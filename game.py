import os
import random
import logging
import math

from webcolors import name_to_rgb as rgb
import cocos
from cocos import sprite, layer, actions, collision_model

import pyglet
from pyglet.window import key as keycode

from primitives import Circle
from util import distance
import config

# Have to do this apparently, otherwise resources aren't found?
pyglet.resource.path = [os.path.join(os.path.realpath(''), 'resources')]
pyglet.resource.reindex()

config.init_log()
log = logging.getLogger('Ballmonster')


def rgba(name, alpha=255):
    return rgb(name) + (alpha,)


class CollidableSprite(sprite.Sprite):

    def __init__(self, image, width_multi=0.9, height_multi=0.9):
        super(CollidableSprite, self).__init__(image)
        # self.cshape = collision_model.CircleShape(self.position,
        #                                           self.height/2)
        box_width = int(self.width*width_multi)
        box_height = int(self.height*height_multi)
        self.cshape = collision_model.AARectShape(self.position,
                                                  box_width/2,
                                                  box_height/2)
        self.schedule(self.update)

        print log.level
        if log.level == logging.DEBUG:
            print 'doing debug'
            r,g,b,a = rgba('yellow', 100)
            c = layer.ColorLayer(r,g,b,a,
                                 width=self.cshape.rx*2,
                                 height=self.cshape.ry*2)
            self.add(c, name='cbox')
            c.position = -c.width/2, -c.height/2

        self.rect = self.get_rect()
        self.rect.width = int(self.width*width_multi)
        self.rect.height = int(self.height*height_multi)

    def update(self, *args, **kwargs):

        if self.are_actions_running():
            self.rect.center = self.position
        else:
            self.position = self.rect.center
        self.cshape.center = self.position

    def set_rect(self, attr, value):
        """
        Set a rect attribute and update sprite position accordingly.
        """
        self.rect.__setattr__(attr, value)
        self.position = self.rect.center

    def draw(self):
        super(CollidableSprite, self).draw()

        ## Debug draw circle cshape
        # x,y = self.cshape.center
        # c = Circle(x=x, y=y, width=self.cshape.r*2,  color=(0,0,0,0.4))
        # c.render()


class SteiniMove(actions.Action):
    """Move the target based on parameters on the target.

    For movement the parameters are::

        target.position = (x, y)
        target.velocity = (dx, dy)
        target.acceleration = (ddx, ddy) = (0, 0)
        target.gravity = 0

    And rotation::

        target.rotation
        target.dr
        target.ddr
    """

    def init(self, *args, **kwargs):
        self.fall_time = 0

    def step(self, dt):
        x, y = self.target.position
        dx, dy = self.target.velocity
        ddx, ddy = getattr(self.target, 'acceleration', (0, 0))
        if self.target.is_on_solid_ground():
            gravity = 0
            self.fall_time = 0.0
        else:
            gravity = getattr(self.target, 'gravity', 0)
            gravity = gravity*self.fall_time
            log.debug('Gravity: %s, fall_time: %s', gravity, self.fall_time)
            self.fall_time += dt
        dx += ddx * dt
        dy += gravity
        #dy = ddy + gravity
        #dy += (ddy + gravity) * dt
        self.target.velocity = (dx, dy)
        x += dx * dt
        y += dy * dt
        # Save old bottom position (for one way platforms)
        self.target.old_bottom = self.target.rect.bottom
        self.target.old_left = self.target.rect.left
        self.target.old_right = self.target.rect.right
        self.target.position = (x, y)
        dr = getattr(self.target, 'dr', 0)
        ddr = getattr(self.target, 'ddr', 0)
        if dr or ddr:
            dr = self.target.dr = dr + ddr*dt
        if dr:
            self.target.rotation += dr * dt


class Player(CollidableSprite):

    def __init__(self, image, *args, **kwargs):
        super(Player, self).__init__(image, *args, **kwargs)
        self.left_image = self.left_image or image
        self.right_image = self.right_image or image

        self.floors_enabled = True

        # Maximum walk speed
        self.walk_speed = kwargs.get('walk_speed', 3000)
        self.walk_velocity_cap = kwargs.get('walk_velocity_cap', 2000)
        # Maximum jump strength
        self.jump_strength = kwargs.get('jump_strength', 1200)
        self.velocity = (0, 0)
        self.acceleration = (0, 0)
        self.gravity = -400
        self.walking = False

        self.move_action = SteiniMove()
        self.do(self.move_action)
        self.old_bottom = self.rect.bottom
        self.old_left = self.rect.left
        self.old_right = self.rect.right

    def _set_x_accel(self, value):
        self.acceleration = value, self.acceleration[1]

    def _set_y_accel(self, value):
        self.acceleration = self.acceleration[0], value

    def _set_x_velocity(self, value):
        self.velocity = value, self.velocity[1]

    def _set_y_velocity(self, value):
        self.velocity = self.velocity[0], value

    def walk_left(self):
        self.image = self.left_image
        log.debug(self.image)
        self.walking = True
        if self.velocity[0] <= -self.walk_velocity_cap:
            self._set_x_accel(0)
        else:
            self.acceleration = -self.walk_speed, self.acceleration[1]

    def walk_right(self):
        self.image = self.right_image
        log.debug(self.image)
        self.walking = True
        if self.velocity[0] >= self.walk_velocity_cap:
            self._set_x_accel(0)
        else:
            self.acceleration = self.walk_speed, self.acceleration[1]

    def stop_walk(self):
        """
        Decellerate to stop position.
        """
        self.walking = False
        if -1 <= self.velocity[0] <= 1:
            self.acceleration = 0, self.acceleration[1]
            self.velocity = 0, self.velocity[1]
            return
        if self.velocity[0] < 0:
            self.acceleration = self.walk_speed, self.acceleration[1]
        elif self.velocity[0] > 0:
            self.acceleration = -self.walk_speed, self.acceleration[1]

    def jump(self):
        if self.velocity[1] == 0:
            self._set_y_velocity(self.jump_strength)
            #self._set_y_accel(self.jump_strength)

    def end_jump(self):
        self._set_y_accel(0)
        self.can_jump = True

    def move_down(self):
        self._set_y_accel(-self.jump_strength)

    def move_down_through_platform(self):
        if self.can_jump:
            self._set_y_velocity(-1)
            self.floors_enabled = False
            self.can_jump = False

    def reset_move(self):
        pass

    def is_on_solid_ground(self):
        for platform in self.parent.get('platforms_layer').get_children():
            if (platform.rect.top == self.rect.bottom
                and (self.rect.right > platform.rect.left
                     and self.rect.left < platform.rect.right)):
                return True
        return False

    def stop_on_platform(self):
        # if self.velocity[1] >= 0:
        #     return
        for ob in self.parent.cm.objs_colliding(self):
            if not isinstance(ob, Platform):
                continue
            if ((self.floors_enabled or not ob.can_traverse_down)
                and self.old_bottom >= ob.rect.top and ob.is_walkable
                and self.velocity[1] < 0):
                self.set_rect('bottom', ob.rect.top)
                self._set_y_accel(0)
                self._set_y_velocity(0)
                return
            if ob.is_wall:
                # Only do X collisions if
                # player is partially outside of x-bounds
                # of box.
                # This is to make sure that player can jump into the
                # bottom of the box without being cast to the side of it.
                # Same with Y collisions, only if bottom of the player is
                # outside the box
                if (self.velocity[0] > 0
                    and self.rect.right > ob.rect.left
                    and self.rect.left < ob.rect.left):
                    # moving right
                    self.set_rect('right', ob.rect.left)
                    self._set_x_velocity(self.velocity[0]*-0.8)
                    self._set_x_accel(-self.walk_speed)
                elif (self.velocity[0] < 0
                      and self.rect.left < ob.rect.right
                      and self.rect.right > ob.rect.right):
                    # moving left
                    self.set_rect('left', ob.rect.right)
                    self._set_x_velocity(self.velocity[0]*-0.8)
                    self._set_x_accel(self.walk_speed)
                if (self.velocity[1] > 0
                    and self.rect.top > ob.rect.bottom
                    and self.rect.bottom < ob.rect.bottom):
                    # Moving up
                    self.set_rect('top', ob.rect.bottom)
                    self._set_y_velocity(-self.velocity[1])


    # Attack stuff
    def ranged_attack(self, target):
        bullet = Bullet(player=self)
        bullet.position = self.position
        bullet.go(target)

    def update(self, *args, **kwargs):

        super(Player, self).update(*args, **kwargs)
        self.stop_on_platform()
        log.debug('Velocity: %s, Acceleration: %s',
                  self.velocity, self.acceleration)
        if not self.walking:
            self.stop_walk()
        # Enable floors again after movement is done
        # so we won't keep falling through
        self.floors_enabled = True
        if abs(self.velocity[0]) > self.walk_velocity_cap:
            xvel = self.velocity[0]
            if xvel > 0:
                newvel = self.walk_velocity_cap
            else:
                newvel = -self.walk_velocity_cap
            self._set_x_velocity(newvel)


class AIPlayer(Player):

    def __init__(self, *args, **kwargs):
        super(AIPlayer, self).__init__(*args, **kwargs)


class BallMan(Player):

    def __init__(self, *args, **kwargs):
        self.right_image = pyglet.resource.image('ballman128x128.png')
        self.left_image = pyglet.resource.image('ballman128x128left.png')
        super(BallMan, self).__init__(self.right_image,
                                      *args, width_multi=0.6, height_multi=0.8,
                                      **kwargs)




class Bullet(CollidableSprite):

    def __init__(self, *args, **kwargs):
        image = pyglet.resource.image('bullet8x8.png')
        self.player = kwargs.pop('player')
        super(Bullet, self).__init__(image, *args,
                                     width_multi=1, height_multi=1,
                                     **kwargs)
        self.player.parent.add(self, z=3)
        self.velocity = 1000

    def go(self, target):
        log.debug('Bullet position: %s, player.position: %s, target: %s',
                  self.position, self.player.position, target)
        d = distance(target, self.position)
        t = d/self.velocity
        self.do(actions.MoveTo(target, t)+actions.CallFunc(self.kill))

    def kill_self(self, *args, **kwargs):
        self.kill()

class Platform(CollidableSprite):

    def __init__(self, image):
        super(Platform, self).__init__(image,
                                       width_multi=1,
                                       height_multi=1)
        self.can_traverse_down = True
        self.is_wall = False
        self.is_walkable = True


class GreyPlatform(Platform):
    def __init__(self):
        super(GreyPlatform, self).__init__('greyplatform256x24.png')


class Wall(Platform):
    def __init__(self):
        super(Wall, self).__init__('wall.png')
        self.is_wall = True
        self.is_walkable = False

class ObstacleBox(Platform):

    def __init__(self):
        super(ObstacleBox, self).__init__('obstaclebox128x128.png')
        self.is_wall = True
        self.is_walkable = True
        self.can_traverse_down = False

class MouseDisplay(layer.Layer):

    def __init__(self, *args, **kwargs):
        super(MouseDisplay, self).__init__(*args, **kwargs)
        self.width = 1280
        self.height = 720
        self.crosshair = sprite.Sprite('crosshair.png')
        self.add(self.crosshair)
        self.is_event_handler = True
        self.crosshair.position = 500, 500

    def on_mouse_motion(self, x, y, dx, dy):
        self.update_crosshair(x, y)

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        self.update_crosshair(x, y)

    def update_crosshair(self, x, y):
        self.crosshair.position = x, y

class Level(layer.Layer):

    def __init__(self, player):
        super(Level, self).__init__()
        # Get the platforms layer which contains all the
        # platforms as children
        self.add(player, name='player', z=2)
        self.add(self.build_platforms(), name='platforms_layer', z=1)
        self.add(self.build_background(), name='background_layer', z=0)
        self.cm = collision_model.CollisionManagerBruteForce()
        self.cm.add(player)
        for p in self.get('platforms_layer').get_children():
            self.cm.add(p)

        self.schedule(self.update)
        self.is_event_handler = True
        self.keys_pressed = set()
        self.add(MouseDisplay(), name='mouse_display', z=3)

    def build_platforms():
        return layer.Layer()

    def build_background():
        return layer.Layer()

    def on_key_press(self, key, modifiers):
        self.keys_pressed.add(key)

    def on_key_release(self, key, modifiers):
        self.keys_pressed.remove(key)

    def on_mouse_press(self, x, y, buttons, modifiers):
        p = self.get('player')
        p.ranged_attack((x, y))

    def update(self, *args, **kwargs):
        p = self.get('player')
        pressed = self.keys_pressed
        left = {keycode.A, keycode.LEFT}
        right = {keycode.D, keycode.RIGHT}
        jump = {keycode.SPACE, keycode.W, keycode.UP}
        down = {keycode.DOWN, keycode.S}

        if left.intersection(pressed):
            p.walk_left()
        elif right.intersection(pressed):
            p.walk_right()
        else:
            p.stop_walk()
        if len(jump.union(down).intersection(pressed)) >= 2:
            p.move_down_through_platform()
        elif jump.intersection(pressed):
            p.jump()
        else:
            p.end_jump()


class Level0(Level):
    def __init__(self, player):
        super(Level0, self).__init__(player)
        # Place player on a random platform
        platform = None
        while True:
            platform = random.choice(
                self.get('platforms_layer').get_children())
            if not platform.is_wall:
                break
        player.set_rect('bottom', platform.rect.top)
        player.set_rect('left', platform.rect.left+20)

    def build_platforms(self):
        x_pos = 0
        l = layer.Layer()
        for i in range(5):
            p = GreyPlatform()
            p.rect.left, p.rect.bottom = x_pos, 0
            l.add(p)
            x_pos += p.width
            p.can_traverse_down = False
        p = GreyPlatform()
        p.rect.left, p.rect.bottom = 100, 160
        l.add(p)
        p = GreyPlatform()
        p.rect.left, p.rect.top = 500, 200
        l.add(p)
        p = GreyPlatform()
        p.rect.left, p.rect.bottom = 300, 400
        l.add(p)

        # Make an obstacle
        p = ObstacleBox()
        p.rect.left, p.rect.bottom = 500, 30
        l.add(p)

        p = ObstacleBox()
        p.rect.left, p.rect.bottom = 928, 30
        l.add(p)

        p = ObstacleBox()
        p.rect.left, p.rect.bottom = 1050, 30
        l.add(p)

        p = ObstacleBox()
        p.rect.left, p.rect.bottom = 928, 400
        l.add(p)

        window = cocos.director.director.get_window_size()

        w = Wall()
        w.rect.left = 0
        w.rect.bottom = 24
        l.add(w)
        w = Wall()
        w.rect.right = window[0]
        w.rect.bottom = 24
        l.add(w)
        return l

    def build_background(self):
        r, g, b, a = rgb('white') + (255, )
        l = layer.ColorLayer(r, g, b, a)
        return l


cocos.director.director.init(width=1280, height=720,
                             caption='Ballmonster',
                             autoscale=True, resizable=True,
                             fullscreen=False)


#cocos.director.director.fps_display = pyglet.clock.ClockDisplay(color=rgba('blue'))
cocos.director.director.show_FPS = True

#cocos.director.director.fps_display.color = rgba('blue')
level0 = Level0(BallMan())
main_scene = cocos.scene.Scene(level0)

cocos.director.director.run(main_scene)
