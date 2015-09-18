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
from gamepads import dualshock4

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
            pass
            #self.rect.center = self.position
        else:
            self.position = self.rect.center
        self.cshape.center = self.position

    def set_rect(self, attr, value):
        """
        Set a rect attribute and update sprite position accordingly.
        """
        self.rect.__setattr__(attr, value)
        self.position = self.rect.center
        self.cshape.center = self.position

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

class PlatformMove(actions.Action):
    """
    Move and check for collisions in the environment.
    Target should have a cm (collisionmanager) attribute
    """
    def init(self, *args, **kwargs):
        self.fall_time = 0.0

    def step(self, dt):
        target = self.target
        x, y = target.position
        vx, vy = target.velocity
        vxc, vyc = target.velocity_cap
        ax, ay = target.acceleration

        # Acceleration per frame added to velocity
        vx += ax * dt
        vy += ay * dt
        if vxc and abs(vx) > vxc:
            if vx > 0:
                vx = vxc
            elif vx < 0:
                vx = -vxc
        if vyc and abs(vy) > vyc:
            if vy > 0:
                vy = vyc
            elif vy < 0:
                vy = -vyc
        target.velocity = [vx, vy]
        # Start moving on X axis, then check for collisions
        old_x = target.x
        target.set_rect('left', target.rect.left+(vx*dt))

        log.debug('Target position: %s', target.position)
        log.debug('Target.rect center: %s', target.rect.center)
        log.debug('Target cshape pos: %s', target.cshape.center)
        for ob in target.cm.objs_colliding(target):
            if not isinstance(ob, Platform):
                continue
            if ob.is_wall:
                if vx > 0:
                    target.set_rect('right', ob.rect.left)
                    target.velocity[0] = 0
                    target.acceleration[0] = 0
                elif vx < 0:
                    target.set_rect('left', ob.rect.right)
                    target.velocity[0] = 0
                    target.acceleration[0] = 0
        # target.y += vy * dt
        old_bottom = target.rect.bottom
        old_top = target.rect.top
        target.set_rect('top', target.rect.top+(vy*dt))
        collides = False
        for ob in target.cm.objs_colliding(target):
            if not isinstance(ob, Platform):
                continue
            if ((target.floors_enabled or not ob.can_traverse_down)
                 and vy < 0 and old_bottom >= ob.rect.top):
                target.set_rect('bottom', ob.rect.top)
                target.velocity[1] = 0
                target.acceleration[1] = 0
                collides = True
            elif (ob.is_wall and vy > 0 and old_top <= ob.rect.bottom):
                target.set_rect('top', ob.rect.bottom)
                target.velocity[1] = 0
                target.acceleration[1] = 0
        if collides:
            gravity = 0
            self.fall_time = 0.0
        else:
            gravity = target.gravity
            self.fall_time += dt
            target.velocity[1] += (gravity*dt)*self.fall_time
            #target.velocity[1] += (gravity * self.fall_time * dt)

class Player(CollidableSprite):

    def __init__(self, image, *args, **kwargs):
        super(Player, self).__init__(image, *args, **kwargs)
        self.left_image = self.left_image or image
        self.right_image = self.right_image or image

        self.floors_enabled = True

        # Maximum walk speed
        self.walk_acceleration = kwargs.get('walk_acceleration', 2000)
        # Maximum jump strength
        self.jump_strength = kwargs.get('jump_strength', 1000)

        self.velocity_cap = kwargs.get('velocity_cap', [1000, 0])
        # Use this for dynamic velocity cap, using a multiplier
        self._base_velocity_cap = tuple([x for x in self.velocity_cap])
        self.velocity = [0, 0]
        self.acceleration = [0, 0]
        self.gravity = -20000
        self.walking = False

        self.cm = None

        # X direction either 1 or -1
        self.direction = 1

    def _set_x_accel(self, value):
        self.acceleration[0] = value

    def _set_y_accel(self, value):
        self.acceleration[1] = value

    def _set_x_velocity(self, value):
        self.velocity[0] = value

    def _set_y_velocity(self, value):
        self.velocity[1] = value

    def turn(self, direction):
        if direction > 0:
            self.image = self.right_image
            self.direction = direction
        elif direction < 0:
            self.image = self.left_image
            self.direction = direction
        else:
            self.image = self.right_image
            self.direction = 1

    def walk_left(self, accel_mod=None):
        accel_mod = accel_mod or -1
        self.turn(accel_mod)
        accel = accel_mod*self.walk_acceleration
        self.velocity_cap[0] = abs(accel_mod)*self._base_velocity_cap[0]
        if self.velocity[0] > 0:
            # Is currently walking right
            # Help slow down
            accel -= 1000
        self.walking = True
        self.acceleration[0] = accel
        self._set_x_accel(accel)

    def walk_right(self, accel_mod=None):
        accel_mod = accel_mod or 1
        self.turn(accel_mod)
        accel = accel_mod*self.walk_acceleration
        self.velocity_cap[0] = abs(accel_mod)*self._base_velocity_cap[0]
        if self.velocity[0] < 0:
            # Is currently walking left
            accel += 1000
        self.walking = True
        self.acceleration[0] = accel

    def stop_walk(self):
        """
        Decellerate to stop position.
        """
        self.walking = False
        if -5 <= self.velocity[0] <= 5:
            self.acceleration[0] = 0
            self.velocity[0] = 0
            return
        #self._set_x_velocity(-self.velocity[0]/2)
        #return
        if self.velocity[0] < 0:
            self.acceleration[0] = 4000
        elif self.velocity[0] > 0:
            self.acceleration[0] = -4000

    def jump(self):
        if self.velocity >= 0:
            # Can't jump if already falling
            self.velocity[1] += 95

    def end_jump(self):
        return
        # TODO erase this method
        self._set_y_accel(0)
        self.can_jump = True

    def move_down(self):
        self._set_y_accel(-self.jump_strength)

    def move_down_through_platform(self):
        if self.velocity[1] == 0:
            self._set_y_velocity(-1)
            self.floors_enabled = False

    def reset_move(self):
        pass

    def is_on_solid_ground(self):
        for platform in self.parent.get('platforms_layer').get_children():
            if (platform.rect.top == self.rect.bottom
                and (self.rect.right > platform.rect.left
                     and self.rect.left < platform.rect.right)):
                return True
        return False

    # Attack stuff
    def ranged_attack(self, offset=(1, 1)):
        """
        Give axis offset, x, y.
        """
        # TODO: Make range an attribute for attack class
        arange = 500
        bullet = Bullet(player=self)
        bullet.position = self.position
        log.info('Ranged attack: player.pos:%s, offset: %s',
                 self.position, offset)
        degrees = math.atan2(offset[1], offset[0])*180/math.pi
        rads = degrees * (math.pi/180)
        endpoint = (self.x+math.cos(rads)*arange,
                    self.y+math.sin(rads)*arange)
        log.info('Ranged attack: degrees: %s, rads: %s, endpoint: %s',
                 degrees, rads, endpoint)
        bullet.go(endpoint)

    def update(self, *args, **kwargs):

        super(Player, self).update(*args, **kwargs)
        #self.stop_on_platform()
        log.debug('Velocity: %s, Acceleration: %s',
                  self.velocity, self.acceleration)
        if not self.walking:
            self.stop_walk()
        # Enable floors again after movement is done
        # so we won't keep falling through
        self.floors_enabled = True

class AIPlayer(Player):

    def __init__(self, *args, **kwargs):
        super(AIPlayer, self).__init__(*args, **kwargs)


class BallMan(Player):

    def __init__(self, *args, **kwargs):
        self.right_image = pyglet.resource.image('ballman72x72.png')
        self.left_image = pyglet.resource.image('ballman72x72left.png')
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
        player.cm = self.cm

        self.schedule(self.update)
        self.is_event_handler = True
        self.keys_pressed = set()
        self.add(MouseDisplay(), name='mouse_display', z=3)

        # Joysticks
        log.info('Joysticks: %s', pyglet.input.get_joysticks())
        self.joysticks = pyglet.input.get_joysticks()
        for j in self.joysticks:
            log.info('Opening joystick: %s', j)
            j.open()
            j.set_handler('on_joybutton_press', self.on_joybutton_press)
            j.set_handler('on_joybutton_release', self.on_joybutton_release)
            j.set_handler('on_joyaxis_motion', self.on_joyaxis_motion)

        if self.joysticks:
            self.joystick_player = {self.joysticks[0]: player}
            self.key_handler = self.joystick_handler
        else:
            self.key_handler = self.keyboard_handler

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

    def on_joybutton_press(self, joystick, button):
        pass

    def on_joybutton_release(self, joystick, button):
        pass

    def on_joyaxis_motion(self, joystick, axis, value):
        pass

    def joystick_handler(self):
        joystick = self.joysticks[0]
        p = self.get('player')
        x = joystick.x
        y = joystick.y
        deadzone = 0.4
        down = y > deadzone

        pressed = joystick.buttons
        ds = dualshock4

        if pressed[ds['x']] and down:
            p.move_down_through_platform()
        elif pressed[ds['x']]:
            p.jump()
        else:
            p.end_jump()
        if abs(x) <= deadzone:
            p.stop_walk()
        elif x < 0:
            p.walk_left(x)
        else:
            p.walk_right(x)
        if pressed[ds['circle']]:
            log.info('Joystick: X axis: %s, Y axis: %s', x, y)
            if abs(x) > deadzone or abs(y) > deadzone:
                y *= -1
                p.turn(x)
            else:
                x = p.direction
                y = 0
            # offset = (x if abs(x) > deadzone or abs(y) > deadzone else p.direction,
            #           y*-1 if abs(y) > deadzone or abs(x) > deadzone else 0)
            p.ranged_attack(offset=(x,y))
            p.stop_walk()

    def keyboard_handler(self):
        p = self.get('player')
        pressed = self.keys_pressed
        left = {keycode.LEFT}
        right = {keycode.RIGHT}
        jump = {keycode.SPACE}
        down = {keycode.DOWN}
        ranged_attack = {keycode.S}
        up = {keycode.UP}

        axis = [0,0]

        if left.intersection(pressed):
            axis[0] = -1
            p.walk_left()
        elif right.intersection(pressed):
            axis[0] = 1
            p.walk_right()
        else:
            p.stop_walk()
        if up.intersection(pressed):
            axis[1] = 1
        elif down.intersection(pressed):
            axis[1] = -1
        if len(jump.union(down).intersection(pressed)) >= 2:
            p.move_down_through_platform()
        elif jump.intersection(pressed):
            p.jump()
        else:
            p.end_jump()
        if ranged_attack.intersection(pressed):
            p.ranged_attack(offset=axis)
            p.stop_walk()


    def update(self, *args, **kwargs):
        self.key_handler()

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
        m = PlatformMove(cm=self.cm)
        player.do(m)

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
