import os
import random
import logging
import math
import warnings

from webcolors import name_to_rgb as rgb
import cocos
from cocos import sprite, layer, actions, collision_model
from cocos.rect import Rect

import pyglet
from pyglet.window import key as keycode

from primitives import Circle, Line
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


class RangedAttack(object):

    def __init__(self, player, targets):
        self.player = player
        self.targets = targets
        self.timer = 0.0
        self.started = False
        # Cool down period in seconds between shots
        self.cool_down_time = 0.3
        # Weapon range in pixels
        self.range = 1000

    def tick(self, dt):
        if self.started:
            self.timer += dt
            if self.timer >= self.cool_down_time:
                self.timer = 0.0
                self.stop()

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def can_perform(self):
        return self.timer == 0

    def perform(self, target):
        raise NotImplementedError

    def update(self, dt, *args, **kwargs):
        self.tick(dt)
        log.info('Gun timer: %s', self.timer)


class Gun(RangedAttack):

    def __init__(self, player, targets):
        super(Gun, self).__init__(player, targets)

    def get_target(self, offset=(1, 1)):
        arange = self.range
        degrees = math.atan2(offset[1], offset[0])*180/math.pi
        rads = degrees * (math.pi/180)
        endpoint = (self.player.x+math.cos(rads)*arange,
                    self.player.y+math.sin(rads)*arange)
        return endpoint

    def perform(self, offset=(1, 1)):
        if not self.can_perform():
            return
        bullet = Bullet(player=self.player)
        bullet.position = self.player.position
        target = self.get_target(offset)
        self.start()
        bullet.go(target)


class BoundRect(Rect):

    """
    A Rect object that updates sprite position accordingly on any position changes.
    """

    def __init__(self, *args, **kwargs):
        self.sprite = kwargs.pop('sprite')
        super(BoundRect, self).__init__(*args, **kwargs)

    def _set_property(self, name, value):
        object.__setattr__(self, name, value)
        self.sprite.position = self.center
        # TODO: Set sprite cshape as well
        if hasattr(self.sprite, 'cshape'):
            self.sprite.cshape.center = self.sprite.position

    def __setattr__(self, name, value):
        posattrs = ('bottom', 'top', 'left', 'right',
                    'center', 'midtop', 'midbottom',
                    'midleft', 'midright', 'topleft', 'topright',
                    'bottomleft', 'bottomright')
        if name in posattrs:
            self._set_property(name, value)
        else:
            object.__setattr__(self, name, value)


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

    def update(self, dt, *args, **kwargs):
        pass

    def get_rect(self):
        """
        Overriden to return a BoundRect insted of cocos.rect.Rect
        """
        x, y = self.position
        x -= self.image_anchor_x
        y -= self.image_anchor_y
        return BoundRect(x, y, self.width, self.height, sprite=self)

    def draw(self):
        super(CollidableSprite, self).draw()
        if not isinstance(self, BallMan):
            return

        if log.level == logging.DEBUG:
            path_nodes = self.parent.path_nodes
            for node in path_nodes:
                x,y = node
                c = Circle(x,y,width=10,color=(0.,.9,0.,1.))
                c.render()

        ## Debug draw circle cshape
        # x,y = self.cshape.center
        # c = Circle(x=x, y=y, width=self.cshape.r*2,  color=(0,0,0,0.4))
        # c.render()

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
        if vxc and abs(vx) > vxc:
            if vx > 0:
                vx = vxc
            elif vx < 0:
                vx = -vxc
        if vyc and vy >= vyc:
            # No downward velocity cap
            vy = vyc
            target.acceleration[1] = 0
        else:
            vy += (ay * dt)
        vy -= (target.gravity*dt)
        target.velocity = [vx, vy]
        # Start moving on X axis, then check for collisions
        old_x = target.x
        target.rect.left += (vx*dt)
        log.debug('Target.rect center: %s', target.rect.center)
        for ob in target.cm.objs_colliding(target):
            if not isinstance(ob, Platform):
                continue
            if ob.is_wall:
                if vx > 0:
                    target.rect.right = ob.rect.left
                    target.velocity[0] = 0
                    target.acceleration[0] = 0
                elif vx < 0:
                    target.rect.left = ob.rect.right
                    target.velocity[0] = 0
                    target.acceleration[0] = 0
        old_bottom = target.rect.bottom
        old_top = target.rect.top
        target.rect.top += (vy*dt)
        collides = False
        for ob in target.cm.objs_colliding(target):
            if not isinstance(ob, Platform):
                continue
            if ((target.floors_enabled or not ob.can_traverse_down)
                 and vy < 0 and old_bottom >= ob.rect.top):
                target.rect.bottom = ob.rect.top
                target.velocity[1] = 0
                target.acceleration[1] = 0
                collides = True
            elif (ob.is_wall and vy > 0 and old_top <= ob.rect.bottom):
                target.rect.top = ob.rect.bottom
                target.velocity[1] = 0
                target.acceleration[1] = 0


class Player(CollidableSprite):

    def __init__(self, image, *args, **kwargs):
        super(Player, self).__init__(image, *args, **kwargs)
        self.left_image = self.left_image or image
        self.right_image = self.right_image or image

        self.floors_enabled = True

        self.walk_acceleration = kwargs.get('walk_acceleration',
                                            15*config.METER)
        self.jump_strength = kwargs.get('jump_strength', 45*config.METER)
        self.velocity_cap = kwargs.get('velocity_cap', [6*config.METER,
                                                        7.5*config.METER])

        self._base_velocity_cap = tuple([x for x in self.velocity_cap])
        self.velocity = [0, 0]
        self.acceleration = [0, 0]
        self.gravity = config.GRAVITY
        self.walking = False

        self.cm = None

        # X direction either 1 or -1
        self.direction = 1

        # Attacks
        self.attacks = {'ranged': Gun(self, [])}
        for a in self.attacks.values():
            self.schedule(a.update)

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

    def walk(self, accel_mod=1):
        accel = accel_mod*self.walk_acceleration
        self.velocity_cap[0] = abs(accel_mod)*self._base_velocity_cap[0]
        self.turn(accel_mod)
        if self.velocity[0] * accel < 0:
            # Is switching directions
            accel += accel_mod*self.walk_acceleration
        self.walking = True
        self.acceleration[0] = accel

    def stop_walk(self):
        """
        Decellerate to stop position.
        """
        self.walking = False
        if self.velocity[0]*self.direction < 0:
            # To make sure player doesn't accelerate much beyond 0
            # will stop as soon as he starts travelling in the opposite
            # direction he's facing
            self.acceleration[0] = 0
            self.velocity[0] = 0
            return
        elif abs(self.velocity[0]) > 0:
            # Accelerate opposite current walk direction
            self.acceleration[0] = -self.direction*self.walk_acceleration

    def jump(self):
        if self.velocity[1] == 0:
            # Set acceleration  only when player is standing on ground
            # Accelration is stopped in Move Action once player
            # reaches Y velocity cap
            self.acceleration[1] = self.jump_strength

    def end_jump(self):
        self.acceleration[1] = 0

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
        a = self.attacks['ranged']
        a.perform(offset)

    def update(self, dt, *args, **kwargs):
        super(Player, self).update(dt, *args, **kwargs)
        log.debug('%s: Velocity: %s, Acceleration: %s',
                  self, self.velocity, self.acceleration)
        log.info(dt)
        if not self.walking:
            self.stop_walk()
        self.floors_enabled = True


class AIPlayer(Player):

    def __init__(self, *args, **kwargs):
        super(AIPlayer, self).__init__(*args, **kwargs)
        # TODO: Calculate correct jump distance
        self.max_jump_distance = 200
        self.max_jump_height = 200

    def get_edges(self, node, all_nodes):
        _test_edges = set()
        edges = set()
        # Get all nodes with same y value
        same_y = [n for n in all_nodes if node[1] == n[1]]
        # Sort them by x value
        same_y = sorted(same_y)
        index = same_y.index(node)
        if index == 0:
            _test_edges.add(same_y[1])
        elif index == len(same_y)-1:
            _test_edges.add(same_y[index-1])
        else:
            _test_edges.add(same_y[index+1])
            _test_edges.add(same_y[index-1])
        while len(_test_edges) > 0:
            edge = _test_edges.pop()
            if abs(edge[0] - node[0]) > self.max_jump_distance:
                continue
            # Check for obstacles between the two nodes
            # start by placing player on the current node
            x, y = node
            ex, ey = edge
            left = False
            right = False
            if ex < x:
                # edge is left of
                left = True
                movemod = -1
            elif ex > x:
                right = True
                movemod = 1
            self.rect.left, self.rect.bottom = x, y
            path_blocked = False
            while not path_blocked:
                if left and self.rect.left < ex:
                    break
                elif right and self.rect.left > ex:
                    break
                for obstacle in self.cm.objs_colliding(self):
                    if obstacle.is_wall:
                        path_blocked = True
                        break
                self.rect.left += movemod
            if not path_blocked:
                edges.add(edge)
        different_y = sorted([n for n in all_nodes
                              if node[1] != n[1]])
        higher = sorted([n for n in all_nodes
                         if n[1] > node[1]])
        lower = sorted([n for n in all_nodes
                        if n[1] < node[1]])
        # TODO: get edges for platforms on different y positions
        # pedge = potential edge
        for pedge in lower:
            if abs(pedge[0] - node[0]) > self.max_jump_distance:
                continue
            x, y = node
            ex, ey = pedge
            left, right = False, False
            if ex < x:
                left = True
                movemod = -1
            elif ex > x:
                right = True
                movemod = 1
            self.rect.left, self.rect.bottom = node
            path_blocked = False
            while not path_blocked:
                # Start by moving x towards node, then y
                # Check for obstacles on each iteration
                # log.debug('Checking edge %s against %s', pedge, node)
                # log.debug('Dummy position: %s',
                #           (self.rect.left, self.rect.bottom))
                if self.rect.left == ex and self.rect.bottom == ey:
                    break
                if not self.rect.bottom == ey:
                    self.rect.bottom -= 1
                if not self.rect.left == ex:
                    self.rect.left += movemod
                for obstacle in self.cm.objs_colliding(self):
                    if obstacle.is_wall or not obstacle.can_traverse_down:
                        path_blocked = True
                        break
            if not path_blocked:
                edges.add(pedge)


        # for pedge in different_y:
        #     if (abs(pedge[0]-node[0]) > self.max_jump_distance
        #         or abs(pedge[1]-node[1] > self.max_jump_height)):
        #         # Edge is too far away, carry on
        #         continue


        return edges


class EvilBallman(AIPlayer):

    def __init__(self, *args, **kwargs):
        self.right_image = pyglet.resource.image('ballman72x72.png')
        self.left_image = pyglet.resource.image('ballman72x72left.png')
        super(EvilBallman, self).__init__(self.right_image,
                                          *args,
                                          width_multi=0.6, height_multi=0.8,
                                          **kwargs)


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
        self.add(self.build_enemies(), name='enemies_layer', z=2)
        self.cm = collision_model.CollisionManagerBruteForce()
        #self.cm.add(player)
        for p in self.get('platforms_layer').get_children():
            self.cm.add(p)
        player.cm = self.cm

        # Path nodes will be {node: {player class: set(edges)}}
        # edges = self.path_nodes[node][player.__class__]
        self.path_nodes = self.build_pathnodes()

        for e in self.get('enemies_layer').get_children():
            e.cm = self.cm
            for node in self.path_nodes:
                edges = e.get_edges(node, self.path_nodes)
                self.path_nodes[node][e.__class__] = edges
                log.info('Edges for %s: %s',
                         node, edges)

        if log.level == logging.DEBUG:
            #self.add(layer.Layer(), name='debug_layer', z=2)
            dlayer = cocos.layer.Layer()
            self.add(dlayer, name='debug_layer', z=2)
            batch = cocos.batch.BatchableNode()
            dlayer.add(batch)

            # for i in xrange(50):
            #     a = (random.randrange(1280),
            #          random.randrange(720))
            #     b = (random.randrange(1280),
            #          random.randrange(720))
            #     color = rgba('green', 100)
            #     l = cocos.draw.Line(a, b, color)
            #     batch.add(l)


            e = self.get('enemies_layer').get_children()[0]
            linecount = 0
            for node in self.path_nodes:
                for edge in self.path_nodes[node][e.__class__]:
                    alpha = 60
                    if edge[1] > node[1]:
                        color = rgba('green', alpha)
                    elif edge[1] < node[1]:
                        color = rgba('red', alpha)
                    else:
                        color = rgba('purple', alpha)
                    l = cocos.draw.Line(node, edge, color)
                    linecount +=1
                    batch.add(l)
                    #self.get('debug_layer').add(l)
            log.debug('LineCount: %s', linecount)

        m = PlatformMove(cm=self.cm)
        player.do(m)
        for e in self.get('enemies_layer').get_children():
            m = PlatformMove(cm=self.cm)
            e.do(m)

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

    def build_platforms(self):
        return layer.Layer()

    def build_background(self):
        return layer.Layer()

    def build_enemies(self):
        return layer.Layer()

    def build_pathnodes(self):
        log.debug('Building pathnodes for level %s', self)
        p = BallMan()
        nodes = set()
        for platform in self.get('platforms_layer').get_children():
            log.debug('Building pathnodes, platform: %s', platform)
            if not platform.is_walkable:
                continue
            p.rect.left = platform.rect.left
            p.rect.bottom = platform.rect.top
            last = False
            while True:
                obstacles = self.cm.objs_colliding(p)
                blocked = False
                for obs in obstacles:
                    if obs.is_wall:
                        blocked = True
                        break
                if blocked:
                    log.debug('Blocked at pos: (%s, %s), moving 1 px right',
                              p.rect.left, p.rect.bottom)
                    p.rect.left += 1
                else:
                    node = (p.rect.left, platform.rect.top)
                    log.debug('Clear, creating node at: %s', node)
                    nodes.add(node)
                    p.rect.left += config.METER/2
                if p.rect.left >= platform.rect.right:
                    if not last and not blocked:
                        p.rect.left = platform.rect.right
                        last = True
                    else:
                        break
        _nodes = {}
        for node in nodes:
            _nodes[node] = {}
        return _nodes

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
        if abs(x) < deadzone:
            p.stop_walk()
        else:
            p.walk(x)
        if pressed[ds['circle']]:
            log.info('Joystick: X axis: %s, Y axis: %s', x, y)
            if abs(x) > deadzone or abs(y) > deadzone:
                y *= -1
                p.turn(x)
            else:
                x = p.direction
                y = 0
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
            p.walk(-1)
        elif right.intersection(pressed):
            axis[0] = 1
            p.walk(1)
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
            x = axis[0] if axis[0] or axis[1] else p.direction
            p.ranged_attack(offset=(x, axis[1]))
            p.stop_walk()

    def draw(self):

        super(Level, self).draw()



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
        player.rect.left, player.rect.bottom = (
            platform.rect.left+20, 500)

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
        p.rect.left, p.rect.bottom = 500, 200
        l.add(p)
        p = GreyPlatform()
        p.rect.left, p.rect.bottom = 300, 400
        l.add(p)

        p = ObstacleBox()
        p.rect.left, p.rect.bottom = 928, 400
        l.add(p)

        p = ObstacleBox()
        p.rect.left, p.rect.bottom = 500, 24
        l.add(p)

        window = cocos.director.director.get_window_size()

        w = Wall()
        w.rect.left, w.rect.bottom = 0, 24
        l.add(w)
        w = Wall()
        w.rect.right, w.rect.bottom = window[0], 24
        l.add(w)
        return l

    def build_background(self):
        r, g, b, a = rgb('white') + (255, )
        l = layer.ColorLayer(r, g, b, a)
        return l

    def build_enemies(self):
        l = layer.Layer()
        for i in range(2):
            e = EvilBallman()
            l.add(e)
            e.rect.left, e.rect.top = 200, 600
        return l

cocos.director.director.init(width=1280, height=720,
                             caption='Ballmonster',
                             autoscale=True, resizable=True,
                             fullscreen=False)
cocos.director.director.show_FPS = True

level0 = Level0(BallMan())
main_scene = cocos.scene.Scene(level0)

cocos.director.director.run(main_scene)
