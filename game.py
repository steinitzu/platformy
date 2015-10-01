import os
import random
import logging
import math
import warnings
from collections import namedtuple
import threading

from webcolors import name_to_rgb as rgb
import cocos
from cocos import sprite, layer, actions, collision_model
from cocos.rect import Rect

import pyglet
from pyglet.window import key as keycode

from primitives import Circle
from util import distance, PriorityQueue
import config
from gamepads import dualshock4
from pathfinding import PathNode

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
            path_nodes = self.get_ancestor(Level).path_nodes
            for node in path_nodes:
                x,y = node.x, node.y
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
        self._halt_timer = 0

    def step(self, dt):
        # dt is hard coded to 60 fps
        # Game will run slower at lower than 60 and faster at higher
        # Prevents "bullet through paper" at low fps
        if dt > 1.0/30:
            dt = 1.0/60
        target = self.target
        x, y = target.position
        vx, vy = target.velocity
        vxc, vyc = target.velocity_cap
        ax, ay = target.acceleration

        if hasattr(target, 'halt_time') and target.halt_time:
            if self._halt_timer < target.halt_time:
                self._halt_timer += dt
                vx = 0
                ax = 0
                ay = 0
            else:
                target.halt_time = 0
                self._halt_timer = 0

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
        self.is_jumping = False

        # Attacks
        self.attacks = {'ranged': Gun(self, [])}
        for a in self.attacks.values():
            self.schedule(a.update)
        self.current_node = None

    def get_path_node(self):
        return PathNode(self.rect.left,
                        self.rect.bottom)

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
        if self.is_jumping:
            return
        self.is_jumping = True
        if self.velocity[1] == 0:
            # Set acceleration  only when player is standing on ground
            # Accelration is stopped in Move Action once player
            # reaches Y velocity cap
            self.acceleration[1] = self.jump_strength

    def end_jump(self):
        self.is_jumping = False
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

    def set_current_node(self):
        if self.velocity[1] != 0:
            # Don't set while in air
            return
        all_nodes = self.get_ancestor(Level).path_nodes
        midpoint = (self.rect.left+self.rect.right)/2
        nearestd = None
        nearestnode = None
        for node in all_nodes:
            if node.y != self.rect.bottom:
                continue
            d = abs(midpoint - node.x)
            # d = distance((midpoint, self.rect.bottom),
            #              (node.x, node.y))
            if not nearestnode or d < nearestd:
                nearestnode = node
                nearestd = d
        if nearestnode:
            self.current_node = nearestnode

        log.debug('%s current node is %s', self, self.current_node)

    # Attack stuff
    def ranged_attack(self, offset=(1, 1)):
        a = self.attacks['ranged']
        a.perform(offset)

    def update(self, dt, *args, **kwargs):
        super(Player, self).update(dt, *args, **kwargs)
        log.debug('%s: Velocity: %s, Acceleration: %s',
                  self, self.velocity, self.acceleration)
        if not self.walking:
            self.stop_walk()
        self.floors_enabled = True
        self.set_current_node()


class AIPlayer(Player):

    def __init__(self, *args, **kwargs):
        super(AIPlayer, self).__init__(*args, **kwargs)
        # TODO: Calculate correct jump distance
        self.max_jump_distance = 400
        self.max_jump_height = 240

        # Target player
        self.target = None
        self.last_target_node = None
        self.destination = None
        self.path = []

    def _heuristic(self, a, b):
        return a.distance(b)
        (x1, y1) = a.x, a.y
        (x2, y2) = b.x, b.y
        return abs(x1 - x2) + abs(y1 - y2)

    def get_path(self, goal):
        current_node = self.current_node
        if not current_node or not goal:
            return []
        elif current_node == goal:
            return [goal]
        if not self.last_target_node:
            self.last_target_node = goal
        elif self.last_target_node == goal:
            if self.destination not in self.current_node.get_edges(self):
                pass
            else:
                log.info('Reusing last path: %s', self.path)
                return self.path
        else:
            self.last_target_node = goal

        start = current_node
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()
            if current is goal:
                break
            # Get current node edges
            for next in current.get_edges(self):
                new_cost = cost_so_far[current] + current.cost(next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self._heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
        path = []
        path.append(goal)
        current = goal
        while True:
            try:
                current = came_from[current]
            except KeyError:
                log.warning('No available path to %s', current)
                break
            if not current:
                break
            path.append(current)
        # Pop current node from path
        path.pop()
        return path

    def follow(self):
        # TODO: Don't pop until player is at destination
        if not self.target:
            return
        path = self.get_path(self.target.current_node)
        if (not self.destination
            or self.destination == self.current_node
            or path != self.path
            or (len(path) > 0 and self.destination != path[-1])):
            try:
                self.destination = self.path.pop()
            except IndexError:
                self.path = path
                return
        self.path = path
        log.info('Path: %s', self.path)
        log.info('Next destination: %s', self.destination)
        log.info('Current node: %s', self.current_node)
        self.go_to_destination(self.destination)
        return


    def go_to_destination(self, dest):
        cx, cy = self.rect.left, self.rect.bottom
        cmidx = (self.rect.left+self.rect.right)/2
        dx = dest.x - cmidx
        # No jump button release, can always jump
        self.is_jumping = False
        if self.current_node == dest:
            self.stop_walk()
        elif dx < -10:
            self.walk(-1)
        elif dx > 10:
            self.walk(1)
        else:
            self.stop_walk()
        if dest.y > cy:
            self.jump()
        else:
            self.end_jump()
        cnode = self.current_node
        if dest.y < cy:
            if (self.velocity[1] == 0
                and cnode.platform.can_traverse_down):
                self.move_down_through_platform()

    def halt(self, seconds=1):
        self.halt_time = seconds

    def update(self, dt, *args, **kwargs):
        super(AIPlayer, self).update(dt, *args, **kwargs)
        self.follow()
        if random.randrange(1000) == 3:
           log.info('%s halting', self)
           self.halt(2)


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
        self.width = 2560
        self.height = 2048
        # Get the platforms layer which contains all the
        # platforms as children
        self.scroller = layer.scrolling.ScrollableLayer()
        self.scroller.px_width, self.scroller.px_height = (
             self.width, self.height)
        V = namedtuple('V', ['width', 'height'])
        v = V(1280*2, 720)
        self.scroll_man = layer.scrolling.ScrollingManager()
        self.scroll_man.add(self.scroller)
        self.add(self.scroll_man)
        self.platforms = self.build_platforms()
        self.background = self.build_background()
        self.enemies = self.build_enemies()
        self.player = player

        self.scroller.add(player, name='player', z=2)
        self.scroller.add(self.platforms, name='platforms_layer', z=1)
        self.scroller.add(self.background, name='background_layer', z=0)
        self.scroller.add(self.enemies, name='enemies_layer', z=2)
        #self.cm = collision_model.CollisionManagerBruteForce()
        self.cm = collision_model.CollisionManagerGrid(
            0, self.width, 0, self.height,
            128, 128)

        #self.cm.add(player)
        for p in self.platforms.get_children():
            self.cm.add(p)
        player.cm = self.cm

        self.path_nodes = self.build_pathnodes()

        # Generate graph of edges for each player class per pathnode
        for e in self.enemies.get_children():
            # Give the players a reference to the platform
            # collision manager
            e.cm = self.cm
            for node in self.path_nodes:
                node.get_edges(e)
                log.info('Edges for %s: %s',
                         node, node.edges)

        if log.level == logging.DEBUG:
            #dlayer = cocos.layer.Layer()
            dlayer = cocos.batch.BatchableNode()
            self.scroller.add(dlayer, name='debug_layer', z=2)
            #batch = cocos.batch.BatchableNode()
            #dlayer.add(batch)
            batch = dlayer
            e = self.enemies.get_children()[0]
            linecount = 0
            for node in self.path_nodes:
                for edge in node.get_edges(e):
                    if edge.y > node.y:
                        color = rgba('green', 60)
                    elif edge.y < node.y:
                        color = rgba('red', 50)
                    else:
                        color = rgba('purple', 50)
                    l = cocos.draw.Line(
                        (node.x, node.y), (edge.x, edge.y), color)
                    linecount += 1
                    batch.add(l)
            log.debug('LineCount: %s', linecount)

        m = PlatformMove(cm=self.cm)
        player.do(m)
        for e in self.enemies.get_children():
            m = PlatformMove(cm=self.cm)
            e.do(m)

        self.schedule(self.update)
        self.is_event_handler = True
        self.keys_pressed = set()
        self.modifiers = None

        # Joysticks
        log.info('Joysticks: %s', pyglet.input.get_joysticks())
        self.joysticks = pyglet.input.get_joysticks()
        for j in self.joysticks:
            log.info('Opening joystick: %s', j)
            j.open()

        if self.joysticks:
            self.joystick_player = {self.joysticks[0]: player}
            self.key_handler = self.joystick_handler
        else:
            self.key_handler = self.keyboard_handler

        # TODO: higher screen resolution instead of this
        # eye = self.camera.eye
        # eye.z += 200
        # self.camera.eye = eye

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
        for platform in self.platforms.get_children():
            log.debug('Building pathnodes, platform: %s', platform)
            if not platform.is_walkable:
                continue
            p.rect.left = platform.rect.left
            p.rect.bottom = platform.rect.top
            last = False
            while True:
                if p.rect.left > platform.rect.right:
                    break
                if p.rect.left >= self.width:
                    break
                obstacles = self.cm.objs_colliding(p)
                blocked = False
                for obs in obstacles:
                    if obs.is_wall:
                        blocked = True
                        break
                if blocked:
                    log.debug('Blocked at pos: (%s, %s), moving 1 px right',
                              p.rect.left, p.rect.bottom)
                    p.rect.left += 128
                else:
                    node = PathNode(p.rect.left, platform.rect.top, platform)
                    log.debug('Clear, creating node at: %s', node)
                    nodes.add(node)
                    node.all_nodes = nodes
                    p.rect.left += 128
                # if p.rect.left >= platform.rect.right:
                #     if not last and not blocked:
                #         p.rect.left = platform.rect.right
                #         last = True
                #     else:
                #         break
        return nodes

    def on_key_press(self, key, modifiers):
        self.keys_pressed.add(key)
        self.modifiers = modifiers

    def on_key_release(self, key, modifiers):
        try:
            self.keys_pressed.remove(key)
        except KeyError:
            pass
        self.modifiers = modifiers

    def on_mouse_press(self, x, y, buttons, modifiers):
        p = self.player
        p.ranged_attack((x, y))

    def joystick_handler(self):
        joystick = self.joysticks[0]
        p = self.player
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
        p = self.player
        pressed = self.keys_pressed
        modifiers = self.modifiers
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

        #Eye = namedtuple('Eye', ('x', 'y', 'z'))
        eye = self.camera.eye
        if keycode.Y in pressed:
            if modifiers & keycode.MOD_CTRL:
                eye.y -= 10
            else:
                eye.y += 10
        x, y = self.scroller.view_x, self.scroller.view_y
        w, h = self.scroller.view_w, self.scroller.view_h
        if keycode.X in pressed:
            if modifiers & keycode.MOD_CTRL:
                x -= 10
            else:
                x += 10
        self.scroller.set_view(x, y, w, h)
        if keycode.Z in pressed:
            if modifiers & keycode.MOD_CTRL:

                eye.z -= 10
            else:
                eye.z += 10

        self.camera.eye = eye

    def update(self, *args, **kwargs):
        self.key_handler()
        self.scroll_man.set_focus(self.player.x, self.player.y)
        # eye = self.camera.eye
        # Eye = namedtuple('Eye', ['x', 'y', 'z'])
        # e = Eye(0, 0, eye.z)
        # self.camera.center = e
        # log.info('Enemies width: %s, height:%s',
        #          self.enemies.width, self.enemies.height)


class Level0(Level):
    def __init__(self, player):
        super(Level0, self).__init__(player)
        # Place player on a random platform
        platform = None
        while True:
            platform = random.choice(
                self.platforms.get_children())
            if not platform.is_wall:
                break
        player.rect.left, player.rect.bottom = (
            platform.rect.left+20, 500)
        for e in self.enemies.get_children():
            e.rect.left, e.rect.bottom = 200,500
            e.target = player

    def build_platforms_random(self):
        count = 0
        ps = []
        while count < 38:
            x = random.randrange(40, self.width-300)
            y = random.randrange(40, self.height-100)
            p = GreyPlatform()
            p.rect.left, p.rect.bottom = x, y
            ps.append(p)
            count += 1
        return ps


    def build_platforms(self):
        x_pos = 0
        l = layer.Layer()
        while x_pos <= self.width-256:
        #for i in range(10):
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
        p = GreyPlatform()
        p.rect.left, p.rect.bottom = 800, 200
        l.add(p)

        p = ObstacleBox()
        p.rect.left, p.rect.bottom = 928, 400
        l.add(p)

        p = ObstacleBox()
        p.rect.left, p.rect.bottom = 500, 24
        l.add(p)

        p = ObstacleBox()
        p.rect.left, p.rect.bottom = 1056, 420
        l.add(p)

        window = cocos.director.director.get_window_size()

        w = Wall()
        w.rect.left, w.rect.bottom = 0, 24
        l.add(w)
        w = Wall()
        w.rect.right, w.rect.bottom = 2560, 24
        l.add(w)
        for p in self.build_platforms_random():
            l.add(p)
        return l

    def build_background(self):
        r, g, b, a = rgb('white') + (255, )
        l = layer.ColorLayer(r, g, b, a, self.width, self.height)
        return l

    def build_enemies(self):
        #l = layer.Layer()
        l = cocos.batch.BatchNode()
        for i in range(15):
            e = EvilBallman()
            l.add(e)
            e.rect.left, e.rect.top = 200, 600
        return l

cocos.director.director.init(width=1920, height=1080,
                             caption='Ballmonster',
                             autoscale=True, resizable=True,
                             fullscreen=False)
cocos.director.director.show_FPS = True

level0 = Level0(BallMan())
main_scene = cocos.scene.Scene(level0)

cocos.director.director.run(main_scene)
