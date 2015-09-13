import os
import random
import logging

from webcolors import name_to_rgb as rgb
import cocos
from cocos import sprite, layer, actions, collision_model, draw, euclid

from cocos.actions import Reverse, Repeat, ScaleBy, Rotate
import pyglet

from primitives import Circle, Rect
import config

# Have to do this apparently, otherwise resources aren't found?
pyglet.resource.path = [os.path.join(os.path.realpath(''), 'resources')]
pyglet.resource.reindex()

log = logging.getLogger('Ballmonster')
#log.setLevel(config.LOG_LEVEL)

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

    def update(self, *args, **kwargs):
        self.cshape.center = self.position
        self.position = self.rect.center

    def draw(self):
        super(CollidableSprite, self).draw()

        ## Debug draw circle cshape
        # x,y = self.cshape.center
        # c = Circle(x=x, y=y, width=self.cshape.r*2,  color=(0,0,0,0.4))
        # c.render()

class Player(CollidableSprite):

    def __init__(self, image, *args, **kwargs):
        super(Player, self).__init__(image, *args, **kwargs)

    def walk_left(self):

        pass

    def walk_right(self):
        pass

    def jump(self):
        pass

    def move_down(self):
        pass


class BallMan(Player):

    def __init__(self, *args, **kwargs):
        super(BallMan, self).__init__('ballman128x128.png',
                                      *args, width_multi=0.8, height_multi=0.8,
                                      **kwargs)

class Platform(CollidableSprite):

    def __init__(self, image):
        super(Platform, self).__init__(image,
                                       width_multi=1,
                                       height_multi=1)


class GreyPlatform(Platform):
    def __init__(self):
        super(GreyPlatform, self).__init__('greyplatform256x64.png')


class Level(layer.Layer):

    def __init__(self, player):
        super(Level, self).__init__()
        # Get the platforms layer which contains all the
        # platforms as children
        self.add(player, name='player', z=2)
        self.add(self.build_platforms(), name='platforms_layer', z=1)
        self.add(self.build_background(), name='background_layer')
        self.cm = collision_model.CollisionManagerBruteForce()
        self.cm.add(player)
        for p in self.get('platforms_layer').get_children():
            self.cm.add(p)

    def build_platforms():
        return layer.Layer()

    def build_background():
        return layer.Layer()


class Level0(Level):
    def __init__(self, player):
        super(Level0, self).__init__(player)
        # Place player on a random platform
        platform = random.choice(self.get('platforms_layer').get_children())
        #player.position = platform.x, (platform.y+platform.height/2)+player.height/2
        #player.get_rect().bottom = platform.get_rect().top
        #player.position = 20,20
        player.rect.bottom = platform.rect.top
        player.rect.left = platform.rect.left

    def build_platforms(self):
        x_pos = 0
        l = layer.Layer()
        for i in range(5):
            p = GreyPlatform()
            p.rect.left, p.rect.bottom = x_pos, 0
            l.add(p)
            x_pos += p.width
        p = GreyPlatform()
        p.rect.left, p.rect.bottom = 200, 160
        l.add(p)
        p = GreyPlatform()
        p.rect.left, p.rect.bottom = 600, 160
        l.add(p)
        p = GreyPlatform()
        p.rect.left, p.rect.bottom = 400, 400
        l.add(p)
        return l

    def build_background(self):
        r, g, b, a = rgb('white') + (255, )
        l = layer.ColorLayer(r, g, b, a)
        return l


cocos.director.director.init(width=1280, height=720,
                             caption='Ballmonster',
                             autoscale=True, resizable=True,
                             fullscreen=False)
level0 = Level0(BallMan())
main_scene = cocos.scene.Scene(level0)

cocos.director.director.run(main_scene)



class HelloWorld(cocos.layer.Layer):
    def __init__(self):
        super(HelloWorld, self).__init__()
        label = cocos.text.Label('HelloWorld',
                                 font_name='Arial',
                                 font_size=32,
                                 anchor_x='center', anchor_y='center')
        label.position = 320,240
        self.add(label)

        sprite = cocos.sprite.Sprite('ballman.png')
        sprite.position = 320, 240
        sprite.scale = 1
        self.add(sprite, z=1)

        scale = ScaleBy(3, duration=2)
        label.do(Repeat(scale + Reverse(scale)))
        rotate = Rotate(360, duration=5)
        sprite.do(Repeat(rotate))
