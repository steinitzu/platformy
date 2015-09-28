from util import distance

class PathNode(object):

    def __init__(self, x, y, platform=None, all_nodes=None):
        self.x = x
        self.y = y
        self.platform = platform
        self.all_nodes = all_nodes

    def __getitem__(self, index):
        return (self.x, self.y, self.platform)[index]

    def __str__(self):
        return '{}, {}'.format(self.x, self.y)

    def __repr__(self):
        return str(self)

    def __lt__(self, other):
        return self.x < other.x

    def intersects_platform_x(self, platform):
        """
        Chek if node is within given platform's
        horizontal bounds.
        """
        return platform.rect.left < self.x < platform.rect.right

    def distance(self, othernode):
        return distance((self.x, self.y),
                        (othernode.x, othernode.y))

    def get_edges(self, player):
        if not self.all_nodes:
            return set()
        return self.all_nodes[self][player.__class__]
