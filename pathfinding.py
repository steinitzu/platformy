from util import distance

class PathNode(object):

    def __init__(self, x, y, platform=None, all_nodes=None):
        self.x = x
        self.y = y
        self.platform = platform
        self.all_nodes = all_nodes
        # Stores edges as {player.__class__: set()}
        self.edges = {}
        self.unreachable = False

    def __getitem__(self, index):
        return (self.x, self.y, self.platform)[index]

    def __str__(self):
        return '({}, {})'.format(self.x, self.y)

    def __repr__(self):
        return str(self)

    def __lt__(self, other):
        return self.x < other.x

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        try:
            return (other.x, other.y) == (self.x, self.y)
        except AttributeError:
            return False

    def __ne__(self, other):
        try:
            return (other.x, other.y) != (self.x, self.y)
        except AttributeError:
            return True

    def intersects_platform_x(self, platform):
        """
        Chek if node is within given platform's
        horizontal bounds.
        """
        return platform.rect.left < self.x < platform.rect.right

    def distance(self, othernode):
        return distance((self.x, self.y),
                        (othernode.x, othernode.y))

    def cost(self, othernode):
        xcost = abs(self.x-othernode.x)
        ycost = othernode.y - self.y
        if ycost < 0:
            ycost = 0
        return xcost+ycost
        return self.distance(othernode)

    def old_get_edges(self, player):
        if not self.all_nodes:
            return set()
        return self.all_nodes[self][player.__class__]

    def get_edges(self, player, force_update=False):
        if player.__class__ in self.edges:
            if force_update:
                pass
            else:
                return self.edges[player.__class__]

        # How many pixels to move at a time when checking for
        # collisions
        px_move = 20

        all_nodes = self.all_nodes
        _test_edges = set()
        edges = set()
        # Get all nodes with same y value
        same_y = sorted([n for n in all_nodes if self.y == n.y])
        # Sort them by x value
        index = same_y.index(self)
        single = len(same_y) == 1
        if index == 0 and not single:
            _test_edges.add(same_y[1])
        elif index == len(same_y)-1:
            _test_edges.add(same_y[index-1])
        else:
            _test_edges.add(same_y[index+1])
            _test_edges.add(same_y[index-1])
        while len(_test_edges) > 0:
            edge = _test_edges.pop()
            if abs(edge.x - self.x) > player.max_jump_distance:
                continue
            # Check for obstacles between the two nodes
            # start by placing player on the current node
            x, y = self.x, self.y
            ex, ey = edge.x, edge.y
            if ex < x:
                # edge is left of
                movemod = -px_move
            elif ex > x:
                movemod = px_move
            else:
                movemod = 0
            player.rect.left, player.rect.bottom = x, y
            path_blocked = False
            while not path_blocked:
                if ((movemod < 0 and player.rect.left <= ex)
                    or (movemod > 0 and player.rect.left >= ex)):
                    player.rect.left = ex
                if player.rect.left == ex:
                    break
                for obstacle in player.cm.objs_colliding(player):
                    if obstacle.is_wall:
                        path_blocked = True
                        break
                player.rect.left += movemod
            if not path_blocked:
                edges.add(edge)

        higher = [n for n in all_nodes
                         if n[1] > self[1]]
        lower = [n for n in all_nodes
                        if n[1] < self[1]]
        # TODO: get edges for platforms on different y positions
        # pedge = potential edge
        for pedge in lower:
            if abs(pedge[0] - self[0]) > player.max_jump_distance:
                continue
            if (pedge.intersects_platform_x(self.platform)
                and not self.platform.can_traverse_down):
                continue
            x, y = self.x, self.y
            ex, ey = pedge.x, pedge.y
            if ex < x:
                movemod = -px_move
            elif ex > x:
                movemod = px_move
            else:
                movemod = 0
            player.rect.left, player.rect.bottom = self.x, self.y
            path_blocked = False
            while not path_blocked:
                if ((movemod < 0 and player.rect.left <= ex)
                    or (movemod > 0 and player.rect.left >= ex)):
                    player.rect.left = ex
                if player.rect.bottom <= ey:
                    player.rect.bottom = ey
                # Start by moving x towards node, then y
                # Check for obstacles on each iteration
                # log.debug('Checking edge %s against %s', pedge, node)
                # log.debug('Dummy position: %s',
                #           (player.rect.left, player.rect.bottom))
                if player.rect.left == ex and player.rect.bottom == ey:
                    break
                if not player.rect.left == ex:
                    player.rect.left += movemod
                else:
                    player.rect.bottom -= px_move
                if player.rect.bottom <= ey:
                    player.rect.bottom = ey

                for obstacle in player.cm.objs_colliding(player):
                    if (obstacle == self.platform
                        and obstacle.can_traverse_down):
                        continue
                    path_blocked = True
                    break
            if not path_blocked:
                edges.add(pedge)
        for pedge in higher:
            if (pedge.y - self.y > player.max_jump_height
                or abs(pedge.x - self.x) > player.max_jump_distance):
                continue
            if (self.intersects_platform_x(pedge.platform)
                and pedge.platform.is_wall):
                continue
            x, y = self.x, self.y
            ex, ey = pedge.x, pedge.y
            if ex < x:
                movemod = -px_move
            elif ex > x:
                movemod = px_move
            else:
                movemod = 0
            player.rect.left, player.rect.bottom = x, y
            path_blocked = False
            while not path_blocked:
                if ((movemod < 0 and player.rect.left <= ex)
                    or (movemod > 0 and player.rect.left >= ex)):
                    player.rect.left = ex
                if player.rect.bottom > ey:
                    player.rect.bottom = ey
                if player.rect.left == ex and player.rect.bottom == ey:
                    break
                if not player.rect.bottom == ey:
                    player.rect.bottom += px_move
                else:
                    player.rect.left += movemod
                for obstacle in player.cm.objs_colliding(player):
                    if obstacle.is_wall:
                        path_blocked = True
                        break
            if not path_blocked:
                edges.add(pedge)
        self.edges[player.__class__] = edges
        return edges
