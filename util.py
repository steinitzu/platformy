import math


def slope(pointa, pointb):
    """
    Return the slope of a line as float.
    """
    x1, y1 = pointa
    x2, y2 = pointb
    try:
        slope = float(float((y2-y1))/float((x2-x1)))
    except ZeroDivisionError:
        # Vertical line, slope is undefined
        slope = None
    return slope


def distance(pointa, pointb):
    """
    _distance((x,y), (x, y)) -> float
    Get the distance between two points.
    """
    return math.sqrt(
        (pointa[0]-pointb[0])**2
        + (pointa[1]-pointb[1])**2)


def short_id(obj, length=3):
    return str(id(obj))[0-length:]


def midpoint(numa, numb):
    return (numa+numb)/2


def string_list(alist, joiner=', '):
    """
    Return all items in list joined into string by joiner.
    """
    return joiner.join([str(x) for x in alist])


import collections

class Queue:
    def __init__(self):
        self.elements = collections.deque()

    def empty(self):
        return len(self.elements) == 0

    def put(self, x):
        self.elements.append(x)

    def get(self):
        return self.elements.popleft()


import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


from math import atan2, pi, degrees


def angle(origin, target):
    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    rads = atan2(-dy, dx)
    rads %= 2*pi
    degs = degrees(rads)
    return degs
