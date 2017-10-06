# coding: utf-8
# GDB用のpretty-printer
import math
import frea.printer
import re
import gdb

def buildPrinter(obj):
    obj.pretty_printers.append(frea.printer.Lookup)
    obj.pretty_printers.append(Lookup)

Re_Beat = re.compile("^beat::(.+)$")
def Lookup(val):
    name = val.type.strip_typedefs().name
    if name == None:
        return None
    obj = Re_Beat.match(name)
    if obj:
        return LookupBeat(obj.group(1), val)
    return None


class Iterator:
    def next(self):
        return self.__next__()

class Point2d(frea.printer.Vector, object):
    def __init__(self, val):
        super(Point2d, self).__init__(val)
    def to_string(self):
        return "Pt2" + self._raw_string()
class Point2dM(Point2d, object):
    def __init__(self, val):
        super(Point2dM, self).__init__(val)
    def to_string(self):
        return "Pt2M" + self._raw_string()

class Line2d:
    def __init__(self, val):
        self._pos = frea.printer.Vector(val["pos"])
        self._dir = frea.printer.Vector(val["dir"])
    def _raw_string(self):
        return "{pos=%s, dir=%s}" % (self._pos._raw_string(), self._dir._raw_string())
    def to_string(self):
        return "Line2" + self._raw_string()
class Line2dM(Line2d, object):
    def __init__(self, val):
        super(Line2dM, self).__init__(val)
    def to_string(self):
        return "Line2M" + self._raw_string()

class Ray2d(Line2d, object):
    def __init__(self, val):
        super(Ray2d, self).__init__(val)
    def to_string(self):
        return "Ray2" + self._raw_string()
class Ray2dM(Ray2d, object):
    def __init__(self, val):
        super(Ray2dM, self).__init__(val)
    def to_string(self):
        return "Ray2M" + self._raw_string()

class Segment2d:
    def __init__(self, val):
        self._from = frea.printer.Vector(val["from"])
        self._to = frea.printer.Vector(val["to"])
    def _raw_string(self):
        return "{from=%s, to=%s}" % (self._from._raw_string(), self._to._raw_string())
    def to_string(self):
        return "Seg2" + self._raw_string()

class Segment2dM(Segment2d, object):
    def __init__(self, val):
        super(Segment2dM, self).__init__(val)
    def to_string(self):
        return "Seg2M" + self._raw_string()

class Circle2d:
    def __init__(self, val):
        self._center = frea.printer.Vector(val["center"])
        self._radius = val["radius"]
    def _raw_string(self):
        return "{center=%s, radius=%.5g}" % (self._center._raw_string(), float(self._radius))
    def to_string(self):
        return "Cir2" + self._raw_string()

class Circle2dM(Circle2d, object):
    def __init__(self, val):
        super(Circle2dM, self).__init__(val)
    def to_string(self):
        return "Cir2M" + self._raw_string()

class Capsule2d:
    def __init__(self, val):
        self._from = frea.printer.Vector(val["from"])
        self._to = frea.printer.Vector(val["to"])
        self._radius = val["radius"]
    def _raw_string(self):
        return "{from=%s, to=%s, radius=%.5g}" % (
                    self._from._raw_string(),
                    self._to._raw_string(),
                    float(self._radius)
                )
    def to_string(self):
        return "Cap2" + self._raw_string()

class Capsule2dM(Capsule2d, object):
    def __init__(self, val):
        super(Capsule2dM, self).__init__(val)
    def to_string(self):
        return "Cap2M" + self._raw_string()

class AABB2d:
    def __init__(self, val):
        self._min = frea.printer.Vector(val["min"])
        self._max = frea.printer.Vector(val["max"])
    def _raw_string(self):
        return "{min=%s, max=%s}" % (
                    self._min._raw_string(),
                    self._max._raw_string()
                )
    def to_string(self):
        return "AB2" + self._raw_string()

class AABB2dM(AABB2d, object):
    def __init__(self, val):
        super(AABB2dM, self).__init__(val)
    def to_string(self):
        return "AB2M" + self._raw_string()

class Triangle2d:
    pass
class Convex2d:
    pass

class fooPrinter:
    class _iterator(Iterator):
        def __init__(self, parent):
            self._i = 0
            self._parent = parent
        def __iter__(self):
            return self
        def __next__(self):
            if self._i == 10:
                raise StopIteration
            self._i += 1
            return ("[{0}]".format(self._i), self._i)

    def __init__(self, val):
        self.val = val
    def to_string(self):
        return ("{x=" + str(float(123.54)) + " | y=" + str(int(1000)) + "}")
    def children(self):
        return self._iterator(self.val["value0"])
    def display_hint(self):
        return 'array'

def MakeRE(name):
    return re.compile(r"^g2::%s$" % name)
def MakeModelRE(name):
    return re.compile(r"^g2::Model<beat::g2::%s>$" % name)
Re_Ar = []
def AddRE(name, cls0, cls1):
    Re_Ar.append((MakeRE(name), cls0))
    Re_Ar.append((MakeModelRE(name), cls1))

AddRE("Point", Point2d, Point2dM)
AddRE("Line", Line2d, Line2dM)
AddRE("Ray", Ray2d, Ray2dM)
AddRE("Segment", Segment2d, Segment2dM)
AddRE("Circle", Circle2d, Circle2dM)
AddRE("Capsule", Capsule2d, Capsule2dM)
AddRE("AABB", AABB2d, AABB2dM)

def LookupBeat(name, val):
    for i in range(len(Re_Ar)):
        if Re_Ar[i][0].match(name):
            return Re_Ar[i][1](val)
    return None
