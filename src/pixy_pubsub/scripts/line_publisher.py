#!/usr/bin/env python2
from __future__ import print_function
import rospy
from std_msgs.msg import String
from ctypes import *
from geometry_msgs.msg import Twist
import pixy
from pixy import *

get_all_features = True
pixy.init()
pixy.change_prog("line")

class Vector (Structure):
  _fields_ = [
    ("m_x0", c_uint),
    ("m_y0", c_uint),
    ("m_x1", c_uint),
    ("m_y1", c_uint),
    ("m_index", c_uint),
    ("m_flags", c_uint) ]
class IntersectionLine (Structure):
  _fields_ = [
    ("m_index", c_uint),
    ("m_reserved", c_uint),
    ("m_angle", c_uint) ]

vectors = VectorArray(100)
intersections = IntersectionArray(100)
frame = 0

while 1:
  if get_all_features:
    line_get_all_features ()
  else:
    line_get_main_features ()
  i_count = line_get_intersections (100, intersections)
  v_count = line_get_vectors (100, vectors)
  if i_count > 0 or v_count > 0:
    print('frame %3d:' % (frame))
    frame = frame + 1
    for index in range (0, i_count):
      print('[INTERSECTION: X=%d Y=%d BRANCHES=%d]' % (intersections[index].m_x, intersections[index].m_y, intersections[index].m_n))
      for lineIndex in range (0, intersections[index].m_n):
        print('  [LINE: INDEX=%d ANGLE=%d]' % (intersections[index].getLineIndex(lineIndex), intersections[index].getLineAngle(lineIndex)))

