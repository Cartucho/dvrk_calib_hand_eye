#!/usr/bin/env python
import sys

import rospy
import rospkg

class AdjustHSV:
  def __init__(self):
    print("heyyy")


def main(args):
  rospy.init_node('green_marker_adjust_hsv', anonymous=True)
  a = AdjustHSV()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
  main(sys.argv)

