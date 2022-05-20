#!/usr/bin/env python
import sys
import os

import rospy
import rospkg

class AdjustHSV:
  def __init__(self, path_images, path_green_marker_data):
    print(path_images)
    print(path_green_marker_data)


def main(args):
  rospy.init_node('green_marker_adjust_hsv', anonymous=True)
  rospack = rospkg.RosPack()
  path_package = rospack.get_path('dvrk_calib_hand_eye')
  path_images = os.path.join(path_package, 'data', 'cam_interpolated')
  path_green_marker_data = os.path.join(path_package, 'cylmarker', 'data')
  a = AdjustHSV(path_images, path_green_marker_data)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
  main(sys.argv)

