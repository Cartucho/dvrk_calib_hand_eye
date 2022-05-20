#!/usr/bin/env python
import sys
import os
import yaml

import rospy
import rospkg


class AdjustHSV:
  def __init__(self, path_images, path_green_marker, h_min, h_max, s_min, v_min):
    sys.path.append(path_green_marker)
    from cylmarker.pose_estimation import adjust_hsv
    print("success")
    #path_green_marker_data = os.path.join(path_green_marker, 'data')
    settings = {'img_dir_path': path_images,
                'img_format': '.png',
                'h_min': h_min,
                'h_max': h_max,
                's_min': s_min,
                'v_min': v_min
               }
    adjust_hsv.improve_segmentation(settings)


def main(args):
  rospy.init_node('green_marker_adjust_hsv', anonymous=True)
  rospack = rospkg.RosPack()
  path_package = rospack.get_path('dvrk_calib_hand_eye')
  path_images = os.path.join(path_package, 'data', 'cam_interpolated')
  path_green_marker = os.path.join(path_package, 'cylmarker')
  # Load h_min, h_max, s_min, v_min
  path_config_file = os.path.join(path_package, 'config.yaml')
  with open(path_config_file, 'rb') as yaml_file:
    config_data = yaml.safe_load(yaml_file)
  gm = config_data['green_marker']
  a = AdjustHSV(path_images, path_green_marker, gm['h_min'], gm['h_max'], gm['s_min'], gm['v_min'])
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
  main(sys.argv)

