#!/usr/bin/env python
import sys
import os
import glob

import rospy
import rospkg
import cv2 as cv
import numpy as np


def calib_monocular(criteria, objp, size, all_im, window_name):
  # Arrays to store object points and image points from all the images.
  all_ob_p = [] # 3d point in real world space
  all_im_p = [] # 2d points in image plane.
  for fname in all_im:
    im = cv.imread(fname)
    gray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, size, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE)
    # If found, add object points, image points (after refining them)
    if ret:
      all_ob_p.append(objp)
      corners_refined = cv.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)
      all_im_p.append(corners_refined)
      # Draw and display the corners
      cv.drawChessboardCorners(im, size, corners_refined, ret)
      cv.imshow(window_name, im)
      cv.waitKey(0)
  cv.destroyWindow(window_name)
  _, K, D, _, _ = cv.calibrateCamera(all_ob_p, all_im_p, gray.shape[::-1], None, None)
  return K, D


def cam_calib(path_data):
  # NOTE: Values hardcoded for this specific package's calib pattern
  n_cols = 23 - 1
  n_rows = 18 - 1
  size = (n_cols, n_rows) # size of checkerboard
  criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 300, 0.00001) # termination criteria
  sq_mm = 5
  objp = np.zeros((n_rows*n_cols, 3), np.float32)
  height_mm = n_rows*sq_mm
  width_mm = n_cols*sq_mm
  objp[:,:2] = np.mgrid[0:width_mm:sq_mm, 0:height_mm:sq_mm].T.reshape(-1,2)
  im_path_list = glob.glob(os.path.join(path_data, "*.png"))
  K1, D1 = calib_monocular(criteria, objp, size, im_path_list, "calib_images")
  np.set_printoptions(threshold=sys.maxsize, suppress=True)
  print("Copy and paste these values into the `camera_calibration.yaml` file!\n")
  print("intrinsic: {}".format(K1.tolist()))
  print("distortion: {}".format(D1[0].tolist()))


def main(args):
  rospy.init_node('calib_camera', anonymous=True)
  rospack = rospkg.RosPack()
  path_package = rospack.get_path('dvrk_calib_hand_eye')
  path_data = os.path.join(path_package, 'data', 'cam_calib')
  if os.path.isdir(path_data):
    cam_calib(path_data)
  else:
    print("No data found in path: {}".format(path_data))
    print("\t please run first: `rosrun dvrk_calib_hand_eye record_image_calib.py`")


if __name__ == '__main__':
  main(sys.argv)
