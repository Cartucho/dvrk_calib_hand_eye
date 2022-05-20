# dvrk_calib_hand_eye

-- REPO UNDER CONSTRUCTION --

The goal of this code is to estimate the geometric transformations between a camera and the robotic arms of a da Vinci dVRK robot. E.g., the transformation from the left-stereo camera to the PSM1's base. This is a fundamental calibration for tasks such as visual servoeing, where you want to move the PSM given input endoscopic images.

Surgical instrument and projected skeleton after calibrating the transformation:
TODO: giff with tool moving

Assumptions:
- I will assume that you will be using a `8.35 [mm]` surgical instrument, which is the standard diameter of the shaft;
- You will have `"neon green"` sticker paper, which you can buy on Amazon or any another store. This sticker paper will be used to wrap a marker around the shaft of the surgical instrument;

  <img src="https://user-images.githubusercontent.com/15831541/165297939-ebcb03ef-c781-4ad2-bd5b-16712d0d018d.png" width="20%">

- You have a monocular (or stereo) `camera at a fixed position`. If you are using the ECM, you can lock the ECM's joints to a fixed state;

## Step 0 - Setting up

1. Clone and build this repo as a catkin package. Do not forget to add `--recurse-submodules` as shown below (this flag is used to clone the [cylmarker](https://github.com/Cartucho/cylmarker) submodule):

```
cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/Cartucho/dvrk_calib_hand_eye.git
catkin build dvrk_calib_hand_eye
```

2. When the build is completed, open a new terminal and set-up the cylmarker code:

```
roscd dvrk_calib_hand_eye/cylmarker
python3.9 -m pip install --user virtualenv
python3.9 -m virtualenv venv
source venv/bin/activate
pip install -r requirements.txt
```

3. Edit the rostopics in [config.yaml](https://github.com/Cartucho/dvrk_calib_arms_to_camera/blob/main/config.yaml) to the target PSM and camera.

## Step 1 - Camera calibration

First, we will calculate both the `distortion` and `intrinsic camera parameters` of the camera that you will be using as pivot. In the case of a stereo endoscope, we suggest that you use the left-stereo camera as pivot. It is **very important that the camera is well calibrated**, since this will have a significant impact in the accuracy of the estimated transformations. If you have already accurately calibrated your camera just then edit the values in the [camera_calibration.yaml](https://github.com/Cartucho/dvrk_calib_arms_to_camera/blob/main/camera_calibration.yaml) file and skip to step 2, otherwise follow the [camera_calibration.md](https://github.com/Cartucho/dvrk_calib_arms_to_camera/blob/main/camera_calibration.md) instructions.

## Step 2 - Record data (by moving the surgical instrument)

First, print the marker on `green sticker paper`, [PDF file](https://github.com/Cartucho/dvrk_calib_arms_to_camera/blob/main/to_print/green_marker_pattern.pdf)
Then, you cut one of the printed markers and wrap it around the shaft of a `8.3 [mm]` surgical instrument.
Launch both the camera and the PSM.

After doing that, let's record the green marker at multiple poses:

`rosrun dvrk_calib_hand_eye record_image_and_joints.py`

Then the robot will automatically move the PSM through the recorded poses and interpolate points in between.
This is both to guarantee that the images are taken at a stable pose (no motion blur) and that we have enough data for the calibration.
`Home` the PSM and then run this command:

`rosrun dvrk_calib_hand_eye replay_interpolated.py`

This step will take a few minutes to be completed. You can check that the data is being recorded in `data/cam_interpolated` and `data/joint_interpolated.yaml`.

## Step 3 - Get green marker pose in recorded data

```
roscd dvrk_calib_hand_eye/cylmarker
source venv/bin/activate
unset PYTHONPATH
```

Then check if the HSV values are good for segmenting the green marker:
```
python main.py --task a --path ..
```
You should see the `entire` marker highlighted in red in most of the images. If you don't you will have to adjust the HSV ranges in the [config.yaml](https://github.com/Cartucho/dvrk_calib_arms_to_camera/blob/main/config.yaml) file. If you move your mouse over the images you will see the HSV value at each point of the image.


After adjusting the HSV ranges, let's get the poses (which will be automatically saved if the marker is successfully detected):
```
python main.py --task p --path ..
```

Remember always to activate the virtual environment `(venv)` and unset ROS's default old OpenCV `unset PYTHONPATH` before running the command `python main.py --task p --path ..`

## Step 4 - Calculate the transformation (cam_T_basePSM)

Final step, do the hand-eye calibration:
```
rosrun dvrk_calib_hand_eye calib_hand_eye.py
```

## FAQ

1. `I moved the set-up joints, what should I do?`
If you move the set-up joints you need to repeat the calibration (step 2, 3 and 4).

2. `I moved the camera, what should I do?`
The easiest is to repeat the calibration (step 2, 3 and 4). If you are using the ECM you could try sending the camera to the same initial state (same joint values as when you did the calibration). Please note that when locking and unlocking motors the joint values will always change slightly, which adds error to the calibration. Read question 4. to learn how to compensate for the ECM joint motions.

3. `What if I want to use rectified images instead of the ones captured directly from the camera?`
When you rectify the camera rotates, therefore you need to update the transformation according to that rotation. Specifically, if you are using OpenCV's `... R1 ... = cv.stereoRectify()` you have to multiply `R1 = camRect_T_cam` with the estimated transformation `cam_T_basePSM`, since `camRect_T_basePSM = camRect_T_cam @ cam_T_basePSM`. Note that you will have to make `R1` homogeneous and 4x4 for doing this product. Another alternative (Option B) is to rectify the images captured in step 2 and repeat step 3 and 4. Note that if you do (Option B) you will also need to update the camera calibration values, set all the distortion to 0, since rectified images have no distortion and set the intrinsic with the first 3x3 values of your rectified intrinsics, you get this from `P1` if you use `cv.stereoRectify()`.

4. `How to update the cam_T_basePSM transformation in real-time if I move the ECM?`
To achieve this you have to perform another hand-eye calibration to estimate the rigid transformation from the ECM's centre (`/ECM/measured_cp`), to the camera. This way, every time the ECM joint values change you can re-adjust the `cam_T_basePSM` transformation. However, keep in mind that at any time, if you move the set-up joints, you need to calibrate `cam_T_basePSM` from scratch (step 2, 3 and 4).
