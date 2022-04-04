# dvrk_calib_arms_to_camera

-- REPO UNDER CONSTRUCTION --

The goal of this code is to estimate the geometric transformations between a camera and the robotic arms of a da Vinci dVRK robot. E.g., the transformation from the left-stereo camera to the PSM1's base. This is a fundamental calibration for tasks such as visual servoeing, where you want to move the PSM given input endoscopic images.

Surgical instrument and projected skeleton after calibrating the transformation:
TODO: giff with tool moving

Assumptions:
- I will assume that you will be using a `8.3 [mm]` surgical instrument, which is the standard diameter of the shaft;
- You will have `"neon green"` sticker paper, which you can buy on Amazon or any another store. This sticker paper will be used to wrap a marker around the shaft of the surgical instrument;
- You have a monocular camera (or stereo) at a fixed position. If you are using the ECM, you can lock the ECM's joints to a fixed state;

## Step 1 - Camera calibration

First, we will calculate both the `distortion` and `intrinsic camera parameters` of the camera that you will be using as pivot. In the case of a stereo endoscope, we suggest that you use the left-stereo camera as pivot. It is **very important that the camera is well calibrated**, since this will have a significant impact in the accuracy of the estimated transformations. If you have already accurately calibrated your camera, please skip to step 2, otherwise follow the [camera_calibration.md](https://github.com/Cartucho/dvrk_calib_arms_to_camera/blob/main/camera_calibration.md) instructions.

## Step 2 - Record data + move the robotic arm

First, you will need to print the green marker on sticker paper, to wrap the marker around the shaft of the surgical instrument. 

TODO:

## Step 3 - Get green marker pose in recorded data

TODO:

## Step 4 - Calculate the transformation

TODO:

## FAQ

- `I moved to set-up joints, what should I do?`
If you move the set-up joints you need to repeat the calibration.

 - `I moved the camera, what should I do?`
 You need to put the camera back in the original pose, otherwise repeat the calibration. If you are using the ECM, all you have to do is put the ECM back to the same joint state as you initially had, when you did the transformation calibration! An alternative solution, is to do the advanced calibration decribed below. With the advanced calibration you can update the transformations even when you move the ECM, as long as you do not move the set-up joints.


## Advanced calibration: How to update the transformation in real-time if I move the ECM?

If you move the ECM, you change the ECMS' joints state, and therefore the transformation between the camera and the base of the PSM's also changes.
TODO: explain that there is a rigid transformation
