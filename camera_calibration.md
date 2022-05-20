# How to calibrate the camera?

## 1. Preparation 

- Print the camera calibration pattern [PDF file](https://github.com/Cartucho/dvrk_calib_arms_to_camera/blob/main/to_print/camera_calibration_pattern.pdf);
- Check the dimensions of the printed rectangle - black line around the pattern - the width should be 100 [mm] and the height 125 [mm], this is an important step to verify that your printer is not automatically scaling the pattern;
<img src="https://user-images.githubusercontent.com/15831541/162568372-52c59ed6-58be-42ca-87fe-948e7898212e.png" width="60%">

- Attach the pattern to a rigid and straight surface (never bend the calibration pattern!);
- Place the pattern approximately 20 [cm] away from the camera, and focus the cameras to that working distance;
- Sometimes the endoscopic cameras get dirty, so you may need to clean the tip of the endoscope.

## 2. Take pictures

Capture images to calibrate the camera by running:
`rosrun dvrk_calib_hand_eye record_image_calib.py`
If the pattern is being successfully detected it will be drawn in the image, like the following example:

TODO: add image here


Take `50 pictures` pictures of the calibration pattern in different poses, while respecting the following rules:
- Keep the distance between the camera and the pattern around 20 [cm];
- Adjust the brightness of the light source, so that the pattern is detected. Get images `without reflections` and `not too dark`. Tip: you may use your phone's flashlight to get the right brightness;

  <img src="https://user-images.githubusercontent.com/15831541/165295443-d4e74908-5d34-4001-82eb-e0b7defb35d9.png" width="50%">
- Do not move the pattern or the camera while taking a picture! This would cause motion blur. Also, do not hold the pattern in your hand, your hand will shake, and again cause motion blur. Just make sure that it is static while taking each picture;

  <img src="https://user-images.githubusercontent.com/15831541/165124470-50de5b22-3839-4f9b-bf2f-cd1fb82067a1.png" width="50%">
- Keep the rectangular line inside the field of view of the camera;

  <img src="https://user-images.githubusercontent.com/15831541/165296055-05a5afff-5ddd-458c-a393-62d0aaebbaed.png" width="50%">
- Do not to take pictures with the pattern completely parallel to the image plane, add an inclination. On the other hand, do not exaggerate on the inclination, try to keep it less than 45 degrees;
  [TODO: add picture]

Example of a calibration set:

<img src="https://user-images.githubusercontent.com/15831541/165122385-6fa200d3-7146-4458-96a7-59a12440af5b.png" width="100%">

TODO add matlab image of poses

## 3. Calibrate the camera

`rosrun dvrk_calib_hand_eye calib_camera.py`

Please edit these values in the [camera_calibration.yaml](https://github.com/Cartucho/dvrk_calib_arms_to_camera/blob/main/camera_calibration.yaml) file!

Alternative: Use `Matlab` and install the `Computer Vision toolbox`. Then click `Apps > camera calibration`, select the captured images `data/cam_calib/` and proceed with the calibration.

Example of calibration parameters, when using the:

- First generation endoscopes:
    - Distortion: TODO
    - Intrinsic: TODO

- HD endoscope
    - Distortion: TODO
    - Intrinsic: TODO
