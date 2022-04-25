# How to calibrate the camera?

## 1. Preparation 

- Print the camera calibration pattern [PDF file](https://raw.githubusercontent.com/Cartucho/dvrk_calib_arms_to_camera/main/to_print/camera_calibration_pattern.pdf);
- Check the dimensions of the printed rectangle - black line around the pattern - the width should be 100 [mm] and the height 125 [mm], this is an important step to verify that your printer is not automatically scaling the pattern;
<img src="https://user-images.githubusercontent.com/15831541/162568372-52c59ed6-58be-42ca-87fe-948e7898212e.png" width="60%">

- Attach the pattern to a rigid and straight surface (never bend the calibration pattern!);
- Place the pattern approximately 20 [cm] away from the camera, and focus the cameras to that working distance;
- Sometimes the endoscopic cameras get dirty, so you may need to clean the tip of the endoscope.

## 2. Take pictures

I will ask you to take 50 pictures in total, but before taking these 50 pictures, please take just one or two pictures to check that the pattern is successfully detected. If it is not detected please read carefully the indications below and give it another try!

Take `50 pictures` pictures of the calibration pattern in different poses, while respecting the following rules:
- Keep the distance between the camera and the pattern around to 20 [cm];
- Adjust the brightness of the light source, so that the pattern is detected. You need enough light to detect the pattern but you also do not want reflections which would make the corners hard to detect. Tip: you may use your phone's flashlight to get the right brightness;
  [TODO: add picture]
- Do not move the pattern or the camera while taking a picture! This would cause motion blur. Also, do not hold the pattern in your hand, your hand will shake, and again cause motion blur. Just make sure that it is static while taking each picture;
  <img src="https://user-images.githubusercontent.com/15831541/165124470-50de5b22-3839-4f9b-bf2f-cd1fb82067a1.png" width="50%">
- Keep the rectangular line inside the field of view of the camera;
  [TODO: add picture]
- Try not to take pictures with the pattern parallel to the image plane, add inclinations. On the other hand, do not exaggerate on the inclination, try to keep it less than 45 degrees;
  [TODO: add picture]

Example of a calibration set:

<img src="https://user-images.githubusercontent.com/15831541/165122385-6fa200d3-7146-4458-96a7-59a12440af5b.png" width="100%">

## 3. Calibrate the camera

Easiest: Use `Matlab` and install the `Computer Vision toolbox`. Then clicks `Apps > camera calibration` and select the images that were captured in Step 2.

Example of calibration parameters, when using the first generation endoscopes:
- Distortion:
- Intrinsic:
