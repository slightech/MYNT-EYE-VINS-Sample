## Calibrate camera parameters
1\. launch `mynt_eye_ros_wrapper`
```
cd MYNT-EYE-SDK-2
make ros
source wrappers/ros/devel/setup.bash
roslaunch mynt_eye_ros_wrapper mynteye.launch
```
2\. launch `calibration_images`

`calibration_images pkg` is used to get the calibration image list, press the `w`key to save image to `mynt_images file. We collect about 30 images to calibrate the camera parameters. here is [chessbord_9_6](./chessbord_9*6.jpg), grid_size is the measured value, unit millimeters.


```
roslaunch calibration_images calibration_images.launch
```

3\. run `camera_model` node to extrat camera parameters **in mynt_images**, copy the contents to `<vins>/config/mynteye/mynteye_config.yaml`.

```
roscd vins_estimator/../calibration_images/mynt_images
rosrun camera_model Calibration -w 9 -h 6 -s 25.16 -p fisheye_ -e .jpg -i . --camera-model mei
```

The `camera_camera_calib.yaml` file is generated in the **mynt_images** after the calibration is completed.
