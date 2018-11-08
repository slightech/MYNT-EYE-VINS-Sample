## Calibrate camera parameters

1. launch `mynt_eye_ros_wrapper`
   ```
    cd MYNT-EYE-S-SDK
    make ros
    source wrappers/ros/devel/setup.bash
    roslaunch mynt_eye_ros_wrapper mynteye.launch
    ```
2. launch `calibration_images`

    `calibration_images pkg` is used to get the calibration image list, press the `w`key to save image to mynt_images_fisheye or mynt_images_pinhole file. We collect about 30 images to calibrate the camera parameters. here is [chessbord_9_6](./chessbord_9*6.jpg), grid_size is the measured value, unit millimeters.

    **fisheye model, follow the type:**
    ```
     roslaunch calibration_images calibration_fisheye.launch
    ```
    **pinhole model, follow the type:**
    ```
     roslaunch calibration_images calibration_pinhole.launch
    ```
3. run `camera_model` node to extrat camera parameters, copy the contents to `<vins>/config/mynteye/mynteye_config.yaml`.

    **fisheye model, follow the type:**

    ```
      roscd calibration_images/mynt_images_fisheye
      rosrun camera_model Calibration -w 9 -h 6 -s 25.16 -p fisheye_ -e .jpg -i . --camera-model mei
    ```

    **pinhole model, follow the type:**

    ```
      roscd calibration_images/mynt_images_pinhole
      rosrun camera_model Calibration -w 9 -h 6 -s 25.16 -p color_ -e .jpg -i . --camera-model pinhole
    ```
   The `camera_camera_calib.yaml` file is generated in the **mynt_images_fisheye** or **mynt_images_pinhole** file when the calibration is completed.
