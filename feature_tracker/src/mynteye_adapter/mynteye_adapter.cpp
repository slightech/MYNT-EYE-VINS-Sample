// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include "mynteye_adapter.h"
#include "mynt_eye_ros_wrapper_422c9/GetInfo.h"
#include "mynteye_wrapper_d_27490/GetParams.h"

#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"
using namespace configuru;

static bool check_tag;
static bool check_success_l_tag;
static bool check_success_r_tag;
static std::string config_file = "";

const std::string info_l = INFO_L;
const std::string info_r = INFO_R;

void cameraParamsLCallback(const sensor_msgs::CameraInfoConstPtr &info_msg);
void cameraParamsRCallback(const sensor_msgs::CameraInfoConstPtr &info_msg);

void ConversionFromDeviceVINSFUSION(const std::string& path,
    const sensor_msgs::CameraInfoConstPtr & img_intri_info) {
  cv::FileStorage calib_intri_fs(path, cv::FileStorage::WRITE);
  if (img_intri_info->distortion_model == "KANNALA_BRANDT") {
    calib_intri_fs << "model_type" << "KANNALA_BRANDT";
    calib_intri_fs << "camera_name" << "kannala-brandt";
    calib_intri_fs << "image_width" << (int)(img_intri_info->width);
    calib_intri_fs << "image_height" << (int)(img_intri_info->height);
    calib_intri_fs << "projection_parameters";
    calib_intri_fs << "{";
    calib_intri_fs << "k2" << (double)(img_intri_info->D[0]);
    calib_intri_fs << "k3" << (double)(img_intri_info->D[1]);
    calib_intri_fs << "k4" << (double)(img_intri_info->D[2]);
    calib_intri_fs << "k5" << (double)(img_intri_info->D[3]);
    calib_intri_fs << "mu" << (double)(img_intri_info->K[0]);
    calib_intri_fs << "mv" << (double)(img_intri_info->K[4]);
    calib_intri_fs << "u0" << (double)(img_intri_info->K[2]);
    calib_intri_fs << "v0" << (double)(img_intri_info->K[5]);
    calib_intri_fs << "}";
  } else if (img_intri_info->distortion_model == "plumb_bob") {
    calib_intri_fs << "model_type" << "PINHOLE";
    calib_intri_fs << "camera_name" << "pinhole";
    calib_intri_fs << "image_width" << (int)(img_intri_info->width);
    calib_intri_fs << "image_height" << (int)(img_intri_info->height);
    calib_intri_fs << "distortion_parameters";
    calib_intri_fs << "{";
    calib_intri_fs << "k1" << (double)(img_intri_info->D[0]);
    calib_intri_fs << "k2" << (double)(img_intri_info->D[1]);
    calib_intri_fs << "p1" << (double)(img_intri_info->D[2]);
    calib_intri_fs << "p2" << (double)(img_intri_info->D[3]);
    calib_intri_fs << "}";
    calib_intri_fs << "projection_parameters";
    calib_intri_fs << "{";
    calib_intri_fs << "fx" << (double)(img_intri_info->K[0]);
    calib_intri_fs << "fy" << (double)(img_intri_info->K[4]);
    calib_intri_fs << "cx" << (double)(img_intri_info->K[2]);
    calib_intri_fs << "cy" << (double)(img_intri_info->K[5]);
    calib_intri_fs << "}";
  }
  calib_intri_fs.release();
}

void ConversionFromDeviceVINSMONO(const std::string& path,
    const sensor_msgs::CameraInfoConstPtr & img_intri_info) {
  cv::FileStorage calib_intri_fs(path, cv::FileStorage::WRITE);
  if (img_intri_info->distortion_model == "KANNALA_BRANDT") {
    calib_intri_fs << "model_type" << "KANNALA_BRANDT";
    calib_intri_fs << "camera_name" << "kannala-brandt";
    calib_intri_fs << "image_width" << (int)(img_intri_info->width);
    calib_intri_fs << "image_height" << (int)(img_intri_info->height);
    calib_intri_fs << "projection_parameters";
    calib_intri_fs << "{";
    calib_intri_fs << "k2" << (double)(img_intri_info->D[0]);
    calib_intri_fs << "k3" << (double)(img_intri_info->D[1]);
    calib_intri_fs << "k4" << (double)(img_intri_info->D[2]);
    calib_intri_fs << "k5" << (double)(img_intri_info->D[3]);
    calib_intri_fs << "mu" << (double)(img_intri_info->K[0]);
    calib_intri_fs << "mv" << (double)(img_intri_info->K[4]);
    calib_intri_fs << "u0" << (double)(img_intri_info->K[2]);
    calib_intri_fs << "v0" << (double)(img_intri_info->K[5]);
    calib_intri_fs << "}";
  } else if (img_intri_info->distortion_model == "plumb_bob") {
    calib_intri_fs << "model_type" << "PINHOLE";
    calib_intri_fs << "camera_name" << "pinhole";
    calib_intri_fs << "image_width" << (int)(img_intri_info->width);
    calib_intri_fs << "image_height" << (int)(img_intri_info->height);
    calib_intri_fs << "distortion_parameters";
    calib_intri_fs << "{";
    calib_intri_fs << "k1" << (double)(img_intri_info->D[0]);
    calib_intri_fs << "k2" << (double)(img_intri_info->D[1]);
    calib_intri_fs << "p1" << (double)(img_intri_info->D[2]);
    calib_intri_fs << "p2" << (double)(img_intri_info->D[3]);
    calib_intri_fs << "}";
    calib_intri_fs << "projection_parameters";
    calib_intri_fs << "{";
    calib_intri_fs << "fx" << (double)(img_intri_info->K[0]);
    calib_intri_fs << "fy" << (double)(img_intri_info->K[4]);
    calib_intri_fs << "cx" << (double)(img_intri_info->K[2]);
    calib_intri_fs << "cy" << (double)(img_intri_info->K[5]);
    calib_intri_fs << "}";
  }
  calib_intri_fs.release();
}

bool MynteyeAdapter::readmynteyeConfig() {
  ROS_INFO("Now try to read mynteye device param ...");
  ros::NodeHandle n("~");
  config_file = getConfigPath();

  // Create a ROS subscriber for the input point cloud
  sub1L = n.subscribe(info_l, 100, cameraParamsLCallback,
                    ros::TransportHints().tcpNoDelay());
  sub1R = n.subscribe(info_r, 100, cameraParamsRCallback,
                    ros::TransportHints().tcpNoDelay());

  check_tag = true;
  for (size_t i = 0; i < 3; i++) {
    ros::spinOnce();
    sleep(1);
    if (check_success_l_tag && check_success_r_tag) return true;
  }
  ROS_WARN("read mynteye img params fialed! \nyou shoud calib the img extri manually!! and fill it to [%s]", config_file.c_str());
  return false;
}

void cameraParamsLCallback(
    const sensor_msgs::CameraInfoConstPtr &info_msg) {
  static bool check_local_dl = true;
  if (check_tag && check_local_dl) {
    check_local_dl = false;
    std::cout << "left camera info:" << *info_msg << std::endl;

    int pn__ = config_file.find_last_of('/');
    std::string configPath__ = config_file.substr(0, pn__);
    std::string device_info_path_left =
        configPath__ + "/" + CLIB_INFO_FILE_NAME_L;
    ConversionFromDeviceVINSMONO(device_info_path_left, info_msg);
    check_success_l_tag = true;
  }
}
void cameraParamsRCallback(
    const sensor_msgs::CameraInfoConstPtr &info_msg) {
  static bool check_local_dr = true;
  if (check_tag && check_local_dr) {
    check_local_dr = false;
    std::cout << "right camera info:" << *info_msg << std::endl;

    int pn__ = config_file.find_last_of('/');
    std::string configPath__ = config_file.substr(0, pn__);
    std::string device_info_path_right =
        configPath__ + "/" + CLIB_INFO_FILE_NAME_R;
    ConversionFromDeviceVINSMONO(device_info_path_right, info_msg);
    check_success_r_tag = true;
  }
}

