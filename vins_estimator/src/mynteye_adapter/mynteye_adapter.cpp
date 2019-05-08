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

bool ConversionIMUFromDeviceVINSFUSION(
    const std::string& path, const Config &config, const Config &config_r2l) {
    if (abs((double)config["rotation"][0]) < 0.0000001) {
      ROS_WARN("The imu extri param is invalid!, you shoud calib the imu extri manually!! and fill it to [%s]", config_file.c_str());
      return false;
    }

  std::cout << "device_info_path_imu: " << path << std::endl;
  cv::FileStorage imu_params_fs(path, cv::FileStorage::WRITE);
  double l2imu_proj[4][4] = { {(double)config["rotation"][0], (double)config["rotation"][1], (double)config["rotation"][2], (double)config["translation"][0] / 1000.0},
                              {(double)config["rotation"][3], (double)config["rotation"][4], (double)config["rotation"][5], (double)config["translation"][1] / 1000.0},
                              {(double)config["rotation"][6], (double)config["rotation"][7], (double)config["rotation"][8], (double)config["translation"][2] / 1000.0},
                              {0., 0., 0., 1.} };
  double l2r[4][4] = { {(double)config_r2l["rotation"][0], (double)config_r2l["rotation"][1], (double)config_r2l["rotation"][2], (double)config_r2l["translation"][0] / 1000.0},
                       {(double)config_r2l["rotation"][3], (double)config_r2l["rotation"][4], (double)config_r2l["rotation"][5], (double)config_r2l["translation"][1] / 1000.0},
                       {(double)config_r2l["rotation"][6], (double)config_r2l["rotation"][7], (double)config_r2l["rotation"][8], (double)config_r2l["translation"][2] / 1000.0},
                       {0., 0., 0., 1.} };

  cv::Mat body_T_cam0(4, 4, CV_64FC1, l2imu_proj);
  cv::Mat l2r_pr(4, 4, CV_64FC1, l2r);
  cv::Mat l2r_pr_n(4, 4, CV_64FC1);
  cv::invert(l2r_pr, l2r_pr_n);

  imu_params_fs << "body_T_cam0" << body_T_cam0;
  imu_params_fs << "body_T_cam1" << body_T_cam0 * l2r_pr_n;

  imu_params_fs.release();
  return true;
}

std::string get_d_imu_intri_info() {
  ros::NodeHandle ns;
  ros::ServiceClient client = ns.serviceClient<mynteye_wrapper_d::GetParams>("/mynteye_wrapper_d_node/get_params");  // NOLINT
  mynteye_wrapper_d::GetParams srv;
  // IMG_INTRINSICS = 0u,
  // IMG_EXTRINSICS_RTOL = 1u,
  // IMU_INTRINSICS = 2u,
  // IMU_EXTRINSICS = 3u,
  srv.request.key = 2u;  // IMG_INTRINSICS
  if (client.call(srv)) {
    return srv.response.value;
  } else {
    ROS_ERROR("Failed to call service GetParams, make sure you have launch mynteye device SDK nodelet"); // NOLINT
    return "null";
  }
}

std::string get_d_imu_extri_info() {
  ros::NodeHandle ns;
  ros::ServiceClient client = ns.serviceClient<mynteye_wrapper_d::GetParams>("/mynteye_wrapper_d_node/get_params"); // NOLINT
  mynteye_wrapper_d::GetParams srv;
  // IMG_INTRINSICS = 0u,
  // IMG_EXTRINSICS_RTOL = 1u,
  // IMU_INTRINSICS = 2u,
  // IMU_EXTRINSICS = 3u,
  srv.request.key = 3u;
  if (client.call(srv)) {
    return srv.response.value;
  } else {
    ROS_ERROR("Failed to call service GetParams, make sure you have launch mynteye device SDK nodelet");  // NOLINT
    return "null";
  }
}

std::string get_d_extri_l2r() {
  ros::NodeHandle ns;
  ros::ServiceClient client = ns.serviceClient<mynteye_wrapper_d::GetParams>("/mynteye_wrapper_d_node/get_params"); // NOLINT
  mynteye_wrapper_d::GetParams srv;
  // IMG_INTRINSICS = 0u,
  // IMG_EXTRINSICS_RTOL = 1u,
  // IMU_INTRINSICS = 2u,
  // IMU_EXTRINSICS = 3u,
  srv.request.key = 1u;
  if (client.call(srv)) {
    return srv.response.value;
  } else {
    ROS_ERROR("Failed to call service GetParams, make sure you have launch mynteye device SDK nodelet");  // NOLINT
    return "null";
  }
}

std::string get_s_imu_intri_info() {
    ros::NodeHandle ns;
    ros::ServiceClient client =
        ns.serviceClient<mynt_eye_ros_wrapper::GetInfo>("/mynteye/get_info");
    mynt_eye_ros_wrapper::GetInfo srv;
    // DEVICE_NAME = 0u,
    // SERIAL_NUMBER = 1u,
    // FIRMWARE_VERSION = 2u,
    // HARDWARE_VERSION = 3u,
    // SPEC_VERSION = 4u,
    // LENS_TYPE = 5u,
    // IMU_TYPE = 6u,
    // NOMINAL_BASELINE = 7u,
    // AUXILIARY_CHIP_VERSION = 8u,
    // ISP_VERSION = 9u,
    // IMG_INTRINSICS = 10u,
    // IMG_EXTRINSICS_RTOL = 11u,
    // IMU_INTRINSICS = 12u,
    // IMU_EXTRINSICS = 13u,
    srv.request.key = 12u;
    if (client.call(srv)) {
      return srv.response.value;
    } else {
      ROS_ERROR("Failed to call service GetInfo , make sure you have launch mynteye device SDK nodelet");  // NOLINT
      return "null";
    }
}

std::string get_s_extri_l2r() {
    ros::NodeHandle ns;
    ros::ServiceClient client =
        ns.serviceClient<mynt_eye_ros_wrapper::GetInfo>("/mynteye/get_info");
    mynt_eye_ros_wrapper::GetInfo srv;
    // DEVICE_NAME = 0u,
    // SERIAL_NUMBER = 1u,
    // FIRMWARE_VERSION = 2u,
    // HARDWARE_VERSION = 3u,
    // SPEC_VERSION = 4u,
    // LENS_TYPE = 5u,
    // IMU_TYPE = 6u,
    // NOMINAL_BASELINE = 7u,
    // AUXILIARY_CHIP_VERSION = 8u,
    // ISP_VERSION = 9u,
    // IMG_INTRINSICS = 10u,
    // IMG_EXTRINSICS_RTOL = 11u,
    // IMU_INTRINSICS = 12u,
    // IMU_EXTRINSICS = 13u,
    srv.request.key = 11u;
    if (client.call(srv)) {
      return srv.response.value;
    } else {
      ROS_ERROR("Failed to call service GetInfo , make sure you have launch mynteye device SDK nodelet");  // NOLINT
      return "null";
    }
}

std::string get_s_imu_extri_info() {
    ros::NodeHandle ns;
    ros::ServiceClient client =
        ns.serviceClient<mynt_eye_ros_wrapper::GetInfo>("/mynteye/get_info");
    mynt_eye_ros_wrapper::GetInfo srv;
    // DEVICE_NAME = 0u,
    // SERIAL_NUMBER = 1u,
    // FIRMWARE_VERSION = 2u,
    // HARDWARE_VERSION = 3u,
    // SPEC_VERSION = 4u,
    // LENS_TYPE = 5u,
    // IMU_TYPE = 6u,
    // NOMINAL_BASELINE = 7u,
    // AUXILIARY_CHIP_VERSION = 8u,
    // ISP_VERSION = 9u,
    // IMG_INTRINSICS = 10u,
    // IMG_EXTRINSICS_RTOL = 11u,
    // IMU_INTRINSICS = 12u,
    // IMU_EXTRINSICS = 13u,
    srv.request.key = 13u;
    if (client.call(srv)) {
      return srv.response.value;
    } else {
      ROS_ERROR("Failed to call service GetInfo , make sure you have launch mynteye device SDK nodelet");  // NOLINT
      return "null";
    }
}

bool MynteyeAdapter::readmynteyeConfig() {
  ROS_INFO("Now try to read mynteye device param ...");
  ros::NodeHandle n("~");
  config_file = getConfigPath();

  Config imu_intri_info;
  Config imu_extri_info;
  Config extri_l2r;
  std::string imu_intri = "null";
  std::string imu_extri = "null";
  std::string extri_l2r_string = "null";
  if (imu_srv_ == "s") {
    imu_intri = get_s_imu_intri_info();
    imu_extri = get_s_imu_extri_info();
    extri_l2r_string = get_s_extri_l2r();
  } else if (imu_srv_ == "d") {
    imu_intri = get_d_imu_intri_info();
    imu_extri = get_d_imu_extri_info();
    extri_l2r_string = get_d_extri_l2r();
  }

  if (imu_intri != "null" && imu_extri != "null") {
    imu_intri_info = parse_string(imu_intri.c_str(), JSON, "log");
    imu_extri_info = parse_string(imu_extri.c_str(), JSON, "log");
    extri_l2r = parse_string(extri_l2r_string.c_str(), JSON, "log");
    int pn__ = config_file.find_last_of('/');
    std::string configPath__ = config_file.substr(0, pn__);
    std::string device_info_path_imu =
        configPath__ + "/" + IMU_PARAMS_FILE_NAME;
    if (ConversionIMUFromDeviceVINSFUSION(
            device_info_path_imu, imu_extri_info, extri_l2r)) {
      std::cout << "L2Imu extrinsics: \n" << imu_extri_info << std::endl;
      std::cout << "L2R extrinsics: \n" << extri_l2r << std::endl;
      ROS_INFO("Imu params is load to build the params");
      imu_res = true;
    }
  } else {
    if (imu_srv_ == "s" || imu_srv_ == "d") {
      ROS_WARN("check the list below:");
      ROS_WARN("1. the mynteye device ROS nodelet not been launched");
      ROS_WARN("2. the mynteye device SDK version may be too old");
      ROS_WARN("3. the device calib data may not correct");
    }
  }
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
    ConversionFromDeviceVINSFUSION(device_info_path_left, info_msg);
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
    ConversionFromDeviceVINSFUSION(device_info_path_right, info_msg);
    check_success_r_tag = true;
  }
}

