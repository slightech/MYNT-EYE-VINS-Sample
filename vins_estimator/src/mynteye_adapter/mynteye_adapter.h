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

#pragma once

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/visualization.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>

#define INFO_L "/mynteye/left/camera_info";
#define INFO_R "/mynteye/right/camera_info";
#define CLIB_INFO_FILE_NAME_L "device_params_left.yaml"
#define CLIB_INFO_FILE_NAME_R "device_params_right.yaml"
#define IMU_PARAMS_FILE_NAME "device_imu_params.yaml"

class MynteyeAdapter {
 public:
  MynteyeAdapter(const std::string &config_path, const std::string &imu_srv)
      :config_path_(config_path), imu_srv_(imu_srv), imu_res(false) {}
  ~MynteyeAdapter() {}
  bool readmynteyeConfig();
  inline bool getImuRes() {
    return imu_res;
  }
  inline const std::string getConfigPath() const {
    return config_path_;
  }
  inline const std::string getImusrv() const {
    return imu_srv_;
  }
  inline bool setConfigPath(const std::string& config_path) {
    config_path_ = config_path;
    return true;
  }
  inline bool setMynteyeIMUsrv(const std::string &srv) {
    imu_srv_ = srv;
    return true;
  }
 private:
  ros::Subscriber sub1L;
  ros::Subscriber sub1R;
  std::string config_path_;
  std::string imu_srv_;
  bool imu_res;
};
