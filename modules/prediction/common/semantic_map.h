/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/

#pragma once

#include <future>
#include <unordered_map>

#include "opencv2/opencv.hpp"

#include "cyber/common/macros.h"
#include "modules/prediction/proto/feature.pb.h"

namespace apollo {
namespace prediction {

//SemanticMap
class SemanticMap {
 public:
  SemanticMap(); //构造函数 类实例化

  virtual ~SemanticMap() = default; //析构函数

  void Init(); //初始化

  void RunCurrFrame(
      const std::unordered_map<int, ObstacleHistory>& obstacle_id_history_map);//unordered_map,获取当前帧的历史map

  bool GetMapById(const int obstacle_id, cv::Mat* feature_map);//通过obstacle_id来获取feature_map

 private:
  cv::Point2i GetTransPoint(const double x, const double y, const double base_x,
                            const double base_y) {
    return cv::Point2i(static_cast<int>((x - base_x) / 0.1),
                       static_cast<int>(2000 - (y - base_y) / 0.1));//基本点,转换坐标(x,y) , 0.1m
  }

  void DrawBaseMap(const double x, const double y, const double base_x,
                   const double base_y); //画出基本地图 1

  void DrawBaseMapThread();//画出基本地图线程

  void DrawRoads(const common::PointENU& center_point, const double base_x,
                 const double base_y,
                 const cv::Scalar& color = cv::Scalar(64, 64, 64)); //画出道路 2

  void DrawJunctions(const common::PointENU& center_point, const double base_x,
                     const double base_y,
                     const cv::Scalar& color = cv::Scalar(128, 128, 128)); //画出交叉路口 灰色 3

  void DrawCrosswalks(const common::PointENU& center_point, const double base_x,
                      const double base_y,
                      const cv::Scalar& color = cv::Scalar(192, 192, 192)); //画出人行横道 4

  void DrawLanes(const common::PointENU& center_point, const double base_x,
                 const double base_y,
                 const cv::Scalar& color = cv::Scalar(255, 255, 255));//画出车道 白色 5

  cv::Scalar HSVtoRGB(double H = 1.0, double S = 1.0, double V = 1.0); //HSV转RGB(H,S,V=1.0)

  void DrawRect(const Feature& feature, const cv::Scalar& color,
                const double base_x, const double base_y, cv::Mat* img); //画出矩形

  void DrawPoly(const Feature& feature, const cv::Scalar& color,
                const double base_x, const double base_y, cv::Mat* img); //画出多线段

  void DrawHistory(const ObstacleHistory& history, const cv::Scalar& color,
                   const double base_x, const double base_y, cv::Mat* img); //画出历史

  // Draw adc trajectory in semantic map
  void DrawADCTrajectory(const cv::Scalar& color, const double base_x,
                         const double base_y, cv::Mat* img); //画出ADC轨迹在semantic_map

  cv::Mat CropArea(const cv::Mat& input_img, const cv::Point2i& center_point,
                   const double heading);//裁减区域

  cv::Mat CropByHistory(const ObstacleHistory& history, const cv::Scalar& color,
                        const double base_x, const double base_y);//通过历史进行裁减

 private:
  // base_image, base_x, and base_y to be updated by async thread
  cv::Mat base_img_; //基本图
  double base_x_ = 0.0;
  double base_y_ = 0.0;

  std::mutex draw_base_map_thread_mutex_;

  // base_image, base_x, and base_y to be used in the current cycle
  cv::Mat curr_img_;//当前图
  double curr_base_x_ = 0.0;
  double curr_base_y_ = 0.0;

  std::unordered_map<int, ObstacleHistory> obstacle_id_history_map_; //0:map_1; 1:map_1; 2:map_2; 3:map_3
  Feature ego_feature_;

  std::future<void> task_future_;

  bool started_drawing_ = false;
};

}  // namespace prediction
}  // namespace apollo
