/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
//
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "cyber/common/macros.h"

#include "modules/common/proto/geometry.pb.h"
#include "modules/map/proto/map_clear_area.pb.h"
#include "modules/map/proto/map_crosswalk.pb.h"
#include "modules/map/proto/map_junction.pb.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/map/proto/map_overlap.pb.h"
#include "modules/map/proto/map_parking_space.pb.h"
#include "modules/map/proto/map_road.pb.h"
#include "modules/map/proto/map_signal.pb.h"
#include "modules/map/proto/map_speed_bump.pb.h"
#include "modules/map/proto/map_stop_sign.pb.h"
#include "modules/map/proto/map_yield_sign.pb.h"

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_impl.h"  //hdmap_impl

/**
 * @namespace apollo::hdmap
 * @brief apollo::hdmap
 */
//apollo.hdmap
namespace apollo {
namespace hdmap {

//API
/**
 * @class HDMap
 *
 * @brief High-precision map loader interface.
 */

//class
class HDMap {
 public:
  /**
   * @brief load map from local file  从本地文件加载地图
   * @param map_filename path of map data file  地图数据地址
   * @return 0:success, otherwise failed    0为成功,其他为失败
   */
  int LoadMapFromFile(const std::string& map_filename); //LoaMapFromFile

  /**
   * @brief load map from a given protobuf message. 从给定的protobuf信息中加载map
   * @param map_proto map data in protobuf format   protobuf格式的map数据
   * @return 0:success, otherwise failed
   */
  int LoadMapFromProto(const Map& map_proto);   //LoadMapFromProto

  LaneInfoConstPtr GetLaneById(const Id& id) const;  //LaneInfo
  JunctionInfoConstPtr GetJunctionById(const Id& id) const;  //Junction
  SignalInfoConstPtr GetSignalById(const Id& id) const;    
  CrosswalkInfoConstPtr GetCrosswalkById(const Id& id) const;  //Crosswalk
  StopSignInfoConstPtr GetStopSignById(const Id& id) const;   
  YieldSignInfoConstPtr GetYieldSignById(const Id& id) const;  
  ClearAreaInfoConstPtr GetClearAreaById(const Id& id) const; 
  SpeedBumpInfoConstPtr GetSpeedBumpById(const Id& id) const;   
  OverlapInfoConstPtr GetOverlapById(const Id& id) const;      
  RoadInfoConstPtr GetRoadById(const Id& id) const;            //Road
  ParkingSpaceInfoConstPtr GetParkingSpaceById(const Id& id) const;
  PNCJunctionInfoConstPtr GetPNCJunctionById(const Id& id) const;
  RSUInfoConstPtr GetRSUById(const Id& id) const;

  /**
   * 车道
   * @brief get all lanes in certain range  在一定范围内,获取所有的车道
   * @param point the central point of the range  该范围内的中心点
   * @param distance the search radius            搜索的半径
   * @param lanes store all lanes in target range  在目标范围内,存储所有的车道
   * @return 0:success, otherwise failed
   */
  int GetLanes(const apollo::common::PointENU& point, double distance,
               std::vector<LaneInfoConstPtr>* lanes) const;
  /**
   * 交叉口
   * @brief get all junctions in certain range  在一定范围内,获取所有的交叉路口
   * @param point the central point of the range  该范围内的中心点
   * @param distance the search radius             搜索半径
   * @param junctions store all junctions in target range  在目标范围内,存储所有的交叉口
   * @return 0:success, otherwise failed
   */
  int GetJunctions(const apollo::common::PointENU& point, double distance,
                   std::vector<JunctionInfoConstPtr>* junctions) const;
  /**
   * @brief get all signals in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param signals store all signals in target range
   * @return 0:success, otherwise failed
   */
  int GetSignals(const apollo::common::PointENU& point, double distance,
                 std::vector<SignalInfoConstPtr>* signals) const;
  /**
   * 人行横道
   * @brief get all crosswalks in certain range  在一定范围内获取人行横道
   * @param point the central point of the range  该范围内的中心点
   * @param distance the search radius             搜索半径
   * @param crosswalks store all crosswalks in target range   在该目标范围内,存储所有的交叉口
   * @return 0:success, otherwise failed
   */
  int GetCrosswalks(const apollo::common::PointENU& point, double distance,
                    std::vector<CrosswalkInfoConstPtr>* crosswalks) const;
  /**
   * @brief get all stop signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param stop signs store all stop signs in target range
   * @return 0:success, otherwise failed
   */
  int GetStopSigns(const apollo::common::PointENU& point, double distance,
                   std::vector<StopSignInfoConstPtr>* stop_signs) const;
  /**
   * @brief get all yield signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param yield signs store all yield signs in target range
   * @return 0:success, otherwise failed
   */
  int GetYieldSigns(const apollo::common::PointENU& point, double distance,
                    std::vector<YieldSignInfoConstPtr>* yield_signs) const;
  /**
   * @brief get all clear areas in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param clear_areas store all clear areas in target range
   * @return 0:success, otherwise failed
   */
  int GetClearAreas(const apollo::common::PointENU& point, double distance,
                    std::vector<ClearAreaInfoConstPtr>* clear_areas) const;
  /**
   * @brief get all speed bumps in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param speed_bumps store all speed bumps in target range
   * @return 0:success, otherwise failed
   */
  int GetSpeedBumps(const apollo::common::PointENU& point, double distance,
                    std::vector<SpeedBumpInfoConstPtr>* speed_bumps) const;
  /**
   * roads
   * @brief get all roads in certain range   在一定范围内所有的道路
   * @param point the central point of the range  该范围内的中心点
   * @param distance the search radius            搜索半径
   * @param roads store all roads in target range   在目标范围内存储所有的道路
   * @return 0:success, otherwise failed
   */
  int GetRoads(const apollo::common::PointENU& point, double distance,
               std::vector<RoadInfoConstPtr>* roads) const;
  /**
   * @brief get all parking spaces in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param parking spaces store all clear areas in target range
   * @return 0:success, otherwise failed
   */
  int GetParkingSpaces(
      const apollo::common::PointENU& point, double distance,
      std::vector<ParkingSpaceInfoConstPtr>* parking_spaces) const;
  /**
   * @brief get all pnc junctions in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param junctions store all junctions in target range
   * @return 0:success, otherwise failed
   */
  int GetPNCJunctions(
      const apollo::common::PointENU& point, double distance,
      std::vector<PNCJunctionInfoConstPtr>* pnc_junctions) const;
  /**
   * 最邻近车道
   * @brief get nearest lane from target point,
   * @param point the target point
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLane(const apollo::common::PointENU& point,
                     LaneInfoConstPtr* nearest_lane, double* nearest_s,
                     double* nearest_l) const;
  /**
   * 获取带有heading的最近车道
   * @brief get the nearest lane within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLaneWithHeading(const apollo::common::PointENU& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;
  /**
   * @brief get all lanes within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading 基本heading
   * @param max_heading_difference the heading range
   * @param nearest_lane all lanes that match search conditions 存储匹配一定条件的所有车道
   * @return 0:success, otherwise, failed.
   */
  int GetLanesWithHeading(const apollo::common::PointENU& point,
                          const double distance, const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr>* lanes) const;
  /**
   * 道路边界
   * @brief get all road and junctions boundaries within certain range
   * @param point the target position
   * @param radius the search radius
   * @param road_boundaries the roads' boundaries
   * @param junctions the junctions' boundaries
   * @return 0:success, otherwise failed
   */
  int GetRoadBoundaries(const apollo::common::PointENU& point, double radius,
                        std::vector<RoadROIBoundaryPtr>* road_boundaries,
                        std::vector<JunctionBoundaryPtr>* junctions) const;
  /**
   * @brief get all road boundaries and junctions within certain range
   * @param point the target position
   * @param radius the search radius
   * @param road_boundaries the roads' boundaries
   * @param junctions the junctions
   * @return 0:success, otherwise failed
   */
  int GetRoadBoundaries(const apollo::common::PointENU& point, double radius,
                        std::vector<RoadRoiPtr>* road_boundaries,
                        std::vector<JunctionInfoConstPtr>* junctions) const;
  /**
   * @brief get ROI within certain range
   * @param point the target position
   * @param radius the search radius
   * @param roads_roi the roads' boundaries
   * @param polygons_roi the junctions' boundaries
   * @return 0:success, otherwise failed
   */
  int GetRoi(const apollo::common::PointENU& point, double radius,
             std::vector<RoadRoiPtr>* roads_roi,
             std::vector<PolygonRoiPtr>* polygons_roi);
  /**
   * @brief get forward nearest signals within certain range on the lane
   *        if there are two signals related to one stop line,
   *        return both signals.
   * @param point the target position
   * @param distance the forward search distance
   * @param signals all signals match conditions
   * @return 0:success, otherwise failed
   */
  int GetForwardNearestSignalsOnLane(
      const apollo::common::PointENU& point, const double distance,
      std::vector<SignalInfoConstPtr>* signals) const;

  /**
   * @brief get all other stop signs associated with a stop sign
   *        in the same junction
   * @param id id of stop sign
   * @param stop_signs stop signs associated
   * @return 0:success, otherwise failed
   */
  int GetStopSignAssociatedStopSigns(
      const Id& id, std::vector<StopSignInfoConstPtr>* stop_signs) const;

  /**
   * @brief get all lanes associated with a stop sign in the same junction
   * @param id id of stop sign
   * @param lanes all lanes match conditions
   * @return 0:success, otherwise failed
   */
  int GetStopSignAssociatedLanes(const Id& id,
                                 std::vector<LaneInfoConstPtr>* lanes) const;

  /**
   * @brief get a local map which is identical to the origin map except that all
   * map elements without overlap with the given region are deleted. 获取本区域的与原始地图一样的本地地图
   * @param point the target position   目标位置
   * @param range the size of local map region, [width, height]  范围
   * @param local_map local map in proto format      proto格式的本地地图
   * @return 0:success, otherwise failed
   */
  int GetLocalMap(const apollo::common::PointENU& point,
                  const std::pair<double, double>& range, Map* local_map) const;

  /**
   * @brief get forward nearest rsus within certain range
   * @param point the target position
   * @param distance the forward search distance
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param rsus all rsus that match search conditions
   * @return 0:success, otherwise failed
   */
  int GetForwardNearestRSUs(const apollo::common::PointENU& point,
                    double distance, double central_heading,
                    double max_heading_difference,
                    std::vector<RSUInfoConstPtr>* rsus) const;

 private:
  HDMapImpl impl_;//impl, implement
};

}  // namespace hdmap
}  // namespace apollo
