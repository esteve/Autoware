/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LATTICE_VELOCITY_SET_HPP
#define LATTICE_VELOCITY_SET_HPP

#define UNUSED(x) (void)(x)

#include "libvelocity_set.h"
#include "autoware_msgs/ConfigLatticeVelocitySet.h"
#include "waypoint_follower/libwaypoint_follower.h"

namespace autoware
{
namespace planner
{
namespace lattice
{
class VelocitySet
{
  class PathVset : public WayPoints
  {
  private:
    autoware_msgs::lane temporal_waypoints_;

  public:
    void changeWaypoints(VelocitySet* velocity_set, int stop_waypoint);
    void avoidSuddenBraking(VelocitySet* velocity_set);
    void avoidSuddenAceleration(VelocitySet* velocity_set);
    void setDeceleration(VelocitySet* velocity_set);

    //===============================
    //       class function
    //===============================

    // check if waypoint number is valid
    bool checkWaypoint(int num, const char* name) const;
    void setTemporalWaypoints(VelocitySet* velocity_set);
    autoware_msgs::lane getTemporalWaypoints() const
    {
      return temporal_waypoints_;
    }
  };

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  int loop_rate_;

  bool use_crosswalk_detection_;

  ros::Subscriber localizer_sub_;
  ros::Subscriber control_pose_sub_;
  ros::Subscriber vscan_sub_;
  ros::Subscriber base_waypoint_sub_;
  ros::Subscriber obj_pose_sub_;
  ros::Subscriber current_vel_sub_;
  ros::Subscriber config_sub_;

  ros::Subscriber dtlane_sub_;
  ros::Subscriber area_sub_;
  ros::Subscriber line_sub_;
  ros::Subscriber point_sub_;

  ros::Publisher closest_waypoint_pub_;

  geometry_msgs::TwistStamped current_twist_;
  geometry_msgs::PoseStamped localizer_pose_;  // pose of sensor
  geometry_msgs::PoseStamped control_pose_;    // pose of base_link
  pcl::PointCloud<pcl::PointXYZ> vscan_;

  const std::string pedestrian_sound = "pedestrian";
  bool pose_flag_ = false;
  bool path_flag_ = false;
  bool vscan_flag_ = false;
  int obstacle_waypoint_ = -1;
  double deceleration_search_distance_ = 30;
  double search_distance_ = 60;
  int closest_waypoint_ = -1;
  double current_vel_ = 0.0;  // (m/s) subscribe estimated_vel
  autoware::planner::velocity_set::CrossWalk vmap;
  autoware::planner::velocity_set::ObstaclePoints obstacle_;

  /* Config Parameter */
  double detection_range_ = 0;       // if obstacle is in this range, stop
  double deceleration_range_ = 1.8;  // if obstacle is in this range, decelerate
  int threshold_points_ = 15;
  double detection_height_top_ = 2.0;  // actually +2.0m
  double detection_height_bottom_ = -2.0;
  double others_distance_ = 8.0;            // meter: stopping distance from obstacles (using VSCAN)
  double decel_ = 1.5;                      // (m/s) deceleration
  double velocity_change_limit_ = 2.778;    // (m/s) about 10 km/h
  double temporal_waypoints_size_ = 100.0;  // meter

  // Publisher
  ros::Publisher range_pub_;
  ros::Publisher deceleration_range_pub_;
  ros::Publisher sound_pub_;
  ros::Publisher safety_waypoint_pub_;
  ros::Publisher temporal_waypoints_pub_;
  ros::Publisher crosswalk_points_pub_;
  ros::Publisher obstacle_pub_;

  WayPoints path_dk_;

  PathVset path_change_;

  void configCallback(const autoware_msgs::ConfigLatticeVelocitySetConstPtr& config);

  void currentVelCallback(const geometry_msgs::TwistStampedConstPtr& msg);

  void baseWaypointCallback(const autoware_msgs::laneConstPtr& msg);

  void objPoseCallback(const visualization_msgs::MarkerConstPtr& msg);

  void vscanCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  void controlCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void localizerCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void updateClosesWaypoint();

  //===============================
  //          Callback
  //===============================

  void displayObstacle(const autoware::planner::velocity_set::EControl& kind);

  void displayDetectionRange(const int& crosswalk_id, const int& num,
                             const autoware::planner::velocity_set::EControl& kind);

  int findCrossWalk();

  autoware::planner::velocity_set::EControl crossWalkDetection(const int& crosswalk_id);

  autoware::planner::velocity_set::EControl vscanDetection();

  autoware::planner::velocity_set::EControl obstacleDetection();

  void changeWaypoint(const autoware::planner::velocity_set::EControl& detection_result);

public:
  VelocitySet(ros::NodeHandle& nh, ros::NodeHandle& private_nh, int loop_rate)
    : nh_(nh), private_nh_(private_nh), loop_rate_(loop_rate)
  {
    localizer_sub_ = nh_.subscribe("localizer_pose", 1, &VelocitySet::localizerCallback, this);
    control_pose_sub_ = nh_.subscribe("current_pose", 1, &VelocitySet::controlCallback, this);
    vscan_sub_ = nh_.subscribe("vscan_points", 1, &VelocitySet::vscanCallback, this);
    base_waypoint_sub_ = nh_.subscribe("base_waypoints", 1, &VelocitySet::baseWaypointCallback, this);
    obj_pose_sub_ = nh_.subscribe("obj_pose", 1, &VelocitySet::objPoseCallback, this);
    current_vel_sub_ = nh_.subscribe("current_velocity", 1, &VelocitySet::currentVelCallback, this);
    config_sub_ = nh_.subscribe("config/lattice_velocity_set", 10, &VelocitySet::configCallback, this);

    //------------------ Vector Map ----------------------//
    dtlane_sub_ = nh_.subscribe("vector_map_info/cross_walk", 1,
                                &autoware::planner::velocity_set::CrossWalk::crossWalkCallback, &vmap);
    area_sub_ =
        nh_.subscribe("vector_map_info/area", 1, &autoware::planner::velocity_set::CrossWalk::areaCallback, &vmap);
    line_sub_ =
        nh_.subscribe("vector_map_info/line", 1, &autoware::planner::velocity_set::CrossWalk::lineCallback, &vmap);
    point_sub_ =
        nh_.subscribe("vector_map_info/point", 1, &autoware::planner::velocity_set::CrossWalk::pointCallback, &vmap);
    //----------------------------------------------------//

    range_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detection_range", 0);
    sound_pub_ = nh_.advertise<std_msgs::String>("sound_player", 10);
    temporal_waypoints_pub_ = nh_.advertise<autoware_msgs::lane>("temporal_waypoints", 1000, true);
    closest_waypoint_pub_ = nh_.advertise<std_msgs::Int32>("closest_waypoint", 1000);
    obstacle_pub_ = nh_.advertise<visualization_msgs::Marker>("obstacle", 0);

    private_nh_.param<bool>("use_crosswalk_detection", use_crosswalk_detection_, true);
  }

  void updateCrossWalkPoints();

  void updateClosestWaypoint();

  void process();

  void publishClosestWaypoint();

  void updateDetectionWaypoint();
};
}  // namespace lattice
}  // namespace planner
}  // namespace autoware
#endif /* LATTICE_VELOCITY_SET_HPP */
