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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "autoware_msgs/ConfigVelocitySet.h"
#include "autoware_msgs/ConfigLatticeVelocitySet.h"
#include <iostream>
#pragma GCC diagnostic pop

#include "autoware_msgs/lane.h"
#include "lattice_velocity_set.hpp"
#include "libvelocity_set.h"
#include "waypoint_follower/libwaypoint_follower.h"

namespace autoware
{
namespace planner
{
namespace lattice
{
bool VelocitySet::PathVset::checkWaypoint(VelocitySet* velocity_set, int num, const char* name) const
{
  if (num < 0 || num >= getSize())
  {
    return false;
  }
  return true;
}

// set about '_temporal_waypoints_size' meter waypoints from closest waypoint
void VelocitySet::PathVset::setTemporalWaypoints(VelocitySet* velocity_set)
{
  if (velocity_set->closest_waypoint_ < 0)
    return;
  int size = (int)(velocity_set->temporal_waypoints_size_ / getInterval()) + 1;

  temporal_waypoints_.waypoints.clear();
  temporal_waypoints_.header = current_waypoints_.header;
  temporal_waypoints_.increment = current_waypoints_.increment;
  // push current pose
  autoware_msgs::waypoint current_point;

  current_point.pose = velocity_set->control_pose_;
  current_point.twist = current_waypoints_.waypoints[velocity_set->closest_waypoint_].twist;
  current_point.dtlane = current_waypoints_.waypoints[velocity_set->closest_waypoint_].dtlane;
  temporal_waypoints_.waypoints.push_back(current_point);
  for (int i = 0; i < size; i++)
  {
    if (velocity_set->closest_waypoint_ + i >= getSize())
      return;
    temporal_waypoints_.waypoints.push_back(current_waypoints_.waypoints[velocity_set->closest_waypoint_ + i]);
  }

  return;
}

void VelocitySet::PathVset::setDeceleration(VelocitySet* velocity_set)
{
  int velocity_change_range = 5;
  double intervel = getInterval();
  double temp1 = velocity_set->current_vel_ * velocity_set->current_vel_;
  double temp2 = 2 * velocity_set->decel_ * intervel;
  double deceleration_minimum = kmph2mps(4.0);

  for (int i = 0; i < velocity_change_range; i++)
  {
    if (!checkWaypoint(velocity_set, velocity_set->closest_waypoint_ + i, "setDeceleration"))
      continue;
    double waypoint_velocity = current_waypoints_.waypoints[velocity_set->closest_waypoint_ + i].twist.twist.linear.x;
    double changed_vel = temp1 - temp2;
    if (changed_vel < 0)
    {
      changed_vel = deceleration_minimum * deceleration_minimum;
    }
    if (sqrt(changed_vel) > waypoint_velocity || deceleration_minimum > waypoint_velocity)
      continue;
    if (sqrt(changed_vel) < deceleration_minimum)
    {
      current_waypoints_.waypoints[velocity_set->closest_waypoint_ + i].twist.twist.linear.x = deceleration_minimum;
      continue;
    }
    current_waypoints_.waypoints[velocity_set->closest_waypoint_ + i].twist.twist.linear.x = sqrt(changed_vel);
  }

  return;
}

void VelocitySet::PathVset::avoidSuddenAceleration(VelocitySet* velocity_set)
{
  double changed_vel;
  double interval = getInterval();
  double temp1 = velocity_set->current_vel_ * velocity_set->current_vel_;
  double temp2 = 2 * velocity_set->decel_ * interval;
  double velocity_offset = 1.389;  // m/s

  for (int i = 0;; i++)
  {
    if (!checkWaypoint(velocity_set, velocity_set->closest_waypoint_ + i, "avoidSuddenAceleration"))
      return;
    changed_vel = sqrt(temp1 + temp2 * (double)(i + 1)) + velocity_offset;
    if (changed_vel > current_waypoints_.waypoints[velocity_set->closest_waypoint_ + i].twist.twist.linear.x)
      return;
    current_waypoints_.waypoints[velocity_set->closest_waypoint_ + i].twist.twist.linear.x = changed_vel;
  }

  return;
}

void VelocitySet::PathVset::avoidSuddenBraking(VelocitySet* velocity_set)
{
  int i = 0;
  int fill_in_zero = 20;
  int fill_in_vel = 15;
  int examin_range = 1;  // need to change according to waypoint interval?
  int num;
  double interval = getInterval();
  double changed_vel;

  for (int j = -1; j < examin_range; j++)
  {
    if (!checkWaypoint(velocity_set, velocity_set->closest_waypoint_ + j, "avoidSuddenBraking"))
      return;
    if (getWaypointVelocityMPS(velocity_set->closest_waypoint_ + j) <
        velocity_set->current_vel_ - velocity_set->velocity_change_limit_)  // we must change
                                                                            // waypoints
      break;
    if (j == examin_range - 1)  // we don't have to change waypoints
      return;
  }

  // fill in waypoints velocity behind vehicle
  for (num = velocity_set->closest_waypoint_ - 1; fill_in_vel > 0; fill_in_vel--)
  {
    if (!checkWaypoint(velocity_set, num - fill_in_vel, "avoidSuddenBraking"))
      continue;
    current_waypoints_.waypoints[num - fill_in_vel].twist.twist.linear.x = velocity_set->current_vel_;
  }

  // decelerate gradually
  double temp1 = (velocity_set->current_vel_ - velocity_set->velocity_change_limit_ + 1.389) *
                 (velocity_set->current_vel_ - velocity_set->velocity_change_limit_ + 1.389);
  double temp2 = 2 * velocity_set->decel_ * interval;
  for (num = velocity_set->closest_waypoint_ - 1;; num++)
  {
    if (num >= getSize())
      return;
    if (!checkWaypoint(velocity_set, num, "avoidSuddenBraking"))
      continue;
    changed_vel = temp1 - temp2 * (double)i;  // sqrt(v^2 - 2*a*x)
    if (changed_vel <= 0)
      break;
    current_waypoints_.waypoints[num].twist.twist.linear.x = sqrt(changed_vel);

    i++;
  }

  for (int j = 0; j < fill_in_zero; j++)
  {
    if (!checkWaypoint(velocity_set, num + j, "avoidSuddenBraking"))
      continue;
    current_waypoints_.waypoints[num + j].twist.twist.linear.x = 0.0;
  }

  return;
}

void VelocitySet::PathVset::changeWaypoints(VelocitySet* velocity_set, int stop_waypoint)
{
  int i = 0;
  int close_waypoint_threshold = 4;
  int fill_in_zero = 20;
  double changed_vel;
  double interval = getInterval();

  // change waypoints to decelerate
  for (int num = stop_waypoint; num > velocity_set->closest_waypoint_ - close_waypoint_threshold; num--)
  {
    if (!checkWaypoint(velocity_set, num, "changeWaypoints"))
      continue;

    changed_vel = sqrt(2.0 * velocity_set->decel_ * (interval * i));  // sqrt(2*a*x)

    autoware_msgs::waypoint initial_waypoint = velocity_set->path_dk_.getCurrentWaypoints().waypoints[num];
    if (changed_vel > initial_waypoint.twist.twist.linear.x)
    {  // avoid acceleration
      current_waypoints_.waypoints[num].twist.twist.linear.x = initial_waypoint.twist.twist.linear.x;
    }
    else
    {
      current_waypoints_.waypoints[num].twist.twist.linear.x = changed_vel;
    }

    i++;
  }

  // fill in 0
  for (int j = 1; j < fill_in_zero; j++)
  {
    if (!checkWaypoint(velocity_set, stop_waypoint + j, "changeWaypoints"))
      continue;
    current_waypoints_.waypoints[stop_waypoint + j].twist.twist.linear.x = 0.0;
  }

  return;
}

void VelocitySet::configCallback(const autoware_msgs::ConfigLatticeVelocitySetConstPtr& config)
{
  others_distance_ = config->others_distance;
  detection_range_ = config->detection_range;
  threshold_points_ = config->threshold_points;
  detection_height_top_ = config->detection_height_top;
  detection_height_bottom_ = config->detection_height_bottom;
  decel_ = config->deceleration;
  velocity_change_limit_ = kmph2mps(config->velocity_change_limit);
  deceleration_range_ = config->deceleration_range;
  temporal_waypoints_size_ = config->temporal_waypoints_size;
}

void VelocitySet::currentVelCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
  current_vel_ = msg->twist.linear.x;
}

void VelocitySet::baseWaypointCallback(const autoware_msgs::laneConstPtr& msg)
{
  path_dk_.setPath(*msg);
  path_change_.setPath(*msg);
  if (path_flag_ == false)
  {
    path_flag_ = true;
  }
}

void VelocitySet::objPoseCallback(const visualization_msgs::MarkerConstPtr& msg)
{
  // ROS_INFO("subscribed obj_pose\n");
}

void VelocitySet::vscanCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> vscan_raw;
  pcl::fromROSMsg(*msg, vscan_raw);

  vscan_.clear();
  for (const auto& v : vscan_raw)
  {
    if (v.x == 0 && v.y == 0)
      continue;
    if (v.z > detection_height_top_ || v.z < detection_height_bottom_)
      continue;
    vscan_.push_back(v);
  }

  if (vscan_flag_ == false)
  {
    vscan_flag_ = true;
  }
}

void VelocitySet::controlCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (!pose_flag_)
    pose_flag_ = true;

  control_pose_.header = msg->header;
  control_pose_.pose = msg->pose;
}

void VelocitySet::localizerCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  localizer_pose_.header = msg->header;
  localizer_pose_.pose = msg->pose;
}

//===============================
//          Callback
//===============================

void VelocitySet::displayObstacle(const autoware::planner::velocity_set::EControl& kind)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = obstacle_.getObstaclePoint(kind);
  if (kind == autoware::planner::velocity_set::EControl::OTHERS)
    marker.pose.position = obstacle_.getPreviousDetection();
  marker.pose.orientation = localizer_pose_.pose.orientation;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 2.0;
  marker.color.a = 0.7;
  if (kind == autoware::planner::velocity_set::EControl::STOP)
  {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  else
  {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  marker.lifetime = ros::Duration(0.1);
  marker.frame_locked = true;

  obstacle_pub_.publish(marker);
}

void VelocitySet::displayDetectionRange(const int& crosswalk_id, const int& num,
                                        const autoware::planner::velocity_set::EControl& kind)
{
  // set up for marker array
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker crosswalk_marker;
  visualization_msgs::Marker waypoint_marker_stop;
  visualization_msgs::Marker waypoint_marker_decelerate;
  visualization_msgs::Marker stop_line;
  crosswalk_marker.header.frame_id = "/map";
  crosswalk_marker.header.stamp = ros::Time();
  crosswalk_marker.id = 0;
  crosswalk_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  crosswalk_marker.action = visualization_msgs::Marker::ADD;
  waypoint_marker_stop = crosswalk_marker;
  waypoint_marker_decelerate = crosswalk_marker;
  stop_line = crosswalk_marker;
  stop_line.type = visualization_msgs::Marker::CUBE;

  // set each namespace
  crosswalk_marker.ns = "Crosswalk Detection";
  waypoint_marker_stop.ns = "Stop Detection";
  waypoint_marker_decelerate.ns = "Decelerate Detection";
  stop_line.ns = "Stop Line";

  // set scale and color
  double scale = 2 * detection_range_;
  waypoint_marker_stop.scale.x = scale;
  waypoint_marker_stop.scale.y = scale;
  waypoint_marker_stop.scale.z = scale;
  waypoint_marker_stop.color.a = 0.2;
  waypoint_marker_stop.color.r = 0.0;
  waypoint_marker_stop.color.g = 1.0;
  waypoint_marker_stop.color.b = 0.0;
  waypoint_marker_stop.frame_locked = true;

  scale = 2 * (detection_range_ + deceleration_range_);
  waypoint_marker_decelerate.scale.x = scale;
  waypoint_marker_decelerate.scale.y = scale;
  waypoint_marker_decelerate.scale.z = scale;
  waypoint_marker_decelerate.color.a = 0.15;
  waypoint_marker_decelerate.color.r = 1.0;
  waypoint_marker_decelerate.color.g = 1.0;
  waypoint_marker_decelerate.color.b = 0.0;
  waypoint_marker_decelerate.frame_locked = true;

  if (obstacle_waypoint_ > -1)
  {
    stop_line.pose.position = path_dk_.getWaypointPosition(obstacle_waypoint_);
    stop_line.pose.orientation = path_dk_.getWaypointOrientation(obstacle_waypoint_);
  }
  stop_line.pose.position.z += 1.0;
  stop_line.scale.x = 0.1;
  stop_line.scale.y = 15.0;
  stop_line.scale.z = 2.0;
  stop_line.color.a = 0.3;
  stop_line.color.r = 1.0;
  stop_line.color.g = 0.0;
  stop_line.color.b = 0.0;
  stop_line.lifetime = ros::Duration(0.1);
  stop_line.frame_locked = true;

  if (crosswalk_id > 0)
    scale = vmap.getDetectionPoints(crosswalk_id).width;
  crosswalk_marker.scale.x = scale;
  crosswalk_marker.scale.y = scale;
  crosswalk_marker.scale.z = scale;
  crosswalk_marker.color.a = 0.5;
  crosswalk_marker.color.r = 0.0;
  crosswalk_marker.color.g = 1.0;
  crosswalk_marker.color.b = 0.0;
  crosswalk_marker.frame_locked = true;

  // set marker points coordinate
  for (int i = 0; i < search_distance_; i++)
  {
    if (num < 0 || i + num > path_dk_.getSize() - 1)
      break;

    geometry_msgs::Point point;
    point = path_dk_.getWaypointPosition(num + i);

    waypoint_marker_stop.points.push_back(point);

    if (i > deceleration_search_distance_)
      continue;
    waypoint_marker_decelerate.points.push_back(point);
  }

  if (crosswalk_id > 0)
  {
    for (const auto& p : vmap.getDetectionPoints(crosswalk_id).points)
      crosswalk_marker.points.push_back(p);
  }

  // publish marker
  marker_array.markers.push_back(crosswalk_marker);
  marker_array.markers.push_back(waypoint_marker_stop);
  marker_array.markers.push_back(waypoint_marker_decelerate);
  if (kind == autoware::planner::velocity_set::EControl::STOP)
    marker_array.markers.push_back(stop_line);
  range_pub_.publish(marker_array);
  marker_array.markers.clear();
}

int VelocitySet::findCrossWalk()
{
  if (!vmap.set_points || closest_waypoint_ < 0)
    return -1;

  double find_distance = 2.0 * 2.0;      // meter
  double ignore_distance = 20.0 * 20.0;  // meter
  static std::vector<int> bdid = vmap.getBDID();
  // Find near cross walk
  for (int num = closest_waypoint_; num < closest_waypoint_ + search_distance_; num++)
  {
    geometry_msgs::Point waypoint = path_dk_.getWaypointPosition(num);
    waypoint.z = 0.0;  // ignore Z axis
    for (const auto& i : bdid)
    {
      // ignore far crosswalk
      geometry_msgs::Point crosswalk_center = vmap.getDetectionPoints(i).center;
      crosswalk_center.z = 0.0;
      if (autoware::planner::velocity_set::calcSquareOfLength(crosswalk_center, waypoint) > ignore_distance)
        continue;

      for (auto p : vmap.getDetectionPoints(i).points)
      {
        p.z = waypoint.z;
        if (autoware::planner::velocity_set::calcSquareOfLength(p, waypoint) < find_distance)
        {
          vmap.setDetectionCrossWalkID(i);
          return num;
        }
      }
    }
  }

  vmap.setDetectionCrossWalkID(-1);
  return -1;  // no near crosswalk
}

autoware::planner::velocity_set::EControl VelocitySet::crossWalkDetection(const int& crosswalk_id)
{
  double search_radius = vmap.getDetectionPoints(crosswalk_id).width / 2;

  // Search each calculated points in the crosswalk
  for (const auto& p : vmap.getDetectionPoints(crosswalk_id).points)
  {
    geometry_msgs::Point detection_point = calcRelativeCoordinate(p, localizer_pose_.pose);
    tf::Vector3 detection_vector = point2vector(detection_point);
    detection_vector.setZ(0.0);

    int stop_count = 0;  // the number of points in the detection area
    for (const auto& vscan : vscan_)
    {
      tf::Vector3 vscan_vector(vscan.x, vscan.y, 0.0);
      double distance = tf::tfDistance(vscan_vector, detection_vector);
      if (distance < search_radius)
      {
        stop_count++;
        geometry_msgs::Point vscan_temp;
        vscan_temp.x = vscan.x;
        vscan_temp.y = vscan.y;
        vscan_temp.z = vscan.z;
        obstacle_.setStopPoint(calcAbsoluteCoordinate(vscan_temp, localizer_pose_.pose));
      }
      if (stop_count > threshold_points_)
        return autoware::planner::velocity_set::EControl::STOP;
    }

    obstacle_.clearStopPoints();
  }

  return autoware::planner::velocity_set::EControl::KEEP;  // find no obstacles
}

autoware::planner::velocity_set::EControl VelocitySet::vscanDetection()
{
  if (vscan_.empty() == true || closest_waypoint_ < 0)
    return autoware::planner::velocity_set::EControl::KEEP;

  int decelerate_or_stop = -10000;
  int decelerate2stop_waypoints = 15;

  for (int i = closest_waypoint_; i < closest_waypoint_ + search_distance_; i++)
  {
    obstacle_.clearStopPoints();
    if (!obstacle_.isDecided())
      obstacle_.clearDeceleratePoints();

    decelerate_or_stop++;
    if (decelerate_or_stop > decelerate2stop_waypoints || (decelerate_or_stop >= 0 && i >= path_dk_.getSize() - 1) ||
        (decelerate_or_stop >= 0 && i == closest_waypoint_ + search_distance_ - 1))
      return autoware::planner::velocity_set::EControl::DECELERATE;
    if (i > path_dk_.getSize() - 1)
      return autoware::planner::velocity_set::EControl::KEEP;

    // Detection for cross walk
    if (i == vmap.getDetectionWaypoint())
    {
      if (crossWalkDetection(vmap.getDetectionCrossWalkID()) == autoware::planner::velocity_set::EControl::STOP)
      {
        obstacle_waypoint_ = i;
        return autoware::planner::velocity_set::EControl::STOP;
      }
    }

    // waypoint seen by vehicle
    geometry_msgs::Point waypoint = calcRelativeCoordinate(path_dk_.getWaypointPosition(i), localizer_pose_.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int stop_point_count = 0;
    int decelerate_point_count = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = vscan_.begin(); item != vscan_.end(); item++)
    {
      tf::Vector3 vscan_vector((double)item->x, (double)item->y, 0);

      // 2D distance between waypoint and vscan points(obstacle)
      // ---STOP OBSTACLE DETECTION---
      double dt = tf::tfDistance(vscan_vector, tf_waypoint);
      if (dt < detection_range_)
      {
        stop_point_count++;
        geometry_msgs::Point vscan_temp;
        vscan_temp.x = item->x;
        vscan_temp.y = item->y;
        vscan_temp.z = item->z;
        obstacle_.setStopPoint(calcAbsoluteCoordinate(vscan_temp, localizer_pose_.pose));
      }
      if (stop_point_count > threshold_points_)
      {
        obstacle_waypoint_ = i;
        return autoware::planner::velocity_set::EControl::STOP;
      }

      // without deceleration range
      if (deceleration_range_ < 0.01)
        continue;
      // deceleration search runs "decelerate_search_distance" waypoints from closest
      if (i > closest_waypoint_ + deceleration_search_distance_ || decelerate_or_stop >= 0)
        continue;

      // ---DECELERATE OBSTACLE DETECTION---
      if (dt > detection_range_ && dt < detection_range_ + deceleration_range_)
      {
        bool count_flag = true;

        // search overlaps between DETECTION range and DECELERATION range
        for (int waypoint_search = -5; waypoint_search <= 5; waypoint_search++)
        {
          if (i + waypoint_search < 0 || i + waypoint_search >= path_dk_.getSize() || !waypoint_search)
            continue;
          geometry_msgs::Point temp_waypoint =
              calcRelativeCoordinate(path_dk_.getWaypointPosition(i + waypoint_search), localizer_pose_.pose);
          tf::Vector3 waypoint_vector = point2vector(temp_waypoint);
          waypoint_vector.setZ(0);
          // if there is a overlap, give priority to DETECTION range
          if (tf::tfDistance(vscan_vector, waypoint_vector) < detection_range_)
          {
            count_flag = false;
            break;
          }
        }
        if (count_flag)
        {
          decelerate_point_count++;
          geometry_msgs::Point vscan_temp;
          vscan_temp.x = item->x;
          vscan_temp.y = item->y;
          vscan_temp.z = item->z;
          obstacle_.setDeceleratePoint(calcAbsoluteCoordinate(vscan_temp, localizer_pose_.pose));
        }
      }

      // found obstacle to DECELERATE
      if (decelerate_point_count > threshold_points_)
      {
        obstacle_waypoint_ = i;
        decelerate_or_stop = 0;  // for searching near STOP obstacle
        obstacle_.setDecided(true);
      }
    }
  }

  return autoware::planner::velocity_set::EControl::KEEP;  // no obstacles
}

/*
void soundPlay()
{
std_msgs::String string;
string.data = pedestrian_sound;
sound_pub_.publish(string);
}
*/

autoware::planner::velocity_set::EControl VelocitySet::obstacleDetection()
{
  static int false_count = 0;
  static autoware::planner::velocity_set::EControl prev_detection = autoware::planner::velocity_set::EControl::KEEP;

  autoware::planner::velocity_set::EControl vscan_result = vscanDetection();
  displayDetectionRange(vmap.getDetectionCrossWalkID(), closest_waypoint_, vscan_result);

  if (prev_detection == autoware::planner::velocity_set::EControl::KEEP)
  {
    if (vscan_result != autoware::planner::velocity_set::EControl::KEEP)
    {  // found obstacle
      displayObstacle(vscan_result);
      prev_detection = vscan_result;
      // SoundPlay();
      false_count = 0;
      return vscan_result;
    }
    else
    {  // no obstacle
      prev_detection = autoware::planner::velocity_set::EControl::KEEP;
      return vscan_result;
    }
  }
  else
  {  // prev_detection = STOP or DECELERATE
    if (vscan_result != autoware::planner::velocity_set::EControl::KEEP)
    {  // found obstacle
      displayObstacle(vscan_result);
      prev_detection = vscan_result;
      false_count = 0;
      return vscan_result;
    }
    else
    {  // no obstacle
      false_count++;

      // fail-safe
      if (false_count >= loop_rate_ / 2)
      {
        obstacle_waypoint_ = -1;
        false_count = 0;
        prev_detection = autoware::planner::velocity_set::EControl::KEEP;
        return vscan_result;
      }
      else
      {
        displayObstacle(autoware::planner::velocity_set::EControl::OTHERS);
        return prev_detection;
      }
    }
  }
}

void VelocitySet::updateClosestWaypoint()
{
  closest_waypoint_ = getClosestWaypoint(path_change_.getCurrentWaypoints(), control_pose_.pose);
}

void VelocitySet::changeWaypoint(const autoware::planner::velocity_set::EControl& detection_result)
{
  int obs = obstacle_waypoint_;

  if (detection_result == autoware::planner::velocity_set::EControl::STOP)
  {  // STOP for obstacle
    // stop_waypoint is about others_distance_ meter away from obstacles
    int stop_waypoint = obs - ((int)(others_distance_ / path_change_.getInterval()));
    // change waypoints to stop by the stop_waypoint
    path_change_.changeWaypoints(this, stop_waypoint);
    path_change_.avoidSuddenBraking(this);
    path_change_.setTemporalWaypoints(this);
    temporal_waypoints_pub_.publish(path_change_.getTemporalWaypoints());
  }
  else if (detection_result == autoware::planner::velocity_set::EControl::DECELERATE)
  {  // DECELERATE for obstacles
    path_change_.setPath(path_dk_.getCurrentWaypoints());
    path_change_.setDeceleration(this);
    path_change_.setTemporalWaypoints(this);
    temporal_waypoints_pub_.publish(path_change_.getTemporalWaypoints());
  }
  else
  {  // ACELERATE or KEEP
    path_change_.setPath(path_dk_.getCurrentWaypoints());
    path_change_.avoidSuddenAceleration(this);
    path_change_.avoidSuddenBraking(this);
    path_change_.setTemporalWaypoints(this);
    temporal_waypoints_pub_.publish(path_change_.getTemporalWaypoints());
  }

  return;
}

void VelocitySet::updateCrossWalkPoints()
{
  if (vmap.loaded_all && !vmap.set_points)
    vmap.setCrossWalkPoints();
}

void VelocitySet::publishClosestWaypoint()
{
  std_msgs::Int32 closest_waypoint;
  closest_waypoint.data = closest_waypoint_;
  closest_waypoint_pub_.publish(closest_waypoint);
}

void VelocitySet::updateDetectionWaypoint()
{
  if (use_crosswalk_detection_)
  {
    vmap.setDetectionWaypoint(findCrossWalk());
  }
  autoware::planner::velocity_set::EControl detection_result = obstacleDetection();

  changeWaypoint(detection_result);

  vscan_.clear();
}

void VelocitySet::process()
{
  updateCrossWalkPoints();

  if (pose_flag_ == false || path_flag_ == false)
  {
    return;
  }

  updateClosestWaypoint();

  publishClosestWaypoint();

  updateDetectionWaypoint();
}
}  // namespace lattice
}  // namespace planner
}  // namespace autoware

//======================================
//                 main
//======================================

static constexpr int LOOP_RATE = 10;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lattice_velocity_set");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  autoware::planner::lattice::VelocitySet velocity_set(nh, private_nh, LOOP_RATE);

  ros::Rate loop_rate(LOOP_RATE);

  while (ros::ok())
  {
    ros::spinOnce();

    velocity_set.process();

    loop_rate.sleep();
  }

  return 0;
}
