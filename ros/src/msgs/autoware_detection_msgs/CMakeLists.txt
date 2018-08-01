cmake_minimum_required(VERSION 2.8.3)
project(autoware_detection_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  autoware_msgs
  geometry_msgs
  std_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
    CloudCluster.msg
    CloudClusterArray.msg
    DetectedObject.msg
    DetectedObjectArray.msg
    ExtractedPosition.msg
    ImageLaneObjects.msg
    ImageObjects.msg
    PointsImage.msg
    ScanImage.msg
    ImageObj.msg
    ImageObjRanged.msg
    ImageObjTracked.msg
    ImageRect.msg
    ImageRectRanged.msg
    ObjLabel.msg
    ObjPose.msg
    Signals.msg
    TrafficLightResult.msg
    TrafficLightResultArray.msg
    Lane.msg
    LaneArray.msg
    Waypoint.msg
    WaypointState.msg
    DTLane.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  autoware_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  autoware_msgs
  geometry_msgs
  std_msgs
)
