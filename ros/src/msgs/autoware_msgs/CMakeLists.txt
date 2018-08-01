cmake_minimum_required(VERSION 2.8.3)
project(autoware_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
        )


## Generate messages in the 'msg' folder
add_message_files(
        DIRECTORY msg
        FILES
            ControlCommandStamped.msg
            ColorSet.msg
            ControlCommand.msg
            TunedResult.msg
            ValueSet.msg
            Centroids.msg
            GeometricRectangle.msg
            ICPStat.msg
            NDTStat.msg
            ProjectionMatrix.msg
            VscanTracked.msg
            VscanTrackedArray.msg
            VehicleCmd.msg
            VehicleStatus.msg

            AccelCmd.msg
            AdjustXY.msg
            BrakeCmd.msg
            IndicatorCmd.msg
            LampCmd.msg
            SteerCmd.msg
            TrafficLight.msg
            StateCmd.msg
            State.msg
            ## Sync
            SyncTimeMonitor.msg
            SyncTimeDiff.msg

            ## Remote Control
            RemoteCmd.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
)
