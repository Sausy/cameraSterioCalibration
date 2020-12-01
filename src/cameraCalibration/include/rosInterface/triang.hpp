#pragma once


#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <roboy_middleware_msgs/DarkRoomSensorV2.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <roboy_middleware_msgs/DarkRoom.h>
#include <roboy_middleware_msgs/DarkRoomStatistics.h>
#include <roboy_middleware_msgs/DarkRoomStatus.h>
#include <roboy_middleware_msgs/DarkRoomOOTX.h>
#include <roboy_middleware_msgs/DarkRoomSensor.h>
#include <roboy_middleware_msgs/DarkRoomSensorV2.h>
#include <roboy_middleware_msgs/LighthousePoseCorrection.h>
#include <ros/package.h>


#include <roboy_control_msgs/SetControllerParameters.h>
#include <std_srvs/Empty.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <darkroom/TrackedObject.hpp>
#include <darkroom/LighthouseSimulator.hpp>
#include <darkroom/Transform.hpp>

#include <boost/filesystem.hpp>
#include <common_utilities/rviz_visualization.hpp>
#include <common_utilities/CommonDefinitions.h>

#include <sstream>
#include <string>

#include <datatype_htcDongle.h>

//public cardsflow::vrpuppet::Robot
class TrackerClass {
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
     TrackerClass();

     void correctPose(const roboy_middleware_msgs::LighthousePoseCorrection &msg);


     void imuGet(const roboy_middleware_msgs::LighthousePoseCorrection &msg);
     void imuGet_pos(const roboy_middleware_msgs::DarkRoomSensor::ConstPtr &msg);
     void interactiveMarkersFeedback(const visualization_msgs::InteractiveMarkerFeedback &msg);
     void resetLighthousePoses();
     void updateTrackedObjectInfo();
     void transformPublisher();

     bool addTrackedObject(const char *config_file_path);
     void removeTrackedObject();

     void showRays();
     void startTriangulation();


     void receiveSensorStatus(const roboy_middleware_msgs::DarkRoomStatus::ConstPtr &msg);

     void publishSensorData(std::vector<rawRayData> *ray);

     void receiveArucoPose(const roboy_middleware_msgs::ArucoPose::ConstPtr &msg);



    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::Subscriber pose_correction_sub, interactive_marker_sub, sensor_status_sub, imu_sub, imu_pos, aruco_pose_sub;
    ros::Publisher sensorPub;


    //ros::AsyncSpinner *spinner;

    boost::shared_ptr<ros::AsyncSpinner> spinner;


    boost::shared_ptr<tf::TransformListener> listener;
    vector<TrackedObjectPtr> trackedObjects;
    //vector<TrackedObjectInfo> trackedObjectsInfo;
    vector<LighthouseSimulatorPtr> lighthouseSimulator;

    //bool publish_transform = true;
    tf::TransformBroadcaster tf_broadcaster;
    mutex mux;

    boost::shared_ptr<std::thread> transform_thread = nullptr, update_tracked_object_info_thread = nullptr;

    static tf::Transform lighthouse1, lighthouse2, tf_world, tf_map,
            simulated_object_lighthouse1, simulated_object_lighthouse2, imu_foo;
    vector<pair<LighthouseSimulatorPtr,LighthouseSimulatorPtr>> lighthouse_simulation;


    rviz_visualization rvizVis;

    float random_pose_x = 0, random_pose_y = 0, random_pose_z = 0, random_pose_roll = 0, random_pose_pitch = 0, random_pose_yaw = 0;

    bool random_pose = false;
    bool simulated = false;
    vector<string> trackedObjectsIDs;
    int object_counter = 0;
    atomic<bool> publish_transform, update_tracked_object_info;

    map<int,Vector3d> aruco_position_mean;
    map<int,Vector3d> aruco_position_variance;
    map<int,long> receive_counter;
    LighthouseCalibration calibration[2][2];



    /**
     * Toggles poseestimation thread
     */
    void startPoseEstimationSensorCloud();
    /**
     * Toggles object poseestimation thread
     */
    void startObjectPoseEstimationSensorCloud(bool run);
    /**
     * Toggles distance estimation thread
     */
    void startEstimateSensorPositionsUsingRelativeDistances();
    /**
     * Toggles relative pose estimation thread
     */
    void startEstimateObjectPoseUsingRelativeDistances();
    /**
     * Toggles relative pose estimation epnp thread
     */
    void startEstimateObjectPoseEPNP(bool run);
    /**
     * Toggles object pose estimation using multi lighthouse approach
     */
    void startEstimateObjectPoseMultiLighthouse(bool run);
};
