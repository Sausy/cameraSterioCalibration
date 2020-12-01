// C++
#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include <vector>

#include "triang.hpp"

tf::Transform TrackerClass::lighthouse1;
tf::Transform TrackerClass::lighthouse2;
tf::Transform TrackerClass::simulated_object_lighthouse1;
tf::Transform TrackerClass::simulated_object_lighthouse2;
tf::Transform TrackerClass::tf_world;
tf::Transform TrackerClass::tf_map;
tf::Transform TrackerClass::imu_foo;


TrackerClass::TrackerClass(){

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "tracker");
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);


    sensorPub = nh->advertise<roboy_middleware_msgs::DarkRoomSensorV2>("/roboy/middleware/DarkRoom/sensorsLH2", 1);


    pose_correction_sub = nh->subscribe("/roboy/middleware/DarkRoom/lhcorrect", 1,
                                        &TrackerClass::correctPose, this);
    imu_sub = nh->subscribe("/roboy/middleware/DarkRoom/imuLH2", 1, &TrackerClass::imuGet, this);
    imu_pos = nh->subscribe("/roboy/middleware/DarkRoom/sensor_location", 1, &TrackerClass::imuGet_pos, this);



    /*
    sensor_sub = nh->subscribe("/roboy/middleware/DarkRoom/sensors", 1, &TrackerClass::receiveSensorData, this);

    statistics_sub = nh->subscribe("/roboy/middleware/DarkRoom/Statistics", 2, &TrackerClass::receiveStatistics, this);
    ootx_sub = nh->subscribe("/roboy/middleware/DarkRoom/ootx", 1, &TrackerClass::receiveOOTXData, this);
    */
    aruco_pose_sub = nh->subscribe("/roboy/middleware/ArucoPose", 1, &TrackerClass::receiveArucoPose, this);


    sensor_status_sub = nh->subscribe("/roboy/middleware/DarkRoom/status", 1, &TrackerClass::receiveSensorStatus, this);



    /*
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->startObjectPoseEstimationSensorCloud();
    */
    //spinner = new ros::AsyncSpinner(0);
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    resetLighthousePoses();

    publish_transform = true;
    transform_thread = boost::shared_ptr<std::thread>(new std::thread(&TrackerClass::transformPublisher, this));
    transform_thread->detach();

    update_tracked_object_info = true;
    update_tracked_object_info_thread = boost::shared_ptr<std::thread>(
            new std::thread(&TrackerClass::updateTrackedObjectInfo, this));
    update_tracked_object_info_thread->detach();

    interactive_marker_sub = nh->subscribe("/interactive_markers/feedback", 1,
                                           &TrackerClass::interactiveMarkersFeedback, this);

    /*
    if (nh->hasParam("simulated")) {
        nh->getParam("simulated", simulated);
    }*/


    rvizVis.make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, lighthouse1.getOrigin(),
                   true, 0.1, "world", "lighthouse1", "");
    rvizVis.make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, lighthouse2.getOrigin(),
                   true, 0.1, "world", "lighthouse2", "");




    std::string msg;
    std::stringstream ss;
    //ss << ros::package::getPath("robots") << "/" << "exohaptic" << "/lighthouseSensors/left_link5.yaml";
    ss <<  "/usr/games/robots" << "/" << "exohaptic" << "/lighthouseSensors/base.yaml";
    msg = ss.str();

    addTrackedObject(msg.c_str());



    vector<string> joint_names;
    nh->getParam("joint_names", joint_names);


    showRays();

    startTriangulation();
    //startEstimateObjectPoseMultiLighthouse();
    //switchLighthouses();

    //LighthouseEstimator::publishRays
    //publishRays




    ROS_INFO("Finished setup");
}

void TrackerClass::imuGet_pos(const roboy_middleware_msgs::DarkRoomSensor::ConstPtr &msg) {
    mux.lock();

    tf::Transform tf;
    //tf::transformMsgToTF(msg.tf, tf);
    tf::Vector3 tf_vec;

    geometry_msgs::Vector3 v;
    v = msg->position[0];

    tf::vector3MsgToTF(v, tf_vec);

    imu_foo.setOrigin(tf_vec);

    tf_broadcaster.sendTransform(tf::StampedTransform(imu_foo, ros::Time::now(), "world", "imu"));

    mux.unlock();
}

void TrackerClass::imuGet(const roboy_middleware_msgs::LighthousePoseCorrection &msg) {
  mux.lock();

  tf::Transform tf;
  tf::transformMsgToTF(msg.tf, tf);

  //if (msg.type == 0) // relativ
  //    imu_foo = tf * imu_foo;
  //else    // absolut
  imu_foo = tf;

  //tf::Transform foo;
  //tf::TransformBroadcaster foo_br;

  imu_foo.setOrigin(tf::Vector3(0.0,2.0,0.0));
  //foo.setRotation(tf::Quaternion(0,0,0,1));
  //foo_br.sendTransform(tf::StampedTransform(foo, ros::Time::now(), "world", "imu"));

  tf_broadcaster.sendTransform(tf::StampedTransform(imu_foo, ros::Time::now(), "world", "imu"));

  //foo.setOrigin(tf::Vector3(3.0,0.0,0.0));
  //foo.setRotation(tf::Quaternion(0,0,0,1));

  //foo_br.sendTransform(tf::StampedTransform(foo, ros::Time::now(), "world", "foo"));

  mux.unlock();

}

void TrackerClass::startTriangulation() {
    //std::cout<<"amount of tracked Objects " << trackedObjects.size();
    ROS_INFO("amount of tread objects %d", trackedObjects.size());
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();

            ROS_INFO("starting tracking thread");
            trackedObjects[i]->tracking = true;
            trackedObjects[i]->tracking_thread = boost::shared_ptr<boost::thread>(
                    new boost::thread(
                            [this, i]() { this->trackedObjects[i]->triangulateSensors(); }
                    ));
            trackedObjects[i]->tracking_thread->detach();

        trackedObjects[i]->mux.unlock();
    }
}

void TrackerClass::startPoseEstimationSensorCloud() {
    ROS_DEBUG("pose_correction_sensor_cloud clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
        ROS_INFO("starting pose estimation thread");
        trackedObjects[i]->poseestimating = true;
        trackedObjects[i]->poseestimation_thread = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->lighthousePoseEstimationLeastSquares();
                }));
        trackedObjects[i]->poseestimation_thread->detach();
        trackedObjects[i]->mux.unlock();
    }
}

void TrackerClass::startObjectPoseEstimationSensorCloud(bool run) {
    ROS_DEBUG("object pose estimation clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
        if (run) {
            ROS_INFO("starting pose estimation thread");
            if(run){
                ROS_INFO("starting tracking thread");
                trackedObjects[i]->tracking = true;
                trackedObjects[i]->tracking_thread = boost::shared_ptr<boost::thread>(
                        new boost::thread(
                                [this, i]() { this->trackedObjects[i]->triangulateSensors(); }
                        ));
                trackedObjects[i]->tracking_thread->detach();
            }
            trackedObjects[i]->objectposeestimating = true;
            trackedObjects[i]->objectposeestimation_thread = boost::shared_ptr<boost::thread>(
                    new boost::thread([this, i]() {
                        this->trackedObjects[i]->objectPoseEstimationLeastSquares();
                    }));
            trackedObjects[i]->objectposeestimation_thread->detach();
        } else {
            if (trackedObjects[i]->objectposeestimation_thread != nullptr) {
                ROS_INFO("stopping pose estimation thread");
                trackedObjects[i]->objectposeestimating = false;
                if (trackedObjects[i]->objectposeestimation_thread->joinable()) {
                    ROS_INFO_THROTTLE(1, "Waiting for pose estimation thread to terminate");
                    trackedObjects[i]->objectposeestimation_thread->join();
                }
            }
        }
        trackedObjects[i]->mux.unlock();
    }
}

void TrackerClass::startEstimateSensorPositionsUsingRelativeDistances() {
    ROS_DEBUG("position_estimation_relativ_sensor_distances clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
        ROS_INFO("starting relativ distance thread for lighthouse 1");
        trackedObjects[i]->distance_thread_1 = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_A);
                }));
        trackedObjects[i]->distance_thread_1->detach();
        ROS_INFO("starting relativ distance thread for lighthouse 2");
        trackedObjects[i]->distance_thread_2 = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_B);
                }));
        trackedObjects[i]->distance_thread_2->detach();
        trackedObjects[i]->mux.unlock();
    }
}

void TrackerClass::startEstimateObjectPoseUsingRelativeDistances() {
    ROS_DEBUG("pose_estimation_relativ_sensor_distances clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
        ROS_INFO("starting relativ pose thread");
        trackedObjects[i]->relative_pose_thread = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->estimateObjectPoseUsingRelativeDistances();
                }));
        trackedObjects[i]->relative_pose_thread->detach();
        trackedObjects[i]->mux.unlock();
    }
}

void TrackerClass::startEstimateObjectPoseEPNP(bool run) {
    ROS_DEBUG("pose_estimation_epnp clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
        if(run) {
            ROS_INFO("starting relativ pose epnp thread");
            trackedObjects[i]->poseestimating_epnp = true;
            trackedObjects[i]->relative_pose_epnp_thread = boost::shared_ptr<boost::thread>(
                    new boost::thread([this, i]() {
                        this->trackedObjects[i]->estimateObjectPoseEPNP();
                    }));
            trackedObjects[i]->relative_pose_epnp_thread->detach();
        }else{
            trackedObjects[i]->poseestimating_epnp = false;
        }
        trackedObjects[i]->mux.unlock();
    }
}


void TrackerClass::startEstimateObjectPoseMultiLighthouse(bool run) {
    ROS_DEBUG("pose_estimation_multi_lighthouse clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
        if(run) {
            ROS_INFO("starting multi lighthouse pose estimation thread");
            trackedObjects[i]->poseestimating_multiLighthouse = true;
            trackedObjects[i]->object_pose_estimation_multi_lighthouse_thread = boost::shared_ptr<boost::thread>(
                    new boost::thread([this, i]() {
                        this->trackedObjects[i]->estimateObjectPoseMultiLighthouse();
                    }));
            trackedObjects[i]->object_pose_estimation_multi_lighthouse_thread->detach();
        }else{
            trackedObjects[i]->poseestimating_multiLighthouse = false;
        }
        trackedObjects[i]->mux.unlock();
    }
}



void TrackerClass::receiveSensorStatus(const roboy_middleware_msgs::DarkRoomStatus::ConstPtr &msg){
  int active_sensors = 0;
  for(auto status:msg->sensor_state){
      if(status == 1)
          active_sensors++;
  }
  ptrdiff_t pos = find(trackedObjectsIDs.begin(), trackedObjectsIDs.end(), msg->object_id) - trackedObjectsIDs.begin();
  if(pos<trackedObjects.size())
      trackedObjects[pos]->active_sensors = active_sensors;
}

void TrackerClass::correctPose(const roboy_middleware_msgs::LighthousePoseCorrection &msg) {
    mux.lock();
    tf::Transform tf;
    tf::transformMsgToTF(msg.tf, tf);

    if (msg.id == LIGHTHOUSE_A) {
        if (msg.type == 0) // relativ
            lighthouse1 = tf * lighthouse1;
        else    // absolut
            lighthouse1 = tf;
    } else {
        if (msg.type == 0) // relativ
            lighthouse2 = tf * lighthouse2;
        else    // absolut
            lighthouse2 = tf;
    }

    mux.unlock();
}

/*
void TrackerClass::transformPublisher() {
    ros::Rate rate(60);
    while (publish_transform && ros::ok()) {
        tf_broadcaster.sendTransform(
                tf::StampedTransform(lighthouse1, ros::Time::now(), "world", "lighthouse1"));
        tf_broadcaster.sendTransform(
                tf::StampedTransform(lighthouse2, ros::Time::now(), "world", "lighthouse2"));
        rate.sleep();
    }
}

*/

void TrackerClass::interactiveMarkersFeedback(const visualization_msgs::InteractiveMarkerFeedback &msg) {
    mux.lock();
    tf::Vector3 position(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    tf::Quaternion orientation(msg.pose.orientation.x, msg.pose.orientation.y,
                               msg.pose.orientation.z, msg.pose.orientation.w);
    if (strcmp(msg.marker_name.c_str(), "lighthouse1") == 0) {
        lighthouse1.setOrigin(position);
        lighthouse1.setRotation(orientation);
    } else if (strcmp(msg.marker_name.c_str(), "lighthouse2") == 0) {
        lighthouse2.setOrigin(position);
        lighthouse2.setRotation(orientation);
    } else if (strcmp(msg.marker_name.c_str(), "trackedObject") == 0) {
        for (auto &object:trackedObjects) {
            object->pose.setOrigin(position);
            object->pose.setRotation(orientation);
        }
    }
    mux.unlock();
}


void TrackerClass::resetLighthousePoses() {
    ROS_DEBUG("reset lighthouse poses");
    tf_world.setOrigin(tf::Vector3(0, 0, 0));
    tf_map.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion quat;
    quat.setRPY(0, 0, 0);
    tf_world.setRotation(quat);
    tf_map.setRotation(quat);
    bool ok;
    quat.setRPY(0, 0, 0);
    lighthouse1.setRotation(quat);
//    lighthouse1.setOrigin(tf::Vector3(1, -2, 0));
    lighthouse1.setOrigin(tf::Vector3(0, -1, 0));
    lighthouse2.setRotation(quat);
//    lighthouse2.setOrigin(tf::Vector3(-1, -2, 0));
    lighthouse2.setOrigin(tf::Vector3(-0.825, -1,0));
    imu_foo.setOrigin(tf::Vector3(2, -1,0.5));

}


void TrackerClass::updateTrackedObjectInfo() {
    ros::Rate rate(1);
    while (update_tracked_object_info) {
        mux.lock();
        for (int i = 0; i < trackedObjects.size(); i++) {
            char str[100];
            sprintf(str, "%d/%d", trackedObjects[i]->active_sensors, (int) trackedObjects[i]->sensors.size());
            //trackedObjectsInfo[i].activeSensors->setText(str);
        }
        mux.unlock();
        rate.sleep();
    }
}


void TrackerClass::transformPublisher() {
    ros::Rate rate(30);
    Vector3d pos(0,0,0), vel(0.3,0.3,0.3);
    double boundary = 0.5;
    while (publish_transform) {
        mux.lock();

        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse1, ros::Time::now(), "world", "lighthouse1"));
        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse2, ros::Time::now(), "world", "lighthouse2"));

        for (auto &object:trackedObjects) {
            tf_broadcaster.sendTransform(tf::StampedTransform(object->pose, ros::Time::now(),
                                                              "world", object->name.c_str()));
        }
//        }
        mux.unlock();
        rate.sleep();
    }
}


/*
bool TrackerClass::addTrackedObject(const char *config_file_path) {
    mux.lock();
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject(nh));


    if (!newObject->init(config_file_path))
        return false;
    if (simulated) {
        pair<LighthouseSimulatorPtr, LighthouseSimulatorPtr> simulation;

        vector<fs::path> parts = {config_file_path};

        simulation.first.reset(new LighthouseSimulator(LIGHTHOUSE_A, parts));
        simulation.second.reset(new LighthouseSimulator(LIGHTHOUSE_B, parts));
        simulation.first->startSensorPublisher();
        simulation.second->startSensorPublisher();
        lighthouse_simulation.push_back(simulation);
    }


    ROS_DEBUG_STREAM("adding tracked object " << config_file_path);
    trackedObjects.push_back(newObject);
    trackedObjectsIDs.push_back(newObject->objectID);
    //object_counter++;

    //TrackedObjectInfo info;


    //trackedObjectsInfo.push_back(info);

    mux.unlock();

    return true;
}
*/


void TrackerClass::showRays() {
    ROS_DEBUG("show rays clicked");

    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();

        ROS_INFO("starting rays thread i:%d", i);
        trackedObjects[i]->rays = true;
        trackedObjects[i]->rays_thread = boost::shared_ptr<boost::thread>(
                new boost::thread(
                        [this, i]() { this->trackedObjects[i]->publishRays(); }
                ));
        trackedObjects[i]->rays_thread->detach();

        trackedObjects[i]->mux.unlock();
    }
}

void TrackerClass::publishSensorData(std::vector<rawRayData> *ray){


  roboy_middleware_msgs::DarkRoomSensorV2 ros_msg;

  for(std::vector<rawRayData>::iterator vIt = ray->begin(); vIt != ray->end(); ++vIt){
    //======[ROS Msg]=====
    ros_msg.object_id = "noclue";
    ros_msg.base = vIt->ch;
    ros_msg.SensorID = (uint8_t)vIt->id;
    ros_msg.elevation = vIt->elevation;
    ros_msg.azimuth = vIt->azimuth;

    //ros_msg.elevation = vIt->elMean;
    //ros_msg.azimuth = vIt->azMean;


    sensorPub.publish(ros_msg);
  }


}

bool TrackerClass::addTrackedObject(const char *config_file_path) {
    mux.lock();
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject(nh));

    if (!newObject->init(config_file_path))
        return false;

    ROS_DEBUG_STREAM("adding tracked object " << config_file_path);
    trackedObjects.push_back(newObject);
    trackedObjectsIDs.push_back(newObject->objectID);
    object_counter++;


    mux.unlock();
    return true;
}

/**
 * removes the selected tracked object
 */
void TrackerClass::removeTrackedObject() {
    mux.lock();
    publish_transform = false;

    if (transform_thread->joinable()) {
        ROS_INFO("waiting for transform thread to shut down");
        transform_thread->join();
    }
    update_tracked_object_info = false;
    if (update_tracked_object_info_thread->joinable()) {
        ROS_INFO("waiting for update_tracked_object_info_thread to shut down");
        update_tracked_object_info_thread->join();
    }
    int i = 0;
  /*  for (auto it = trackedObjects.begin(); it != trackedObjects.end();) {
        if (trackedObjectsInfo[i].selected->isChecked()) {
            // remove gui elements
            delete trackedObjectsInfo[i].name;
            delete trackedObjectsInfo[i].activeSensors;
            delete trackedObjectsInfo[i].selected;
            delete trackedObjectsInfo[i].widget;
            trackedObjectsInfo.erase(trackedObjectsInfo.begin() + i);
            // if we are simulating, shutdown lighthouse simulators
            // erase the trackedObject
            it = trackedObjects.erase(it);
        } else {
            i++;
            ++it;
        }
    }
    */
    publish_transform = true;
    transform_thread = boost::shared_ptr<std::thread>(new std::thread(&TrackerClass::transformPublisher, this));
    transform_thread->detach();

    update_tracked_object_info = true;
    update_tracked_object_info_thread = boost::shared_ptr<std::thread>(
            new std::thread(&TrackerClass::updateTrackedObjectInfo, this));
    update_tracked_object_info_thread->detach();

    mux.unlock();
}

void TrackerClass::receiveArucoPose(const roboy_middleware_msgs::ArucoPose::ConstPtr &msg){
    int i=0;
    // running mean and variance (cf https://www.johndcook.com/blog/standard_deviation/ )
    stringstream str;
    for(int id:msg->id){
        if(receive_counter.find(id)==receive_counter.end()){
            receive_counter[id] = 1;
            aruco_position_mean[id].setZero();
            aruco_position_variance[id].setZero();
        }

        Vector3d pos(msg->pose[i].position.x,
                     msg->pose[i].position.y,
                     msg->pose[i].position.z);
        // using lazy average at this point
        Vector3d new_mean = aruco_position_mean[id]*0.9 + pos * 0.1;
        aruco_position_variance[id](0) += (pos(0)-aruco_position_mean[id](0))*(pos(0)-new_mean(0));
        aruco_position_variance[id](1) += (pos(1)-aruco_position_mean[id](1))*(pos(1)-new_mean(1));
        aruco_position_variance[id](2) += (pos(2)-aruco_position_mean[id](2))*(pos(2)-new_mean(2));
        aruco_position_mean[id] = new_mean;
        str << "\naruco id " << id << " \nmean " << aruco_position_mean[id].transpose() << " \nvariance " << aruco_position_variance[id].transpose() << endl;
        i++;
        receive_counter[id]++;
    }

    for(auto &object:trackedObjects){
        object->pose.setOrigin(tf::Vector3(aruco_position_mean[282](0),aruco_position_mean[282](1),aruco_position_mean[282](2)));
    }


    ROS_INFO_STREAM_THROTTLE(1,str.str());
}
