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
    aruco_pose_sub = nh->subscribe("/roboy/middleware/ArucoPose", 1, &TrackerClass::receiveArucoPose, this);
    */

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

    //interactive_marker_sub = nh->subscribe("/interactive_markers/feedback", 1,
    //                                       &TrackerClass::interactiveMarkersFeedback, this);



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
            if (random_pose) {
                // randomly moves all objects
                for (auto &object:trackedObjects) {
                    object->pose.setOrigin( tf::Vector3(pos(0),pos(1),pos(2)));
                    tf::Quaternion q(0, 0, 0, 1);
                    q.setRPY(random_pose_roll, random_pose_pitch, random_pose_yaw);
                    object->pose.setRotation(q);

                    pos(0) += vel(0)*1/1000.0+(rand()/(double)RAND_MAX-0.5)*0.001;
                    pos(1) += vel(1)*1/1000.0+(rand()/(double)RAND_MAX-0.5)*0.001;
                    pos(2) += vel(2)*1/1000.0+(rand()/(double)RAND_MAX-0.5)*0.001;
                    random_pose_roll += rand()/(double)RAND_MAX*1/1000.0;
                    random_pose_pitch += rand()/(double)RAND_MAX*1/1000.0;
                    random_pose_yaw += rand()/(double)RAND_MAX*1/1000.0;

                    if(pos(0)>boundary || pos(0)<-boundary)
                        vel(0) = -(vel(0)+(rand()/(double)RAND_MAX-0.5));
                    if(pos(1)>boundary || pos(1)<-boundary)
                        vel(1) = -(vel(1)+(rand()/(double)RAND_MAX-0.5));
                    if(pos(2)>boundary || pos(2)<-boundary)
                        vel(2) = -(vel(2)+(rand()/(double)RAND_MAX-0.5));
//
                }
            }
            for (auto &object:trackedObjects) {
                tf_broadcaster.sendTransform(tf::StampedTransform(object->pose, ros::Time::now(),
                                                                  "world", object->name.c_str()));
            }
//        }
        mux.unlock();
        rate.sleep();
    }
}


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
