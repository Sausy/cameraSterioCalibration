#ifndef DATAMATCHER_H_
#define DATAMATCHER_H_

#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <unistd.h>

#include <opencv2/core/types.hpp>

// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <roboy_middleware_msgs/DarkRoomSensorV2.h>
#include <roboy_middleware_msgs/LighthousePoseCorrection.h>
#include <tf/tf.h>
#include <ros/package.h>


#include "DataMatcher.hpp"
#include "Utils.hpp"

#include <filter.hpp>

#include <datatype_htcDongle.h>




#define MIN_POSITIV_BASE_EVENTS 100
#define MAX_BASE_AMOUNT 16//we have 32Polynomes and per base 2 Polynomes are used
#define MAX_SENSOR_CNT 30

#define FILTER_HISTORY_LEN 10
#define FILTER_HISTORY_LEN_CH 50

#define MAX_DATA_HISTORY 20

class DataMatcher{
  public:
      DataMatcher(double * params_Lighthouse_, int maxSensorAvailable_);
      int maxSensorAvailable;
      //if we get a certain amount of data from a base Station
      //it will be added to the list of known Base Stations
      int registNewBaseStation(const std::vector<int> channel);
      int registNewBaseStation(const std::vector<rawRayData> ray_);

      //to start a new run matchData has to be properly reseted
      void reset_matchData();

      //match Data returns true if enough valid data was captured
      bool matchData( const std::vector<std::vector<float>> inVec,\
                      const std::vector<int> id,\
                      const std::vector<float> azimuth,\
                      const std::vector<float> elevation,\
                      const std::vector<int> channel\
                    );
      bool matchData( const std::vector<std::vector<double>> inVec,\
                      const std::vector<int> id,\
                      const std::vector<float> azimuth,\
                      const std::vector<float> elevation,\
                      const std::vector<int> channel\
                    );

      bool twoCameraMatcher();

      void filterfunc(std::vector<rawRayData> *ray);


      std::vector<cv::Point3f> list_points3d[MAX_BASE_AMOUNT]; // container for the model 3D coordinates found in the scene
      std::vector<cv::Point2f> list_points2d[MAX_BASE_AMOUNT]; // container for the model 2D coordinates found in the scene
      std::vector<int> list_id[MAX_BASE_AMOUNT];

      std::vector<cv::Point2f> dataImg1_2D;
      std::vector<cv::Point2f> dataImg2_2D;

      bool availableBaseStations[MAX_BASE_AMOUNT];
      bool dataReady[MAX_BASE_AMOUNT];

      std::vector<double> azimuthHistory[MAX_BASE_AMOUNT][MAX_SENSOR_CNT];
      std::vector<double> elevatiHistory[MAX_BASE_AMOUNT][MAX_SENSOR_CNT];

    private:
      double params_Lighthouse[4];
      uint16_t BaseStationsEventCount[MAX_BASE_AMOUNT];
      int goodCount;

      bool idIsTaken[MAX_BASE_AMOUNT][MAX_SENSOR_CNT];
      uint8_t idIsTaken_cnt[MAX_BASE_AMOUNT][MAX_SENSOR_CNT];

      std::vector<double> azimuth_buffer[MAX_BASE_AMOUNT][MAX_SENSOR_CNT];
      std::vector<double> elevation_buffer[MAX_BASE_AMOUNT][MAX_SENSOR_CNT];

      bool customFilter(int id_,double *azimuth_, double *elevation_, int channel_);

      //filter *f;
      //the additional 2 is needed, because of azimuth&elevation
      filter *filterClass[MAX_BASE_AMOUNT][MAX_SENSOR_CNT][2];
      filter *filterClass_perCh[MAX_BASE_AMOUNT][2];

      int returnAmountOfBaseStations;

      bool wasRepos[3];
      uint8_t wasRepos_cnt;
      std::vector<cv::Point2f> dataImg1_2D_buffer;
      std::vector<cv::Point2f> dataImg2_2D_buffer;


      ros::NodeHandlePtr  nh;

      int lastFilter_id[MAX_BASE_AMOUNT];



};

#endif
