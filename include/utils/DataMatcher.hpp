#ifndef DATAMATCHER_H_
#define DATAMATCHER_H_

#include <filter.hpp>

#define MIN_POSITIV_BASE_EVENTS 100
#define MAX_BASE_AMOUNT 16//we have 32Polynomes and per base 2 Polynomes are used
#define MAX_SENSOR_CNT 30

#define FILTER_HISTORY_LEN 200

#define MAX_DATA_HISTORY 20

class DataMatcher{
  public:
      DataMatcher(double * params_Lighthouse_);

      //if we get a certain amount of data from a base Station
      //it will be added to the list of known Base Stations
      void registNewBaseStation(const std::vector<int> channel);

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


      std::vector<cv::Point3f> list_points3d; // container for the model 3D coordinates found in the scene
      std::vector<cv::Point2f> list_points2d; // container for the model 2D coordinates found in the scene

      bool availableBaseStations[MAX_BASE_AMOUNT];

      std::vector<double> azimuthHistory[MAX_BASE_AMOUNT][MAX_SENSOR_CNT];
      std::vector<double> elevatiHistory[MAX_BASE_AMOUNT][MAX_SENSOR_CNT];

    private:
      double params_Lighthouse[4];
      uint16_t BaseStationsEventCount[MAX_BASE_AMOUNT];
      int goodCount;
      bool idIsTaken[MAX_SENSOR_CNT][MAX_BASE_AMOUNT];

      std::vector<double> azimuth_buffer[MAX_BASE_AMOUNT][MAX_SENSOR_CNT];
      std::vector<double> elevation_buffer[MAX_BASE_AMOUNT][MAX_SENSOR_CNT];

      bool customFilter(int id_,double *azimuth_, double *elevation_, int channel_);

      //filter *f;
      //the additional 2 is needed, because of azimuth&elevation
      filter *filterClass[MAX_BASE_AMOUNT][2];

};

#endif
