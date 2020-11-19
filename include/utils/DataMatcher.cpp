#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>

#include <opencv2/core/types.hpp>

#include "DataMatcher.hpp"

#include "Utils.hpp"

DataMatcher::DataMatcher(double * params_Lighthouse_):\
                        BaseStationsEventCount{0},\
                        availableBaseStations{false}
{
    for (size_t i = 0; i < 4; i++) {
      params_Lighthouse[i] = *params_Lighthouse_;
      params_Lighthouse_++;
    }

}

//if we get a certain amount of data from a base Station
//it will be added to the list of known Base Stations
void DataMatcher::registNewBaseStation(const std::vector<int> channel){

  for (size_t i = 0; i < channel.size(); i++) {
    if(channel[i] < MAX_BASE_AMOUNT){
      //if we get enough  base events, the base will be added to the available list
      if(BaseStationsEventCount[channel[i]] < MIN_POSITIV_BASE_EVENTS){
          BaseStationsEventCount[channel[i]]++;
      }else{
        availableBaseStations[channel[i]] = true;
      }
    }
  }



  //deregister base if it doesn't ocure in a while
  for (size_t j = 0; j < channel.size(); j++) {
    if(channel[j] < MAX_BASE_AMOUNT){
    //check if base was already registerd
      if(availableBaseStations[channel[j]]){
        //std::cout<<"\nBase Event Counter: " << BaseStationsEventCount[channel[j]];
        if(BaseStationsEventCount[channel[j]] > 10){
            BaseStationsEventCount[channel[j]]--;
        }else{
            //if the amount of base events false under 10 .. it will be degristerd
            availableBaseStations[channel[j]] = false;
        }

      }
    }
  }

}


//overloaded function to also allow double as input
bool DataMatcher::matchData(const std::vector<std::vector<double>> inVec,\
                            const std::vector<int> id,\
                            const std::vector<float> azimuth,\
                            const std::vector<float> elevation,\
                            const std::vector<int> channel\
                          ){

  std::vector<std::vector<float>> vec;
  vec.reserve(inVec.size());
  for (auto&& v : inVec) vec.emplace_back(std::begin(v), std::end(v));

  return (matchData(vec,id,azimuth,elevation,channel));
}

//match 2d with 3d data
bool DataMatcher::matchData(const std::vector<std::vector<float>> inVec,\
                            const std::vector<int> id,\
                            const std::vector<float> azimuth,\
                            const std::vector<float> elevation,\
                            const std::vector<int> channel\
                          ){

  //std::vector<std::vector<double>> * vbuf = &sensorData_buffer;
  //std::vector<std::vector<double>> * v2d = &sensorData_solver_2d;
  //std::vector<std::vector<double>> * v4d = &sensorData_solver_3d;
  std::cout << "\nMatch Data: size=" << id.size();

  bool idIsTaken[64] = {0};
  uint8_t goodCount = 0;

  for(size_t i = 0; i < id.size(); i++){
    //std::cout << "\nMatch Data: size=" << id.size() << "/i: " << i;

    //check if base station id is one of the registered ones
    if(availableBaseStations[channel[i]]){
      //TODO: this should be some kind of kalman filter setup ...
      if(idIsTaken[id[i]]){
      //if(false){
          std::cout<<"\nID: " << id[i] << " was already taken";
      }else{
          //std::cout<<"\nID: " << id[i] << " ac: " << azimuth[i];
          //TODO: DIRTY WORKAROUND FOR THE TEST DATA
          //NEEDS Prober filtering
          if(elevation[i] < 20.0){
              //sensorData_solver_2d.push_back(std::vector<float>(0,0));
              //sensorData_solver_3d.push_back(std::vector<float>(0,0));

              //translate azimuth and elevation to a 2D plane
              std::vector<double> Vec2D = azimuthTo2D(azimuth[i], elevation[i], &params_Lighthouse[0]);
              //std::vector<float> buf2d(Vec2D.begin(),Vec2D.end());

              //load the 2d and 3d to a vector
              cv::Point2f buf2d;
              cv::Point3f buf3d;

              buf2d.x = (float)Vec2D[0];
              buf2d.y = (float)Vec2D[1];
              buf3d.x = (float)inVec[id[i]][0];
              buf3d.y = (float)inVec[id[i]][1];
              buf3d.z = (float)inVec[id[i]][2];

              list_points2d.push_back(buf2d);
              list_points3d.push_back(buf3d);

              //sensorData_solver_2d[goodCount].insert(sensorData_solver_2d[goodCount].begin(), buf2d.begin(), buf2d.end());
              //sensorData_solver_3d[goodCount].insert(sensorData_solver_3d[goodCount].begin(), inVec[id[i]].begin(), inVec[id[i]].end());

              //list_points2d.push_back(buf2d);
              //list_points3d.push_back(buf3d);

              idIsTaken[id[i]] = true;

              goodCount++;
        }
      }
    }else{
      std::cout<<"\nBaseStation:"<<channel[i]<<" was not registerd yet";
    }





    if(goodCount > 20){
      break;
    }
  }

  return false;

}
