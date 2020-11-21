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
    goodCount = 0;

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
  //std::cout << "\nMatch Data: size=" << id.size();


  for(size_t i = 0; i < id.size(); i++){
    double az_ = azimuth[i];
    double ele_ = elevation[i];
    int ch_ = channel[i];
    //std::cout << "\nMatch Data: size=" << id.size() << "/i: " << i;

    //check if base station id is one of the registered ones
    if(availableBaseStations[ch_]){
      //TODO: this should be some kind of kalman filter setup ...
      //std::cout<<"\nGOOD COUnT: " << goodCount;
      if(idIsTaken[id[i]][ch_]){
      //if(false){
          //std::cout<<"\nID: " << id[i] << " was already taken";
          ;//
      }else{
          if(filter(id[i], &az_, &ele_, ch_)){
            //after we found an average, we register the coresponding id
            idIsTaken[id[i]][ch_] = true;

            //translate azimuth and elevation to a 2D plane
            std::vector<double> Vec2D = azimuthTo2D(az_, ele_, &params_Lighthouse[0]);
            //std::vector<float> buf2d(Vec2D.begin(),Vec2D.end());

            //load the 2d and 3d to a vector
            cv::Point2f buf2d;
            cv::Point3f buf3d;

            //int id_ = id.push_back();

            buf2d.x = (float)Vec2D[0];
            buf2d.y = (float)Vec2D[1];
            buf3d.x = (float)inVec[id[i]][0];
            buf3d.y = (float)inVec[id[i]][1];
            buf3d.z = (float)inVec[id[i]][2];

            list_points2d.push_back(buf2d);
            list_points3d.push_back(buf3d);

            goodCount++;
          }
      }
    }



    if(goodCount >= 7){
      goodCount = 7;
      return true;
      //break;
    }
  }

  return false;

}




bool DataMatcher::filter(int id_, double * azimuth_, double * elevation_, int channel_){

  std::vector<double> *azBuf = &azimuth_buffer[channel_][id_];
  std::vector<double> *elBuf = &elevation_buffer[channel_][id_];

  //std::cout << "\n[" << id_ << "/" << channel_  << "]filter:" << azBuf->size();


  azBuf->push_back(*azimuth_);
  elBuf->push_back(*elevation_);

  if(azBuf->size() == FILTER_HISTORY_LEN){
    std::cout << "\n" << id_ << "/"<< channel_ << "\t";
    for (size_t i = 0; i < FILTER_HISTORY_LEN; i++) {
      std::cout << (*azBuf)[i] << " | " << (*elBuf)[i];
    }

  }else if(azBuf->size() > FILTER_HISTORY_LEN){
    int inc_ = azBuf->size() - FILTER_HISTORY_LEN - 1;
    azBuf->erase(azBuf->begin()+inc_);
    elBuf->erase(elBuf->begin()+inc_);
  }else{
    return false;
  }

  std::cout << "\n[" << id_ << "/"<< channel_ << "]\tNow lets calc the mean:";

  double meanAzi = calcMean(*azBuf);
  double meanEle = calcMean(*elBuf);

  std::cout << "\n[" << id_ << "/"<< channel_ << "]\tNow lets calc the deviation:";
  double deviAzi = standardDeviation(*azBuf,meanAzi);
  double deviEle = standardDeviation(*elBuf,meanEle);


  //now erase outliers
  for (size_t i = 0; i < azBuf->size(); i++) {
    //erase outliers from azimuth
    if(((meanAzi+deviAzi) < (*azBuf)[i]) || ((meanAzi-deviAzi) > (*azBuf)[i])){
      azBuf->erase(azBuf->begin()+i);
    }
    //erase outliers from azimuth
    if(((meanEle+deviEle) < (*elBuf)[i]) || ((meanEle-deviEle) > (*elBuf)[i])){
      elBuf->erase(elBuf->begin()+i);
    }
  }

  std::cout << "\nSize of remaining vec: " << elBuf->size() << " | " << azBuf->size();

  //calculate the aritmetric mean
  //and return with sucess
  //*azimuth_ = calcMean(*azBuf);
  //*elevation_ = calcMean(*elBuf);
  *azimuth_ = (*azBuf)[0];
  *elevation_ = (*elBuf)[0];


  return true;
}
