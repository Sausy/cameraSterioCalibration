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

    //init filters .. there should be one for each lighthouse base
    for (size_t i = 0; i < MAX_BASE_AMOUNT; i++){
      for (size_t j = 0; j < 2; j++) {
        filterClass[i][j] = new filter(FILTER_HISTORY_LEN);
      }
    }


    goodCount = 0;

}


//resets all data proberly
//to start a new run
//INFO: doesn't reset known lighthouse bases
void DataMatcher::reset_matchData(){
  list_points2d.clear();
  list_points3d.clear();

  //clear old filter
  for (size_t i = 0; i < MAX_BASE_AMOUNT; i++){
    for (size_t j = 0; j < 2; j++) {
      //clear old filter
      delete(filterClass[i][j]);

      //init them again
      filterClass[i][j] = new filter(FILTER_HISTORY_LEN);


    }

    for (size_t j = 0; j < MAX_SENSOR_CNT; j++) {
      //reset all taken flags from all sensors;
      idIsTaken[j][i] = false;
    }
  }

  goodCount = 0;
}


//if we get a certain amount of data from a base Station
//it will be added to the list of known Base Stations
void DataMatcher::registNewBaseStation(const std::vector<int> channel){

  for (size_t i = 0; i < channel.size(); i++) {
    //check if channel data is valid,
    //it shall not exeed 16base stations
    if(channel[i] < MAX_BASE_AMOUNT){
      //if we get enough  base events, the base will be added to the available list
      if(BaseStationsEventCount[channel[i]] < MIN_POSITIV_BASE_EVENTS){
          BaseStationsEventCount[channel[i]]++;
      }else{
        //if the base wasn't already registered we output an info
        if(availableBaseStations[channel[i]] == false){
          std::cout<<"\n[FOUND BASE:] " << channel[i];
        }
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

  for(size_t i = 0; i < id.size(); i++){
    double az_ = azimuth[i];
    double ele_ = elevation[i];
    int ch_ = channel[i];

    //check if base station id is one of the registered ones
    if(availableBaseStations[ch_]){
      //TODO: this should be some kind of kalman filter setup ...
      //std::cout<<"\nGOOD COUnT: " << goodCount;

      if(customFilter(id[i], &az_, &ele_, ch_)){
        //push azimuth and elevation onto history buffer
        if(azimuthHistory[ch_][id[i]].size() == MAX_DATA_HISTORY){
          azimuthHistory[ch_][id[i]].push_back(az_);
          elevatiHistory[ch_][id[i]].push_back(ele_);
        }else if(azimuthHistory[ch_][id[i]].size() > MAX_DATA_HISTORY){
          int inc_ = azimuthHistory[ch_][id[i]].size() - MAX_DATA_HISTORY - 1;

          azimuthHistory[ch_][id[i]].erase(azimuthHistory[ch_][id[i]].begin()+inc_);
          elevatiHistory[ch_][id[i]].erase(elevatiHistory[ch_][id[i]].begin()+inc_);
        }

        //only use one sample per sensor and lighhouse base
        if(idIsTaken[id[i]][ch_]){
            ;
        }else{

            std::cout<<"\n\n[ID:" << id[i] << "/" << ch_;
            std::cout<<"]\taz: " << az_ << "\tele: " << ele_;
            std::cout<<"\tMeanAZ: " << filterClass[ch_][0]->mean * 180/M_PI;
            std::cout<<"\tMeanEL: " << filterClass[ch_][1]->mean * 180/M_PI;

            //after we found a valid azimuth and elevation value
            //, we register the coresponding id
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



    if(goodCount >= 9){
      goodCount = 9;
      return true;
      //break;
    }
  }

  return false;

}




bool DataMatcher::customFilter(int id_, double * azimuth_, double * elevation_, int channel_){

  //assign the matching lighthouse base and
  //the azimuth 0 / elevation 1 to a easy to read pointer
  filter *faz = filterClass[channel_][0];
  filter *fele = filterClass[channel_][1];

  //std::vector<double> *azBuf = &azimuth_buffer[channel_][id_];
  //std::vector<double> *elBuf = &elevation_buffer[channel_][id_];
  bool dataIsRdy = false;
  dataIsRdy = faz->stdDeviationFilter(*azimuth_);
  dataIsRdy = dataIsRdy & fele->stdDeviationFilter(*elevation_);
  if(dataIsRdy){
    //for both values there are upper and lower boundaries
    bool upperBound_az = (*azimuth_ <= (faz->mean + faz->variance));
    bool lowerBound_az = (*azimuth_ >= (faz->mean - faz->variance));
    bool upperBound_el = (*elevation_ <= (fele->mean + fele->variance));
    bool lowerBound_el = (*elevation_ >= (fele->mean - fele->variance));

    //if both values are with a variance range of 1*var
    //we shall consider the as good
    if(upperBound_az && lowerBound_az && upperBound_el && lowerBound_el){
      //std::cout<<"\n[ID:" << id_ << "/" << channel_ << "]";
      //std::cout<<"\tMean:" << faz->mean * 180.0/M_PI << "\tVar:" << faz->variance * 180.0/M_PI;
      //std::cout<<"\tMean:" << fele->mean * 180.0/M_PI << "\tVar:" << fele->variance * 180.0/M_PI;
      return true;
    }
  }
  return false;



  /*
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

  */
  return true;

}
