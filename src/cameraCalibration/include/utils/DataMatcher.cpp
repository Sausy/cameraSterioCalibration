#include "DataMatcher.hpp"
/*#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>

#include <opencv2/core/types.hpp>

#include "DataMatcher.hpp"

#include "Utils.hpp"

#include <unistd.h>
*/



DataMatcher::DataMatcher(double * params_Lighthouse_, int maxSensorAvailable_):\
                        BaseStationsEventCount{0},\
                        availableBaseStations{false},\
                        dataReady{false}
{
    for (size_t i = 0; i < 4; i++) {
      params_Lighthouse[i] = *params_Lighthouse_;
      params_Lighthouse_++;
    }

    //init filters .. there should be one for each lighthouse base
    for (size_t i = 0; i < MAX_BASE_AMOUNT; i++){
      for (size_t j = 0; j < 2; j++) {
        filterClass_perCh[i][j] = new filter(FILTER_HISTORY_LEN_CH);
        for (size_t k = 0; k < MAX_SENSOR_CNT; k++) {
          filterClass[i][k][j] = new filter(FILTER_HISTORY_LEN);
        }

      }
    }

    returnAmountOfBaseStations = 0;
    goodCount = 0;

    maxSensorAvailable = maxSensorAvailable_;

    wasRepos_cnt = 0;
    for (size_t i = 0; i < 3; i++) {
      wasRepos[i] = false;
    }


    //============ ROS INIT============

    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "lighthouse2ToRos");
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    ROS_INFO("init[STARTED]...lighthouse2Data");


}


//resets all data proberly
//to start a new run
//INFO: doesn't reset known lighthouse bases
void DataMatcher::reset_matchData(){


  //clear old filter
  for (size_t i = 0; i < MAX_BASE_AMOUNT; i++){
    dataReady[i] = false;
    list_points2d[i].clear();
    list_points3d[i].clear();

    delete(filterClass_perCh[i][0]);
    delete(filterClass_perCh[i][1]);

    filterClass_perCh[i][0] = new filter(FILTER_HISTORY_LEN_CH);
    filterClass_perCh[i][1] = new filter(FILTER_HISTORY_LEN_CH);

    for (size_t k = 0; k < maxSensorAvailable; k++) {

      for (size_t j = 0; j < 2; j++) {
        //clear old filter
        delete(filterClass[i][k][j]);

        //init them again
        filterClass[i][k][j] = new filter(FILTER_HISTORY_LEN);


      }

      //reset all taken flags from all sensors;
      idIsTaken[i][k] = false;
      idIsTaken_cnt[i][k] = 0;
    }
  }

  goodCount = 0;
}


int DataMatcher::registNewBaseStation(const std::vector<rawRayData> ray_){
  std::vector<int> ch_;
  //ch_.reserve(ray.size());

  for (const auto& structElement : ray_){
    ch_.push_back(structElement.ch);
  }

  return(registNewBaseStation(ch_));
}

//if we get a certain amount of data from a base Station
//it will be added to the list of known Base Stations
//returns amount of base stations active
int DataMatcher::registNewBaseStation(const std::vector<int> channel){

  int ch_;

  for (size_t i = 0; i < channel.size(); i++) {
    ch_ = channel[i];
    //check if channel data is valid,
    //it shall not exeed 16base stations
    if(ch_ < MAX_BASE_AMOUNT && ch_ >= 0){
      //if we get enough  base events, the base will be added to the available list
      if(BaseStationsEventCount[ch_] < MIN_POSITIV_BASE_EVENTS){
          BaseStationsEventCount[ch_]++;
      }else{
        //if the base wasn't already registered we output an info
        if(availableBaseStations[ch_] == false){
          returnAmountOfBaseStations++;
          std::cout<<"\n[FOUND BASE:] " << ch_;
        }
        availableBaseStations[ch_] = true;

      }
    }
  }



  //deregister base if it doesn't ocure in a while
  for (size_t j = 0; j < channel.size(); j++) {
    ch_ = channel[j];

    if(ch_ < MAX_BASE_AMOUNT && ch_ >= 0){
    //check if base was already registerd
      if(availableBaseStations[ch_]){
        //std::cout<<"\nBase Event Counter: " << BaseStationsEventCount[ch_];
        if(BaseStationsEventCount[ch_] > 10){
            BaseStationsEventCount[ch_]--;
        }else{
            if(returnAmountOfBaseStations  > 0)
              returnAmountOfBaseStations--;
            //if the amount of base events false under 10 .. it will be degristerd
            availableBaseStations[ch_] = false;
        }

      }
    }
  }

  return returnAmountOfBaseStations;

}



//overloaded function to also allow double as input
bool DataMatcher::matchData(const std::vector<std::vector<double>> inVec,\
                            const std::vector<int> id,\
                            const std::vector<float> azimuth,\
                            const std::vector<float> elevation,\
                            const std::vector<int> channel\
                          ){

  if(channel.size() <= 0)
    return false;

  std::vector<std::vector<float>> vec;
  vec.reserve(inVec.size());
  for (auto&& v : inVec) vec.emplace_back(std::begin(v), std::end(v));

  return (matchData(vec,id,azimuth,elevation,channel));
}

bool DataMatcher::matchData(const std::vector<std::vector<double>> inVec, std::vector<rawRayData> *ray_){

  std::vector<int> id_;
  std::vector<float> az_;
  std::vector<float> el_;
  std::vector<int> ch_;

  for(std::vector<rawRayData>::iterator vIt = ray_->begin(); vIt != ray_->end(); ++vIt){
    ch_.push_back(vIt->ch);
    id_.push_back(vIt->id);
    az_.push_back(vIt->azimuth);
    el_.push_back(vIt->elevation);
  }

  return (matchData(inVec, id_, az_, el_, ch_));
}


//match 2d with 3d data
bool DataMatcher::matchData(const std::vector<std::vector<float>> inVec,\
                            const std::vector<int> id,\
                            const std::vector<float> azimuth,\
                            const std::vector<float> elevation,\
                            const std::vector<int> channel\
                          ){

  //load the 2d and 3d to a vector
  cv::Point2f buf2d;
  cv::Point3f buf3d;

  for(size_t i = 0; i < id.size(); i++){
    double az_ = azimuth[i];
    double ele_ = elevation[i];
    unsigned int ch_ = channel[i];

    if(ch_ >= MAX_BASE_AMOUNT){
      return false;
    }
    if(id[i] >= maxSensorAvailable){
      return false;
    }


    std::vector<double> *azHist = &azimuthHistory[ch_][id[i]];
    std::vector<double> *elHist = &elevatiHistory[ch_][id[i]];

    //check if base station id is one of the registered ones
    if(availableBaseStations[ch_]){
      //TODO: this should be some kind of kalman filter setup ...
      //std::cout<<"\nGOOD COUnT: " << goodCount;

      if(customFilter(id[i], &az_, &ele_, ch_)){
      //if(true){
        //only use one sample per sensor and lighhouse base
        if(idIsTaken[ch_][id[i]]){
          ;//std::cout << "\nnot taken " << std::flush;
        }else{
            std::cout<<"\n\n[ID:" << id[i] << "/" << ch_;
            std::cout<<"]\taz: " << az_ << "\tele: " << ele_;
            std::cout<<"\tMeanAZ: " << filterClass[ch_][id[i]][0]->mean * 180/M_PI;
            std::cout<<"\tMeanEL: " << filterClass[ch_][id[i]][1]->mean * 180/M_PI;
            std::cout<<"\tmaxSensor" << maxSensorAvailable;
            //after we found a valid azimuth and elevation value
            //, we register the coresponding id
            idIsTaken[ch_][id[i]] = true;

            //translate azimuth and elevation to a 2D plane
            std::vector<double> Vec2D = azimuthTo2D(az_, ele_, &params_Lighthouse[0]);

            //int id_ = id.push_back();
            buf2d.x = (float)Vec2D[0];
            buf2d.y = (float)Vec2D[1];

            buf3d.x = (float)inVec[id[i]][0];
            buf3d.y = (float)inVec[id[i]][1];
            buf3d.z = (float)inVec[id[i]][2];


            //list_points3d_dump.push_back(buf3d);
            //list_points2d_dump.push_back(buf2d);

            list_points2d[ch_].push_back(buf2d);
            list_points3d[ch_].push_back(buf3d);
            list_id[ch_].push_back(id[i]);

            goodCount++;


        }

        /*
        //push azimuth and elevation onto history buffer
        if(azHist->size() < MAX_DATA_HISTORY){
          azHist->push_back(az_);
          elHist->push_back(ele_);
        }else if(azHist->size() > MAX_DATA_HISTORY){
          int inc_ = azHist->size() - MAX_DATA_HISTORY - 1;

          azHist->erase(azHist->begin()+inc_);
          elHist->erase(elHist->begin()+inc_);
        }else{
          //if we have enough history data from each sensor we shall continue

          //only use one sample per sensor and lighhouse base
          if(idIsTaken[ch_][id[i]]){
              ;
          }else{
              std::cout<<"\n\n[ID:" << id[i] << "/" << ch_;
              std::cout<<"]\taz: " << az_ << "\tele: " << ele_;
              std::cout<<"\tMeanAZ: " << filterClass[ch_][0]->mean * 180/M_PI;
              std::cout<<"\tMeanEL: " << filterClass[ch_][1]->mean * 180/M_PI;

              //after we found a valid azimuth and elevation value
              //, we register the coresponding id
              idIsTaken[ch_][id[i]] = true;

              //TODO: this looks super hack ... should be more elegant
              //but first we calculate the mean of the data ... to get better results
              az_ = filterClass[0][0]->calcMean(*azHist); //calcMean doesn't need global or specific variabel
              ele_ = filterClass[0][0]->calcMean(*elHist);

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
        */


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



//DataMatcher::customFilter_indi

void DataMatcher::filterfunc(std::vector<rawRayData> *ray){
  std::vector<rawRayData> retRay;

  //for (size_t i = 0; i < ray->size(); i++) {
  int i = 0;
  for(std::vector<rawRayData>::iterator vIt = ray->begin(); vIt != ray->end(); ++vIt){


    //Filter out obvious errors
    if(vIt->elevation < (-48.0 * M_PI/180.0) || vIt->elevation  > (48.0 * M_PI/180.0) ){
      if(vIt->azimuth < (30.0 * M_PI/180.0) || vIt->azimuth > (160.0 * M_PI/180.0)){
        return;
      }
    }
    int ch_ = vIt->ch;
    int id_ = vIt->id;
    double azTmp = vIt->azimuth;
    double elTmp = vIt->elevation;
    if(customFilter(vIt->id, &azTmp, &elTmp, vIt->ch)){
      rawRayData ret_data;
      ret_data.id = vIt->id;
      ret_data.ch = vIt->ch;
      ret_data.azimuth = azTmp;
      ret_data.elevation = elTmp;
      ret_data.azMean = filterClass[ch_][id_][0]->mean;
      ret_data.azVar = filterClass[ch_][id_][0]->variance;
      ret_data.elMean = filterClass[ch_][id_][1]->mean;
      ret_data.elVar = filterClass[ch_][id_][1]->variance;
      retRay.push_back(ret_data);
    }
    i++;
  }

  ray->clear();
  ray->insert(ray->begin(),retRay.begin(),retRay.end());
}


bool DataMatcher::customFilter(int id_, double * azimuth_, double * elevation_, int channel_){
  if(*elevation_ < (-48.0 * M_PI/180.0) || *elevation_ > (48.0 * M_PI/180.0) ){
    if(*azimuth_ < (34.0 * M_PI/180.0) || *azimuth_ > (160.0 * M_PI/180.0)){
      return false;
    }
  }


  filter *faz_perCh = filterClass_perCh[channel_][0];
  filter *fele_perCh = filterClass_perCh[channel_][1];

  //std::vector<double> *azBuf = &azimuth_buffer[channel_][id_];
  //std::vector<double> *elBuf = &elevation_buffer[channel_][id_];
  bool isBaseRdy = false;
  isBaseRdy = faz_perCh->stdDeviationFilter(*azimuth_);
  isBaseRdy = isBaseRdy & fele_perCh->stdDeviationFilter(*elevation_);

  if(isBaseRdy){
    //bool upperBound_az = (*azimuth_ <= (faz_perCh->mean + faz_perCh->variance));
    //bool lowerBound_az = (*azimuth_ >= (faz_perCh->mean - faz_perCh->variance));
    //bool upperBound_el = (*elevation_ <= (fele_perCh->mean + fele_perCh->variance));
    //bool lowerBound_el = (*elevation_ >= (fele_perCh->mean - fele_perCh->variance));
    bool upperBound_az = (*azimuth_ <= (faz_perCh->mean + 0.3));
    bool lowerBound_az = (*azimuth_ >= (faz_perCh->mean - 0.3));
    bool upperBound_el = (*elevation_ <= (fele_perCh->mean +  0.3));
    bool lowerBound_el = (*elevation_ >= (fele_perCh->mean -  0.3));


    if(!upperBound_az || !lowerBound_az || !upperBound_el || !lowerBound_el){
      return false;
    }
    /*if(upperBound_az && lowerBound_az && upperBound_el && lowerBound_el){
      dataReady[channel_] = true;
      return true;
    }*/
  }else{
    return false;
  }

  //assign the matching lighthouse base and
  //the azimuth 0 / elevation 1 to a easy to read pointer
  filter *faz = filterClass[channel_][id_][0];
  filter *fele = filterClass[channel_][id_][1];

  bool dataIsRdy = false;
  dataIsRdy = faz->stdDeviationFilter(*azimuth_);
  dataIsRdy = dataIsRdy & fele->stdDeviationFilter(*elevation_);

  if(dataIsRdy){
    //for both values there are upper and lower boundaries
    bool upperBound_az = (*azimuth_ <= (faz->mean + 0.2));
    bool lowerBound_az = (*azimuth_ >= (faz->mean - 0.2));
    bool upperBound_el = (*elevation_ <= (fele->mean +  0.2));
    bool lowerBound_el = (*elevation_ >= (fele->mean -  0.2));

    //if both values are with a variance range of 1*var
    //we shall consider the as good
    //if(upperBound_az && lowerBound_az && upperBound_el && lowerBound_el){
    //  dataReady[channel_] = true;
    //  return true;
    //}
    if(!upperBound_az || !lowerBound_az || !upperBound_el || !lowerBound_el){
      return false;
    }
  }else{
    return false;
  }






  //return true;



  dataReady[channel_] = true;
  return true;




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

}

bool DataMatcher::twoCameraMatcher(){
  //todo create vector for this
  const int histMatchSize = 2;
  const int numIterationMax = 9;

  std::vector<int> ch;


  dataImg1_2D.clear();
  dataImg2_2D.clear();

  //find 2 base channels that we want to process
  for (uint8_t i = 0; i < MAX_BASE_AMOUNT; i++) {
    if(dataReady[i]==true){
      //std::cout << "\nfound ch " << i << "/" << maxSensorAvailable << std::flush;
      ch.push_back(i);
    }
  }

  //find matching sensor ID's and push them to dataImgx_2D
  int matchingCounter = 0;
  if(ch.size() >= 2){
    //std::cout << "\n\nDo we have common Points " << ch.size() << std::flush;;

    for (size_t i = 0; i < ch.size()-1; i++) {
      //std::cout << "\ndbg2 " << i;
      //std::cout << "\nsize list " << list_points2d[ch[i]].size();
      for (size_t k = 0; k < list_points2d[ch[i+1]].size(); k++) {
        for (size_t j = 0; j < list_points2d[ch[i]].size(); j++) {
          if(list_id[ch[i]][j] ==  list_id[ch[i+1]][k]){
            matchingCounter ++;
            dataImg1_2D.push_back(list_points2d[ch[i]][j]);
            dataImg2_2D.push_back(list_points2d[ch[i+1]][k]);
          }
        }
      }
    }
  }

  //if we don't have enough matches func shall return false
  if(matchingCounter < histMatchSize)
    return false;

  //if we have enough matching data we shall continue
  //and push the data to a buffer
  for (size_t i = 0; i < dataImg1_2D.size(); i++) {
    dataImg1_2D_buffer.push_back(dataImg1_2D[i]);
    dataImg2_2D_buffer.push_back(dataImg2_2D[i]);
  }

  twoCameraMatcher_itCnt++;
  //we no have enough matches in this run
  //but for prober data we need multiple runs
  if(twoCameraMatcher_itCnt >= numIterationMax ){
    //if we have enough runs and enough data per run
    //we push back all the data to dataImgx_2D and return true
    dataImg1_2D.clear();
    dataImg2_2D.clear();

    for (size_t i = 0; i < dataImg1_2D_buffer.size(); i++) {
      std::cout << "\n[1] " << dataImg1_2D_buffer[i].x << "\t" <<  dataImg1_2D_buffer[i].y;
      std::cout << "\n[2] " << dataImg2_2D_buffer[i].x << "\t" << dataImg2_2D_buffer[i].y;
      dataImg1_2D.push_back(dataImg1_2D_buffer[i]);
      dataImg2_2D.push_back(dataImg2_2D_buffer[i]);
    }

    //we shall now clear the buffer
    dataImg1_2D_buffer.clear();
    dataImg2_2D_buffer.clear();
    twoCameraMatcher_itCnt = 0;

    return true;

  }else{
    //if we don't have enough data
    //print out a info msg on how to proceed
    std::cout << "\n======================================";
    std::cout << "\n\tInfo";
    std::cout << "\n======================================";
    std::cout << "\n please move the tracking object a bit" << std::flush;
    usleep(4*1000000);
    reset_matchData();
    std::cout << "\n now keep it steady again" << std::flush;
    usleep(2*1000000);
  }

  return false;

  /*

  if(wasRepos[wasRepos_cnt] == false){
    if(matchingCounter >= histMatchSize){
      for (size_t i = 0; i < dataImg1_2D.size(); i++) {

        dataImg1_2D_buffer.push_back(dataImg1_2D[i]);
        dataImg2_2D_buffer.push_back(dataImg2_2D[i]);
      }
      iterationCnt++;

      std::cout << "\n======================================";
      std::cout << "\n\tInfo";
      std::cout << "\n======================================";
      std::cout << "\n please move the tracking object a bit" << std::flush;
      usleep(5*1000000);
      reset_matchData();
      std::cout << "\n now keep it steady again" << std::flush;
      usleep(3*1000000);
      std::cout << "\nplease wait ..." << std::flush;
      wasRepos[wasRepos_cnt] = true;
      if(wasRepos_cnt < numIterationMax-1)
        wasRepos_cnt++;
    }
  }else{
    if(matchingCounter >= histMatchSize){
      //push the buffer back onto the datapoints
      for (size_t i = 0; i < dataImg1_2D_buffer.size(); i++) {
        dataImg1_2D.push_back(dataImg1_2D_buffer[i]);
        dataImg2_2D.push_back(dataImg2_2D_buffer[i]);

      }
    }
  }


  if(iterationCnt >= numIterationMax){
    wasRepos[wasRepos_cnt] = false;
    std::cout << "\n found " << dataImg1_2D.size() << std::flush;
    return true;
  }
  */
    /*

    dataImg1_2D[MAX_BASE_AMOUNT];
    std::vector<cv::Point2f> dataImg2_2D[MAX_BASE_AMOUNT];

    int point_count = 100;
    vector<Point2f> points1(point_count);
    vector<Point2f> points2(point_count);
    // initialize the points here ...
    for( int i = 0; i < point_count; i++ )
    {
        points1[i] = ...;
        points2[i] = ...;
    }
    Mat fundamental_matrix =  findFundamentalMat(points1, points2, FM_RANSAC, 3, 0.99);
    */
  //}
  return false;

}
