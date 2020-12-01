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
#include <opencv2/calib3d.hpp>

DataMatcher::DataMatcher(double * params_Lighthouse_, int maxSensorAvailable_, tf::Vector3 p_):\
                        BaseStationsEventCount{0},\
                        availableBaseStations{false},\
                        dataReady{false}
{

    for (size_t i = 0; i < 4; i++) {
      params_Lighthouse[i] = *params_Lighthouse_;
      params_Lighthouse_++;
    }

    //init Kalman Filter
    int nStates = 18;            // the number of states
    int nMeasurements = 6;       // the number of measured states
    int nInputs = 0;             // the number of action control
    double dt = 0.125;           // time between measurements (1/FPS)
    initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function

    cv::Mat m_(nMeasurements, 1, CV_64FC1);
    measurements = m_;
    measurements.setTo(cv::Scalar(0));


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

    pubHandl_correctBase = nh->advertise<roboy_middleware_msgs::LighthousePoseCorrection>("/roboy/middleware/DarkRoom/lhcorrect", 1);
    tf::Transform tf;

    float x = 0.7;
    float y = -1.0;
    float z = 0.0;

    lhmsg.id = 1;
    lhmsg.type = 1; //relativ==0 ; absolut ==1

    tf::Vector3 position(p_);
    tf::Quaternion orientation;//(x,quat_y,quat_z,quat_w);
    orientation.setRPY( 0, 0, 0 );

    tf.setOrigin(position);
    tf.setRotation(orientation);

    tf::transformTFToMsg(tf, lhmsg.tf);
    pubHandl_correctBase.publish(lhmsg);


}


void DataMatcher::initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt){
  KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
                 /* DYNAMIC MODEL */
  //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
  // position
  KF.transitionMatrix.at<double>(0,3) = dt;
  KF.transitionMatrix.at<double>(1,4) = dt;
  KF.transitionMatrix.at<double>(2,5) = dt;
  KF.transitionMatrix.at<double>(3,6) = dt;
  KF.transitionMatrix.at<double>(4,7) = dt;
  KF.transitionMatrix.at<double>(5,8) = dt;
  KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);
  // orientation
  KF.transitionMatrix.at<double>(9,12) = dt;
  KF.transitionMatrix.at<double>(10,13) = dt;
  KF.transitionMatrix.at<double>(11,14) = dt;
  KF.transitionMatrix.at<double>(12,15) = dt;
  KF.transitionMatrix.at<double>(13,16) = dt;
  KF.transitionMatrix.at<double>(14,17) = dt;
  KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);
       /* MEASUREMENT MODEL */
  //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
  KF.measurementMatrix.at<double>(0,0) = 1;  // x
  KF.measurementMatrix.at<double>(1,1) = 1;  // y
  KF.measurementMatrix.at<double>(2,2) = 1;  // z
  KF.measurementMatrix.at<double>(3,9) = 1;  // roll
  KF.measurementMatrix.at<double>(4,10) = 1; // pitch
  KF.measurementMatrix.at<double>(5,11) = 1; // yaw
}


void DataMatcher::fillMeasurements( cv::Mat &measurements,
                   const cv::Mat &translation_measured, const cv::Mat &rotation_measured)
{
    // Convert rotation matrix to euler angles
    cv::Mat measured_eulers(3, 1, CV_64F);
    measured_eulers = rot2euler(rotation_measured);
    // Set measurement to predict
    measurements.at<double>(0) = translation_measured.at<double>(0); // x
    measurements.at<double>(1) = translation_measured.at<double>(1); // y
    measurements.at<double>(2) = translation_measured.at<double>(2); // z
    measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
    measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
    measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}

void DataMatcher::updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement, cv::Mat &translation_estimated, cv::Mat &rotation_estimated )
{
    // First predict, to update the internal statePre variable
    cv::Mat prediction = KF.predict();

    // The "correct" phase that is going to use the predicted value and our measurement
    cv::Mat estimated = KF.correct(measurement);

    // Estimated translation
    translation_estimated.at<double>(0) = estimated.at<double>(0);
    translation_estimated.at<double>(1) = estimated.at<double>(1);
    translation_estimated.at<double>(2) = estimated.at<double>(2);

    // Estimated euler angles
    cv::Mat eulers_estimated(3, 1, CV_64F);
    eulers_estimated.at<double>(0) = estimated.at<double>(9);
    eulers_estimated.at<double>(1) = estimated.at<double>(10);
    eulers_estimated.at<double>(2) = estimated.at<double>(11);

    // Convert estimated quaternion to rotation matrix
    rotation_estimated = euler2rot(eulers_estimated);
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
  const int histMatchSize = 1;
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

}

std::vector<int> DataMatcher::listChannel(){
  std::vector<int> ch;

  for (uint8_t i = 0; i < MAX_BASE_AMOUNT; i++) {
    if(availableBaseStations[i]==true){
      ch.push_back(i);
    }
  }
  return ch;
}

bool DataMatcher::gatherDataForCalib(const std::vector<std::vector<float>> inVec, std::vector<rawRayData> *ray){
  std::vector<int> ch = listChannel();

  if(ch.size() < 2)
    return false;

  for(std::vector<rawRayData>::iterator vIt = ray->begin(); vIt != ray->end(); ++vIt){
    std::vector<double> Vec2D = azimuthTo2D(vIt->azimuth, vIt->elevation, &params_Lighthouse[0]);

    cv::Point2f buf2d;
    cv::Point3f buf3d;

    buf2d.x = (float)Vec2D[0];
    buf2d.y = (float)Vec2D[1];

    pushData_idHistory[vIt->ch].push_back(vIt->id);
    pushData_historyPoint[vIt->ch].push_back(buf2d);
    buf3d.x = (float)inVec[vIt->id][0];
    buf3d.y = (float)inVec[vIt->id][1];
    buf3d.z = (float)inVec[vIt->id][2];


    //make it a ring buffer
    if(pushData_historyPoint[vIt->ch].size() >= 10){
      std::vector<cv::Point2f> data_temp(pushData_historyPoint[vIt->ch]);
      pushData_historyPoint[vIt->ch].clear();
      pushData_historyPoint[vIt->ch].insert(pushData_historyPoint[vIt->ch].begin(),data_temp.begin()+1,data_temp.end());

      std::vector<int> id_temp(pushData_idHistory[vIt->ch]);
      pushData_idHistory[vIt->ch].clear();
      pushData_idHistory[vIt->ch].insert(pushData_idHistory[vIt->ch].begin(),id_temp.begin()+1,id_temp.end());
    }


    //if()
    //list_points2d[vIt->ch].push_back(buf2d);
    //list_points3d[vIt->ch].push_back(buf3d);

  }


  std::vector<cv::Point2f> *sDat = &pushData_historyPoint[ch[0]];
  std::vector<cv::Point2f> *lDat = &pushData_historyPoint[ch[1]];

  if(sDat->size() < 9)
    return false;
  if(lDat->size() < 9)
    return false;




  //limit to 2 Lighthouse Bases
  std::vector<int> *sId = &pushData_idHistory[ch[0]];
  std::vector<int> *lId = &pushData_idHistory[ch[1]];

  for (size_t i = 0; i < sId->size(); i++) {
    for (size_t j = 0; j < lId->size(); j++) {
      if(sId->at(i) == lId->at(j)){
        dataImg1_2D.push_back(sDat->at(i));
        dataImg2_2D.push_back(lDat->at(j));
        sId->clear();
        lId->clear();
        sDat->clear();
        lDat->clear();

        //i = sId->size();
        //break;
      }
    }
  }

  std::cout << "\nmatched data : " << dataImg2_2D.size();
  if (dataImg2_2D.size() >= 100){
    return true;
  }

  return false;


}
void DataMatcher::resetCalibration(){
  for (size_t i = 0; i < MAX_BASE_AMOUNT; i++) {
    pushData_idHistory[i].clear();
    pushData_historyPoint[i].clear();
  }

}

void DataMatcher::calcCalibrationMatrix(){

  cv::Mat essentialMatrix,fundamental_matrix;
  cv::Mat rotMatrix, transMatrix, mask;
  //cv::Mat K = cv::Mat::eye(3,3,CV_64F);
  cv::Mat K = cv::Mat::zeros(3,3,CV_64FC1);

  K.at<double>(0, 0) = params_Lighthouse[0];       //      [ fx   0  cx ]
  K.at<double>(1, 1) = params_Lighthouse[1];       //      [  0  fy  cy ]
  K.at<double>(0, 2) = params_Lighthouse[2];       //      [  0   0   1 ]
  K.at<double>(1, 2) = params_Lighthouse[3];
  K.at<double>(2, 2) = 1;


  std::cout << "\n\n============\nDifferent Approach\nLMEDS\nf=0.1\noutl=1.0\n============\n";
  essentialMatrix = cv::findEssentialMat(dataImg1_2D, dataImg1_2D, 0.1,cv::Point2d(params_Lighthouse[2],params_Lighthouse[3]),  cv::LMEDS, 0.999, 4.0, mask);
  std::cout << "\nessential: " << essentialMatrix << std::flush;

  cv::recoverPose(essentialMatrix, dataImg1_2D, dataImg1_2D, K, rotMatrix, transMatrix, mask);
  std::cout << "\nTransl: " << transMatrix  << std::flush;

  std::cout << "\n\n============\nDifferent Approach\nRANSAC\nK matrix\n============\n";


  essentialMatrix = cv::findEssentialMat(dataImg1_2D, dataImg2_2D, K, cv::RANSAC, 0.999, 4.0, mask);
  std::cout << "\nessential: " << essentialMatrix << std::flush;

  cv::recoverPose(essentialMatrix, dataImg1_2D, dataImg2_2D, K, rotMatrix, transMatrix, mask);
  std::cout << "\nTransl: " << transMatrix  << std::flush;


  std::cout << "\n\n============\nDifferent Approach\nRANSAC\nf=1.0\noutl=1.0\n============\n";
  essentialMatrix = cv::findEssentialMat(dataImg1_2D, dataImg2_2D, 1.0,cv::Point2d(params_Lighthouse[2],params_Lighthouse[3]),  cv::RANSAC, 0.999, 1.0, mask);
  std::cout << "\nessential: " << essentialMatrix << std::flush;

  cv::recoverPose(essentialMatrix, dataImg1_2D, dataImg2_2D, K, rotMatrix, transMatrix, mask);
  std::cout << "\nTransl: " << transMatrix  << std::flush;
  //std::cout << "\nRotation: " << rotMatrix  << std::flush;

  std::cout << "\n\n============\nDifferent Approach\nRANSAC\nf=0.1\noutl=1.0\n============\n";
  essentialMatrix = cv::findEssentialMat(dataImg1_2D, dataImg2_2D, 0.1,cv::Point2d(params_Lighthouse[2],params_Lighthouse[3]),  cv::RANSAC, 0.999, 1.0, mask);
  std::cout << "\nessential: " << essentialMatrix << std::flush;

  cv::recoverPose(essentialMatrix, dataImg1_2D, dataImg2_2D, K, rotMatrix, transMatrix, mask);
  std::cout << "\nTransl: " << transMatrix  << std::flush;
  //std::cout << "\nRotation: " << rotMatrix  << std::flush;

  std::cout << "\n\n============\nDifferent Approach\nRANSAC\nf=0.1\noutl=1.0\n============\n";
  essentialMatrix = cv::findEssentialMat(dataImg1_2D, dataImg2_2D, 1.0,cv::Point2d(0,0),  cv::RANSAC, 0.999, 1.0, mask);
  std::cout << "\nessential: " << essentialMatrix << std::flush;

  cv::recoverPose(essentialMatrix, dataImg1_2D, dataImg2_2D, K, rotMatrix, transMatrix, mask);
  std::cout << "\nTransl: " << transMatrix  << std::flush;
  //std::cout << "\nRotation: " << rotMatrix  << std::flush;


  double retRol, retPitch, retYaw;
  double A[3][3]= {
    {rotMatrix.at<double>(0,0),rotMatrix.at<double>(0,1),rotMatrix.at<double>(0,2)},
    {rotMatrix.at<double>(1,0),rotMatrix.at<double>(1,1),rotMatrix.at<double>(1,2)},
    {rotMatrix.at<double>(2,0),rotMatrix.at<double>(2,1),rotMatrix.at<double>(2,2)}
  };


}

void DataMatcher::resetProjection(int ch_){
  //clear old filter
  //for (size_t i = 0; i < MAX_BASE_AMOUNT; i++){
    dataReady[ch_] = false;
    list_points2d[ch_].clear();
    list_points3d[ch_].clear();

    delete(filterClass_perCh[ch_][0]);
    delete(filterClass_perCh[ch_][1]);
    filterClass_perCh[ch_][0] = new filter(FILTER_HISTORY_LEN_CH);
    filterClass_perCh[ch_][1] = new filter(FILTER_HISTORY_LEN_CH);

    for (size_t k = 0; k < maxSensorAvailable; k++) {
      for (size_t j = 0; j < 2; j++) {
        //clear old filter
        delete(filterClass[ch_][k][j]);
        //init them again
        filterClass[ch_][k][j] = new filter(FILTER_HISTORY_LEN);
      }
      //reset all taken flags from all sensors;
      idIsTaken[ch_][k] = false;
      idIsTaken_cnt[ch_][k] = 0;
    }
  //}
  goodCount = 0;
  //pnpCounter = 0;
}

bool DataMatcher::findProjectedPosition(int ch_){

  std::vector<int> ch_list = listChannel();
  int minInliersKalman = 6;

  PnPProblem pnp_(params_Lighthouse);
  // PnP parameters
  //int pnpMethod = cv::SOLVEPNP_ITERATIVE;
  int pnpMethod = cv::SOLVEPNP_EPNP;
  int iterationsCount = 500;      // number of Ransac iterations.
  float reprojectionError = 3.0;  // maximum allowed distance to consider it an inlier.
  double confidence = 0.99;       // ransac successful confidence.
  cv::Mat inliers_idx;

  cv::Mat essentialMatrix,fundamental_matrix;
  cv::Mat rotMatrix, transMatrix, mask;
  cv::Mat translation_measured(3, 1, CV_64F);
  cv::Mat rotation_measured(3, 3, CV_64F);
  cv::Mat K = cv::Mat::zeros(3,3,CV_64FC1);


  //if(list_points3d[ch_].size()-pnpCounter < 8)
  //    return;

  //pnpCounter = list_points3d[ch_].size();
  if(list_points3d[ch_].size() < 8)
    return false;


  //==== pos solver ====
  std::vector<cv::Point3f> d3D(list_points3d[ch_]);
  std::vector<cv::Point2f> d2D(list_points2d[ch_]);
  std::cout << "\nRansac solver " << ch_;
  std::cout << "\n3dList size\t " << d3D.size() << "\t" <<  std::flush;
//  std::cout << "\n3dList \t " << d3D << "\t" <<  std::flush;

  std::cout << "\n2dList size\t " << d2D.size() << "\t" <<  std::flush;
//  std::cout << "\n2dList \t " << d2D << "\t" <<  std::flush;

  pnp_.estimatePoseRANSAC( d3D, d2D, pnpMethod, inliers_idx, iterationsCount, reprojectionError, confidence );


  std::cout << "\n\nInliers Index " << inliers_idx.rows << std::flush;

  //if( inliers_idx.rows >= minInliersKalman ){
    std::cout << "\nEnough Inliers";

    // Get the translation

    translation_measured = pnp_.get_t_matrix();
    std::cout << "\nTransl\n" << translation_measured;
    //std::cout << "\nX\n" << translation_measured.at<double>(0);
    //std::cout << "\nX\n" << translation_measured.at<float>(0);
    // Get the rotation

    rotation_measured = pnp_.get_R_matrix();
    std::cout << "\nRotation\n" << rotation_measured;

    //fillMeasurements(measurements,translation_measured,rotation_measured);
  //}

  // update the Kalman filter with good measurements, otherwise with previous valid measurements
  cv::Mat translation_estimated(3, 1, CV_64FC1);
  cv::Mat rotation_estimated(3, 3, CV_64FC1);
  updateKalmanFilter( KF, measurements, translation_estimated, rotation_estimated);

  // -- Step 6: Set estimated projection matrix
  //pnp_detection_est.set_P_matrix(rotation_estimated, translation_estimated);

  std::cout << "\n===\nEST.\n====\nTransl\n" << translation_estimated;
  // Get the rotation
  cv::Mat rot_es(3, 3, CV_64F);
  rot_es = pnp_.get_R_matrix();
  std::cout << "\nRotation\n" << rot_es;

  resetProjection(ch_);

  cv::Mat euler(rot2euler(rotation_measured));

  float Yrot = (float)euler.at<double>(0);
  float Zrot = (float)euler.at<double>(1);
  float Xrot = (float)euler.at<double>(2);
  /*

  double A[3][3]= {
    {rotation_measured.at<double>(0,0),rotation_measured.at<double>(0,1),rotation_measured.at<double>(0,2)},
    {rotation_measured.at<double>(1,0),rotation_measured.at<double>(1,1),rotation_measured.at<double>(1,2)},
    {rotation_measured.at<double>(2,0),rotation_measured.at<double>(2,1),rotation_measured.at<double>(2,2)}
  };
  //converRotMatrixToEuler(A, retRol, retPitch, retYaw);
  std::cout << "\n======\nRotationEULER:\tRol: " << retRol << "\tPitch: " << retPitch << "\tYaw: "  << retYaw  << std::flush;

  */
  std::cout << "\n======\nRotationEULER:\tYrot: " << Yrot << "\tYitch: " << Zrot << "\tXaw: "  << Xrot  << std::flush;


  tf::Transform tf;
  float lh_x = (float)translation_measured.at<double>(0)/1000;
  float lh_y = (float)translation_measured.at<double>(2)/1000;
  float lh_z = (float)translation_measured.at<double>(1)/1000;
  lh_y = -1 * lh_y;

  std::cout << "\nTranslation:\tX: " << lh_x << "\tY: " << lh_y << "\tZ: "  << lh_z  << std::flush;

  if(ch_list[0] == ch_){
    lhmsg.id = 1;
  }else{
    lhmsg.id = 0;
  }

  //Now we can transpose the center in respekt to World0
  lhmsg.type = 1; //relativ==0 ; absolut ==1
  //due to different coordinate system we need to map x,y,z proberly

  //send Translation and Rotation via ros to rviz
  ///double foo = transMatrix(0);
  tf::Vector3 position;
  tf::Quaternion orientation;//(x,quat_y,quat_z,quat_w);

  position.setValue(lh_x,lh_y,lh_z);
  orientation.setRPY(0.0, 0.0, 0.0);//retPitch
  //orientation.setEulerZYX(retRol, retPitch, retYaw);
  //orientation.setRPY(0.0,0.0,retYaw);

  std::cout << "\nSet transform data"  << std::flush;
  tf.setOrigin(position);
  tf.setRotation(orientation);

  std::cout << "\nTransform Msg"  << std::flush;
  tf::transformTFToMsg(tf, lhmsg.tf);

  pubHandl_correctBase.publish(lhmsg);
  ros::spinOnce();

  return true;


}
