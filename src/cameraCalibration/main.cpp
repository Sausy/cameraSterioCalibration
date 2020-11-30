// C++
#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include <vector>
//to open a file
#include <fstream>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
/* ROS */
#include <ros/ros.h>
#include <ros/package.h>

#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include "std_msgs/Float32MultiArray.h"

#include <roboy_middleware_msgs/DarkRoomSensorV2.h>
#include <roboy_middleware_msgs/LighthousePoseCorrection.h>

#include <tf/tf.h>


//custom ros interface
#include <triang.hpp>

// PnP Tutorial
#include <Mesh.hpp>
#include <PnPProblem.hpp>
#include <RobustMatcher.hpp>
#include <ModelRegistration.hpp>
#include <Utils_openCV.hpp>

//common Utils
#include <Utils.hpp>
#include <datatype_htcDongle.h>

// test
//#include <testData.hpp>

// Usb driver
#include <interface_htcDongle.h>

// 3D sensor position
#include <3dModel.h>

// to match 2D with 3D data
#include <DataMatcher.hpp>

// add a filter include
#include <filter.hpp>


using namespace cv;
using namespace std;

// Some basic colors
const Scalar red(0, 0, 255);
const Scalar green(0,255,0);
const Scalar blue(255,0,0);
const Scalar yellow(0,255,255);

static bool wasCalibrated = false;

filter *filterClass_[MAX_BASE_AMOUNT][2];
filter *filterClass_perSensor[MAX_BASE_AMOUNT][MAX_SENSORS][2];

void cFilter_init(int history_N){
  for (size_t i = 0; i < MAX_BASE_AMOUNT; i++){
    for (size_t j = 0; j < 2; j++) {
      filterClass_[i][j] = new filter(history_N);
      for (size_t k = 0; k < MAX_SENSORS; k++) {
        filterClass_perSensor[i][k][j] = new filter(history_N);
      }
    }
  }
}

bool cFilter(int channel_, float *azimuth_, float *elevation_){
  filter *faz = filterClass_[channel_][0];
  filter *fele = filterClass_[channel_][1];

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
      //dataReady[channel_] = true;
      return true;
    }
  }
  return false;

}



bool cFilter_sensor( int id_,  float *azimuth_, float *elevation_, int channel_){
  filter *faz = filterClass_perSensor[channel_][id_][0];
  filter *fele = filterClass_perSensor[channel_][id_][1];

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
      //dataReady[channel_] = true;
      return true;
    }
  }
  return false;
}

bool cFilter_sensor(std::vector<rawRayData> *ray_){
  std::vector<rawRayData> retRay;

  //for (size_t i = 0; i < ray->size(); i++) {
  int i = 0;
  for(std::vector<rawRayData>::iterator vIt = ray_->begin(); vIt != ray_->end(); ++vIt){
    int ch_ = vIt->ch;
    int id_ = vIt->id;
    float azTmp = vIt->azimuth;
    float elTmp = vIt->elevation;

    if(cFilter_sensor(vIt->id, &azTmp, &elTmp, vIt->ch)){
      rawRayData ret_data;
      ret_data.id = vIt->id;
      ret_data.ch = vIt->ch;
      ret_data.azimuth = azTmp;
      ret_data.elevation = elTmp;
      ret_data.azMean = filterClass_perSensor[ch_][id_][0]->mean;
      ret_data.azVar = filterClass_perSensor[ch_][id_][0]->variance;
      ret_data.elMean = filterClass_perSensor[ch_][id_][1]->mean;
      ret_data.elVar = filterClass_perSensor[ch_][id_][1]->variance;
      retRay.push_back(ret_data);
    }
    i++;
  }

  ray_->clear();
  ray_->insert(ray_->begin(),retRay.begin(),retRay.end());
}


int main(int argc, char const *argv[]) {


  //==================[ROS]==================

  ros::Publisher pubHandl_correctBase;
  ros::Publisher pubHandl_swapBase;

  roboy_middleware_msgs::LighthousePoseCorrection lhmsg;
  std_msgs::Bool lhswap;

  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "mainLH");
  }
  //nh = ros::NodeHandlePtr(new ros::NodeHandle);
  ros::NodeHandle nh;
  //ros::Rate loop_rate(5);

  ros::Publisher pubHandl_Sensor;
  roboy_middleware_msgs::DarkRoomSensorV2 ros_msg;

  pubHandl_correctBase = nh.advertise<roboy_middleware_msgs::LighthousePoseCorrection>("/roboy/middleware/DarkRoom/lhcorrect", 1);
  pubHandl_swapBase = nh.advertise<std_msgs::Bool>("/roboy/middleware/DarkRoom/lhswap", 1);
  //pubHandl_Sensor = nh.advertise<roboy_middleware_msgs::DarkRoomSensorV2>("/roboy/middleware/DarkRoom/sensorsLH2", 1);


  tf::Transform tf;

  float x = 0.0;
  float y = 0.0;
  float z = 0.0;

  lhmsg.id = 1;
  lhmsg.type = 1; //relativ==0 ; absolut ==1

  tf::Vector3 position(x,y,z);
  tf::Quaternion orientation;//(x,quat_y,quat_z,quat_w);
  orientation.setRPY( 0, 0, 0 );

  tf.setOrigin(position);
  tf.setRotation(orientation);

  tf::transformTFToMsg(tf, lhmsg.tf);
  pubHandl_correctBase.publish(lhmsg);

  TrackerClass robot;



  //==================[define paths]==================
  //find the path to 3DModel file  that includes the
  //relativ 3D positions of the Lighthouse Sensors
  std::stringstream ss;
  //"Search Path" is an Absolut path, it was defined in CMAKELists.txt
  ss << SEARCH_PATH << "/cameraCalibration/3DModel/" << "htcViveTrack.yaml";
  string sensor_model_path = cv::samples::findFile(ss.str());          // object 3d coordinates of sensors
  ss.str("");
  ss << SEARCH_PATH << "/cameraCalibration/dump/" << "debug_data_dump.txt";
  string dbg_path = cv::samples::findFile(ss.str());          // object 3d coordinates of sensors
  ss.str("");
  ss << SEARCH_PATH << "/cameraCalibration/dump/" << "plain.jpg";
  string img_path = cv::samples::findFile(ss.str());          // object 3d coordinates of sensors


  //==================[load 3d modell]==================
  //instance model class
  Model model;
  //load 3D sensor positions from a *.yaml file
  model.load(sensor_model_path);

  //========[register Cammera ... in this case lighthouse BASE]========
  //init lighthouse camera matrix
  double params_Lighthouse[4];
  getCameraMatrixForLighthouse(&params_Lighthouse[0]);


  //==================[PNP]==================
  //register the camera Matrix to the pnp-solver
  PnPProblem pnp_registration(params_Lighthouse);
  // PnP parameters
  int pnpMethod = SOLVEPNP_ITERATIVE;
  int iterationsCount = 900;      // number of Ransac iterations.
  float reprojectionError = 6.0;  // maximum allowed distance to consider it an inlier.
  double confidence = 0.999;       // ransac successful confidence.
  cv::Mat inliers_idx;


  //==================[usb driver]==================
  //init vive dongle
  std::cout<<"\n=======\nConnect to usb" << std::flush;
  driverHtcDongle driver;//TODO: find usbDongle automatically


  //============Match Data init ============
  std::cout<<"\n=======\nInit Datamatcher" << std::flush;
  DataMatcher dataM(params_Lighthouse, model.sensorData_3d.size());
  //match 2d data with 3d data



  //todo change this to an infinity loop
  uint16_t sucessCnt = 0;
  std::cout << "\n[STARTING] BASE REGISTRATION] ";
  std::cout << "\nplease wait, this could take a bit ... ";

  //for debug purposis
  uint8_t dbg_itCnt = 0;

  ros::Rate rate(30);

  //==== init Filter ====
  cFilter_init(10);//the parameter defines the history size

  uint8_t cnt_ = 0;



  bool startPubRay = false;
  //============================ Main LOOP ===========================
  while(ros::ok()){
    std::vector<rawRayData> ray;

    //pull/poll data from usb
    if(driver.pullData(&ray)){

      //filter rays with a standardDeviation filter
      dataM.filterfunc(&ray);

      //if there are enough data events per base it will get added
      //to the known list
      int baseN = dataM.registNewBaseStation(ray);
      //we at least need two bases for further processing
      if(startPubRay){
          //robot.publishSensorData(&ray);
          bool dataRdy = dataM.matchData(model.sensorData_3d,&ray);
          dataRdy = dataM.twoCameraMatcher();

          if(dataRdy){

            cv::Mat essentialMatrix,fundamental_matrix;
            cv::Mat rotMatrix, transMatrix, mask;
            //cv::Mat K = cv::Mat::eye(3,3,CV_64F);
            cv::Mat K = cv::Mat::zeros(3,3,CV_64FC1);

            K.at<double>(0, 0) = params_Lighthouse[0];       //      [ fx   0  cx ]
            K.at<double>(1, 1) = params_Lighthouse[1];       //      [  0  fy  cy ]
            K.at<double>(0, 2) = params_Lighthouse[2];       //      [  0   0   1 ]
            K.at<double>(1, 2) = params_Lighthouse[3];
            K.at<double>(2, 2) = 1;

            //cv::Mat K_ = cv::Mat::eye(3,3,CV_64F);

            std::cout << "\n1 data: " << dataM.dataImg1_2D.size() << std::flush;
            std::cout << "\n2 data: " << dataM.dataImg2_2D.size() << std::flush;


            essentialMatrix = cv::findEssentialMat(dataM.dataImg1_2D, dataM.dataImg2_2D, 1.0,cv::Point2d(params_Lighthouse[2],params_Lighthouse[3]),  RANSAC, 0.999, 1.0, mask);
            std::cout << "\nessential: " << essentialMatrix << std::flush;

            cv::recoverPose(essentialMatrix, dataM.dataImg1_2D, dataM.dataImg2_2D, K, rotMatrix, transMatrix, mask);
            std::cout << "\nTransl: " << transMatrix  << std::flush;
            std::cout << "\nRotation: " << rotMatrix  << std::flush;

            double retRol, retPitch, retYaw;
            double A[3][3]= {
              {rotMatrix.at<double>(0,0),rotMatrix.at<double>(0,1),rotMatrix.at<double>(0,2)},
              {rotMatrix.at<double>(1,0),rotMatrix.at<double>(1,1),rotMatrix.at<double>(1,2)},
              {rotMatrix.at<double>(2,0),rotMatrix.at<double>(2,1),rotMatrix.at<double>(2,2)}
            };
            converRotMatrixToEuler(A, retRol, retPitch, retYaw);


            std::cout << "\nRotationEULER:\tRol: " << retRol << "\tPitch: " << retPitch << "\tYaw: "  << retYaw  << std::flush;

            //Now we can transpose the center in respekt to World0
            lhmsg.type = 1; //relativ==0 ; absolut ==1
            //due to different coordinate system we need to map x,y,z proberly
            float lh_x = transMatrix.at<float>(1);
            float lh_y = transMatrix.at<float>(0) - 1; //because lh0 base already has a tranlation of -1
            float lh_z = transMatrix.at<float>(2);
            //send Translation and Rotation via ros to rviz
            ///double foo = transMatrix(0);
            position.setValue(lh_x,lh_y,lh_z);
            orientation.setRPY(retPitch,retRol, retYaw);

            std::cout << "\nSet transform data"  << std::flush;
            tf.setOrigin(position);
            tf.setRotation(orientation);

            std::cout << "\nTransform Msg"  << std::flush;
            tf::transformTFToMsg(tf, lhmsg.tf);

            pubHandl_correctBase.publish(lhmsg);
            ros::spinOnce();

            std::cout << "\nPub msg"  << std::flush;

          }

      }else{
        //only start the system if at least to lighthouseBases ar visible
        if(baseN >= 2){
          //due to system design, the lighthouse with the lower numerical value
          //and therfor lower polynome will be Position Zero
          //to interface with darkroom code we need to ensure that by
          //making a dummy call
          for (size_t lhCnt = 0; lhCnt < MAX_BASE_AMOUNT; lhCnt++) {
            if(dataM.dataReady[lhCnt] == true){
              rawRayData dumyRay;
              dumyRay.ch = lhCnt;
              std::vector<rawRayData> ray_init;
              ray_init.push_back(dumyRay);

              robot.publishSensorData(&ray_init);
              startPubRay = true;
              break;
            }
          }
        }

      }


    }


    /*

    //Picture solver


    std::cout << "\nRansac solver";
    pnp_registration.estimatePoseRANSAC( dataM.list_points3d[1], dataM.list_points2d[1],
                              pnpMethod, inliers_idx, iterationsCount, reprojectionError, confidence );

    // Get the translation
    cv::Mat translation_measured(3, 1, CV_64F);
    translation_measured = pnp_registration.get_t_matrix();
    std::cout << "\nTransl\n" << translation_measured;
    // Get the rotation
    cv::Mat rotation_measured(3, 3, CV_64F);
    rotation_measured = pnp_registration.get_R_matrix();
    std::cout << "\nRotation\n" << rotation_measured;

    dataM.reset_matchData();

    */



    /*
    id.clear();
    azimuth.clear();
    elevation.clear();
    channel.clear();
    */

    /*
    //============Poll usb data ============
    bool dataWasTransmitted = driver.pullData(&id,&azimuth,&elevation, &channel);

    if(dataWasTransmitted && (id.size() > 0) ){

        //if we get a certain amount of data from a base Station
        //it will be added to the list of known Base Stations
        int baseN = dataM.registNewBaseStation(channel);

        sucessCnt++;

        if(wasCalibrated == false){
          bool dataRdy = false;
          dataRdy = dataM.matchData(model.sensorData_3d,id,azimuth,elevation,channel);

          dataRdy = dataM.twoCameraMatcher();
          if(dataRdy){
            cv::Mat essentialMatrix,fundamental_matrix;
            cv::Mat rotMatrix, transMatrix, mask;
            //cv::Mat K = cv::Mat::eye(3,3,CV_64F);
            cv::Mat K = cv::Mat::zeros(3,3,CV_64FC1);

            K.at<double>(0, 0) = params_Lighthouse[0];       //      [ fx   0  cx ]
            K.at<double>(1, 1) = params_Lighthouse[1];       //      [  0  fy  cy ]
            K.at<double>(0, 2) = params_Lighthouse[2];       //      [  0   0   1 ]
            K.at<double>(1, 2) = params_Lighthouse[3];
            K.at<double>(2, 2) = 1;

            //cv::Mat K_ = cv::Mat::eye(3,3,CV_64F);

            std::cout << "\n1 data: " << dataM.dataImg1_2D.size() << std::flush;
            std::cout << "\n2 data: " << dataM.dataImg2_2D.size() << std::flush;


            essentialMatrix = cv::findEssentialMat(dataM.dataImg1_2D, dataM.dataImg2_2D, 1.0,cv::Point2d(params_Lighthouse[2],params_Lighthouse[3]),  RANSAC, 0.999, 1.0, mask);
            std::cout << "\nessential: " << essentialMatrix << std::flush;

            cv::recoverPose(essentialMatrix, dataM.dataImg1_2D, dataM.dataImg2_2D, K, rotMatrix, transMatrix, mask);
            std::cout << "\nTransl: " << transMatrix  << std::flush;
            std::cout << "\nRotation: " << rotMatrix  << std::flush;

            double retRol, retPitch, retYaw;
            double A[3][3]= {
              {rotMatrix.at<double>(0,0),rotMatrix.at<double>(0,1),rotMatrix.at<double>(0,2)},
              {rotMatrix.at<double>(1,0),rotMatrix.at<double>(1,1),rotMatrix.at<double>(1,2)},
              {rotMatrix.at<double>(2,0),rotMatrix.at<double>(2,1),rotMatrix.at<double>(2,2)}
            };
            converRotMatrixToEuler(A, retRol, retPitch, retYaw);


            std::cout << "\nRotationEULER:\tRol: " << retRol << "\tPitch: " << retPitch << "\tYaw: "  << retYaw  << std::flush;

            //Now we can transpose the center in respekt to World0
            lhmsg.type = 1; //relativ==0 ; absolut ==1
            //due to different coordinate system we need to map x,y,z proberly
            double lh_x = transMatrix.at<double>(1);
            double lh_y = transMatrix.at<double>(0) - 1; //because lh0 base already has a tranlation of -1
            double lh_z = transMatrix.at<double>(2);
            //send Translation and Rotation via ros to rviz
            ///double foo = transMatrix(0);
            position.setValue(lh_x,lh_y,lh_z);
            orientation.setRPY(retPitch,retRol, retYaw);

            tf.setOrigin(position);
            tf.setRotation(orientation);

            tf::transformTFToMsg(tf, lhmsg.tf);

            pubHandl_correctBase.publish(lhmsg);
            ros::spinOnce();


            //due to system design, the lighthouse with the numerical lower
            //polynome will be Position Zero
            //to ensure that in the darkroom funktion ... we call the first base once
            for (size_t lhCnt = 0; lhCnt < MAX_BASE_AMOUNT; lhCnt++) {
              if(dataM.dataReady[lhCnt] == true){
                ros_msg.object_id = "noclue";
                ros_msg.base = lhCnt;
                ros_msg.SensorID = 0;
                ros_msg.elevation = -15.0 * M_PI/180.0;
                ros_msg.azimuth = 30.0 * M_PI/180.0;
                pubHandl_Sensor.publish(ros_msg);
                break;
              }
            }





            wasCalibrated = true;
          }
        }else{

          if(baseN > 0){
            //wasCalibrated = true;//todo remove


            for(size_t i = 0; i < id.size(); i++){
              //if(cFilter(channel[i], &azimuth[i], &elevation[i])){
              if(elevation[i] > (-30.0 * M_PI/180.0) && elevation[i] < (30.0 * M_PI/180.0) ){
                if(azimuth[i] > (35.0 * M_PI/180.0) && azimuth[i] < (160.0 * M_PI/180.0)){
                  //if(cFilter(channel[i], &azimuth[i], &elevation[i])){
                  if(cFilter_sensor(channel[i], id[i], &azimuth[i], &elevation[i])){
                    //======[ROS Msg]=====
                    ros_msg.object_id = "noclue";
                    //ros_msg.base = (uint8_t)channel[i];
                    ros_msg.base = channel[i];
                    ros_msg.SensorID = (uint8_t)id[i];

                    ros_msg.elevation = elevation[i];
                    ros_msg.azimuth = azimuth[i];
                    pubHandl_Sensor.publish(ros_msg);
                  }
                }

              }

              //}

            }


          }else{
            ;

          }


        }



    }
    */

    ros::spinOnce();
    //rate.sleep();
  }



  //int k = waitKey(0);
  std::cout << "\n[END PROGRAM]" << std::flush;
  return 0;
}
