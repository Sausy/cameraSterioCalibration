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
// PnP Tutorial
#include <Mesh.h>
#include <PnPProblem.h>
#include <RobustMatcher.h>
#include <ModelRegistration.h>
#include <Utils_openCV.h>

//common Utils
#include <Utils.hpp>

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

/*
#define MIN_AZIMUTH_ANGLE 25.0 //in deg
#define MAX_AZIMUTH_ANGLE ( 180.0 + 30.0 - MIN_AZIMUTH_ANGLE) //in deg

#define MIN_AZIMUTH_ANGLE_RAD ( MIN_AZIMUTH_ANGLE * M_PI/180.0 ) //in deg
#define MAX_AZIMUTH_ANGLE_RAD ( MAX_AZIMUTH_ANGLE * M_PI/180.0 ) //in deg

#define MIN_ELEVATION_ANGLE ( -1 * MIN_AZIMUTH_ANGLE ) //in deg
#define MAX_ELEVATION_ANGLE ( MIN_AZIMUTH_ANGLE ) //in deg

#define MIN_ELEVATION_ANGLE_RAD ( MIN_ELEVATION_ANGLE * M_PI/180.0 ) //in deg
#define MAX_ELEVATION_ANGLE_RAD ( MAX_ELEVATION_ANGLE * M_PI/180.0 ) //in deg
*/

int main(int argc, char const *argv[]) {
  /*std::cout<<"\nMax: " << MIN_AZIMUTH_ANGLE;
  std::cout<<"\tMax: " << MAX_AZIMUTH_ANGLE;
  std::cout<<"\tMin: " << MIN_AZIMUTH_ANGLE_RAD;
  std::cout<<"\tMax: " << MAX_AZIMUTH_ANGLE_RAD;
  std::cout<<"\tMin: " << MIN_ELEVATION_ANGLE;
  std::cout<<"\tMax: " << MAX_ELEVATION_ANGLE;
  return 0;
  */

  //==================[define paths]==================
  //find the path to 3DModel file  that includes the
  //relativ 3D positions of the Lighthouse Sensors
  std::stringstream ss;
  //"Search Path" is an Absolut path, it was defined in CMAKELists.txt
  ss << SEARCH_PATH << "/3DModel/" << "htcViveTrack.yaml";
  string sensor_model_path = cv::samples::findFile(ss.str());          // object 3d coordinates of sensors
  ss.str("");
  ss << SEARCH_PATH << "/dump/" << "debug_data_dump.txt";
  string dbg_path = cv::samples::findFile(ss.str());          // object 3d coordinates of sensors
  ss.str("");
  ss << SEARCH_PATH << "/dump/" << "plain.jpg";
  string img_path = cv::samples::findFile(ss.str());          // object 3d coordinates of sensors


  //==================[load 3d modell]==================
  //instance model class
  Model model;
  //load 3D sensor positions from a *.yaml file
  model.load(sensor_model_path);

  //========[register Cammera ... in this case lighthouse BASE]========
  //init lighthouse camera matrix
  //params_Lighthouse = {fx,fy,cx,cy}

  double params_Lighthouse[4];
  getCameraMatrixForLighthouse(&params_Lighthouse[0]);
  /*
  double f = 55;                           // focal length in mm
  //double sx = 22.3, sy = 14.9;             // sensor size
  double sx = 32, sy = 24.9;             // sensor size
  double width = 640, height = 480;        // image size

  double params_Lighthouse[] = { width*f/sx,   // fx
                             height*f/sy,  // fy
                             width/2,      // cx
                             height/2};    // cy
                             */

  //==================[PNP]==================
  //register the camera Matrix to the pnp-solver
  PnPProblem pnp_registration(params_Lighthouse);
  // PnP parameters
  int pnpMethod = SOLVEPNP_ITERATIVE;
  int iterationsCount = 800;      // number of Ransac iterations.
  float reprojectionError = 6.0;  // maximum allowed distance to consider it an inlier.
  double confidence = 0.99;       // ransac successful confidence.
  cv::Mat inliers_idx;


  //==================[usb driver]==================
  //init vive dongle
  driverHtcDongle driver;//TODO: find usbDongle automatically


  //============Match Data init ============
  DataMatcher dataM(params_Lighthouse, model.sensorData_3d.size());
  //match 2d data with 3d data


  //todo change this to an infinity loop
  uint16_t sucessCnt = 0;
  std::cout << "\n[STARTING] BASE REGISTRATION] ";
  std::cout << "\nplease wait, this could take a bit ... ";

  //for debug purposis
  uint8_t dbg_itCnt = 0;

  std::vector<int> id;
  std::vector<float> azimuth;
  std::vector<float> elevation;
  std::vector<int> channel;

  for(;;){
    id.clear();
    azimuth.clear();
    elevation.clear();
    channel.clear();

    //============Poll usb data ============
    bool dataWasTransmitted = driver.pullData(&id,&azimuth,&elevation, &channel);

    if(dataWasTransmitted && (id.size() > 0) ){

        //if we get a certain amount of data from a base Station
        //it will be added to the list of known Base Stations
        int baseN = dataM.registNewBaseStation(channel);

        sucessCnt++;


        //after this function (matchData()) we get two vectors
        //vector<Point3f> list_points3d;
        //vector<Point2f> list_points2d;
        bool dataRdy = false;
        dataRdy = dataM.matchData(model.sensorData_3d,id,azimuth,elevation,channel);
        /*
        if(dataRdy && (dataM.list_points3d[0].size() > 7)){
            cout << "\nPNP: " <<  std::flush;
            cv::Mat transMatrix(3,1,CV_64F);

            //std::cout<<"\n[STARTING] pnp sovler \tAmount of Data: " << dataM.list_points2d[].size();
            pnp_registration.estimatePoseRANSAC( dataM.list_points3d[0], dataM.list_points2d[0],
                                              pnpMethod, inliers_idx,
                                              iterationsCount, reprojectionError, confidence );


            transMatrix = pnp_registration.get_t_matrix();
            cout << "\nTrans: " << transMatrix.size();
            cout << "\n" << transMatrix;
            cout << "\n" << std::flush;

            bool is_correspondence = pnp_registration.estimatePose(dataM.list_points3d[0], dataM.list_points2d[0], SOLVEPNP_ITERATIVE);

            transMatrix = pnp_registration.get_t_matrix();
            cout << "\nTrans: " << transMatrix.size();
            cout << "\n" << transMatrix;
            cout << "\n";

            return 0;
        }
        */


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

          cv::Mat K_ = cv::Mat::eye(3,3,CV_64F);

          std::cout << "\n1 data: " << dataM.dataImg1_2D.size() << std::flush;
          std::cout << "\n2 data: " << dataM.dataImg2_2D.size() << std::flush;
          //std::cout << "\n2 data: " << dataM.dataImg2_2D[0].pt << std::flush;


          ///fundamental_matrix = findFundamentalMat(dataM.dataImg1_2D, dataM.dataImg2_2D, FM_8POINT, 3,0.99);
          //std::cout << "\nfindFundamentalMat " << essentialMatrix << std::flush;

          //essentialMatrix = cv::findEssentialMat(dataM.dataImg1_2D, dataM.dataImg2_2D, RANSAC, 3, 0.99, mask);
          //std::cout << "\nessential: " << essentialMatrix << std::flush;

          //essentialMatrix = cv::findEssentialMat(dataM.dataImg1_2D, dataM.dataImg2_2D, 0.1,cv::Point2d(1400.0,1400.0),  RANSAC, 0.99, 1.0, mask);
          //std::cout << "\nessential: " << essentialMatrix << std::flush;

          //cv::recoverPose(essentialMatrix, dataM.dataImg1_2D, dataM.dataImg2_2D, K, rotMatrix, transMatrix, mask);
          //std::cout << "\nTransl: " << transMatrix  << std::flush;


          essentialMatrix = cv::findEssentialMat(dataM.dataImg1_2D, dataM.dataImg2_2D, 1.0,cv::Point2d(1400.0,1400.0),  RANSAC, 0.99, 1.0, mask);
          std::cout << "\nessential: " << essentialMatrix << std::flush;

          cv::recoverPose(essentialMatrix, dataM.dataImg1_2D, dataM.dataImg2_2D, K, rotMatrix, transMatrix, mask);
          std::cout << "\nTransl: " << transMatrix  << std::flush;

          /*
          essentialMatrix = cv::findEssentialMat(dataM.dataImg1_2D, dataM.dataImg2_2D, 10.0,cv::Point2d(1400.0,1400.0),  RANSAC, 0.99, 1.0, mask);
          std::cout << "\nessential: " << essentialMatrix << std::flush;

          cv::recoverPose(essentialMatrix, dataM.dataImg1_2D, dataM.dataImg2_2D, K, rotMatrix, transMatrix, mask);
          std::cout << "\nTransl: " << transMatrix  << std::flush;

          essentialMatrix = cv::findEssentialMat(dataM.dataImg1_2D, dataM.dataImg2_2D, 1000.0,cv::Point2d(1400.0,1400.0),  RANSAC, 0.99, 1.0, mask);
          std::cout << "\nessential: " << essentialMatrix << std::flush;

          cv::recoverPose(essentialMatrix, dataM.dataImg1_2D, dataM.dataImg2_2D, K, rotMatrix, transMatrix, mask);
          std::cout << "\nTransl: " << transMatrix  << std::flush;

          */
          //std::cout << "\nK: " << K;
          return 0;
        }

        /*
        if(dataRdy){
            std:cout << "\nAmount of base stations " << baseN;

            //for (size_t k = 0; k < MAX_SENSOR_CNT; k++) {
            //  int data_size = dataM.azimuthHistory[0][k].size();
            //  if(data_size > 0){
            //
            //    std::cout << "\n\nHistory DATA:\nID:"<< k << "\n";
            //    for (size_t l = 0; l < data_size; l++) {
            //      std::cout << dataM.azimuthHistory[0][k][l] * 180/M_PI << "\t";
            //    }
            //  }
            //}


            //std::cout << "\n3d: " << dataM.list_points3d;
            //std::cout << "\n2d: " << dataM.list_points2d;
            //============ PNP Solver============
            cv::Mat transMatrix(3,1,CV_64F);

            //std::cout<<"\n[STARTING] pnp sovler \tAmount of Data: " << dataM.list_points2d[].size();
            pnp_registration.estimatePoseRANSAC( dataM.list_points3d[0], dataM.list_points2d[0],
                                              pnpMethod, inliers_idx,
                                              iterationsCount, reprojectionError, confidence );


            transMatrix = pnp_registration.get_t_matrix();
            cout << "\nTrans: " << transMatrix.size();
            cout << "\n" << transMatrix;
            cout << "\n";




            //draw2DPoints(img, list_points2d, green);
            //imshow("MODEL REGISTRATION", img);

            bool is_correspondence = pnp_registration.estimatePose(dataM.list_points3d[0], dataM.list_points2d[0], SOLVEPNP_ITERATIVE);

            transMatrix = pnp_registration.get_t_matrix();
            cout << "\nTrans: " << transMatrix.size();
            cout << "\n" << transMatrix;
            cout << "\n";

            //to start a new run we need to reset the data
            dataM.reset_matchData();

            dbg_itCnt++;
            if(dbg_itCnt >= 1)
              return 0;
        }
        */
    }
    //===for debuging
    if(sucessCnt > 30000)
      break;
  }



  //open plane image
  /*
  Mat img_in = imread(img_path, IMREAD_COLOR);
  Mat img;
  if(img_in.empty()){
    std::cout << "\No Image found \n";
    return 0;
  }
  img = img_in.clone();


  imshow("MODEL REGISTRATION", img);
  */




  /* ===========[DEBUG STuff]==============*/
  //The following data was dumped via the lighthouse_console
  //this programm can be found under:
  //~/.steam/steam/steamapps/common/SteamVR/tools/lighthouse/bin/linux64
  //afterwards it was converterd into usefull data

  //load test Data
  /*
  testData tdata(&params_Lighthouse[0]);
  tdata.load(dbg_path);
  tdata.matchData(model.sensorData_3d);


  for (size_t i = 0; i < tdata.sensorData_solver_2d.size(); i++) {

    Point2f buf2d;
    buf2d.x = tdata.sensorData_solver_2d[i][0];
    buf2d.y = tdata.sensorData_solver_2d[i][1];
    Point3f buf3d;
    buf3d.x = tdata.sensorData_solver_3d[i][0];
    buf3d.y = tdata.sensorData_solver_3d[i][1];
    buf3d.z = tdata.sensorData_solver_3d[i][2];

    list_points2d.push_back(buf2d);
    list_points3d.push_back(buf3d);

  }

  */
  //drawPoints(img, list_points2d, list_points3d, red);
  //imshow("MODEL REGISTRATION", img);
  //imshow("MODEL REGISTRATION", img);
  /*===========[DEBUG STuff END]= ==========*/



  //int k = waitKey(0);
  std::cout << "\n[END PROGRAM]" << std::flush;
  return 0;
}
