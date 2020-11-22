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
  int iterationsCount = 500;      // number of Ransac iterations.
  float reprojectionError = 6.0;  // maximum allowed distance to consider it an inlier.
  double confidence = 0.99;       // ransac successful confidence.
  cv::Mat inliers_idx;


  //==================[usb driver]==================
  //init vive dongle
  driverHtcDongle driver;//TODO: find usbDongle automatically


  //============Match Data init ============
  DataMatcher dataM(params_Lighthouse);
  //match 2d data with 3d data


  //todo change this to an infinity loop
  uint16_t sucessCnt = 0;
  for(;;){
    std::vector<int> id;
    std::vector<float> azimuth;
    std::vector<float> elevation;
    std::vector<int> channel;

    //============Poll usb data ============
    bool dataWasTransmitted = driver.pullData(&id,&azimuth,&elevation, &channel);

    if(dataWasTransmitted && (id.size() > 0) ){

        //if we get a certain amount of data from a base Station
        //it will be added to the list of known Base Stations
        dataM.registNewBaseStation(channel);

        sucessCnt++;

        //debug output azimuth
        /*
        std::cout<<"\nWe could gather the following data [size:" << id.size();
        for(uint8_t dataCnt = 0; dataCnt < id.size(); dataCnt++){
          std::cout<<"\nid: " << id[dataCnt];
          std::cout<<"\taz: " << azimuth[dataCnt] * 180.0/M_PI;
          std::cout<<"\tel: " << elevation[dataCnt] * 180.0/M_PI;
          std::cout<<"\tch: " << channel[dataCnt];
        }
        */

        //after this function (matchData()) we get two vectors
        //vector<Point3f> list_points3d;
        //vector<Point2f> list_points2d;
        bool dataRdy = false;
        dataRdy = dataM.matchData(model.sensorData_3d,id,azimuth,elevation,channel);

        if(dataRdy){
            std::cout << "\n3d: " << dataM.list_points3d;
            std::cout << "\n2d: " << dataM.list_points2d;
            //============ PNP Solver============
            pnp_registration.estimatePoseRANSAC( dataM.list_points3d, dataM.list_points2d,
                                              pnpMethod, inliers_idx,
                                              iterationsCount, reprojectionError, confidence );

            cv::Mat transMatrix(3,1,CV_64F);
            transMatrix = pnp_registration.get_t_matrix();
            cout << "\nTrans: " << transMatrix.size();
            cout << "\n" << transMatrix;
            cout << "\n";
            //draw2DPoints(img, list_points2d, green);
            //imshow("MODEL REGISTRATION", img);

            bool is_correspondence = pnp_registration.estimatePose(dataM.list_points3d, dataM.list_points2d, SOLVEPNP_ITERATIVE);

            transMatrix = pnp_registration.get_t_matrix();
            cout << "\nTrans: " << transMatrix.size();
            cout << "\n" << transMatrix;
            cout << "\n";

            return 0;
        }
    }
    //===for debuging
    if(sucessCnt > 1000)
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
  return 0;
}
