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
#include <Model.h>
#include <PnPProblem.h>
#include <RobustMatcher.h>
#include <ModelRegistration.h>
#include <Utils.h>

// test
#include <testData.h>

// Usb driver
#include <interface_htcDongle.h>

// 3D sensor position
#include <3dModel.h>

using namespace cv;
using namespace std;

// Some basic colors
const Scalar red(0, 0, 255);
const Scalar green(0,255,0);
const Scalar blue(255,0,0);
const Scalar yellow(0,255,255);


int main(int argc, char const *argv[]) {

  //=========[usb driver]========
  //init vive dongle
  driverHtcDongle driver;//TODO: find usbDongle automatically


  //todo change this to an infinity loop
  uint16_t sucessCnt = 0;
  for(;;){
    std::vector<int> id;
    std::vector<float> azimuth;
    std::vector<float> elevation;
    std::vector<int> channel;

    if(driver.pullData(&id,&azimuth,&elevation, &channel)){
      if(id.size() > 0){
        sucessCnt++;
        std::cout<<"\nWe could gather the following data [size:" << id.size();
        for(uint8_t dataCnt = 0; dataCnt < id.size(); dataCnt++){
          std::cout<<"\nid: " << id[dataCnt];
          std::cout<<"\taz: " << azimuth[dataCnt] * 180.0/M_PI;
          std::cout<<"\tel: " << elevation[dataCnt] * 180.0/M_PI;
          std::cout<<"\tch: " << channel[dataCnt];
        }
      }
    }
    if(sucessCnt > 1000)
      break;
  }

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

  //instance model class ...this doesn't loads the 3d sensor data
  Model model;
  model.load(sensor_model_path);

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


  //init lighthouse camera matrix
  //params_Lighthouse = {fx,fy,cx,cy}
  //double params_Lighthouse[4];
  //getCameraMatrixForLighthouse(&params_Lighthouse[0]);
  double f = 55;                           // focal length in mm
  double sx = 22.3, sy = 14.9;             // sensor size
  double width = 640, height = 480;        // image size

  double params_Lighthouse[] = { width*f/sx,   // fx
                             height*f/sy,  // fy
                             width/2,      // cx
                             height/2};    // cy


  //register the camera Matrix to the pnp-solver
  PnPProblem pnp_registration(params_Lighthouse);


  vector<Point3f> list_points3d; // container for the model 3D coordinates found in the scene
  vector<Point2f> list_points2d; // container for the model 2D coordinates found in the scene



  /* ===========[DEBUG STuff]==============*/
  //The following data was dumped via the lighthouse_console
  //this programm can be found under:
  //~/.steam/steam/steamapps/common/SteamVR/tools/lighthouse/bin/linux64
  //afterwards it was converterd into usefull data

  //load test Data
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

  //drawPoints(img, list_points2d, list_points3d, red);
  //imshow("MODEL REGISTRATION", img);
  //imshow("MODEL REGISTRATION", img);
  /*===========[DEBUG STuff END]= ==========*/

  cout << "\nSize of solver: " << tdata.sensorData_solver_2d.size();

  // PnP parameters
  int pnpMethod = SOLVEPNP_ITERATIVE;
  int iterationsCount = 500;      // number of Ransac iterations.
  float reprojectionError = 6.0;  // maximum allowed distance to consider it an inlier.
  double confidence = 0.99;       // ransac successful confidence.
  Mat inliers_idx;


  pnp_registration.estimatePoseRANSAC( list_points3d, list_points2d,
                                    pnpMethod, inliers_idx,
                                    iterationsCount, reprojectionError, confidence );

  cv::Mat transMatrix(3,1,CV_64F);
  transMatrix = pnp_registration.get_t_matrix();
  cout << "\nTrans: " << transMatrix.size();
  cout << "\n" << transMatrix;
  cout << "\n";
  //draw2DPoints(img, list_points2d, green);
  //imshow("MODEL REGISTRATION", img);

  bool is_correspondence = pnp_registration.estimatePose(list_points3d, list_points2d, SOLVEPNP_ITERATIVE);

  transMatrix = pnp_registration.get_t_matrix();
  cout << "\nTrans: " << transMatrix.size();
  cout << "\n" << transMatrix;
  cout << "\n";

  //int k = waitKey(0);
  return 0;
}
