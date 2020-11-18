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

using namespace cv;
using namespace std;

// Some basic colors
const Scalar red(0, 0, 255);
const Scalar green(0,255,0);
const Scalar blue(255,0,0);
const Scalar yellow(0,255,255);


int main(int argc, char const *argv[]) {

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
  Mat img_in = imread(img_path, IMREAD_COLOR);
  Mat img;
  if(img_in.empty()){
    std::cout << "\No Image found \n";
    return 0;
  }
  img = img_in.clone();


  imshow("MODEL REGISTRATION", img);



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

    //list_points2d.push_back(std::vector<double>(0,0));
    //list_points3d.push_back(std::vector<double>(0,0));

    //list_points2d[i].insert(list_points2d[i].begin(), tdata.sensorData_solver_2d[i].begin(), tdata.sensorData_solver_2d[i].end());
    //list_points3d[i].insert(list_points3d[i].begin(), tdata.sensorData_solver_3d[i].begin(), tdata.sensorData_solver_3d[i].end());
  }

  drawPoints(img, list_points2d, list_points3d, red);
  imshow("MODEL REGISTRATION", img);
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
  draw2DPoints(img, list_points2d, green);
  imshow("MODEL REGISTRATION", img);

  bool is_correspondence = pnp_registration.estimatePose(list_points3d, list_points2d, SOLVEPNP_ITERATIVE);

  transMatrix = pnp_registration.get_t_matrix();
  cout << "\nTrans: " << transMatrix.size();
  cout << "\n" << transMatrix;
  cout << "\n";
  /*

  if ( is_correspondence )
  {
      cout << "Correspondence found" << endl;

      // Compute all the 2D points of the mesh to verify the algorithm and draw it
      //vector<Point2f> list_points2d_mesh = pnp_registration.verify_points(&mesh);

      cv::Mat transMatrix(3,1,CV_64F);
      transMatrix = pnp_registration.get_t_matrix();
      cout << "\nTrans: " << transMatrix.size();
      cout << "\n" << transMatrix;
      cout << "\n";
      draw2DPoints(img, list_points2d, green);
      imshow("MODEL REGISTRATION", img);
      //for (size_t i = 0; i < pnp_registration._t_matrix; i++) {
      //
      //}
  } else {
      cout << "Correspondence not found" << endl << endl;
  }
  */




  /*


  for(unsigned int match_index = 0; match_index < good_matches.size(); ++match_index)
  {
      Point3f point3d_model = list_points3d_model[ good_matches[match_index].trainIdx ];  // 3D point from model
      Point2f point2d_scene = keypoints_scene[ good_matches[match_index].queryIdx ].pt; // 2D point from the scene
      list_points3d_model_match.push_back(point3d_model);         // add 3D point
      list_points2d_scene_match.push_back(point2d_scene);         // add 2D point
  }


  if(good_matches.size() >= 4) // OpenCV requires solvePnPRANSAC to minimally have 4 set of points
  {
      // -- Step 3: Estimate the pose using RANSAC approach
      pnp_detection.estimatePoseRANSAC( list_points3d_model_match, list_points2d_scene_match,
                                        pnpMethod, inliers_idx,
                                        iterationsCount, reprojectionError, confidence );

      // -- Step 4: Catch the inliers keypoints to draw
      for(int inliers_index = 0; inliers_index < inliers_idx.rows; ++inliers_index)
      {
          int n = inliers_idx.at<int>(inliers_index);         // i-inlier
          Point2f point2d = list_points2d_scene_match[n];     // i-inlier point 2D
          list_points2d_inliers.push_back(point2d);           // add i-inlier to list
      }

      // Draw inliers points 2D
      draw2DPoints(frame_vis, list_points2d_inliers, blue);

      // -- Step 5: Kalman Filter

      // GOOD MEASUREMENT
      if( inliers_idx.rows >= minInliersKalman )
      {
          // Get the measured translation
          Mat translation_measured = pnp_detection.get_t_matrix();

          // Get the measured rotation
          Mat rotation_measured = pnp_detection.get_R_matrix();

          // fill the measurements vector
          fillMeasurements(measurements, translation_measured, rotation_measured);
          good_measurement = true;
      }

      // update the Kalman filter with good measurements, otherwise with previous valid measurements
      Mat translation_estimated(3, 1, CV_64FC1);
      Mat rotation_estimated(3, 3, CV_64FC1);
      updateKalmanFilter( KF, measurements,
                          translation_estimated, rotation_estimated);

      // -- Step 6: Set estimated projection matrix
      pnp_detection_est.set_P_matrix(rotation_estimated, translation_estimated);
  }

  */
  //int k = waitKey(0);
  return 0;
}
