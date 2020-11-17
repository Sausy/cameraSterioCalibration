// C++
#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
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

using namespace cv;
using namespace std;

std::vector<double> getSensorModel(std::string sensor_model_path){
  std::vector<double> retVec;



  return retVec;
}

int main(int argc, char const *argv[]) {

  //find the path to 3DModel file  that includes the
  //relativ 3D positions of the Lighthouse Sensors
  std::stringstream ss;
  //"Search Path" is an Absolut path, it was defined in CMAKELists.txt
  ss << SEARCH_PATH << "/3DModel/" << "sensor_position.yml";
  string sensor_model_path = cv::samples::findFile(ss.str());          // object 3d coordinates of sensors

  //Open the yml file



  /* ===========[DEBUG STuff]===========




  */

  vector<Point3f> list_points3d_model_match; // container for the model 3D coordinates found in the scene
  vector<Point2f> list_points2d_scene_match; // container for the model 2D coordinates found in the scene



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


  return 0;
}
