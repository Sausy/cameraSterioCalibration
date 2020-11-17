/*
 * Model.cpp
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#include "Model.h"
#include "CsvWriter.h"
#include <vector>
#include <sstream>
#include <fstream>

//#define YAML_VALUE_SCALE 1000
#define YAML_VALUE_SCALE 1

Model::Model() : n_correspondences_(0), list_points2d_in_(0), list_points2d_out_(0), list_points3d_in_(0), training_img_path_()
{
}

Model::~Model()
{
    // TODO Auto-generated destructor stub
}

void Model::add_correspondence(const cv::Point2f &point2d, const cv::Point3f &point3d)
{
    list_points2d_in_.push_back(point2d);
    list_points3d_in_.push_back(point3d);
    n_correspondences_++;
}

void Model::add_outlier(const cv::Point2f &point2d)
{
    list_points2d_out_.push_back(point2d);
}

void Model::add_descriptor(const cv::Mat &descriptor)
{
    descriptors_.push_back(descriptor);
}

void Model::add_keypoint(const cv::KeyPoint &kp)
{
    list_keypoints_.push_back(kp);
}

void Model::set_trainingImagePath(const std::string &path)
{
    training_img_path_ = path;
}

/** Save a YAML file and fill the object mesh */
void Model::save(const std::string &path)
{
    cv::Mat points3dmatrix = cv::Mat(list_points3d_in_);
    cv::Mat points2dmatrix = cv::Mat(list_points2d_in_);

    cv::FileStorage storage(path, cv::FileStorage::WRITE);
    storage << "points_3d" << points3dmatrix;
    storage << "points_2d" << points2dmatrix;
    storage << "keypoints" << list_keypoints_;
    storage << "descriptors" << descriptors_;
    storage << "training_image_path" << training_img_path_;

    storage.release();
}

/** Load a YAML file using OpenCv functions **/
void Model::load(const std::string &path)
{
  uint8_t vectorSize_MaxM = 0; //means it can grow
  uint8_t vectorSize_MaxN = 3; //because its a 3d vector

  //std::vector<std::vector<double>> retVec;//= std::vector<double>(1,std::vector<double>(vectorSize_MaxN));

  //Open the yml file
  //std::ifstream f(sensor_model_path);
  std::ifstream f(path);
  std::string line;
  std::stringstream ss;
  std::vector<double> vec;

  uint8_t retM = 0; //matrix counter ... mxn
  uint8_t retN = 0; //matrix counter ... mxn


  bool fileHeaderDone = false;


  std::cout << "\n3D Position of the sensors";
  while (std::getline(f,line)) {

    if(fileHeaderDone){

      bool idFlag = true;
      for (auto it=line.cbegin()+3; it!=line.cend(); ++it) { //+3 because the first three chars are irelevant
        if(*it == ',' || *it == ']'){
          //Todo isn't secure agains non numeric strings
          if(!idFlag){
            vec.push_back(stod(ss.str()) * YAML_VALUE_SCALE);
          }
          idFlag = false;
          ss.str("");

          if(*it == ']')
            break;
        }else{
          ss << *it;
        }

      }

      //prevent overflow if data is faulty
      if(vec.size() >= 30){
        vec.erase(vec.begin(),vec.end());
        cout << "\nvector size is way to big in yaml file";
      }

      //load the gathered vector onto the return matrix
      if(vec.size() == vectorSize_MaxN){
        sensorData_3d.push_back(std::vector<double>(0,0));
        sensorData_3d[retM].insert(sensorData_3d[retM].begin(), vec.begin(), vec.end());

        std::cout << "\n";
        //std::cout << "sensorData_3d: " << sensorData_3d.size() << " | " << sensorData_3d[0].size() <<"\n";
        for (size_t i = 0; i < sensorData_3d[0].size(); i++) {
          std::cout << sensorData_3d[retM][i] << " | ";
        }

        retM++;

        vec.erase(vec.begin(),vec.end());
      }




    }

    (void)line.pop_back();//to get rid of the cariage return
    if(line.compare("sensor_relative_locations:") == 0){
      fileHeaderDone = true;

    }

  }


  f.close();
    /*
    cv::Mat points3d_mat;

    cv::FileStorage storage(path, cv::FileStorage::READ);
    storage["points_3d"] >> points3d_mat;
    storage["descriptors"] >> descriptors_;
    if (!storage["keypoints"].empty())
    {
        storage["keypoints"] >> list_keypoints_;
    }
    if (!storage["training_image_path"].empty())
    {
        storage["training_image_path"] >> training_img_path_;
    }

    points3d_mat.copyTo(list_points3d_in_);

    storage.release();
    */
}
