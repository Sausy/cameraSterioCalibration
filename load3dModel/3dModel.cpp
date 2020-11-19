/*
 * Model.cpp
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#include "3dModel.h"
#include <vector>
#include <sstream>
#include <fstream>


Model::Model()
{
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

}
