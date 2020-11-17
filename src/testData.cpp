#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>

#include "testData.h"

#include "Utils.h"



testData::testData(const double * params_Lighthouse_){
  for (size_t i = 0; i < 4; i++) {
    params_Lighthouse[i] = *params_Lighthouse_;
    params_Lighthouse_++;
  }

}

testData::~testData()
{
    // TODO Auto-generated destructor stub
}

void testData::load(const std::string path){
  uint8_t vectorSize_MaxM = 0; //means it can grow
  uint8_t vectorSize_MaxN = 4; //because its a 3d vector with identifier

  //std::vector<int> id;
  //std::vector<int> base;
  //std::vector<double> azimuth;
  //std::vector<double> elevation;


  //bool fileHeaderDone = false;
  //bool idDataFlage = true;
  //bool baseDataFlage = false;
  //bool angleDataFlage = false;

  std::ifstream f(path);
  std::string line;
  std::stringstream ss;

  int foo_cnt = 0;
  while (std::getline(f,line)) {
      for (auto it=line.cbegin()+1; it!=line.cend(); ++it) {
          if(*it == '='){
            id.push_back(stoi(ss.str()));
            ss.str("");
          }else if(*it == ']'){
            base.push_back(stoi(ss.str()));
            ss.str("");
          }else if(*it == '/'){
            azimuth.push_back(stod(ss.str()));
            ss.str("");
          }else{
            ss << *it;
          }
      }

      elevation.push_back(stod(ss.str()));
      ss.str("");


      foo_cnt ++;
      if(foo_cnt >= 50)
        break;
  }

  /*
  for (size_t i = 0; i < elevation.size(); i++) {
    std::cout << "\nAzimuth Elevation: ";
    std::cout << id[i] << "=" << base[i] << " | ";
    std::cout << azimuth[i] << " | " << elevation[i];
  }
  */


}

void testData::matchData(const std::vector<std::vector<double>> inVec){
  std::vector<std::vector<float>> vec;
  vec.reserve(inVec.size());
  for (auto&& v : inVec) vec.emplace_back(std::begin(v), std::end(v));

  matchData(vec);
}

void testData::matchData(const std::vector<std::vector<float>> inVec){



  //std::vector<std::vector<double>> * vbuf = &sensorData_buffer;
  //std::vector<std::vector<double>> * v2d = &sensorData_solver_2d;
  //std::vector<std::vector<double>> * v4d = &sensorData_solver_3d;
  std::cout << "\nMatch Data: size=" << id.size();

  bool idIsTaken[64] = {0};
  uint8_t goodCount = 0;

  for(size_t i = 0; i < id.size(); i++){
    //std::cout << "\nMatch Data: size=" << id.size() << "/i: " << i;

    if(idIsTaken[id[i]]){
    //if(false){
      std::cout<<"\nID: " << id[i] << " was already taken";
    }else{
      //std::cout<<"\nID: " << id[i] << " ac: " << azimuth[i];
      //TODO: DIRTY WORKAROUND FOR THE TEST DATA
      //NEEDS Prober filtering
      if(elevation[i] < 20.0){
        sensorData_solver_2d.push_back(std::vector<float>(0,0));
        sensorData_solver_3d.push_back(std::vector<float>(0,0));

        std::vector<double> buf2d_ = azimuthTo2D(azimuth[i], elevation[i], &params_Lighthouse[0]);
        std::vector<float> buf2d(buf2d_.begin(),buf2d_.end());

        sensorData_solver_2d[goodCount].insert(sensorData_solver_2d[goodCount].begin(), buf2d.begin(), buf2d.end());
        sensorData_solver_3d[goodCount].insert(sensorData_solver_3d[goodCount].begin(), inVec[id[i]].begin(), inVec[id[i]].end());

        idIsTaken[id[i]] = true;

        goodCount++;
      }
    }


    if(goodCount > 20){
      break;
    }
  }


  for (size_t i = 0; i < sensorData_solver_2d.size(); i++) {
    std::cout << "\n2D: Data ";
    for (size_t j = 0; j < sensorData_solver_2d[0].size(); j++) {
      std::cout << sensorData_solver_2d[i][j] << " ";
    }

    std::cout << "\n3D: Data ";
    for (size_t j = 0; j < sensorData_solver_3d[0].size(); j++) {
      std::cout << sensorData_solver_3d[i][j] << " ";
    }
  }

}
