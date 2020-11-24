#ifndef TESTDATA_H_
#define TESTDATA_H_

class testData{
  public:
      testData(const double * params_Lighthouse_);
      virtual ~testData();

      void load(const std::string path);
      void matchData(const std::vector<std::vector<float>> inVec);
      void matchData(const std::vector<std::vector<double>> inVec);

      //std::vector<std::vector<double>> sensorData_buffer;

      std::vector<std::vector<float>> sensorData_solver_2d;
      std::vector<std::vector<float>> sensorData_solver_3d;
      //std::vector<std::vector<double>> sensorData_solver_4d;
    private:
      std::vector<int> id;
      std::vector<int> base;
      std::vector<double> azimuth;
      std::vector<double> elevation;

      double params_Lighthouse[4];

};

#endif
