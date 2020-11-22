#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include <vector>

#include <math.h>

#include "filter.hpp"

filter::filter(uint16_t maxData_){
  N = maxData_;
}

//calculates the Mean and the Standard Deviation and
//and returns it via retMean and retVar
bool filter::stdDeviationFilter(double inData){
  std::vector<double> *v = &dataBuffer;

  v->push_back(inData);
  //if there is enough data pushed onto the vector buffer we can
  //continue without any data changes
  if(v->size() == N){
    ;//do nothing ... was just used for dbg STuff

  //if value bigger than desired ringBuf size it needs to be reduced
  }else if(v->size() > N){
    int inc_ = v->size() - N - 1;
    v->erase(v->begin()+inc_);

  //if we don't have enough datapoints func returns false;
  }else{
    return false;
  }

  mean = calcMean(*v);
  variance = standardDeviation(*v,mean);

  return true;
}


double filter::calcMean(const std::vector<double> data_){
    uint8_t N = data_.size();
    const double *data = &data_[0];
    return (calcMean(data,N));
}

double filter::calcMean(const double * data_, uint8_t N){
  double mean = 0.0;

  for (size_t i = 0; i < N; i++) {
    mean = mean + *data_;
    //std::cout << "\nDATA: " << *data_ << "\tMean: " << mean;
    data_++;
  }
  mean = mean/N;
  return mean;
}


//calculates the standar deviation from a given inData
//and returns it as a double
double filter::standardDeviation(const std::vector<double> data_, double mean_){
  uint8_t N = data_.size();
  const double *data = &data_[0];
  return standardDeviation(data,mean_,N);
}
double filter::standardDeviation(const double * data_, double mean_, uint8_t N){
  double s = 0.0;
  double buffer_ = 0.0;

  for (size_t i = 0; i < N; i++) {
    buffer_ = buffer_ + ((mean_ - *data_)*(mean_ - *data_));
    data_++;
  }

  s = buffer_/(N-1);

  return sqrt(s);
}
