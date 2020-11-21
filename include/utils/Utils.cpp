#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include <vector>

#include <math.h>

#include "Utils.hpp"

#define PIC_WIDTH 2800.0

//double * cameraParamter = {fx,fy,cx,cy}
void getCameraMatrixForLighthouse(double * cameraParamter){
  //if we assume
  //
  //p.s.: umso kleiner die brennweite ist um so mehr weitwinkel
  //brennweitenabstand c = 1mm

  //
  //If we want to have a field of view
  //Fow = 140
  //phi = 90-140/2 = 20
  //
  //now to achiev this the following
  //equation needs to be satisfied
  //fx/tan(phi) + u0 = width/2
  //
  //fx = kx *c = with/2 * tan(phi)
  //
  //if we want to achiev a certain precision
  //lets call it delta phi (dp) and the given equation
  //
  // kx = 1/dx ...where dx are pixel size in [mm]
  // kx * c = fx = 1/[tan(phi + dp) - tan(phi)]
  //
  // this means we have the following equatin
  // 1/[tan(phi + dp) - tan(phi)]  = with/2 * tan(phi)
  //
  // with = 2/{tan(phi)  * [tan(phi + dp) - tan(phi)]}
  //
  // lets say we want a precision of 0.1deg per pixel
  // dw = 2778,3181
  // to keep it clean
  //
  // 2800x2800
  // cx = u0 = 1400

  //const double c = 100.0; //in mm
  const double width = PIC_WIDTH;
  const double height = width;

  const double phi_FOV = 140;
  const double phi = 90-(2*phi_FOV);

  const double fx = width/2 * tan(phi);
  const double fy = height/2 * tan(phi);

  const double cx = width/2;
  const double cy = height/2;

  cameraParamter[0] = fx;
  cameraParamter[1] = fy;
  cameraParamter[2] = cx;
  cameraParamter[3] = cy;

}
//double * cameraParamter = {fx,fy,cx,cy}
//the following function was written acording to openCV documentation about perspective projection
std::vector<double> azimuthTo2D(double azimuth, double elevation, double * cameraParamter){
  std::vector<double> vec2D;

  //azimuth = azimuth * M_PI/180;
  //elevation = elevation * M_PI/180;


  const double fx = cameraParamter[0];
  const double fy = cameraParamter[1];
  const double cx = cameraParamter[2];
  const double cy = cameraParamter[3];

  const double Z = 100.0;
  const double X = Z/tan(azimuth);

  double u = 0;
  //TODO: 100 ist die brennweite, die sollte als Paramter vorhanden sein nicht
  //hard gecoded
  u = 1/tan(azimuth);
  //std::cout << "\nplain U: " << u;
  //u = u * fx;
  //u = u + cx;
  u = u * fx + cx;

  /*
  //calc the help vector length
  double r1 = sqrt(pow(X,2)+pow(Z,2));
  //the minus adresses the axis direction swap from the lighthouse
  //coord system to the picture coord system
  double Y = -r1 * tan(elevation);

  double v = Y/Z * fy + cy;
  */

  //the minus adresses the axis direction swap from the lighthouse
  double v = (tan(-1 * elevation)) * fy + cy;


  std::cout << "\naz: " << azimuth * 180/M_PI;
  std::cout << " el: " << elevation * 180/M_PI;
  std::cout << " u: " << u;
  std::cout << " v: " << v;

  //std::cout << " === " << X;
  //std::cout << " | " << Y;
  std::cout << "\t| cx: " << cx;
  std::cout << " | fx: " << fx;

  vec2D.push_back(u);
  vec2D.push_back(v);

  return vec2D;
}

double calcMean(const std::vector<double> data_){
    uint8_t N = data_.size();
    const double *data = &data_[0];

    return (calcMean(data,N));
}

double calcMean(const double * data_, uint8_t N){
  double mean = 0.0;

  for (size_t i = 0; i < N; i++) {
    mean = mean + *data_;
    std::cout << "\nDATA: " << *data_ << "\tMean: " << mean;
    data_++;
  }
  mean = mean/N;
  return mean;
}

double standardDeviation(const std::vector<double> data_, double mean_){
  uint8_t N = data_.size();
  const double *data = &data_[0];

  return standardDeviation(data,mean_,N);
}

double standardDeviation(const double * data_, double mean_, uint8_t N){
  double s = 0.0;
  double buffer_ = 0.0;

  for (size_t i = 0; i < N; i++) {
    buffer_ = buffer_ + ((mean_ - *data_)*(mean_ - *data_));
    data_++;
  }

  s = buffer_/(N-1);

  return sqrt(s);
}
