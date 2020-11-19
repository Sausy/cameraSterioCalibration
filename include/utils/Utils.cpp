#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include <vector>

#include <math.h>

#include "Utils.hpp"


//double * cameraParamter = {fx,fy,cx,cy}
void getCameraMatrixForLighthouse(double * cameraParamter){
  //if we assume
  //1000x1000 picture size and a
  //brennweitenabstand c = 100mm
  //cx = 500
  //cy = 500
  //
  //This creates a Field of view FOV = 2*(90 - atan(c/cx)) = 157.3
  //phi = 11.3deg
  //
  //If we want a resolution of at least 0.01deg per pixel
  //and a pixel size of dx = c * [tan(11.3 + 0.01) - tan(11.3)] = 0.018
  //this means we have
  // kx = 1/dx = 55.093 Pixels/mm

  const double c = 100.0; //in mm
  const double width = 10000.0;
  const double height = width;

  const double cx = width/2;
  const double cy = height/2;

  const double fov = 12.0 * M_PI/180.0; //~160 deg
  const double precision = 0.01 * M_PI/180.0; //0.01deg

  double dx = 0.0;
  dx = dx + tan(fov + precision);
  dx = dx - tan(fov);
  dx = dx * c;

  const double kx = 1.0/dx;
  std::cout << "\n=====\nKx: " << kx << " | " << dx;
  const double ky = kx;

  cameraParamter[0] = c * kx;
  cameraParamter[1] = c * ky;
  cameraParamter[2] = cx;
  cameraParamter[3] = cy;

}
//double * cameraParamter = {fx,fy,cx,cy}
//the following function was written acording to openCV documentation about perspective projection
std::vector<double> azimuthTo2D(double azimuth, double elevation, double * cameraParamter){
  std::vector<double> vec2D;

  azimuth = azimuth * M_PI/180;
  elevation = elevation * M_PI/180;


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
  std::cout << "\nplain U: " << u;
  //u = u * fx;
  //u = u + cx;
  u = u * fx + cx;


  double buffer1 = pow(X,2);
  double buffer2 = pow(Z,2);

  buffer1 = buffer1 + buffer2;
  double r1 = sqrt(buffer1);
  double Y = -r1 * tan(elevation);

  double v = Y/Z * fy + cy;


  std::cout << "\naz: " << azimuth * 180/M_PI;
  std::cout << " el: " << elevation * 180/M_PI;
  std::cout << " u: " << u;
  std::cout << " v: " << v;

  std::cout << " === " << X;
  std::cout << " | " << Y;
  std::cout << " | cx: " << cx;
  std::cout << " | fx: " << fx;

  vec2D.push_back(u);
  vec2D.push_back(v);

  return vec2D;
}
