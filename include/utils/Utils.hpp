/*
 * Utils.h
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#ifndef UTILS_H_
#define UTILS_H_


//double * cameraParamter = {fx,fy,cx,cy}
void getCameraMatrixForLighthouse(double * cameraParamter);
//double * cameraParamter = {fx,fy,cx,cy}
std::vector<double> azimuthTo2D(double azimuth, double elevation, double * cameraParamter);

//calculates the mean value of N data points
double calcMean(const double * data_, uint8_t N);
double calcMean(const std::vector<double> data_);

//calculates the standardDeviation return (sqrt(abweichung^2))
double standardDeviation(const double * data_, double mean_, uint8_t N);
double standardDeviation(const std::vector<double> data_, double mean_);

#endif
