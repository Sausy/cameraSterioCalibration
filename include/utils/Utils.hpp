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

#endif
