#ifndef UTILS_H_
#define UTILS_H_


//double * cameraParamter = {fx,fy,cx,cy}
void getCameraMatrixForLighthouse(double * cameraParamter);
//double * cameraParamter = {fx,fy,cx,cy}
std::vector<double> azimuthTo2D(double azimuth, double elevation, double * cameraParamter);

void converRotMatrixToEuler(double A[3][3], double &retRol, double &retPitch, double &retYaw);


#endif
