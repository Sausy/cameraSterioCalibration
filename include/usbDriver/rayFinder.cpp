#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <math.h>

#include "rayFinder.h"


void vecCros(double *A, double *B, double *retVec){

  retVec[0] = A[1]*B[2] - A[2]*B[1];
  retVec[1] = A[2]*B[0] - A[0]*B[2];
  retVec[2] = A[0]*B[1] - A[1]*B[0];

}
void matrixDotVector(double  A[][4], double *B, uint8_t m, double *retMatrix){
  double valBuffer = 0.0;
  double multBuffer = 0.0;


  for (uint8_t colCnt = 0; colCnt < 1; colCnt++) {
    for (uint8_t rowCnt = 0; rowCnt < 4; rowCnt++) {
      for (uint8_t i = 0; i < 4; i++) {
        multBuffer = A[rowCnt][i] * B[i];

        valBuffer +=multBuffer;
        //printf("\nA*B=multBuffer\n%f * %f = %f\n=%f",A[rowCnt][i],B[i],multBuffer,valBuffer);
      }
      retMatrix[rowCnt] = valBuffer;
      //printf("\n...multBuffer [%u,%u] = %f\n",colCnt,rowCnt,multBuffer);

      multBuffer=0;
      valBuffer =0;
    }

  }
}

void matrixDotMatrix(double A[][4], double B[][4], uint8_t m, uint8_t n, double retMatrix[][4]){

    double valBuffer = 0;
    double multBuffer = 0;
    for (uint8_t colCnt = 0; colCnt < n; colCnt++) {
      for (uint8_t rowCnt = 0; rowCnt < m; rowCnt++) {
        for (uint8_t i = 0; i < n; i++) {
          multBuffer = A[rowCnt][i] * B[i][colCnt];
          valBuffer +=multBuffer;
        }
        retMatrix[rowCnt][colCnt] = valBuffer;
        multBuffer=0;
        valBuffer =0;
      }

    }

}

void vec2azimuth(double (*ray)[2][3], double *az, double *ele){
    double norm = pow((*ray)[1][0],2) + pow((*ray)[1][1],2) + pow((*ray)[1][2],2);
    norm = sqrt(norm);

    double x = (*ray)[1][0]/norm;
    double y = (*ray)[1][1]/norm;
    double z = (*ray)[1][2]/norm;

    //printf("\nVec [%f,%f,%f]-%f", (*ray)[1][0],(*ray)[1][1],(*ray)[1][2], norm);
    //printf("\nNormVec [%f,%f,%f]-%f", x,y,z, norm);

    //because we normed the vector
    //r has to be 1
    //sqrt(pow(x,2)+pow(y,2)+pow(z),2)) = 1
    double r = 1;

    //in this case the azimuth
    //describes the angle betwenn x and -z
    //*az = atan(y/x);
    *az = atan2(-z,x);

    //in this case the elevation
    //describes the angle betwenn r_{x,-z} and y
    //*ele = atan(z/r);
    double r_xz = sqrt(pow(x,2)+pow(z,2));
    //its unliklely that r_xz will be equal to 0
    *ele = atan(y/r_xz);
}

void baseRotation(double phi, double rotMatrix_y[][4]){
  //double * r_vec = r_vec_;

  phi = -phi;

  rotMatrix_y[0][0] = (double)cos(phi);
  rotMatrix_y[0][1] = 0.0;
  rotMatrix_y[0][2] = (double)sin(phi);
  rotMatrix_y[0][3] = 0.0;

  rotMatrix_y[1][0] = 0.0;
  rotMatrix_y[1][1] = 1.0;
  rotMatrix_y[1][2] = 0.0;
  rotMatrix_y[1][3] = 0.0;

  rotMatrix_y[2][0] = -sin(phi);
  rotMatrix_y[2][1] = 0.0;
  rotMatrix_y[2][2] = (double)cos(phi);
  rotMatrix_y[2][3] = 0.0;

  rotMatrix_y[3][0] = 0.0;
  rotMatrix_y[3][1] = 0.0;
  rotMatrix_y[3][2] = 0.0;
  rotMatrix_y[3][3] = 1.0;
}

//The default vector Tranformation
//transformer for BASE SWEEP 1
#define SINPI4 0.7071067812
#define COSPI4 0.7071067812
void coord_transform_1(double d, double phi, double rotAngle_rad, bool isNeg, bool isSecondSweep, double T_inv[][4]){ //isNeg = False
    //TODO FOV into this shi
    double trans_x = 0.0; //#-1 #* np.sin(rotAngle_rad) * d
    double trans_y = 0.0;

    if(isSecondSweep == true){
      //trans_y = -sin(rotAngle_rad) * d;
      trans_y = - SINPI4 * d;
    }else{
      //trans_y = sin(rotAngle_rad) * d;
      trans_y = SINPI4 * d;
    }
    //double trans_z = cos(rotAngle_rad) * d;
    double trans_z = COSPI4 * d;


    /*
    rotation is counter clockwise
    */
    phi = -phi;

    if (isNeg == true) {
        trans_y = -1.0 * trans_y;
        trans_z = -1.0 * trans_z;
    }

    double TransposMatrix[4][4] = {0};

    TransposMatrix[0][0] = 1;
    TransposMatrix[0][1] = 0;
    TransposMatrix[0][2] = 0;
    TransposMatrix[0][3] = trans_x;

    TransposMatrix[1][0] = 0;
    TransposMatrix[1][1] = 1;
    TransposMatrix[1][2] = 0;
    TransposMatrix[1][3] = trans_y;

    TransposMatrix[2][0] = 0;
    TransposMatrix[2][1] = 0;
    TransposMatrix[2][2] = 1;
    TransposMatrix[2][3] = trans_z;

    TransposMatrix[3][0] = 0;
    TransposMatrix[3][1] = 0;
    TransposMatrix[3][2] = 0;
    TransposMatrix[3][3] = 1;

    /*
    TransposMatrix =   [[1, 0, 0, trans_x],
                        [0, 1, 0 ,trans_y],
                        [0, 0, 1 ,trans_z],
                        [0, 0, 0, 1]]
                        */

    double rotMatrix_y[4][4] = {0};

    rotMatrix_y[0][0] = cos(phi);
    rotMatrix_y[0][1] = 0;
    rotMatrix_y[0][2] = sin(phi);
    rotMatrix_y[0][3] = 0;

    rotMatrix_y[1][0] = 0;
    rotMatrix_y[1][1] = 1;
    rotMatrix_y[1][2] = 0;
    rotMatrix_y[1][3] = 0;

    rotMatrix_y[2][0] = -sin(phi);
    rotMatrix_y[2][1] = 0;
    rotMatrix_y[2][2] = cos(phi);
    rotMatrix_y[2][3] = 0;

    rotMatrix_y[3][0] = 0;
    rotMatrix_y[3][1] = 0;
    rotMatrix_y[3][2] = 0;
    rotMatrix_y[3][3] = 1;

    /*
    rotMatrix_y =      [[np.cos(phi), 0, np.sin(phi), 0],
                        [0, 1, 0, 0],
                        [-np.sin(phi), 0, np.cos(phi), 0],
                        [0, 0, 0, 1]]
                        */

    //T = np.dot(TransposMatrix,rotMatrix_y)
    //T_inv = np.dot(rotMatrix_y, TransposMatrix)
    matrixDotMatrix(rotMatrix_y,TransposMatrix,4,4,T_inv);

    //return [T, T_inv]
}



void plane_transformation(double phi, double * default_vec_, bool sweep, double ret[][4]){
  double * default_vec = default_vec_;

  double T_inv[4][4] = {0};
  //from FRAME 0
  //=== rotate the default point (-1, 0, 0)
  baseRotation(phi, T_inv);
  /*
  for(uint8_t j = 0; j < 4; j++){
      for(uint8_t i = 0; i < 4; i++)
        printf("T_inv [%u,%u] = %g\n",j,i, T_inv[j][i]);
  }
  */

  double r_0[4] = {0,0,0,1};
  double r_0_new[4] = {0,0,0,1};
  double r_0_vec[4] = {0};
  matrixDotVector(T_inv, default_vec, 4,r_0_vec);

  //for(uint8_t j = 0; j < 4; j++){
  //    printf("r_0_vec [%u] = %g\n",j, r_0_vec[j]);
  ///}
  //r_0_new = r_0
  //r_0_vec = np.dot(T_inv,default_vec)


  //from FRAME 1
  //=== rotate the default point (-1, 0, 0)
  if (sweep == 0){
    coord_transform_1(1, phi, M_PI/4.0, false, false, T_inv);
  }else{
    coord_transform_1(1, phi, M_PI/4.0, false, true, T_inv);
  }
  double r_1[4] = {0,0,0,1};
  double r_1_new[4] = {0};
  double r_1_vec[4] = {0};

  matrixDotVector(T_inv, r_1, 4,r_1_new);
  matrixDotVector(T_inv, default_vec, 4,r_1_vec);

  /*
  for(uint8_t j = 0; j < 4; j++){
      printf("r_1_new [%u] = %g\n",j, r_1_new[j]);
  }
  for(uint8_t j = 0; j < 4; j++){
      printf("r_1_vec [%u] = %g\n",j, r_1_vec[j]);
  }
  printf("\n");
  */

  //from FRAME 2
  //=== rotate the default point (-1, 0, 0)
  if (sweep == 0){
    coord_transform_1(1, phi, M_PI/4.0, true, false, T_inv);
  }else{
    coord_transform_1(1, phi, M_PI/4.0, true, true, T_inv);
  }

  double r_2[4] = {0,0,0,1};
  double r_2_new[4] = {0};
  double r_2_vec[4] = {0};

  matrixDotVector(T_inv, r_2, 4,r_2_new);
  matrixDotVector(T_inv, default_vec, 4,r_2_vec);

  ret[0][0] = r_0_vec[0];
  ret[0][1] = r_0_vec[1];
  ret[0][2] = r_0_vec[2];
  ret[0][3] = r_0_vec[3];

  ret[1][0] = r_1_new[0];
  ret[1][1] = r_1_new[1];
  ret[1][2] = r_1_new[2];
  ret[1][3] = r_1_new[3];

  ret[2][0] = r_1_vec[0];
  ret[2][1] = r_1_vec[1];
  ret[2][2] = r_1_vec[2];
  ret[2][3] = r_1_vec[3];

  ret[3][0] = r_2_new[0];
  ret[3][1] = r_2_new[1];
  ret[3][2] = r_2_new[2];
  ret[3][3] = r_2_new[3];

  ret[4][0] = r_2_vec[0];
  ret[4][1] = r_2_vec[1];
  ret[4][2] = r_2_vec[2];
  ret[4][3] = r_2_vec[3];
  //return [r_0_vec, r_1_new,r_1_vec,r_2_new,r_2_vec]

}


//void ray_calculation(double first_phi, double secon_phi, double ray_[2][3]){
void ray_calculation(double first_phi, double secon_phi, double (*ray_)[2][3]){

  //========
  //[120,240] seems to be the center of the device, ... was figured out empiricly
  //so 120 - 90 = 30 ,,,, 240-90 = 150
  //========
  //printf("\nfirst_phi,secon_phi [%f,%f]\n",first_phi,secon_phi);
  first_phi = first_phi - (150.0 * M_PI/180.0);
  secon_phi = secon_phi - (30.0 * M_PI/180.0);

  //printf("first_phi,secon_phi [%f,%f]\n",first_phi,secon_phi);
  //double **ray = r
  double rZero[4] = {0,0,0,1};
  double default_vec[4] = {-1,0,0,1};

  double r_[5][4] = {0};

  plane_transformation(first_phi,default_vec, false, r_);

  /*
  for(uint8_t j = 0; j < 4; j++){
    for(uint8_t i = 0; i < 3; i++)
      printf("r_ [%u,%u] = %g\n",j,i, r_[j][i]);
  }
  printf("\n");
  */

  double p0[3] = {0,0,0};
  double p1[3] = {r_[2][0], r_[2][1], r_[2][2]};
  double p2[3] = {r_[3][0], r_[3][1], r_[3][2]};//r_[3][0:3];
  double p3[3] = {r_[4][0], r_[4][1], r_[4][2]};//r_[4][0:3];

  plane_transformation(secon_phi,default_vec, true, r_);

  /*
  for(uint8_t j = 0; j < 4; j++){
    for(uint8_t i = 0; i < 3; i++)
      printf("r_ [%u,%u] = %g\n",j,i, r_[j][i]);
  }
  */


  double p12[3] = {r_[2][0], r_[2][1], r_[2][2]};//r_[2][0:3]
  double p22[3] = {r_[3][0], r_[3][1], r_[3][2]};//r_[3][0:3]
  double p32[3] = {r_[4][0], r_[4][1], r_[4][2]};//r_[4][0:3]

  //The following stuff is proberly described in the documentation
  //basicly its the default formular for plane intersection

  for (size_t i = 0; i < 3; i++){
    p2[i] = p2[i] - p1[i];
    p3[i] = p3[i] - p1[i];
    p22[i] = p22[i] - p12[i];
    p32[i] = p32[i] - p12[i];
  }

  //First we get a "normale" for eache vector
  double n1[3] = {0};
  double n2[3] = {0};
  double n[3] = {0};
  //n1 = np.cross((p2 - p1),(p3 - p1))
  vecCros(p2,p3,n1);
  //n2 = np.cross((p22 - p12),(p32 - p12))
  vecCros(p22,p32,n2);
  //now according to the equation
  //(a0-a1)x + (b0-b1)y + (c0-c1)z + d = 0
  //where d is always zero in our case because
  //(0,0,0) is always a member of each plane
  for (size_t i = 0; i < 3; i++){//n = n1 - n2
    n[i] = n1[i] - n2[i];
  }


  //now we assume that due to hardware limitation
  //the only valid z > 1
  //hence it can be assumed
  // z = 1

  // TODO: FIGURE OUT WHY DAFUQ IT HAS TO -1
  double z_assumed = -1.0;

  //due to the corelation between the two intersecting planes
  //(b0 - b1) has to be 0
  //solve the equation (a0-a1)x  + (c0-c1)z  = 0
  //for x
  double x = -(z_assumed * n[2])/n[0];

  // instert this equation into one of the plane equations
  // can eather be plane 1 or plane 2
  //a0*x + b0*y + c0*z = 0
  double y = -(n1[0]*x + n1[2]*z_assumed)/n1[1];

  (*ray_)[0][0] = 0;
  (*ray_)[0][1] = 0;
  (*ray_)[0][2] = 0;

  (*ray_)[1][0] = x;
  (*ray_)[1][1] = y;
  (*ray_)[1][2] = z_assumed;

  /*
  ray =  [[0, x],
          [0, y],
          [0, z_assumed]]
  return ray
  */
}
