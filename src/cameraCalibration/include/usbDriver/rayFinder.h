#define MIN_AZIMUTH_ANGLE 25.0 //in deg
#define MAX_AZIMUTH_ANGLE ( 180.0 + 30.0 - MIN_AZIMUTH_ANGLE) //in deg

#define MIN_AZIMUTH_ANGLE_RAD ( MIN_AZIMUTH_ANGLE * M_PI/180.0 ) //in deg
#define MAX_AZIMUTH_ANGLE_RAD ( MAX_AZIMUTH_ANGLE * M_PI/180.0 ) //in deg

#define MIN_ELEVATION_ANGLE ( -90 + MIN_AZIMUTH_ANGLE ) //in deg
#define MAX_ELEVATION_ANGLE ( 90 - MIN_AZIMUTH_ANGLE ) //in deg

#define MIN_ELEVATION_ANGLE_RAD ( MIN_ELEVATION_ANGLE * M_PI/180.0 ) //in deg
#define MAX_ELEVATION_ANGLE_RAD ( MAX_ELEVATION_ANGLE * M_PI/180.0 ) //in deg

void vecCros(double *A, double *B, double *retVec);

void matrixDotVector(double A[][4], double *B, uint8_t m, double *retMatrix);
void matrixDotMatrix(double A[][4], double B[][4], uint8_t m, uint8_t n, double retMatrix[][3]);
void vec2azimuth(double (*ray)[2][3], double *az, double *ele);

void baseRotation(double phi, double rotMatrix_y[][4]);

//The default vector Tranformation
//transformer for BASE SWEEP 1
void coord_transform_1(double d, double phi, double rotAngle_rad, bool isNeg, bool isSecondSweep, double T_inv[][4]);

void plane_transformation(double phi, double * default_vec_, bool sweep, double ret[][4]);

//template <size_t sx, size_t sy>
void ray_calculation(double first_phi, double secon_phi, double (*ray_)[2][3]);
