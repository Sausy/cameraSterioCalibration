#ifndef __DATATYPE_HTCDONGLE_H__
#define __DATATYPE_HTCDONGLE_H__

#define MAX_RAY_HISTORY_SIZE 256

typedef struct rawRayData{
  int ch; //max size 16
  int id;
  float azimuth;
  float elevation;

  float azMean;
  float azVar;

  float elMean;
  float elVar;
}rawRayData;

#endif
