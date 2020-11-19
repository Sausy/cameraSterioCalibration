#ifndef __INTERFACE_HTCDONGLE_H__
#define __INTERFACE_HTCDONGLE_H__

#define MAX_SENSORS 24

#include "rayFinder.h"
#include "driver_htcDongle.h"
#include "parser_htcDongle.h"

//#include "rayFinder.h"
//#include "parser_htcDongle.h"
//#include "driver_htcDongle.h"

class driverHtcDongle
{
  public:
    driverHtcDongle();
    viveParse dataParser;

    bool pullData(std::vector<int> *id, std::vector<float> *azimuth, std::vector<float> *elevation, std::vector<int> *channel);

  private:
    double first_phi[MAX_SENSORS][32];
    //double ray[2][3];
};

#endif
