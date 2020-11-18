#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include <vector>

#include <math.h>

#include "rayFinder.h"
#include "parser_htcDongle.h"
#include "driver_htcDongle.h"

#include "interface_htcDongle.h"

driverHtcDongle::driverHtcDongle(){
    //TODO:BIG TODO
    //automate the path search for the htc usb device
    //this shouldn't be hardcoded .. could lead to huge fuck ups
    (void)HID_init("/dev/hidraw1");
    (void)HID_send_config();
}

bool driverHtcDongle::pullData(std::vector<int> *id, std::vector<float> *azimuth, std::vector<float> *elevation, std::vector<int> *channel){
  int res = HID_read(&dataParser.buffer[0]);

  double az_ = 0.0;
  double ele_ = 0.0;


  if(res != -1){
      //process raw data
      if(dataParser.usbHandl()){
          lightV2 * le = &dataParser.lightData[0];

          for (uint8_t i = 0; i < dataParser.pollLength; i++) {
              //Todo sometimes the timeDiv values are way to big
              double angle_buffer = (double)le->TimeDiv/959000.0;
              double angle_rad = angle_buffer * 2 * M_PI;
              double angle  = angle_buffer * 360.0;

              //TODO:
              // angle 180 is not sofisticated enough ... because both angle can be under 180
              if(angle > 180){
                ray_calculation(angle_rad, first_phi[le->id][le->channel], &ray);
                vec2azimuth(&ray, &az_, &ele_);

                id->push_back(le->id);
                azimuth->push_back((float)az_);
                elevation->push_back((float)ele_);
                channel->push_back(le->channel);
              }
          }
      }
  }
}
