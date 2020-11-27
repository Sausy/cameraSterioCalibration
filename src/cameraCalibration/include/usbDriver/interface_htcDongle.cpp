#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include <vector>

#include <math.h>

#include "interface_htcDongle.h"

driverHtcDongle::driverHtcDongle(){
    //TODO:BIG TODO
    //automate the path search for the htc usb device
    //this shouldn't be hardcoded .. could lead to huge fuck ups
    (void)HID_init("/dev/hidraw2");
    (void)HID_send_config();
}

bool driverHtcDongle::pullData(std::vector<rawRayData> *ray_Data){
  std::vector<rawRayData> r;// = ray_Data;

  std::vector<int> id_;
  std::vector<float> azimuth_;
  std::vector<float> elevation_;
  std::vector<int> channel_;

  bool ret = pullData(&id_, &azimuth_, &elevation_, &channel_);

  if(id_.size() <= 0 || ret == false)
    return false;

  for (size_t i = 0; i < id_.size(); i++) {
    rawRayData ret_data;
    ret_data.id = id_[i];
    ret_data.ch = channel_[i];
    ret_data.azimuth = azimuth_[i];
    ret_data.elevation = elevation_[i];
    r.push_back(ret_data);
  }

  //ray_Data->clear();
  ray_Data->insert(ray_Data->begin(), r.begin(), r.end());

  return (ret);
}

bool driverHtcDongle::pullData(std::vector<int> *id, std::vector<float> *azimuth, std::vector<float> *elevation, std::vector<int> *channel){
  int res = HID_read(&dataParser.buffer[0]);

  double az_ = 0.0;
  double ele_ = 0.0;

  double ray[2][3] = {0};

  double PERIODS[] = {959000, 957000,
         953000, 949000,
         947000, 943000,
         941000, 939000,
         937000, 929000,
         919000, 911000,
         907000, 901000,
         893000, 887000};

  if(res != -1){
      //process raw data
      if(dataParser.usbHandl()){
          lightV2 * le = &dataParser.lightData[0];

          for (uint8_t i = 0; i < dataParser.pollLength; i++) {
            //check if data is valid
            if(le->TimeDiv < 1000000){
                double angle_buffer = (double)le->TimeDiv/PERIODS[le->channel]; //959000.0;
                double angle_rad = angle_buffer * 2 * M_PI;
                double angle  = angle_buffer * 360.0;

                //TODO:
                // angle 180 is not sofisticated enough ... because both angle can be under 180
                if(angle > 180){
                  ray_calculation(angle_rad, first_phi[le->id][le->channel], &ray);
                  //std::cout<< "\n " << angle_rad << "+" << first_phi[le->id][le->channel] ;
                  //std::cout<< "| " << ray[1][0];
                  //std::cout<< " " << ray[1][1];
                  //std::cout<< " " << ray[1][2];
                  vec2azimuth(&ray, &az_, &ele_);

                  //filter out obvious wrong data
                  if((az_ < MAX_AZIMUTH_ANGLE_RAD) && (az_ > MIN_AZIMUTH_ANGLE_RAD) && (ele_ < MAX_ELEVATION_ANGLE_RAD) && (ele_ > MIN_ELEVATION_ANGLE_RAD)){
                    id->push_back(le->id);
                    azimuth->push_back((float)az_);
                    elevation->push_back((float)ele_);
                    channel->push_back(le->channel);
                  }
                  //printf("\n[%u=%u]%f/%f\t%f\t%f",le->id,le->channel,az_*180.0/M_PI, ele_*180.0/M_PI,angle, first_phi[le->id][le->channel]*180.0/M_PI);


                }else{
                  //angle_old[le->id][le->channel] =  angle;
                  if(angle > 10.0 && angle < 210.0){
                    first_phi[le->id][le->channel] = angle_rad;
                  }


                }
              }
              le++;
          }
          return true;
      }
  }

  return false;
}
