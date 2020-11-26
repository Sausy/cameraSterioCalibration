#pragma once

#define MAX_SENSORS 35

typedef struct lightV2{
  uint8_t id;
  uint8_t channel;

  uint32_t TimeStamp;
  int32_t TimeDiv;
}lightV2;


class viveParse
{
  public:
    viveParse();
    bool parseIMU();
    bool parseLightV2(uint8_t * data_, bool cleanCall);

    uint8_t buffer[128];
    uint8_t *rawData;

    float imu[9];

    lightV2 lightData[MAX_SENSORS];
    uint32_t syncTimePerChannel[32];
    uint8_t pollLength;

    bool usbHandl();

};
