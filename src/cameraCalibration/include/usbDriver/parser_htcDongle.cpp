#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "parser_htcDongle.h"

viveParse::viveParse(){
  rawData = &buffer[0];
}


/*=====================================================================
*   Parsing IMU from raw Usb Data
*
*   IMU Data (from Gen2 Hardware Dev Kit HDK)
*   Flag = 0x80 0x??
*   ╔═════════════╦═════════╦═══════╦═══════╦══════╦══════╦════
*   ║Identifier   ║Time     ║Length ║ Time  ║ Flag ║ ??   ║ Data..
*   ╚═════════════╩═════════╩═══════╩═══════╩══════╩══════╩════
*
*    DATA :
*
*   | ACCELERATION(6B) | Gyroscope(6Byte)    | No Clue
*   ╔══════════════════╦═════════════════════╦═══════════
*   ║ aXaX aYaY aZaZ   ║ gXgX gYgY gZgZ      ║  ...?????...
*   ╚══════════════════╩═════════════════════╩════════════
*
*   The raw data has a One's complement
*   and is Scaled with the factor of
*   ~417 (I mean 420 would make more sense humor wise)
*/
bool viveParse::parseIMU(){
	uint8_t * data = rawData;
	float * imuData = &imu[0];

	uint8_t rawbuffer[2] = {0};
	int16_t buffer_ = 0x0;

	//the first gyro data should start at point 6
	data = data + 6;

	for(uint8_t i = 0; i < 6; i++){
		rawbuffer[1] = *data & 0xff;
		data++;
		rawbuffer[0] = *data & 0xff;
		data++;

		buffer_ = ((rawbuffer[0] & 0x00ff) << 8) | (rawbuffer[1] & 0x00ff);

		//detection if data is negativ and if so applie Ones' complement
		if((buffer_ & 0x8000) != 0){
			buffer_ = ~buffer_;
			buffer_ = buffer_ & 0xffff; //because we only have 16 bit
			buffer_ = buffer_ * (-1);

		}

		//TO be honest i Have no clue why 417
		//420 makes more sense xD
		*imuData = (float)buffer_/417.0;
		//printf("\n[%d]: %f | %x:%x = %d",i,*imuData, rawbuffer[0], rawbuffer[1], buffer_);
		imuData++;

	}

  return true;
}
/*=====================================================================*/

bool viveParse::usbHandl(){
  uint8_t * data = rawData;

  uint8_t maxLoop = 0;

  if(data[0] == 0x23){
		maxLoop = 1;
  }else if(data[0] == 0x24){
    maxLoop = 2;
  }else {
    return false;
  }

  uint8_t dataIterator = 0;
  bool lightFound = false;
  pollLength = 0;
  for (uint8_t i = 0; i < maxLoop; i++) {
    if(parseLightV2(&data[dataIterator],false)){
      lightFound = true;
    }else{
      ;//Todo pars IMU Stuff
    }
    dataIterator = 29;
  }

  if(!lightFound)
    return false;

  return true;
}

bool viveParse::parseLightV2(uint8_t * data_, bool cleanCall){

  if(cleanCall)
    pollLength = 0;

  uint8_t * data = data_;
  lightV2 * le = &lightData[0];


  if ((data[4] & 0xff) != 0x10)
		return false;


	uint8_t ident = data[0];
	//uint16_t time = (data[3] << 8) | data[1];
	uint8_t msgLength = data[2];
	//uint16_t flag = (data[4] << 8) | (data[5] & 0x00ff);
  uint8_t flag = (data[4] & 0xff);

	uint8_t dataLength = msgLength - 3; //-3 (timeLSB, Flag1, Flag2)

	//uint32_t lightData[24] = {0};
	//uint32_t syncData = {0};

	int32_t timeDelta = 0;
	double angle = 0.0;

	double printTime = 0.0;

  //printf("\n");
  uint8_t channel = (data[5] >> 4) & 0x0f;


	for (uint8_t i = 0; i < dataLength; i=i+4) {
		uint8_t sensorID = data[9+i] >> 3;
		uint32_t upperData = 0u;
		upperData |= (data[7+i] << 8) | (data[6+i]);
		upperData |= ((data[8+i] & 0xff) << 16);
		upperData |= ((data[9+i] & 0x03) << 24);



		//if SyncFlag is set
		bool sync = ((data[6+i] >> 1) & 0x01);
		if(sync == 0){

      bool ootx = ((data[9+i] >> 2) & 0x01);
      bool g = sensorID & 1u;


      //if sync we only shift 2 times due to ootx Flag
			upperData = upperData >> 2; //shift away the Flag data
			upperData = upperData & 0xFFFFFFu;


      syncTimePerChannel[channel] = upperData;

			//printf("\n[V2][SYNC]%u/ %u | %f .. OOTX:%u | g: %u | %u",channel, upperData, printTime, ootx, g, sensorID);
		}else{
      if(sensorID < MAX_SENSORS && channel < 32){
          le->channel = channel;
          le->id = sensorID;

          bool half_clock_flag = data[9+i] & 0x4u;
          upperData = upperData >> 3; //shift away the Flag data
          upperData = upperData & 0xFFFFFFu;

          le->TimeStamp = upperData;


          //if(syncData > upperData){
          if(syncTimePerChannel[channel] > upperData){
            le->TimeDiv =  0x800000 - syncTimePerChannel[channel];//timeDelta = 0x800000 - syncData;
            //if(syncTimePerChannel[channel] > 0x800000){
            //  printf("\n[ERROR] %u", syncTimePerChannel[channel]);
            //}
            le->TimeDiv =  upperData + le->TimeDiv;//timeDelta = upperData + timeDelta;
          }else{
            le->TimeDiv = upperData - syncTimePerChannel[channel];//timeDelta = upperData - syncData;
          }

          angle = (double)le->TimeDiv/959000.0 * 360.0;

          //printf("\n[V2][%u]%u/ %u | %d | %f", sensorID, channel, upperData, le->TimeDiv, angle);
          pollLength++;
          le++;
      }

		}


	}
  return true;
}
