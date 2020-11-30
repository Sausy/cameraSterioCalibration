/* Linux */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

/*udev*/
//#include <libudev.h>

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <unistd.h>

#include "driver_htcDongle.h"

/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 */
#ifndef HIDIOCSFEATURE
#warning Please have your distro update the userspace kernel headers
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

static int hid_fd;

int HID_init(char * device_interface){
	/*
	struct udev* udev = udev_new();
	if(!udev){
		perror("udev new faild");
		return 1;
	}

	enumerate_devices
	*/

	//int fd;
	int i, res, desc_size = 0;
	uint8_t buf[256];
	struct hidraw_report_descriptor rpt_desc;
	struct hidraw_devinfo info;
	char *device =  device_interface;//"/dev/hidraw1";


	/* Open the Device with non-blocking reads. In real life,
	   don't use a hard coded path; use libudev instead. */
	hid_fd = open(device, O_RDWR|O_NONBLOCK);

	if (hid_fd < 0) {
		perror("Unable to open device");
		return 1;
	}

	memset(&rpt_desc, 0x0, sizeof(rpt_desc));
	memset(&info, 0x0, sizeof(info));
	memset(buf, 0x0, sizeof(buf));

	/* Get Report Descriptor Size */
	res = ioctl(hid_fd, HIDIOCGRDESCSIZE, &desc_size);
	if (res < 0)
		perror("HIDIOCGRDESCSIZE");
	else
		printf("Report Descriptor Size: %d\n", desc_size);

	/* Get Report Descriptor */
	rpt_desc.size = desc_size;
	res = ioctl(hid_fd, HIDIOCGRDESC, &rpt_desc);
	if (res < 0) {
		perror("HIDIOCGRDESC");
	} else {
		printf("Report Descriptor:\n");
		for (i = 0; i < rpt_desc.size; i++)
			printf("%hhx ", rpt_desc.value[i]);
		puts("\n");
	}

	/* Get Raw Name */
	res = ioctl(hid_fd, HIDIOCGRAWNAME(256), buf);
	if (res < 0)
		perror("HIDIOCGRAWNAME");
	else
		printf("Raw Name: %s\n", buf);

	/* Get Physical Location */
	res = ioctl(hid_fd, HIDIOCGRAWPHYS(256), buf);
	if (res < 0)
		perror("HIDIOCGRAWPHYS");
	else
		printf("Raw Phys: %s\n", buf);

	/* Get Raw Info */
	res = ioctl(hid_fd, HIDIOCGRAWINFO, &info);
	if (res < 0) {
		perror("HIDIOCGRAWINFO");
	} else {
		printf("Raw Info:\n");
		//printf("\tbustype: %d (%s)\n",
		//	info.bustype, bus_str(info.bustype));
		printf("\tvendor: 0x%04hx\n", info.vendor);
		printf("\tproduct: 0x%04hx\n", info.product);
	}

  return 0;
}





int HID_send_config(){

  int res;
  uint8_t buf[256];
  int j;
  int i;

  /* Set Feature */
	buf[0] = 0x9; /* Report Number */
	buf[1] = 0xff;
	buf[2] = 0xff;
	buf[3] = 0xff;
  uint8_t txData[] = {\
    0xff,0x96,0x10,0xbe,0x5b,0x32,0x54,0x11,0xcf,0x83,0x75,0x53,0x8a,0x08,0x6a,0x53,\
    0x58,0xd0,0xb1,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 \
  };

	res = ioctl(hid_fd, HIDIOCSFEATURE(0x40), txData);
	if (res < 0)
		perror("HIDIOCSFEATURE");
	else
		printf("ioctl HIDIOCSFEATURE returned: %d\n", res);


  //get the firmwar version of the vive dongle
  //this sometimes takes ages ... not sure why though

  int FirmwareVers = 0;

  for(j = 0; j < 90; j++){
	   /* Get Feature */
	    buf[0] = 0x5; /* Report Number */
	    res = ioctl(hid_fd, HIDIOCGFEATURE(0x41), buf);
	    if (res < 0) {
		        //perror("HIDIOCGFEATURE");
						;
	    } else {
	          printf("ioctl HIDIOCGFEATURE returned: %d\n", res);
            printf("Sucess ... after %d\n", j);
		        for (i = 0; i < res; i++)
			         printf("%hhx ", buf[i]);


						for (i = 0; i < 4; i++){
							FirmwareVers = (FirmwareVers<<8) | (buf[4-i] & 0x00ff);
						}

						printf("\n\n=========\nFirmware Version: %d",FirmwareVers);

						printf(" ");


						for (i = 8*1; i < 8*2; i++)
							printf("%c", buf[i]);

						printf("@");

						for (i = 8*2; i < res-(2*8)-4; i++)
							printf("%c", buf[i]);

						printf("\n\n");

            break;

		        puts("\n");
	    }
  }


  uint8_t txData2[] = {\
    0xff,0x87,0x06,0x01,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 \
  };

  res = -1;
  while(res < 0){
	   res = ioctl(hid_fd, HIDIOCSFEATURE(0x40), txData2);
  }
	//if (res < 0)
	//	perror("HIDIOCSFEATURE");
	//else
	//	printf("ioctl HIDIOCSFEATURE returned: %d\n", res);


  uint8_t txData3[] = {\
    0xff,0x83,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 \
  };
  res = -1;
  while(res < 0){
	   res = ioctl(hid_fd, HIDIOCSFEATURE(0x40), txData3);
  }

  //res = ioctl(hid_fd, HIDIOCSFEATURE(0x40), txData3);
	//if (res < 0)
	//	perror("HIDIOCSFEATURE");
	//else
	//	printf("ioctl HIDIOCSFEATURE returned: %d\n", res);

  buf[0] = 0xff; /* Report Number */
  //res = ioctl(hid_fd, HIDIOCGFEATURE(0x41), buf);

  res = -1;
  while(res < 0){
	   res = ioctl(hid_fd, HIDIOCGFEATURE(0x41), buf); //Should also return Firmware Version
  }

  buf[0] = 0x05; /* Report Number */
  //res = ioctl(hid_fd, HIDIOCGFEATURE(0x41), buf);

  res = -1;
  while(res < 0){
	   res = ioctl(hid_fd, HIDIOCGFEATURE(0x41), buf);
  }



	//=======

	uint8_t txData4[] = {\
    0xff,0x87,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 \
  };
	res = -1;
  while(res < 0){
	   res = ioctl(hid_fd, HIDIOCSFEATURE(0x40), txData4);
  }

	//=======

	buf[0] = 0x01; /* Report Number */
  //res = ioctl(hid_fd, HIDIOCGFEATURE(0x41), buf);

  res = -1;
  while(res < 0){
	   res = ioctl(hid_fd, HIDIOCGFEATURE(0x41), buf);
  }

	//=======

	buf[0] = 0x10; /* Report Number */
  //res = ioctl(hid_fd, HIDIOCGFEATURE(0x41), buf);

  res = -1;
  while(res < 0){
	   res = ioctl(hid_fd, HIDIOCGFEATURE(0x41), buf);
		 usleep(100);
		 //asm("nop");
  }


	usleep(1000);

	//ff 87 06 01 00 00 02
	uint8_t txData5[] = {\
    0xff,0x87,0x06,0x01,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 \
  };
	res = -1;
  while(res < 0){
	   res = ioctl(hid_fd, HIDIOCSFEATURE(0x40), txData5);
  }
	usleep(1);


	//ff 87 06 01 01 00 02
	uint8_t txData6[] = {\
		0xff,0x87,0x06,0x01,0x01,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 \
  };

	for (uint8_t i = 0; i < 2; i++) {
		res = -1;
	  while(res < 0){
		   res = ioctl(hid_fd, HIDIOCSFEATURE(0x40), txData6);
	  }

	}

  return 0;

}



int HID_read(unsigned char * buffer_){
  int res;
  unsigned char *buffer = buffer_;

  res = read(hid_fd, buffer, 59);

  return res;
}

/*
int main(int argc, char const *argv[]) {
  init();
  return 0;
}
*/
