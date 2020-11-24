#ifndef __USBHID_INTERFACE_H__
#define __USBHID_INTERFACE_H__

int HID_init(char * device_interface);
int HID_send_config();
int HID_read(unsigned char * buffer_);


#endif
