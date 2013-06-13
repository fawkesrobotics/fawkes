
/***************************************************************************
 *  kinova_api.cpp - Kinova API for libusb connection
 *
 *  Created: Tue Jun 04 13:13:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "kinova_api.h"
#include <core/exceptions/software.h>

#include <stdio.h>
#include <cstring>

//#include <libusb.h>

#define VENDOR_ID       0x22CD
#define PRODUCT_ID      0x0000

#define EP_IN           (2 | LIBUSB_ENDPOINT_IN)
#define EP_OUT          (2 | LIBUSB_ENDPOINT_OUT)
#define INTR_LENGTH	64

#define CMD_GET_CART_POS        104
#define CMD_GET_ANG_POS         105
#define CMD_START_API_CTRL      302
#define CMD_STOP_API_CTRL       303
#define CMD_SEND_BASIC_TRAJ     308

#define USB_CMD(ep,msg)         \
  (libusb_interrupt_transfer(__lusb_devh, ep, msg.data, INTR_LENGTH, &transferred, 1000))

#define USB_CMD_IN(msg)         (USB_CMD(EP_IN,msg))
#define USB_CMD_OUT(msg)        (USB_CMD(EP_OUT,msg))

#define USB_MSG(msg,pid,pquant,cmdid,cmdsize) { \
  msg.header.IdPacket = pid;                    \
  msg.header.PacketQuantity = pquant;           \
  msg.header.CommandId = cmdid;                 \
  msg.header.CommandSize = cmdsize;           }

/* send data with little endianness
 *
 * HEADER  8 Bytes
 * DATA   56 Bytes
 *
 * HEADER
 *      2 idPacket
 *      2 PacketQuantity
 *      2 CommandID
 *      2 CommandSize
 */
namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class JacoArm <plugins/kinova/kinova_api.h>
 * Class to control a Kinova Jaco arm
 * @author Bahram Maleki-Fard
 */

// Init statics
libusb_context* JacoArm::__lusb_ctx = NULL;

/** Constructor. */
JacoArm::JacoArm() :
  __lusb_devh( 0 )
{
  _init_libusb();

  _get_device_handle();

  _claim_interface();
}

/** Destructor. */
JacoArm::~JacoArm()
{
  printf("destroy usb handle ... ");
  if( __lusb_devh != NULL ) {
    libusb_release_interface(__lusb_devh, 0);
    printf("interface released ... now handle ... ");
    libusb_close(__lusb_devh);
    printf("DONE \n");
  } else {printf("NOT INITIALIZED \n");}

  printf("exit libusb ... ");
  if( __lusb_ctx != NULL ) {
    // Close libusb session
    libusb_exit(__lusb_ctx);
    printf("DONE \n");
  } else {printf("NOT INITIALIZED \n");}
}


/* /================================================\
 *   USB connection establishment (private)
 * \================================================/*/
void
JacoArm::_init_libusb()
{
  printf("check libusb ... ");
  // check libusb initialization
  if( __lusb_ctx == NULL ) {
    printf("init ... ");
    int r = libusb_init(&__lusb_ctx);
    if( r<0 ) {
      printf("FAILED \n");
      throw fawkes::Exception("Kinova_API: Could not init libusb! Error code: %i.", r);
    } else {printf("DONE \n");}
  } else {printf("INITIALIZED \n");}
}

void
JacoArm::_get_device_handle()
{
  if( __lusb_ctx == NULL )
    throw fawkes::Exception("Kinova_API: Could not get device handle, libusb not initialized yet.");

/*
  // print information on devices
  libusb_device **devs;
  libusb_device *dev;

  ssize_t cnt = libusb_get_device_list(__lusb_ctx, &devs);
  if( cnt<0 )
    throw fawkes::Exception("Kinova_API: No USB devices detected! Error code: %i", cnt);

  // print devices info
  int i = 0;
  while ((dev = devs[i++]) != NULL) {
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0)
      throw fawkes::Exception("Kinova_API: Failed to get device descriptor! Error code: %i", r);

    printf("idVendor:%04x  idProduct:%04x  SN:%02x (bus %d, device %d)\n",
    desc.idVendor, desc.idProduct, desc.iSerialNumber,
    libusb_get_bus_number(dev), libusb_get_device_address(dev));
  }

  // Clear devices list.
  libusb_free_device_list(devs, true);
//*/

  // Get handle by vendorId and productId
  printf("get usb handle ... ");
  __lusb_devh = libusb_open_device_with_vid_pid(__lusb_ctx, VENDOR_ID, PRODUCT_ID);
  if( !__lusb_devh ) {
    printf("FAILED \n");
    throw fawkes::Exception("Kinova_API: Failed get handle for Jaco arm!" );
  } else {printf("DONE \n"); }
}

void
JacoArm::_claim_interface()
{
  if( !__lusb_devh )
    throw fawkes::Exception("Kinova_API: Failed claiming interface, no handle for Jaco arm!" );

  printf("claim interface ... ");
  int r = libusb_claim_interface(__lusb_devh, 0);
  if( r<0 ) {
    printf("FAILED \n");
    throw fawkes::Exception("Kinova_API: Could not claim interface 0! Error code: %i.", r);
  } else {printf("DONE \n");}
}



/* /================================================\
 *   Generic USB data transfer commands (private)
 * \================================================/*/
/** Perform an outgoing and then ingoing command.*/
int
JacoArm::_cmd_out_in(message_t &msg, int cmd_size_in)
{
  int r, transferred;

  //printf("\n cmd_out_in send message: \n");
  //print_message(msg);
  r = USB_CMD(EP_OUT, msg);
  if( r < 0 ) {
    fprintf(stderr, "intr error %d\n", r);
    return r;
  } else if( transferred < INTR_LENGTH ) {
    fprintf(stderr, "short write (%d)\n", r);
    return -1;
  }
  //printf("sent interrupt, transferred %i \n", transferred);

  msg.header.CommandSize = cmd_size_in;

  //printf("\n cmd_out_in send modified message: \n");
  //print_message(msg);
  r = USB_CMD_IN(msg);
  if( r < 0 ) {
    fprintf(stderr, "intr error %d\n", r);
    return r;
  } else if( transferred < INTR_LENGTH ) {
    fprintf(stderr, "short read (%d)\n", r);
    return -1;
  }
  //printf("sent interrupt %04x, transfered %i \n", *((uint16_t *) msg.data), transferred);

  return 1;
}

int
JacoArm::_cmd_out(short cmd)
{
  message_t msg;
  USB_MSG(msg, 1, 1, cmd, 8)

  int r, transferred;
  r = USB_CMD(EP_OUT, msg);
  if( r < 0 ) {
    fprintf(stderr, "intr error %d\n", r);
  } else if( transferred < INTR_LENGTH ) {
    fprintf(stderr, "short write (%d)\n", r);
    r = -1;
  }
  return r;
}


/* /================================================\
 *   Jaco specific commands (private)
 * \================================================/*/
int
JacoArm::_get_cart_pos(position_cart_t &pos)
{
  message_t msg;
  USB_MSG(msg, 1, 1, CMD_GET_CART_POS, 8)

  int r = _cmd_out_in(msg, 36);
  if( r >= 0 )
    memcpy(&pos, msg.body, sizeof(pos));

  return r;
}

int
JacoArm::_get_ang_pos(position_ang_t &pos)
{
  message_t msg;
  USB_MSG(msg, 1, 1, CMD_GET_ANG_POS, 8)

  int r = _cmd_out_in(msg, 36);
  if( r >= 0 )
    memcpy(&pos, msg.body, sizeof(pos));

  return r;
}

int
JacoArm::_send_basic_traj(basic_traj_t &traj)
{
  message_t msg;
  USB_MSG(msg, 1, 1, CMD_SEND_BASIC_TRAJ, sizeof(traj))
  memcpy(msg.data, &traj, sizeof(traj));

  int r, transferred;
  r = USB_CMD(EP_OUT, msg);
  if( r < 0 ) {
    fprintf(stderr, "intr error %d\n", r);
  } else if( transferred < INTR_LENGTH ) {
    fprintf(stderr, "short write (%d)\n", r);
    r = -1;
  }
  return r;
}


/* /================================================\
 *   public methods
 * \================================================/*/
void
JacoArm::print_message(message_t &msg)
{
  message_header_t h = msg.header;
  float *b = msg.body;
  printf("header: %i  %i  %i  %i \n", h.IdPacket, h.PacketQuantity, h.CommandId, h.CommandSize);
  printf("body (7 rows, 8 colums, each entry 4 Byte float) \n");
  for(unsigned int i=0; i<7; ++i) {
    for(unsigned int j=0; j<8; ++j) {
      printf("%f   ", *b);
      ++b;
    }
    printf("\n");
  }
}

position_cart_t
JacoArm::get_cart_pos() {
  position_cart_t pos;
  int r = _get_cart_pos(pos);
  if( r < 0 ) {
    throw fawkes::Exception("Kinova_API: Could not get cartesian position! libusb error code: %i.", r);
  }

  return pos;
}

position_ang_t
JacoArm::get_ang_pos() {
  position_ang_t pos;
  int r = _get_ang_pos(pos);
  if( r < 0 ) {
    throw fawkes::Exception("Kinova_API: Could not get angular position! libusb error code: %i.", r);
  }

  return pos;
}

void
JacoArm::start_api_ctrl()
{
  int r = _cmd_out(CMD_START_API_CTRL);
  if( r < 0 ) {
    throw fawkes::Exception("Kinova_API: Could not start API control! libusb error code: %i.", r);
  }
}

void
JacoArm::stop_api_ctrl()
{
  int r = _cmd_out(CMD_STOP_API_CTRL);
  if( r < 0 ) {
    throw fawkes::Exception("Kinova_API: Could not stop API control! libusb error code: %i.", r);
  }
}

void
JacoArm::set_target(basic_traj_t &traj)
{
  int r = _send_basic_traj(traj);
  if( r < 0 ) {
    throw fawkes::Exception("Kinova_API: Could not send basic trajectory! libusb error code: %i.", r);
  }
}

void
JacoArm::set_target_cart(float coord[], float fingers[])
{
  basic_traj_t traj;
  memcpy(traj.target, coord, 6);
  memcpy(traj.fingers, fingers, 3);
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_CARTESIAN;

  set_target(traj);
}

void
JacoArm::set_target_ang(float joints[], float fingers[])
{
  basic_traj_t traj;
  memcpy(traj.target, joints, 6);
  memcpy(traj.fingers, fingers, 3);
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_ANGULAR;

  set_target(traj);
}

void
JacoArm::set_target_cart(float x, float y, float z, float euler_1, float euler_2, float euler_3, float finger_1, float finger_2, float finger_3)
{
  basic_traj_t traj;
  traj.target[0] = x;
  traj.target[1] = y;
  traj.target[2] = z;
  traj.target[3] = euler_1;
  traj.target[4] = euler_2;
  traj.target[5] = euler_3;
  traj.fingers[0] = finger_1;
  traj.fingers[1] = finger_2;
  traj.fingers[2] = finger_3;
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_CARTESIAN;

  set_target(traj);
}

void
JacoArm::set_target_ang(float j1, float j2, float j3, float j4, float j5, float j6, float finger_1, float finger_2, float finger_3)
{
  basic_traj_t traj;
  traj.target[0] = j1;
  traj.target[1] = j2;
  traj.target[2] = j3;
  traj.target[3] = j4;
  traj.target[4] = j5;
  traj.target[5] = j6;
  traj.fingers[0] = finger_1;
  traj.fingers[1] = finger_2;
  traj.fingers[2] = finger_3;
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_ANGULAR;

  set_target(traj);
}

} // end of namespace fawkes