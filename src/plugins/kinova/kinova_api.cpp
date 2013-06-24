
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

#define VENDOR_ID       0x22CD
#define PRODUCT_ID      0x0000

#define EP_IN           (2 | LIBUSB_ENDPOINT_IN)
#define EP_OUT          (2 | LIBUSB_ENDPOINT_OUT)
#define INTR_LENGTH	64

#define CMD_CTRL_ANG            47
#define CMD_CTRL_CART           49
#define CMD_GET_CART_POS        44 //104
#define CMD_GET_ANG_POS         15 //15
#define CMD_ERASE_TRAJECTORIES  301
#define CMD_START_API_CTRL      302
#define CMD_STOP_API_CTRL       303
#define CMD_JOYSTICK            305
#define CMD_SEND_BASIC_TRAJ     308

#define USB_CMD(ep,msg)         \
  (libusb_interrupt_transfer(__lusb_devh, ep, msg.data, INTR_LENGTH, &transferred, 1000))

#define USB_CMD_IN(msg)         (USB_CMD(EP_IN,msg))
#define USB_CMD_OUT(msg)        (USB_CMD(EP_OUT,msg))
#define USB_MSG(msg,pid,pquant,cmdid,cmdsize) { \
  memset(msg.data, 0, sizeof(msg));             \
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

  __lock = false;
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
JacoArm::_cmd_out_in(jaco_message_t &msg, int cmd_size_in)
{
  int r, transferred;
  unsigned short exp = msg.header.CommandId;

  //printf("\n cmd_out_in send message: \n");
  //print_message(msg);
  while(__lock) {
#ifdef __USE_GNU
    pthread_yield();
#else
    usleep(0);
#endif
  }
  __lock = true;
  r = USB_CMD_OUT(msg);
  if( r < 0 ) {
    fprintf(stderr, "intr error %d\n", r);
    __lock = false;
    return r;
  } else if( transferred < INTR_LENGTH ) {
    fprintf(stderr, "short write (%d)\n", r);
    __lock = false;
    return -1;
  }
  //printf("sent interrupt, transferred %i \n", transferred);

  msg.header.CommandSize = cmd_size_in;

  //printf("\n cmd_out_in send modified message: \n");
  //print_message(msg);
  r = USB_CMD_IN(msg);
  if( r < 0 ) {
    fprintf(stderr, "intr error %d\n", r);
    __lock = false;
    return r;
  } else if( transferred < INTR_LENGTH ) {
    fprintf(stderr, "short read (%d)\n", r);
    __lock = false;
    return -1;
  }
  //printf("sent interrupt %04x, transfered %i \n", *((uint16_t *) msg.data), transferred);

  if( exp != msg.header.CommandId)
    throw fawkes::Exception("Kinova_API: Got CMD_ID %i, expected %i!", msg.header.CommandId, exp);

  __lock = false;
  return 0;
}

int
JacoArm::_cmd_out(short cmd)
{
  jaco_message_t msg;
  USB_MSG(msg, 1, 1, cmd, 8);

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
JacoArm::_get_cart_pos(jaco_position_t &pos)
{
  jaco_message_t msg;
  USB_MSG(msg, 1, 1, CMD_GET_CART_POS, 1);

  int r = _cmd_out_in(msg, sizeof(pos));
  if( r < 0 )
    return r;

  //memcpy(&pos, msg.body, sizeof(msg.header.CommandSize));
  memcpy(pos.Position, msg.body + 2, sizeof(pos.Position));
  memcpy(pos.Rotation, msg.body + 8, sizeof(pos.Rotation));

  USB_MSG(msg, 1, 1, 105, 1);
  r = _cmd_out_in(msg, sizeof(pos));
  if( r >= 0 ) {
    memcpy(pos.FingerPosition, msg.body + 6, sizeof(pos.FingerPosition));
  }

  return r;
}

int
JacoArm::_get_ang_pos(jaco_position_t &pos)
{
  jaco_message_t msg;
  USB_MSG(msg, 1, 1, CMD_GET_ANG_POS, 1);

  int r = _cmd_out_in(msg, sizeof(pos.Joints));
  if( r >= 0 )
    memcpy(pos.Joints, msg.body, sizeof(pos.Joints));

  return r;
}

int
JacoArm::_send_basic_traj(jaco_basic_traj_t &traj)
{
  jaco_message_t msg;
  USB_MSG(msg, 1, 1, CMD_SEND_BASIC_TRAJ, 48);
  memcpy(&(msg.body), &traj, 48);

  return _cmd_out_in(msg, 48);
}


/* /================================================\
 *   public methods
 * \================================================/*/
void
JacoArm::print_message(jaco_message_t &msg)
{
  jaco_message_header_t h = msg.header;
  float *b = msg.body;
  printf("h: %i  %i  %i  %i \n", h.IdPacket, h.PacketQuantity, h.CommandId, h.CommandSize);
  printf("b: ");
  for(unsigned int i=0; i<2; ++i) {
    for(unsigned int j=0; j<7; ++j) {
      printf("%f   ", *b);
      ++b;
    }
    printf("\n   ");
  }
}

jaco_position_t
JacoArm::get_cart_pos()
{
  jaco_position_t pos;
  int r = _get_cart_pos(pos);
  if( r < 0 ) {
    throw fawkes::Exception("Kinova_API: Could not get cartesian position! libusb error code: %i.", r);
  }
  return pos;
}

jaco_position_t
JacoArm::get_ang_pos()
{
  jaco_position_t pos;
  int r = _get_ang_pos(pos);
  if( r < 0 ) {
    throw fawkes::Exception("Kinova_API: Could not get angular position! libusb error code: %i.", r);
  }
  return pos;
}

jaco_retract_mode_t
JacoArm::get_status()
{
  jaco_message_t msg;
  jaco_retract_mode_t mode = MODE_ERROR;
  int r;

  for( unsigned int i=1; i<=19; ++i ) {
    USB_MSG(msg, i, 1, 200, 1);
    r = _cmd_out_in(msg, 56);
    if( r < 0 ) {
      throw fawkes::Exception("Kinova_API: Could not get status! libusb error code: %i.", r);
      return MODE_ERROR;
    }

    if( i==2 )
      memcpy(&mode, msg.data+8+52, sizeof(mode));
  }

  return mode;
}

void
JacoArm::start_api_ctrl()
{
  jaco_message_t msg;
  USB_MSG(msg, 1, 1, CMD_START_API_CTRL, 0);
  int r = _cmd_out_in(msg, 0);
  if( r < 0 )
    throw fawkes::Exception("Kinova_API: Could not start API control! libusb error code: %i.", r);
}

void
JacoArm::stop_api_ctrl()
{
  jaco_message_t msg;
  USB_MSG(msg, 1, 1, CMD_STOP_API_CTRL, 0);
  int r = _cmd_out_in(msg, 0);
  if( r < 0 )
    throw fawkes::Exception("Kinova_API: Could not stop API control! libusb error code: %i.", r);
}

void
JacoArm::erase_trajectories()
{
  jaco_message_t msg;
  USB_MSG(msg, 1, 1, CMD_ERASE_TRAJECTORIES, 0);
  int r = _cmd_out_in(msg, 0);
  if( r < 0 )
    throw fawkes::Exception("Kinova_API: Could not erase trajectories! libusb error code: %i.", r);
}

void
JacoArm::set_control_ang()
{
  jaco_message_t msg;
  USB_MSG(msg, 1, 1, CMD_CTRL_ANG, 0);
  int r = _cmd_out_in(msg, 0);
  if( r < 0 )
    throw fawkes::Exception("Kinova_API: Could not set angular control! libusb error code: %i.", r);
}

void
JacoArm::set_control_cart()
{
  jaco_message_t msg;
  USB_MSG(msg, 1, 1, CMD_CTRL_CART, 0);
  int r = _cmd_out_in(msg, 0);
  if( r < 0 )
    throw fawkes::Exception("Kinova_API: Could not set cartesian control! libusb error code: %i.", r);
}

void
JacoArm::push_joystick_button(unsigned short id)
{
  jaco_message_t msg;
  USB_MSG(msg, 1, 1, CMD_JOYSTICK, 56);

  unsigned short *buttons = (unsigned short*)msg.body;
  buttons[id] = 1;

  int r = _cmd_out_in(msg, 56);
  if( r < 0 )
    throw fawkes::Exception("Kinova_API: Could not send joystick command! libusb error code: %i.", r);
}

void
JacoArm::release_joystick()
{
  jaco_message_t msg;
  USB_MSG(msg, 1, 1, CMD_JOYSTICK, 56);

  int r = _cmd_out_in(msg, 56);
  if( r < 0 )
    throw fawkes::Exception("Kinova_API: Could not release joystick! libusb error code: %i.", r);
}

void
JacoArm::set_target(jaco_basic_traj_t &traj)
{
  int r = _send_basic_traj(traj);
  if( r < 0 )
    throw fawkes::Exception("Kinova_API: Could not send basic trajectory! libusb error code: %i.", r);
}

void
JacoArm::set_target_cart(float coord[], float fingers[])
{
  jaco_basic_traj_t traj;
  memcpy(&traj.target, coord, 6);
  memcpy(traj.target.FingerPosition, fingers, 3);
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_CARTESIAN;

  set_target(traj);
}

void
JacoArm::set_target_ang(float joints[], float fingers[])
{
  jaco_basic_traj_t traj;
  memcpy(&traj.target, joints, 24);
  memcpy(traj.target.FingerPosition, fingers, 12);
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_ANGULAR;

  set_target(traj);
}

void
JacoArm::set_target_cart(float x, float y, float z, float euler_1, float euler_2, float euler_3, float finger_1, float finger_2, float finger_3)
{
  jaco_basic_traj_t traj;
  traj.target.Position[0] = x;
  traj.target.Position[1] = y;
  traj.target.Position[2] = z;
  traj.target.Rotation[0] = euler_1;
  traj.target.Rotation[1] = euler_2;
  traj.target.Rotation[2] = euler_3;
  traj.target.FingerPosition[0] = finger_1;
  traj.target.FingerPosition[1] = finger_2;
  traj.target.FingerPosition[2] = finger_3;
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_CARTESIAN;

  set_target(traj);
}

void
JacoArm::set_target_ang(float j1, float j2, float j3, float j4, float j5, float j6, float finger_1, float finger_2, float finger_3)
{
  jaco_basic_traj_t traj;
  traj.target.Joints[0] = j1;
  traj.target.Joints[1] = j2;
  traj.target.Joints[2] = j3;
  traj.target.Joints[3] = j4;
  traj.target.Joints[4] = j5;
  traj.target.Joints[5] = j6;
  traj.target.FingerPosition[0] = finger_1;
  traj.target.FingerPosition[1] = finger_2;
  traj.target.FingerPosition[2] = finger_3;
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_ANGULAR;

  set_target(traj);
}

} // end of namespace fawkes