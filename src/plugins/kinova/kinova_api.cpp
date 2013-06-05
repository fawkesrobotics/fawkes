
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

position_cart_t
JacoArm::_get_cart_pos()
{
  message_t msg;
  position_cart_t pos;

  msg.header.IdPacket = 1;
  msg.header.PacketQuantity = 1;
  msg.header.CommandId = CMD_GET_CART_POS;
  msg.header.CommandSize = 8;

  int r = _cmd_out_in(msg, 40);
  if( r < 0 ) {
    fprintf(stderr, "some error occured %d\n", r);
    return pos;
  }

  memcpy(&pos, msg.body, sizeof(pos));
  return pos;
}

/** Perform an outgoing and then ingoing command.*/
int
JacoArm::_cmd_out_in(message_t msg, int cmd_size_in)
{
  int r, transferred;

  r = libusb_interrupt_transfer(__lusb_devh, EP_OUT, msg.data, INTR_LENGTH, &transferred, 1000);
  if (r < 0) {
    fprintf(stderr, "intr error %d\n", r);
    return r;
  }
  if (transferred < INTR_LENGTH) {
    fprintf(stderr, "short write (%d)\n", r);
    return -1;
  }
  printf("sent interrupt \n");

  msg.header.CommandSize = cmd_size_in;

  r = libusb_interrupt_transfer(__lusb_devh, EP_IN, msg.data, INTR_LENGTH, &transferred, 1000);
  if (r < 0) {
    fprintf(stderr, "intr error %d\n", r);
    return r;
  }
  if (transferred < INTR_LENGTH) {
    fprintf(stderr, "short read (%d)\n", r);
    return -1;
  }

  printf("sent interrupt %04x\n", *((uint16_t *) msg.data));

  return 1;
}
} // end of namespace fawkes