
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
//#include <libusb.h>

#define VENDOR_ID       0x22CD
#define PRODUCT_ID      0x0000

#define EP_INTR         2
#define INTR_LENGTH	64

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
}

/** Destructor. */
JacoArm::~JacoArm()
{
  printf("destroy usb handle ... ");
  if( __lusb_devh != NULL ) {
    // Close libusb session
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

} // end of namespace fawkes