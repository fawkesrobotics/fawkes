
/***************************************************************************
 *  kinova_api.h - Kinova API for libusb connection
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

#ifndef __PLUGINS_KINOVA_KINOVA_API_H_
#define __PLUGINS_KINOVA_KINOVA_API_H_

#include <libusb.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

typedef struct message_header_struct {
  int IdPacket;
  int PacketQuantity;
  int CommandId;
  int CommandSize;
} message_header_t;

typedef struct message_struct {
  union {
    unsigned char data[64];
    struct {
      message_header_t header; //8 Byte
      float body[14]; //56 Byte
    };
  };
} message_t;


class JacoArm
{
 public:
  JacoArm();
  virtual ~JacoArm();

 private:
   static libusb_context  *__lusb_ctx;

   libusb_device_handle   *__lusb_devh;

   void _init_libusb();
   void _get_device_handle();
   void _claim_interface();
};

} // end of namespace fawkes

#endif