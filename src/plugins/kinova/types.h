
/***************************************************************************
 *  types.h - Definition of types for Kinova Jaco
 *
 *  Created: Thu Jun 13 19:14:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __PLUGINS_KINOVA_TYPES_H_
#define __PLUGINS_KINOVA_TYPES_H_

#include <vector>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

typedef struct position_cart_struct {
  float Position[3];
  float Rotation[3];
  float FingerPosition[3];
} position_cart_t;

typedef struct position_ang_struct {
  float Joints[6];
  float FingerPosition[3];
} position_ang_t;

typedef struct message_header_struct {
  short IdPacket;
  short PacketQuantity;
  short CommandId;
  short CommandSize;
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

} // end namespace firevision

#endif
