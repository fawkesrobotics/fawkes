
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
typedef enum position_type_enum {
  POSITION_NO_MOVEMENT  = 0, // No Movements
  POSITION_CARTESIAN    = 1, // Cartesian Position Control
  POSITION_ANGULAR      = 2, // Position Angular Control
  SPEED_CARTESIAN       = 7, // Cartesian speed control
  SPEED_ANGULAR         = 8, // Cartesian Angular control
  TIME_DELA             = 12 // Insert Delay
} position_type_t;

typedef enum hand_mode_enum {
  NO_MOVEMENT   = 0, // Finger movements disabled
  MODE_POSITION = 1, // Finger position control
  MODE_SPEED    = 2  // Finger Speed Control
} hand_mode_t;

typedef struct position_cart_struct {
  float Position[3];
  float Rotation[3];
  float FingerPosition[3];
} position_cart_t;

typedef struct position_ang_struct {
  float Joints[6];
  float FingerPosition[3];
} position_ang_t;

typedef struct basic_traj_struct {
  float target[6];
  float fingers[3];
  float time_delay;
  hand_mode_t hand_mode;
  position_type_t pos_type;
} basic_traj_t;

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
