
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
typedef enum jaco_position_type_enum {
  POSITION_NO_MOVEMENT  = 0, // No Movements
  POSITION_CARTESIAN    = 1, // Cartesian Position Control
  POSITION_ANGULAR      = 2, // Angular Position Control
  SPEED_CARTESIAN       = 7, // Cartesian speed control
  SPEED_ANGULAR         = 8, // Angular speed control
  TIME_DELAY            = 12 // Insert Delay
} jaco_position_type_t;

typedef enum jaco_hand_mode_enum {
  NO_MOVEMENT   = 0, // Finger movements disabled
  MODE_POSITION = 1, // Finger position control
  MODE_SPEED    = 2  // Finger Speed Control
} jaco_hand_mode_t;

typedef enum jaco_retract_mode_enum {
  MODE_NORMAL_TO_READY  = 0, /**< Transition mode indicating that Jaco is in NORMAL state and waiting to go in READY state. */
  MODE_READY_STANDBY    = 1, /**< Transition mode indicating that Jaco is in READY state and waiting to go in STANDBY state. */
  MODE_READY_TO_RETRACT = 2, /**< Transition mode indicating that Jaco is in READY state and waiting to go in RETRACTED state. */
  MODE_RETRACT_STANDBY  = 3, /**< Transition mode indicating that Jaco is in RETRACT state and waiting to go in STANDBY state. */
  MODE_RETRACT_TO_READY = 4, /**< Transition mode indicating that Jaco is in RETRACT state and waiting to go in READY state. */
  MODE_NORMAL           = 5, /**< Transition mode indicating that Jaco is in NORMAL state. */
  MODE_NOINIT           = 6, /**< Transition mode indicating that Jaco is in NO INIT state and waiting to go in READY state. */
  MODE_ERROR            = 25000 /**< This value indicate an error. Most of the time, it is because you received a value that is not part of the enum. */
} jaco_retract_mode_t;

typedef enum jaco_target_type_enum {
  TARGET_CARTESIAN,
  TARGET_ANGULAR,
  TARGET_READY,
  TARGET_RETRACT
} jaco_target_type_t;

typedef struct jaco_position_struct {
  union {
    float Joints[6];
    struct {
      float Position[3];
      float Rotation[3];
    };
  };
  float FingerPosition[3];
} jaco_position_t;

typedef struct jaco_basic_traj_struct {
  jaco_position_t target;
  float time_delay;
  jaco_hand_mode_t hand_mode;
  jaco_position_type_t pos_type;
} jaco_basic_traj_t;

typedef unsigned short jaco_joystick_button_t[16];

typedef struct jaco_message_header_struct {
  unsigned short IdPacket;
  unsigned short PacketQuantity;
  unsigned short CommandId;
  unsigned short CommandSize;
} jaco_message_header_t;

typedef struct jaco_message_struct {
  union {
    unsigned char data[64];
    struct {
      jaco_message_header_t header; //8 Byte
      float body[14]; //56 Byte
    };
  };
} jaco_message_t;


} // end namespace firevision

#endif
