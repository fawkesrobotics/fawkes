
/***************************************************************************
 *  NaoJointPositionInterface.cpp - Fawkes BlackBoard Interface - NaoJointPositionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008-2011  Tim Niemueller
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

#include <interfaces/NaoJointPositionInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class NaoJointPositionInterface <interfaces/NaoJointPositionInterface.h>
 * NaoJointPositionInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to Nao joint positions and movements.
    
 * @ingroup FawkesInterfaces
 */


/** SERVO_head_yaw constant */
const uint32_t NaoJointPositionInterface::SERVO_head_yaw = 1u;
/** SERVO_head_pitch constant */
const uint32_t NaoJointPositionInterface::SERVO_head_pitch = 2u;
/** SERVO_l_shoulder_pitch constant */
const uint32_t NaoJointPositionInterface::SERVO_l_shoulder_pitch = 4u;
/** SERVO_l_shoulder_roll constant */
const uint32_t NaoJointPositionInterface::SERVO_l_shoulder_roll = 8u;
/** SERVO_l_elbow_yaw constant */
const uint32_t NaoJointPositionInterface::SERVO_l_elbow_yaw = 16u;
/** SERVO_l_elbow_roll constant */
const uint32_t NaoJointPositionInterface::SERVO_l_elbow_roll = 32u;
/** SERVO_l_wrist_yaw constant */
const uint32_t NaoJointPositionInterface::SERVO_l_wrist_yaw = 64u;
/** SERVO_l_hand constant */
const uint32_t NaoJointPositionInterface::SERVO_l_hand = 128u;
/** SERVO_l_hip_yaw_pitch constant */
const uint32_t NaoJointPositionInterface::SERVO_l_hip_yaw_pitch = 256u;
/** SERVO_l_hip_roll constant */
const uint32_t NaoJointPositionInterface::SERVO_l_hip_roll = 512u;
/** SERVO_l_hip_pitch constant */
const uint32_t NaoJointPositionInterface::SERVO_l_hip_pitch = 1024u;
/** SERVO_l_knee_pitch constant */
const uint32_t NaoJointPositionInterface::SERVO_l_knee_pitch = 2048u;
/** SERVO_l_ankle_pitch constant */
const uint32_t NaoJointPositionInterface::SERVO_l_ankle_pitch = 4096u;
/** SERVO_l_ankle_roll constant */
const uint32_t NaoJointPositionInterface::SERVO_l_ankle_roll = 8192u;
/** SERVO_r_shoulder_pitch constant */
const uint32_t NaoJointPositionInterface::SERVO_r_shoulder_pitch = 16384u;
/** SERVO_r_shoulder_roll constant */
const uint32_t NaoJointPositionInterface::SERVO_r_shoulder_roll = 32768u;
/** SERVO_r_elbow_yaw constant */
const uint32_t NaoJointPositionInterface::SERVO_r_elbow_yaw = 65536u;
/** SERVO_r_elbow_roll constant */
const uint32_t NaoJointPositionInterface::SERVO_r_elbow_roll = 131072u;
/** SERVO_r_wrist_yaw constant */
const uint32_t NaoJointPositionInterface::SERVO_r_wrist_yaw = 262144u;
/** SERVO_r_hand constant */
const uint32_t NaoJointPositionInterface::SERVO_r_hand = 524288u;
/** SERVO_r_hip_yaw_pitch constant */
const uint32_t NaoJointPositionInterface::SERVO_r_hip_yaw_pitch = 1048576u;
/** SERVO_r_hip_roll constant */
const uint32_t NaoJointPositionInterface::SERVO_r_hip_roll = 2097152u;
/** SERVO_r_hip_pitch constant */
const uint32_t NaoJointPositionInterface::SERVO_r_hip_pitch = 4194304u;
/** SERVO_r_knee_pitch constant */
const uint32_t NaoJointPositionInterface::SERVO_r_knee_pitch = 8388608u;
/** SERVO_r_ankle_pitch constant */
const uint32_t NaoJointPositionInterface::SERVO_r_ankle_pitch = 16777216u;
/** SERVO_r_ankle_roll constant */
const uint32_t NaoJointPositionInterface::SERVO_r_ankle_roll = 33554432u;
/** SERVO_min constant */
const uint32_t NaoJointPositionInterface::SERVO_min = 1u;
/** SERVO_max constant */
const uint32_t NaoJointPositionInterface::SERVO_max = 33554432u;

/** Constructor */
NaoJointPositionInterface::NaoJointPositionInterface() : Interface()
{
  data_size = sizeof(NaoJointPositionInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (NaoJointPositionInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_ENUM, "robot_type", 1, &data->robot_type, "RobotType");
  add_fieldinfo(IFT_UINT8, "robot_version", 4, &data->robot_version);
  add_fieldinfo(IFT_FLOAT, "head_yaw", 1, &data->head_yaw);
  add_fieldinfo(IFT_FLOAT, "head_pitch", 1, &data->head_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_pitch", 1, &data->l_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_roll", 1, &data->l_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "l_elbow_yaw", 1, &data->l_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "l_elbow_roll", 1, &data->l_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "l_wrist_yaw", 1, &data->l_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "l_hand", 1, &data->l_hand);
  add_fieldinfo(IFT_FLOAT, "l_hip_yaw_pitch", 1, &data->l_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "l_hip_roll", 1, &data->l_hip_roll);
  add_fieldinfo(IFT_FLOAT, "l_hip_pitch", 1, &data->l_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "l_knee_pitch", 1, &data->l_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_pitch", 1, &data->l_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_roll", 1, &data->l_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "r_wrist_yaw", 1, &data->r_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "r_hand", 1, &data->r_hand);
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
  add_messageinfo("SetServoMessage");
  add_messageinfo("SetServosMessage");
  add_messageinfo("GotoAngleMessage");
  add_messageinfo("GotoAnglesMessage");
  add_messageinfo("GotoAngleWithSpeedMessage");
  unsigned char tmp_hash[] = {0xfe, 0x28, 0x34, 0xed, 0xa8, 0xfb, 0xea, 0x8e, 0xf, 0x74, 0x5a, 0x71, 0x55, 0x38, 0x4, 0x78};
  set_hash(tmp_hash);
}

/** Destructor */
NaoJointPositionInterface::~NaoJointPositionInterface()
{
  free(data_ptr);
}
/** Convert RobotType constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
NaoJointPositionInterface::tostring_RobotType(RobotType value) const
{
  switch (value) {
  case ROBOTYPE_ACADEMIC: return "ROBOTYPE_ACADEMIC";
  case ROBOTYPE_ROBOCUP: return "ROBOTYPE_ROBOCUP";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get robot_type value.
 * Robot type.
 * @return robot_type value
 */
NaoJointPositionInterface::RobotType
NaoJointPositionInterface::robot_type() const
{
  return (NaoJointPositionInterface::RobotType)data->robot_type;
}

/** Get maximum length of robot_type value.
 * @return length of robot_type value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_robot_type() const
{
  return 1;
}

/** Set robot_type value.
 * Robot type.
 * @param new_robot_type new robot_type value
 */
void
NaoJointPositionInterface::set_robot_type(const RobotType new_robot_type)
{
  data->robot_type = new_robot_type;
  data_changed = true;
}

/** Get robot_version value.
 * 
      Robot version. Fields are in ascending array index major, minor, micro and
      patch level. Currently only the first two are used by Aldebaran, but due to
      struct alignment we add two extra bytes.
    
 * @return robot_version value
 */
uint8_t *
NaoJointPositionInterface::robot_version() const
{
  return data->robot_version;
}

/** Get robot_version value at given index.
 * 
      Robot version. Fields are in ascending array index major, minor, micro and
      patch level. Currently only the first two are used by Aldebaran, but due to
      struct alignment we add two extra bytes.
    
 * @param index index of value
 * @return robot_version value
 * @exception Exception thrown if index is out of bounds
 */
uint8_t
NaoJointPositionInterface::robot_version(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->robot_version[index];
}

/** Get maximum length of robot_version value.
 * @return length of robot_version value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_robot_version() const
{
  return 4;
}

/** Set robot_version value.
 * 
      Robot version. Fields are in ascending array index major, minor, micro and
      patch level. Currently only the first two are used by Aldebaran, but due to
      struct alignment we add two extra bytes.
    
 * @param new_robot_version new robot_version value
 */
void
NaoJointPositionInterface::set_robot_version(const uint8_t * new_robot_version)
{
  memcpy(data->robot_version, new_robot_version, sizeof(uint8_t) * 4);
  data_changed = true;
}

/** Set robot_version value at given index.
 * 
      Robot version. Fields are in ascending array index major, minor, micro and
      patch level. Currently only the first two are used by Aldebaran, but due to
      struct alignment we add two extra bytes.
    
 * @param new_robot_version new robot_version value
 * @param index index for of the value
 */
void
NaoJointPositionInterface::set_robot_version(unsigned int index, const uint8_t new_robot_version)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->robot_version[index] = new_robot_version;
}
/** Get head_yaw value.
 * Head yaw
 * @return head_yaw value
 */
float
NaoJointPositionInterface::head_yaw() const
{
  return data->head_yaw;
}

/** Get maximum length of head_yaw value.
 * @return length of head_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_head_yaw() const
{
  return 1;
}

/** Set head_yaw value.
 * Head yaw
 * @param new_head_yaw new head_yaw value
 */
void
NaoJointPositionInterface::set_head_yaw(const float new_head_yaw)
{
  data->head_yaw = new_head_yaw;
  data_changed = true;
}

/** Get head_pitch value.
 * Head pitch
 * @return head_pitch value
 */
float
NaoJointPositionInterface::head_pitch() const
{
  return data->head_pitch;
}

/** Get maximum length of head_pitch value.
 * @return length of head_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_head_pitch() const
{
  return 1;
}

/** Set head_pitch value.
 * Head pitch
 * @param new_head_pitch new head_pitch value
 */
void
NaoJointPositionInterface::set_head_pitch(const float new_head_pitch)
{
  data->head_pitch = new_head_pitch;
  data_changed = true;
}

/** Get l_shoulder_pitch value.
 * Left shoulder pitch
 * @return l_shoulder_pitch value
 */
float
NaoJointPositionInterface::l_shoulder_pitch() const
{
  return data->l_shoulder_pitch;
}

/** Get maximum length of l_shoulder_pitch value.
 * @return length of l_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_shoulder_pitch() const
{
  return 1;
}

/** Set l_shoulder_pitch value.
 * Left shoulder pitch
 * @param new_l_shoulder_pitch new l_shoulder_pitch value
 */
void
NaoJointPositionInterface::set_l_shoulder_pitch(const float new_l_shoulder_pitch)
{
  data->l_shoulder_pitch = new_l_shoulder_pitch;
  data_changed = true;
}

/** Get l_shoulder_roll value.
 * Left shoulder roll
 * @return l_shoulder_roll value
 */
float
NaoJointPositionInterface::l_shoulder_roll() const
{
  return data->l_shoulder_roll;
}

/** Get maximum length of l_shoulder_roll value.
 * @return length of l_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_shoulder_roll() const
{
  return 1;
}

/** Set l_shoulder_roll value.
 * Left shoulder roll
 * @param new_l_shoulder_roll new l_shoulder_roll value
 */
void
NaoJointPositionInterface::set_l_shoulder_roll(const float new_l_shoulder_roll)
{
  data->l_shoulder_roll = new_l_shoulder_roll;
  data_changed = true;
}

/** Get l_elbow_yaw value.
 * Left elbow yaw
 * @return l_elbow_yaw value
 */
float
NaoJointPositionInterface::l_elbow_yaw() const
{
  return data->l_elbow_yaw;
}

/** Get maximum length of l_elbow_yaw value.
 * @return length of l_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_elbow_yaw() const
{
  return 1;
}

/** Set l_elbow_yaw value.
 * Left elbow yaw
 * @param new_l_elbow_yaw new l_elbow_yaw value
 */
void
NaoJointPositionInterface::set_l_elbow_yaw(const float new_l_elbow_yaw)
{
  data->l_elbow_yaw = new_l_elbow_yaw;
  data_changed = true;
}

/** Get l_elbow_roll value.
 * Left elbow roll
 * @return l_elbow_roll value
 */
float
NaoJointPositionInterface::l_elbow_roll() const
{
  return data->l_elbow_roll;
}

/** Get maximum length of l_elbow_roll value.
 * @return length of l_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_elbow_roll() const
{
  return 1;
}

/** Set l_elbow_roll value.
 * Left elbow roll
 * @param new_l_elbow_roll new l_elbow_roll value
 */
void
NaoJointPositionInterface::set_l_elbow_roll(const float new_l_elbow_roll)
{
  data->l_elbow_roll = new_l_elbow_roll;
  data_changed = true;
}

/** Get l_wrist_yaw value.
 * Left wrist yaw
 * @return l_wrist_yaw value
 */
float
NaoJointPositionInterface::l_wrist_yaw() const
{
  return data->l_wrist_yaw;
}

/** Get maximum length of l_wrist_yaw value.
 * @return length of l_wrist_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_wrist_yaw() const
{
  return 1;
}

/** Set l_wrist_yaw value.
 * Left wrist yaw
 * @param new_l_wrist_yaw new l_wrist_yaw value
 */
void
NaoJointPositionInterface::set_l_wrist_yaw(const float new_l_wrist_yaw)
{
  data->l_wrist_yaw = new_l_wrist_yaw;
  data_changed = true;
}

/** Get l_hand value.
 * Left hand
 * @return l_hand value
 */
float
NaoJointPositionInterface::l_hand() const
{
  return data->l_hand;
}

/** Get maximum length of l_hand value.
 * @return length of l_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_hand() const
{
  return 1;
}

/** Set l_hand value.
 * Left hand
 * @param new_l_hand new l_hand value
 */
void
NaoJointPositionInterface::set_l_hand(const float new_l_hand)
{
  data->l_hand = new_l_hand;
  data_changed = true;
}

/** Get l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @return l_hip_yaw_pitch value
 */
float
NaoJointPositionInterface::l_hip_yaw_pitch() const
{
  return data->l_hip_yaw_pitch;
}

/** Get maximum length of l_hip_yaw_pitch value.
 * @return length of l_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_hip_yaw_pitch() const
{
  return 1;
}

/** Set l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @param new_l_hip_yaw_pitch new l_hip_yaw_pitch value
 */
void
NaoJointPositionInterface::set_l_hip_yaw_pitch(const float new_l_hip_yaw_pitch)
{
  data->l_hip_yaw_pitch = new_l_hip_yaw_pitch;
  data_changed = true;
}

/** Get l_hip_roll value.
 * Left hip roll
 * @return l_hip_roll value
 */
float
NaoJointPositionInterface::l_hip_roll() const
{
  return data->l_hip_roll;
}

/** Get maximum length of l_hip_roll value.
 * @return length of l_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_hip_roll() const
{
  return 1;
}

/** Set l_hip_roll value.
 * Left hip roll
 * @param new_l_hip_roll new l_hip_roll value
 */
void
NaoJointPositionInterface::set_l_hip_roll(const float new_l_hip_roll)
{
  data->l_hip_roll = new_l_hip_roll;
  data_changed = true;
}

/** Get l_hip_pitch value.
 * Left hip pitch
 * @return l_hip_pitch value
 */
float
NaoJointPositionInterface::l_hip_pitch() const
{
  return data->l_hip_pitch;
}

/** Get maximum length of l_hip_pitch value.
 * @return length of l_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_hip_pitch() const
{
  return 1;
}

/** Set l_hip_pitch value.
 * Left hip pitch
 * @param new_l_hip_pitch new l_hip_pitch value
 */
void
NaoJointPositionInterface::set_l_hip_pitch(const float new_l_hip_pitch)
{
  data->l_hip_pitch = new_l_hip_pitch;
  data_changed = true;
}

/** Get l_knee_pitch value.
 * Left knee pitch
 * @return l_knee_pitch value
 */
float
NaoJointPositionInterface::l_knee_pitch() const
{
  return data->l_knee_pitch;
}

/** Get maximum length of l_knee_pitch value.
 * @return length of l_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_knee_pitch() const
{
  return 1;
}

/** Set l_knee_pitch value.
 * Left knee pitch
 * @param new_l_knee_pitch new l_knee_pitch value
 */
void
NaoJointPositionInterface::set_l_knee_pitch(const float new_l_knee_pitch)
{
  data->l_knee_pitch = new_l_knee_pitch;
  data_changed = true;
}

/** Get l_ankle_pitch value.
 * Left ankle pitch
 * @return l_ankle_pitch value
 */
float
NaoJointPositionInterface::l_ankle_pitch() const
{
  return data->l_ankle_pitch;
}

/** Get maximum length of l_ankle_pitch value.
 * @return length of l_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_ankle_pitch() const
{
  return 1;
}

/** Set l_ankle_pitch value.
 * Left ankle pitch
 * @param new_l_ankle_pitch new l_ankle_pitch value
 */
void
NaoJointPositionInterface::set_l_ankle_pitch(const float new_l_ankle_pitch)
{
  data->l_ankle_pitch = new_l_ankle_pitch;
  data_changed = true;
}

/** Get l_ankle_roll value.
 * Left ankle roll
 * @return l_ankle_roll value
 */
float
NaoJointPositionInterface::l_ankle_roll() const
{
  return data->l_ankle_roll;
}

/** Get maximum length of l_ankle_roll value.
 * @return length of l_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_l_ankle_roll() const
{
  return 1;
}

/** Set l_ankle_roll value.
 * Left ankle roll
 * @param new_l_ankle_roll new l_ankle_roll value
 */
void
NaoJointPositionInterface::set_l_ankle_roll(const float new_l_ankle_roll)
{
  data->l_ankle_roll = new_l_ankle_roll;
  data_changed = true;
}

/** Get r_shoulder_pitch value.
 * Right shoulder pitch
 * @return r_shoulder_pitch value
 */
float
NaoJointPositionInterface::r_shoulder_pitch() const
{
  return data->r_shoulder_pitch;
}

/** Get maximum length of r_shoulder_pitch value.
 * @return length of r_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_shoulder_pitch() const
{
  return 1;
}

/** Set r_shoulder_pitch value.
 * Right shoulder pitch
 * @param new_r_shoulder_pitch new r_shoulder_pitch value
 */
void
NaoJointPositionInterface::set_r_shoulder_pitch(const float new_r_shoulder_pitch)
{
  data->r_shoulder_pitch = new_r_shoulder_pitch;
  data_changed = true;
}

/** Get r_shoulder_roll value.
 * Right shoulder roll
 * @return r_shoulder_roll value
 */
float
NaoJointPositionInterface::r_shoulder_roll() const
{
  return data->r_shoulder_roll;
}

/** Get maximum length of r_shoulder_roll value.
 * @return length of r_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_shoulder_roll() const
{
  return 1;
}

/** Set r_shoulder_roll value.
 * Right shoulder roll
 * @param new_r_shoulder_roll new r_shoulder_roll value
 */
void
NaoJointPositionInterface::set_r_shoulder_roll(const float new_r_shoulder_roll)
{
  data->r_shoulder_roll = new_r_shoulder_roll;
  data_changed = true;
}

/** Get r_elbow_yaw value.
 * Right elbow yaw
 * @return r_elbow_yaw value
 */
float
NaoJointPositionInterface::r_elbow_yaw() const
{
  return data->r_elbow_yaw;
}

/** Get maximum length of r_elbow_yaw value.
 * @return length of r_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_elbow_yaw() const
{
  return 1;
}

/** Set r_elbow_yaw value.
 * Right elbow yaw
 * @param new_r_elbow_yaw new r_elbow_yaw value
 */
void
NaoJointPositionInterface::set_r_elbow_yaw(const float new_r_elbow_yaw)
{
  data->r_elbow_yaw = new_r_elbow_yaw;
  data_changed = true;
}

/** Get r_elbow_roll value.
 * Right elbow roll
 * @return r_elbow_roll value
 */
float
NaoJointPositionInterface::r_elbow_roll() const
{
  return data->r_elbow_roll;
}

/** Get maximum length of r_elbow_roll value.
 * @return length of r_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_elbow_roll() const
{
  return 1;
}

/** Set r_elbow_roll value.
 * Right elbow roll
 * @param new_r_elbow_roll new r_elbow_roll value
 */
void
NaoJointPositionInterface::set_r_elbow_roll(const float new_r_elbow_roll)
{
  data->r_elbow_roll = new_r_elbow_roll;
  data_changed = true;
}

/** Get r_wrist_yaw value.
 * Right wrist yaw
 * @return r_wrist_yaw value
 */
float
NaoJointPositionInterface::r_wrist_yaw() const
{
  return data->r_wrist_yaw;
}

/** Get maximum length of r_wrist_yaw value.
 * @return length of r_wrist_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_wrist_yaw() const
{
  return 1;
}

/** Set r_wrist_yaw value.
 * Right wrist yaw
 * @param new_r_wrist_yaw new r_wrist_yaw value
 */
void
NaoJointPositionInterface::set_r_wrist_yaw(const float new_r_wrist_yaw)
{
  data->r_wrist_yaw = new_r_wrist_yaw;
  data_changed = true;
}

/** Get r_hand value.
 * Right hand
 * @return r_hand value
 */
float
NaoJointPositionInterface::r_hand() const
{
  return data->r_hand;
}

/** Get maximum length of r_hand value.
 * @return length of r_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_hand() const
{
  return 1;
}

/** Set r_hand value.
 * Right hand
 * @param new_r_hand new r_hand value
 */
void
NaoJointPositionInterface::set_r_hand(const float new_r_hand)
{
  data->r_hand = new_r_hand;
  data_changed = true;
}

/** Get r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @return r_hip_yaw_pitch value
 */
float
NaoJointPositionInterface::r_hip_yaw_pitch() const
{
  return data->r_hip_yaw_pitch;
}

/** Get maximum length of r_hip_yaw_pitch value.
 * @return length of r_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_hip_yaw_pitch() const
{
  return 1;
}

/** Set r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @param new_r_hip_yaw_pitch new r_hip_yaw_pitch value
 */
void
NaoJointPositionInterface::set_r_hip_yaw_pitch(const float new_r_hip_yaw_pitch)
{
  data->r_hip_yaw_pitch = new_r_hip_yaw_pitch;
  data_changed = true;
}

/** Get r_hip_roll value.
 * Right hip roll
 * @return r_hip_roll value
 */
float
NaoJointPositionInterface::r_hip_roll() const
{
  return data->r_hip_roll;
}

/** Get maximum length of r_hip_roll value.
 * @return length of r_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_hip_roll() const
{
  return 1;
}

/** Set r_hip_roll value.
 * Right hip roll
 * @param new_r_hip_roll new r_hip_roll value
 */
void
NaoJointPositionInterface::set_r_hip_roll(const float new_r_hip_roll)
{
  data->r_hip_roll = new_r_hip_roll;
  data_changed = true;
}

/** Get r_hip_pitch value.
 * Right hip pitch
 * @return r_hip_pitch value
 */
float
NaoJointPositionInterface::r_hip_pitch() const
{
  return data->r_hip_pitch;
}

/** Get maximum length of r_hip_pitch value.
 * @return length of r_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_hip_pitch() const
{
  return 1;
}

/** Set r_hip_pitch value.
 * Right hip pitch
 * @param new_r_hip_pitch new r_hip_pitch value
 */
void
NaoJointPositionInterface::set_r_hip_pitch(const float new_r_hip_pitch)
{
  data->r_hip_pitch = new_r_hip_pitch;
  data_changed = true;
}

/** Get r_knee_pitch value.
 * Right knee pitch
 * @return r_knee_pitch value
 */
float
NaoJointPositionInterface::r_knee_pitch() const
{
  return data->r_knee_pitch;
}

/** Get maximum length of r_knee_pitch value.
 * @return length of r_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_knee_pitch() const
{
  return 1;
}

/** Set r_knee_pitch value.
 * Right knee pitch
 * @param new_r_knee_pitch new r_knee_pitch value
 */
void
NaoJointPositionInterface::set_r_knee_pitch(const float new_r_knee_pitch)
{
  data->r_knee_pitch = new_r_knee_pitch;
  data_changed = true;
}

/** Get r_ankle_pitch value.
 * Right ankle pitch
 * @return r_ankle_pitch value
 */
float
NaoJointPositionInterface::r_ankle_pitch() const
{
  return data->r_ankle_pitch;
}

/** Get maximum length of r_ankle_pitch value.
 * @return length of r_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_ankle_pitch() const
{
  return 1;
}

/** Set r_ankle_pitch value.
 * Right ankle pitch
 * @param new_r_ankle_pitch new r_ankle_pitch value
 */
void
NaoJointPositionInterface::set_r_ankle_pitch(const float new_r_ankle_pitch)
{
  data->r_ankle_pitch = new_r_ankle_pitch;
  data_changed = true;
}

/** Get r_ankle_roll value.
 * Right ankle roll
 * @return r_ankle_roll value
 */
float
NaoJointPositionInterface::r_ankle_roll() const
{
  return data->r_ankle_roll;
}

/** Get maximum length of r_ankle_roll value.
 * @return length of r_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_r_ankle_roll() const
{
  return 1;
}

/** Set r_ankle_roll value.
 * Right ankle roll
 * @param new_r_ankle_roll new r_ankle_roll value
 */
void
NaoJointPositionInterface::set_r_ankle_roll(const float new_r_ankle_roll)
{
  data->r_ankle_roll = new_r_ankle_roll;
  data_changed = true;
}

/** Get time value.
 * 
      Current reference time in ms. For real hardware this is the DCM time.
      Times in messages are always offsets to the current time and the current
      time is added before executing the command.
    
 * @return time value
 */
int32_t
NaoJointPositionInterface::time() const
{
  return data->time;
}

/** Get maximum length of time value.
 * @return length of time value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::maxlenof_time() const
{
  return 1;
}

/** Set time value.
 * 
      Current reference time in ms. For real hardware this is the DCM time.
      Times in messages are always offsets to the current time and the current
      time is added before executing the command.
    
 * @param new_time new time value
 */
void
NaoJointPositionInterface::set_time(const int32_t new_time)
{
  data->time = new_time;
  data_changed = true;
}

/* =========== message create =========== */
Message *
NaoJointPositionInterface::create_message(const char *type) const
{
  if ( strncmp("SetServoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetServoMessage();
  } else if ( strncmp("SetServosMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetServosMessage();
  } else if ( strncmp("GotoAngleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GotoAngleMessage();
  } else if ( strncmp("GotoAnglesMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GotoAnglesMessage();
  } else if ( strncmp("GotoAngleWithSpeedMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GotoAngleWithSpeedMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
NaoJointPositionInterface::copy_values(const Interface *other)
{
  const NaoJointPositionInterface *oi = dynamic_cast<const NaoJointPositionInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(NaoJointPositionInterface_data_t));
}

const char *
NaoJointPositionInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "RobotType") == 0) {
    return tostring_RobotType((RobotType)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class NaoJointPositionInterface::SetServoMessage <interfaces/NaoJointPositionInterface.h>
 * SetServoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_servo initial value for servo
 * @param ini_value initial value for value
 * @param ini_time initial value for time
 */
NaoJointPositionInterface::SetServoMessage::SetServoMessage(const uint32_t ini_servo, const float ini_value, const int32_t ini_time) : Message("SetServoMessage")
{
  data_size = sizeof(SetServoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetServoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->servo = ini_servo;
  data->value = ini_value;
  data->time = ini_time;
  add_fieldinfo(IFT_UINT32, "servo", 1, &data->servo);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}
/** Constructor */
NaoJointPositionInterface::SetServoMessage::SetServoMessage() : Message("SetServoMessage")
{
  data_size = sizeof(SetServoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetServoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "servo", 1, &data->servo);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}

/** Destructor */
NaoJointPositionInterface::SetServoMessage::~SetServoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoJointPositionInterface::SetServoMessage::SetServoMessage(const SetServoMessage *m) : Message("SetServoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetServoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get servo value.
 * A concatenated list of SERVO_* constants to
      define the servos that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
 * @return servo value
 */
uint32_t
NaoJointPositionInterface::SetServoMessage::servo() const
{
  return data->servo;
}

/** Get maximum length of servo value.
 * @return length of servo value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServoMessage::maxlenof_servo() const
{
  return 1;
}

/** Set servo value.
 * A concatenated list of SERVO_* constants to
      define the servos that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
 * @param new_servo new servo value
 */
void
NaoJointPositionInterface::SetServoMessage::set_servo(const uint32_t new_servo)
{
  data->servo = new_servo;
}

/** Get value value.
 * Servo value to set for servos.
 * @return value value
 */
float
NaoJointPositionInterface::SetServoMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServoMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * Servo value to set for servos.
 * @param new_value new value value
 */
void
NaoJointPositionInterface::SetServoMessage::set_value(const float new_value)
{
  data->value = new_value;
}

/** Get time value.
 * 
      Current reference time in ms. For real hardware this is the DCM time.
      Times in messages are always offsets to the current time and the current
      time is added before executing the command.
    
 * @return time value
 */
int32_t
NaoJointPositionInterface::SetServoMessage::time() const
{
  return data->time;
}

/** Get maximum length of time value.
 * @return length of time value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServoMessage::maxlenof_time() const
{
  return 1;
}

/** Set time value.
 * 
      Current reference time in ms. For real hardware this is the DCM time.
      Times in messages are always offsets to the current time and the current
      time is added before executing the command.
    
 * @param new_time new time value
 */
void
NaoJointPositionInterface::SetServoMessage::set_time(const int32_t new_time)
{
  data->time = new_time;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoJointPositionInterface::SetServoMessage::clone() const
{
  return new NaoJointPositionInterface::SetServoMessage(this);
}
/** @class NaoJointPositionInterface::SetServosMessage <interfaces/NaoJointPositionInterface.h>
 * SetServosMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_head_yaw initial value for head_yaw
 * @param ini_head_pitch initial value for head_pitch
 * @param ini_l_shoulder_pitch initial value for l_shoulder_pitch
 * @param ini_l_shoulder_roll initial value for l_shoulder_roll
 * @param ini_l_elbow_yaw initial value for l_elbow_yaw
 * @param ini_l_elbow_roll initial value for l_elbow_roll
 * @param ini_l_hip_yaw_pitch initial value for l_hip_yaw_pitch
 * @param ini_l_hip_roll initial value for l_hip_roll
 * @param ini_l_hip_pitch initial value for l_hip_pitch
 * @param ini_l_knee_pitch initial value for l_knee_pitch
 * @param ini_l_ankle_pitch initial value for l_ankle_pitch
 * @param ini_l_ankle_roll initial value for l_ankle_roll
 * @param ini_l_wrist_yaw initial value for l_wrist_yaw
 * @param ini_l_hand initial value for l_hand
 * @param ini_r_shoulder_pitch initial value for r_shoulder_pitch
 * @param ini_r_shoulder_roll initial value for r_shoulder_roll
 * @param ini_r_elbow_yaw initial value for r_elbow_yaw
 * @param ini_r_elbow_roll initial value for r_elbow_roll
 * @param ini_r_wrist_yaw initial value for r_wrist_yaw
 * @param ini_r_hand initial value for r_hand
 * @param ini_r_hip_yaw_pitch initial value for r_hip_yaw_pitch
 * @param ini_r_hip_roll initial value for r_hip_roll
 * @param ini_r_hip_pitch initial value for r_hip_pitch
 * @param ini_r_knee_pitch initial value for r_knee_pitch
 * @param ini_r_ankle_pitch initial value for r_ankle_pitch
 * @param ini_r_ankle_roll initial value for r_ankle_roll
 * @param ini_time initial value for time
 */
NaoJointPositionInterface::SetServosMessage::SetServosMessage(const float ini_head_yaw, const float ini_head_pitch, const float ini_l_shoulder_pitch, const float ini_l_shoulder_roll, const float ini_l_elbow_yaw, const float ini_l_elbow_roll, const float ini_l_hip_yaw_pitch, const float ini_l_hip_roll, const float ini_l_hip_pitch, const float ini_l_knee_pitch, const float ini_l_ankle_pitch, const float ini_l_ankle_roll, const float ini_l_wrist_yaw, const float ini_l_hand, const float ini_r_shoulder_pitch, const float ini_r_shoulder_roll, const float ini_r_elbow_yaw, const float ini_r_elbow_roll, const float ini_r_wrist_yaw, const float ini_r_hand, const float ini_r_hip_yaw_pitch, const float ini_r_hip_roll, const float ini_r_hip_pitch, const float ini_r_knee_pitch, const float ini_r_ankle_pitch, const float ini_r_ankle_roll, const int32_t ini_time) : Message("SetServosMessage")
{
  data_size = sizeof(SetServosMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetServosMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->head_yaw = ini_head_yaw;
  data->head_pitch = ini_head_pitch;
  data->l_shoulder_pitch = ini_l_shoulder_pitch;
  data->l_shoulder_roll = ini_l_shoulder_roll;
  data->l_elbow_yaw = ini_l_elbow_yaw;
  data->l_elbow_roll = ini_l_elbow_roll;
  data->l_hip_yaw_pitch = ini_l_hip_yaw_pitch;
  data->l_hip_roll = ini_l_hip_roll;
  data->l_hip_pitch = ini_l_hip_pitch;
  data->l_knee_pitch = ini_l_knee_pitch;
  data->l_ankle_pitch = ini_l_ankle_pitch;
  data->l_ankle_roll = ini_l_ankle_roll;
  data->l_wrist_yaw = ini_l_wrist_yaw;
  data->l_hand = ini_l_hand;
  data->r_shoulder_pitch = ini_r_shoulder_pitch;
  data->r_shoulder_roll = ini_r_shoulder_roll;
  data->r_elbow_yaw = ini_r_elbow_yaw;
  data->r_elbow_roll = ini_r_elbow_roll;
  data->r_wrist_yaw = ini_r_wrist_yaw;
  data->r_hand = ini_r_hand;
  data->r_hip_yaw_pitch = ini_r_hip_yaw_pitch;
  data->r_hip_roll = ini_r_hip_roll;
  data->r_hip_pitch = ini_r_hip_pitch;
  data->r_knee_pitch = ini_r_knee_pitch;
  data->r_ankle_pitch = ini_r_ankle_pitch;
  data->r_ankle_roll = ini_r_ankle_roll;
  data->time = ini_time;
  add_fieldinfo(IFT_FLOAT, "head_yaw", 1, &data->head_yaw);
  add_fieldinfo(IFT_FLOAT, "head_pitch", 1, &data->head_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_pitch", 1, &data->l_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_roll", 1, &data->l_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "l_elbow_yaw", 1, &data->l_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "l_elbow_roll", 1, &data->l_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "l_hip_yaw_pitch", 1, &data->l_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "l_hip_roll", 1, &data->l_hip_roll);
  add_fieldinfo(IFT_FLOAT, "l_hip_pitch", 1, &data->l_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "l_knee_pitch", 1, &data->l_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_pitch", 1, &data->l_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_roll", 1, &data->l_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "l_wrist_yaw", 1, &data->l_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "l_hand", 1, &data->l_hand);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "r_wrist_yaw", 1, &data->r_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "r_hand", 1, &data->r_hand);
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}
/** Constructor */
NaoJointPositionInterface::SetServosMessage::SetServosMessage() : Message("SetServosMessage")
{
  data_size = sizeof(SetServosMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetServosMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "head_yaw", 1, &data->head_yaw);
  add_fieldinfo(IFT_FLOAT, "head_pitch", 1, &data->head_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_pitch", 1, &data->l_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_roll", 1, &data->l_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "l_elbow_yaw", 1, &data->l_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "l_elbow_roll", 1, &data->l_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "l_hip_yaw_pitch", 1, &data->l_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "l_hip_roll", 1, &data->l_hip_roll);
  add_fieldinfo(IFT_FLOAT, "l_hip_pitch", 1, &data->l_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "l_knee_pitch", 1, &data->l_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_pitch", 1, &data->l_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_roll", 1, &data->l_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "l_wrist_yaw", 1, &data->l_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "l_hand", 1, &data->l_hand);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "r_wrist_yaw", 1, &data->r_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "r_hand", 1, &data->r_hand);
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}

/** Destructor */
NaoJointPositionInterface::SetServosMessage::~SetServosMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoJointPositionInterface::SetServosMessage::SetServosMessage(const SetServosMessage *m) : Message("SetServosMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetServosMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get head_yaw value.
 * Head yaw
 * @return head_yaw value
 */
float
NaoJointPositionInterface::SetServosMessage::head_yaw() const
{
  return data->head_yaw;
}

/** Get maximum length of head_yaw value.
 * @return length of head_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_head_yaw() const
{
  return 1;
}

/** Set head_yaw value.
 * Head yaw
 * @param new_head_yaw new head_yaw value
 */
void
NaoJointPositionInterface::SetServosMessage::set_head_yaw(const float new_head_yaw)
{
  data->head_yaw = new_head_yaw;
}

/** Get head_pitch value.
 * Head pitch
 * @return head_pitch value
 */
float
NaoJointPositionInterface::SetServosMessage::head_pitch() const
{
  return data->head_pitch;
}

/** Get maximum length of head_pitch value.
 * @return length of head_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_head_pitch() const
{
  return 1;
}

/** Set head_pitch value.
 * Head pitch
 * @param new_head_pitch new head_pitch value
 */
void
NaoJointPositionInterface::SetServosMessage::set_head_pitch(const float new_head_pitch)
{
  data->head_pitch = new_head_pitch;
}

/** Get l_shoulder_pitch value.
 * Left shoulder pitch
 * @return l_shoulder_pitch value
 */
float
NaoJointPositionInterface::SetServosMessage::l_shoulder_pitch() const
{
  return data->l_shoulder_pitch;
}

/** Get maximum length of l_shoulder_pitch value.
 * @return length of l_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_shoulder_pitch() const
{
  return 1;
}

/** Set l_shoulder_pitch value.
 * Left shoulder pitch
 * @param new_l_shoulder_pitch new l_shoulder_pitch value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_shoulder_pitch(const float new_l_shoulder_pitch)
{
  data->l_shoulder_pitch = new_l_shoulder_pitch;
}

/** Get l_shoulder_roll value.
 * Left shoulder roll
 * @return l_shoulder_roll value
 */
float
NaoJointPositionInterface::SetServosMessage::l_shoulder_roll() const
{
  return data->l_shoulder_roll;
}

/** Get maximum length of l_shoulder_roll value.
 * @return length of l_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_shoulder_roll() const
{
  return 1;
}

/** Set l_shoulder_roll value.
 * Left shoulder roll
 * @param new_l_shoulder_roll new l_shoulder_roll value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_shoulder_roll(const float new_l_shoulder_roll)
{
  data->l_shoulder_roll = new_l_shoulder_roll;
}

/** Get l_elbow_yaw value.
 * Left elbow yaw
 * @return l_elbow_yaw value
 */
float
NaoJointPositionInterface::SetServosMessage::l_elbow_yaw() const
{
  return data->l_elbow_yaw;
}

/** Get maximum length of l_elbow_yaw value.
 * @return length of l_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_elbow_yaw() const
{
  return 1;
}

/** Set l_elbow_yaw value.
 * Left elbow yaw
 * @param new_l_elbow_yaw new l_elbow_yaw value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_elbow_yaw(const float new_l_elbow_yaw)
{
  data->l_elbow_yaw = new_l_elbow_yaw;
}

/** Get l_elbow_roll value.
 * Left elbow roll
 * @return l_elbow_roll value
 */
float
NaoJointPositionInterface::SetServosMessage::l_elbow_roll() const
{
  return data->l_elbow_roll;
}

/** Get maximum length of l_elbow_roll value.
 * @return length of l_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_elbow_roll() const
{
  return 1;
}

/** Set l_elbow_roll value.
 * Left elbow roll
 * @param new_l_elbow_roll new l_elbow_roll value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_elbow_roll(const float new_l_elbow_roll)
{
  data->l_elbow_roll = new_l_elbow_roll;
}

/** Get l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @return l_hip_yaw_pitch value
 */
float
NaoJointPositionInterface::SetServosMessage::l_hip_yaw_pitch() const
{
  return data->l_hip_yaw_pitch;
}

/** Get maximum length of l_hip_yaw_pitch value.
 * @return length of l_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_hip_yaw_pitch() const
{
  return 1;
}

/** Set l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @param new_l_hip_yaw_pitch new l_hip_yaw_pitch value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_hip_yaw_pitch(const float new_l_hip_yaw_pitch)
{
  data->l_hip_yaw_pitch = new_l_hip_yaw_pitch;
}

/** Get l_hip_roll value.
 * Left hip roll
 * @return l_hip_roll value
 */
float
NaoJointPositionInterface::SetServosMessage::l_hip_roll() const
{
  return data->l_hip_roll;
}

/** Get maximum length of l_hip_roll value.
 * @return length of l_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_hip_roll() const
{
  return 1;
}

/** Set l_hip_roll value.
 * Left hip roll
 * @param new_l_hip_roll new l_hip_roll value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_hip_roll(const float new_l_hip_roll)
{
  data->l_hip_roll = new_l_hip_roll;
}

/** Get l_hip_pitch value.
 * Left hip pitch
 * @return l_hip_pitch value
 */
float
NaoJointPositionInterface::SetServosMessage::l_hip_pitch() const
{
  return data->l_hip_pitch;
}

/** Get maximum length of l_hip_pitch value.
 * @return length of l_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_hip_pitch() const
{
  return 1;
}

/** Set l_hip_pitch value.
 * Left hip pitch
 * @param new_l_hip_pitch new l_hip_pitch value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_hip_pitch(const float new_l_hip_pitch)
{
  data->l_hip_pitch = new_l_hip_pitch;
}

/** Get l_knee_pitch value.
 * Left knee pitch
 * @return l_knee_pitch value
 */
float
NaoJointPositionInterface::SetServosMessage::l_knee_pitch() const
{
  return data->l_knee_pitch;
}

/** Get maximum length of l_knee_pitch value.
 * @return length of l_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_knee_pitch() const
{
  return 1;
}

/** Set l_knee_pitch value.
 * Left knee pitch
 * @param new_l_knee_pitch new l_knee_pitch value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_knee_pitch(const float new_l_knee_pitch)
{
  data->l_knee_pitch = new_l_knee_pitch;
}

/** Get l_ankle_pitch value.
 * Left ankle pitch
 * @return l_ankle_pitch value
 */
float
NaoJointPositionInterface::SetServosMessage::l_ankle_pitch() const
{
  return data->l_ankle_pitch;
}

/** Get maximum length of l_ankle_pitch value.
 * @return length of l_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_ankle_pitch() const
{
  return 1;
}

/** Set l_ankle_pitch value.
 * Left ankle pitch
 * @param new_l_ankle_pitch new l_ankle_pitch value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_ankle_pitch(const float new_l_ankle_pitch)
{
  data->l_ankle_pitch = new_l_ankle_pitch;
}

/** Get l_ankle_roll value.
 * Left ankle roll
 * @return l_ankle_roll value
 */
float
NaoJointPositionInterface::SetServosMessage::l_ankle_roll() const
{
  return data->l_ankle_roll;
}

/** Get maximum length of l_ankle_roll value.
 * @return length of l_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_ankle_roll() const
{
  return 1;
}

/** Set l_ankle_roll value.
 * Left ankle roll
 * @param new_l_ankle_roll new l_ankle_roll value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_ankle_roll(const float new_l_ankle_roll)
{
  data->l_ankle_roll = new_l_ankle_roll;
}

/** Get l_wrist_yaw value.
 * Left wrist yaw
 * @return l_wrist_yaw value
 */
float
NaoJointPositionInterface::SetServosMessage::l_wrist_yaw() const
{
  return data->l_wrist_yaw;
}

/** Get maximum length of l_wrist_yaw value.
 * @return length of l_wrist_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_wrist_yaw() const
{
  return 1;
}

/** Set l_wrist_yaw value.
 * Left wrist yaw
 * @param new_l_wrist_yaw new l_wrist_yaw value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_wrist_yaw(const float new_l_wrist_yaw)
{
  data->l_wrist_yaw = new_l_wrist_yaw;
}

/** Get l_hand value.
 * Left hand
 * @return l_hand value
 */
float
NaoJointPositionInterface::SetServosMessage::l_hand() const
{
  return data->l_hand;
}

/** Get maximum length of l_hand value.
 * @return length of l_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_l_hand() const
{
  return 1;
}

/** Set l_hand value.
 * Left hand
 * @param new_l_hand new l_hand value
 */
void
NaoJointPositionInterface::SetServosMessage::set_l_hand(const float new_l_hand)
{
  data->l_hand = new_l_hand;
}

/** Get r_shoulder_pitch value.
 * Right shoulder pitch
 * @return r_shoulder_pitch value
 */
float
NaoJointPositionInterface::SetServosMessage::r_shoulder_pitch() const
{
  return data->r_shoulder_pitch;
}

/** Get maximum length of r_shoulder_pitch value.
 * @return length of r_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_shoulder_pitch() const
{
  return 1;
}

/** Set r_shoulder_pitch value.
 * Right shoulder pitch
 * @param new_r_shoulder_pitch new r_shoulder_pitch value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_shoulder_pitch(const float new_r_shoulder_pitch)
{
  data->r_shoulder_pitch = new_r_shoulder_pitch;
}

/** Get r_shoulder_roll value.
 * Right shoulder roll
 * @return r_shoulder_roll value
 */
float
NaoJointPositionInterface::SetServosMessage::r_shoulder_roll() const
{
  return data->r_shoulder_roll;
}

/** Get maximum length of r_shoulder_roll value.
 * @return length of r_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_shoulder_roll() const
{
  return 1;
}

/** Set r_shoulder_roll value.
 * Right shoulder roll
 * @param new_r_shoulder_roll new r_shoulder_roll value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_shoulder_roll(const float new_r_shoulder_roll)
{
  data->r_shoulder_roll = new_r_shoulder_roll;
}

/** Get r_elbow_yaw value.
 * Right elbow yaw
 * @return r_elbow_yaw value
 */
float
NaoJointPositionInterface::SetServosMessage::r_elbow_yaw() const
{
  return data->r_elbow_yaw;
}

/** Get maximum length of r_elbow_yaw value.
 * @return length of r_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_elbow_yaw() const
{
  return 1;
}

/** Set r_elbow_yaw value.
 * Right elbow yaw
 * @param new_r_elbow_yaw new r_elbow_yaw value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_elbow_yaw(const float new_r_elbow_yaw)
{
  data->r_elbow_yaw = new_r_elbow_yaw;
}

/** Get r_elbow_roll value.
 * Right elbow roll
 * @return r_elbow_roll value
 */
float
NaoJointPositionInterface::SetServosMessage::r_elbow_roll() const
{
  return data->r_elbow_roll;
}

/** Get maximum length of r_elbow_roll value.
 * @return length of r_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_elbow_roll() const
{
  return 1;
}

/** Set r_elbow_roll value.
 * Right elbow roll
 * @param new_r_elbow_roll new r_elbow_roll value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_elbow_roll(const float new_r_elbow_roll)
{
  data->r_elbow_roll = new_r_elbow_roll;
}

/** Get r_wrist_yaw value.
 * Right wrist yaw
 * @return r_wrist_yaw value
 */
float
NaoJointPositionInterface::SetServosMessage::r_wrist_yaw() const
{
  return data->r_wrist_yaw;
}

/** Get maximum length of r_wrist_yaw value.
 * @return length of r_wrist_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_wrist_yaw() const
{
  return 1;
}

/** Set r_wrist_yaw value.
 * Right wrist yaw
 * @param new_r_wrist_yaw new r_wrist_yaw value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_wrist_yaw(const float new_r_wrist_yaw)
{
  data->r_wrist_yaw = new_r_wrist_yaw;
}

/** Get r_hand value.
 * Right hand
 * @return r_hand value
 */
float
NaoJointPositionInterface::SetServosMessage::r_hand() const
{
  return data->r_hand;
}

/** Get maximum length of r_hand value.
 * @return length of r_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_hand() const
{
  return 1;
}

/** Set r_hand value.
 * Right hand
 * @param new_r_hand new r_hand value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_hand(const float new_r_hand)
{
  data->r_hand = new_r_hand;
}

/** Get r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @return r_hip_yaw_pitch value
 */
float
NaoJointPositionInterface::SetServosMessage::r_hip_yaw_pitch() const
{
  return data->r_hip_yaw_pitch;
}

/** Get maximum length of r_hip_yaw_pitch value.
 * @return length of r_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_hip_yaw_pitch() const
{
  return 1;
}

/** Set r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @param new_r_hip_yaw_pitch new r_hip_yaw_pitch value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_hip_yaw_pitch(const float new_r_hip_yaw_pitch)
{
  data->r_hip_yaw_pitch = new_r_hip_yaw_pitch;
}

/** Get r_hip_roll value.
 * Right hip roll
 * @return r_hip_roll value
 */
float
NaoJointPositionInterface::SetServosMessage::r_hip_roll() const
{
  return data->r_hip_roll;
}

/** Get maximum length of r_hip_roll value.
 * @return length of r_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_hip_roll() const
{
  return 1;
}

/** Set r_hip_roll value.
 * Right hip roll
 * @param new_r_hip_roll new r_hip_roll value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_hip_roll(const float new_r_hip_roll)
{
  data->r_hip_roll = new_r_hip_roll;
}

/** Get r_hip_pitch value.
 * Right hip pitch
 * @return r_hip_pitch value
 */
float
NaoJointPositionInterface::SetServosMessage::r_hip_pitch() const
{
  return data->r_hip_pitch;
}

/** Get maximum length of r_hip_pitch value.
 * @return length of r_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_hip_pitch() const
{
  return 1;
}

/** Set r_hip_pitch value.
 * Right hip pitch
 * @param new_r_hip_pitch new r_hip_pitch value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_hip_pitch(const float new_r_hip_pitch)
{
  data->r_hip_pitch = new_r_hip_pitch;
}

/** Get r_knee_pitch value.
 * Right knee pitch
 * @return r_knee_pitch value
 */
float
NaoJointPositionInterface::SetServosMessage::r_knee_pitch() const
{
  return data->r_knee_pitch;
}

/** Get maximum length of r_knee_pitch value.
 * @return length of r_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_knee_pitch() const
{
  return 1;
}

/** Set r_knee_pitch value.
 * Right knee pitch
 * @param new_r_knee_pitch new r_knee_pitch value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_knee_pitch(const float new_r_knee_pitch)
{
  data->r_knee_pitch = new_r_knee_pitch;
}

/** Get r_ankle_pitch value.
 * Right ankle pitch
 * @return r_ankle_pitch value
 */
float
NaoJointPositionInterface::SetServosMessage::r_ankle_pitch() const
{
  return data->r_ankle_pitch;
}

/** Get maximum length of r_ankle_pitch value.
 * @return length of r_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_ankle_pitch() const
{
  return 1;
}

/** Set r_ankle_pitch value.
 * Right ankle pitch
 * @param new_r_ankle_pitch new r_ankle_pitch value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_ankle_pitch(const float new_r_ankle_pitch)
{
  data->r_ankle_pitch = new_r_ankle_pitch;
}

/** Get r_ankle_roll value.
 * Right ankle roll
 * @return r_ankle_roll value
 */
float
NaoJointPositionInterface::SetServosMessage::r_ankle_roll() const
{
  return data->r_ankle_roll;
}

/** Get maximum length of r_ankle_roll value.
 * @return length of r_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_r_ankle_roll() const
{
  return 1;
}

/** Set r_ankle_roll value.
 * Right ankle roll
 * @param new_r_ankle_roll new r_ankle_roll value
 */
void
NaoJointPositionInterface::SetServosMessage::set_r_ankle_roll(const float new_r_ankle_roll)
{
  data->r_ankle_roll = new_r_ankle_roll;
}

/** Get time value.
 * 
      Current reference time in ms. For real hardware this is the DCM time.
      Times in messages are always offsets to the current time and the current
      time is added before executing the command.
    
 * @return time value
 */
int32_t
NaoJointPositionInterface::SetServosMessage::time() const
{
  return data->time;
}

/** Get maximum length of time value.
 * @return length of time value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::SetServosMessage::maxlenof_time() const
{
  return 1;
}

/** Set time value.
 * 
      Current reference time in ms. For real hardware this is the DCM time.
      Times in messages are always offsets to the current time and the current
      time is added before executing the command.
    
 * @param new_time new time value
 */
void
NaoJointPositionInterface::SetServosMessage::set_time(const int32_t new_time)
{
  data->time = new_time;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoJointPositionInterface::SetServosMessage::clone() const
{
  return new NaoJointPositionInterface::SetServosMessage(this);
}
/** @class NaoJointPositionInterface::GotoAngleMessage <interfaces/NaoJointPositionInterface.h>
 * GotoAngleMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_servo initial value for servo
 * @param ini_value initial value for value
 * @param ini_time_sec initial value for time_sec
 */
NaoJointPositionInterface::GotoAngleMessage::GotoAngleMessage(const uint32_t ini_servo, const float ini_value, const float ini_time_sec) : Message("GotoAngleMessage")
{
  data_size = sizeof(GotoAngleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAngleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->servo = ini_servo;
  data->value = ini_value;
  data->time_sec = ini_time_sec;
  add_fieldinfo(IFT_UINT32, "servo", 1, &data->servo);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
}
/** Constructor */
NaoJointPositionInterface::GotoAngleMessage::GotoAngleMessage() : Message("GotoAngleMessage")
{
  data_size = sizeof(GotoAngleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAngleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "servo", 1, &data->servo);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
}

/** Destructor */
NaoJointPositionInterface::GotoAngleMessage::~GotoAngleMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoJointPositionInterface::GotoAngleMessage::GotoAngleMessage(const GotoAngleMessage *m) : Message("GotoAngleMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GotoAngleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get servo value.
 * 
      A concatenated list of SERVO_* constants to define the servos
      that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
    
 * @return servo value
 */
uint32_t
NaoJointPositionInterface::GotoAngleMessage::servo() const
{
  return data->servo;
}

/** Get maximum length of servo value.
 * @return length of servo value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAngleMessage::maxlenof_servo() const
{
  return 1;
}

/** Set servo value.
 * 
      A concatenated list of SERVO_* constants to define the servos
      that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
    
 * @param new_servo new servo value
 */
void
NaoJointPositionInterface::GotoAngleMessage::set_servo(const uint32_t new_servo)
{
  data->servo = new_servo;
}

/** Get value value.
 * Servo value to set for servos.
 * @return value value
 */
float
NaoJointPositionInterface::GotoAngleMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAngleMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * Servo value to set for servos.
 * @param new_value new value value
 */
void
NaoJointPositionInterface::GotoAngleMessage::set_value(const float new_value)
{
  data->value = new_value;
}

/** Get time_sec value.
 * 
      Time in seconds when to reach the desired position.
    
 * @return time_sec value
 */
float
NaoJointPositionInterface::GotoAngleMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAngleMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * 
      Time in seconds when to reach the desired position.
    
 * @param new_time_sec new time_sec value
 */
void
NaoJointPositionInterface::GotoAngleMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoJointPositionInterface::GotoAngleMessage::clone() const
{
  return new NaoJointPositionInterface::GotoAngleMessage(this);
}
/** @class NaoJointPositionInterface::GotoAnglesMessage <interfaces/NaoJointPositionInterface.h>
 * GotoAnglesMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_time_sec initial value for time_sec
 * @param ini_head_yaw initial value for head_yaw
 * @param ini_head_pitch initial value for head_pitch
 * @param ini_l_shoulder_pitch initial value for l_shoulder_pitch
 * @param ini_l_shoulder_roll initial value for l_shoulder_roll
 * @param ini_l_elbow_yaw initial value for l_elbow_yaw
 * @param ini_l_elbow_roll initial value for l_elbow_roll
 * @param ini_l_wrist_yaw initial value for l_wrist_yaw
 * @param ini_l_hand initial value for l_hand
 * @param ini_l_hip_yaw_pitch initial value for l_hip_yaw_pitch
 * @param ini_l_hip_roll initial value for l_hip_roll
 * @param ini_l_hip_pitch initial value for l_hip_pitch
 * @param ini_l_knee_pitch initial value for l_knee_pitch
 * @param ini_l_ankle_pitch initial value for l_ankle_pitch
 * @param ini_l_ankle_roll initial value for l_ankle_roll
 * @param ini_r_shoulder_pitch initial value for r_shoulder_pitch
 * @param ini_r_shoulder_roll initial value for r_shoulder_roll
 * @param ini_r_elbow_yaw initial value for r_elbow_yaw
 * @param ini_r_elbow_roll initial value for r_elbow_roll
 * @param ini_r_wrist_yaw initial value for r_wrist_yaw
 * @param ini_r_hand initial value for r_hand
 * @param ini_r_hip_yaw_pitch initial value for r_hip_yaw_pitch
 * @param ini_r_hip_roll initial value for r_hip_roll
 * @param ini_r_hip_pitch initial value for r_hip_pitch
 * @param ini_r_knee_pitch initial value for r_knee_pitch
 * @param ini_r_ankle_pitch initial value for r_ankle_pitch
 * @param ini_r_ankle_roll initial value for r_ankle_roll
 */
NaoJointPositionInterface::GotoAnglesMessage::GotoAnglesMessage(const float ini_time_sec, const float ini_head_yaw, const float ini_head_pitch, const float ini_l_shoulder_pitch, const float ini_l_shoulder_roll, const float ini_l_elbow_yaw, const float ini_l_elbow_roll, const float ini_l_wrist_yaw, const float ini_l_hand, const float ini_l_hip_yaw_pitch, const float ini_l_hip_roll, const float ini_l_hip_pitch, const float ini_l_knee_pitch, const float ini_l_ankle_pitch, const float ini_l_ankle_roll, const float ini_r_shoulder_pitch, const float ini_r_shoulder_roll, const float ini_r_elbow_yaw, const float ini_r_elbow_roll, const float ini_r_wrist_yaw, const float ini_r_hand, const float ini_r_hip_yaw_pitch, const float ini_r_hip_roll, const float ini_r_hip_pitch, const float ini_r_knee_pitch, const float ini_r_ankle_pitch, const float ini_r_ankle_roll) : Message("GotoAnglesMessage")
{
  data_size = sizeof(GotoAnglesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAnglesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->time_sec = ini_time_sec;
  data->head_yaw = ini_head_yaw;
  data->head_pitch = ini_head_pitch;
  data->l_shoulder_pitch = ini_l_shoulder_pitch;
  data->l_shoulder_roll = ini_l_shoulder_roll;
  data->l_elbow_yaw = ini_l_elbow_yaw;
  data->l_elbow_roll = ini_l_elbow_roll;
  data->l_wrist_yaw = ini_l_wrist_yaw;
  data->l_hand = ini_l_hand;
  data->l_hip_yaw_pitch = ini_l_hip_yaw_pitch;
  data->l_hip_roll = ini_l_hip_roll;
  data->l_hip_pitch = ini_l_hip_pitch;
  data->l_knee_pitch = ini_l_knee_pitch;
  data->l_ankle_pitch = ini_l_ankle_pitch;
  data->l_ankle_roll = ini_l_ankle_roll;
  data->r_shoulder_pitch = ini_r_shoulder_pitch;
  data->r_shoulder_roll = ini_r_shoulder_roll;
  data->r_elbow_yaw = ini_r_elbow_yaw;
  data->r_elbow_roll = ini_r_elbow_roll;
  data->r_wrist_yaw = ini_r_wrist_yaw;
  data->r_hand = ini_r_hand;
  data->r_hip_yaw_pitch = ini_r_hip_yaw_pitch;
  data->r_hip_roll = ini_r_hip_roll;
  data->r_hip_pitch = ini_r_hip_pitch;
  data->r_knee_pitch = ini_r_knee_pitch;
  data->r_ankle_pitch = ini_r_ankle_pitch;
  data->r_ankle_roll = ini_r_ankle_roll;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "head_yaw", 1, &data->head_yaw);
  add_fieldinfo(IFT_FLOAT, "head_pitch", 1, &data->head_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_pitch", 1, &data->l_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_roll", 1, &data->l_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "l_elbow_yaw", 1, &data->l_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "l_elbow_roll", 1, &data->l_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "l_wrist_yaw", 1, &data->l_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "l_hand", 1, &data->l_hand);
  add_fieldinfo(IFT_FLOAT, "l_hip_yaw_pitch", 1, &data->l_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "l_hip_roll", 1, &data->l_hip_roll);
  add_fieldinfo(IFT_FLOAT, "l_hip_pitch", 1, &data->l_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "l_knee_pitch", 1, &data->l_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_pitch", 1, &data->l_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_roll", 1, &data->l_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "r_wrist_yaw", 1, &data->r_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "r_hand", 1, &data->r_hand);
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
}
/** Constructor */
NaoJointPositionInterface::GotoAnglesMessage::GotoAnglesMessage() : Message("GotoAnglesMessage")
{
  data_size = sizeof(GotoAnglesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAnglesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "head_yaw", 1, &data->head_yaw);
  add_fieldinfo(IFT_FLOAT, "head_pitch", 1, &data->head_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_pitch", 1, &data->l_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_roll", 1, &data->l_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "l_elbow_yaw", 1, &data->l_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "l_elbow_roll", 1, &data->l_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "l_wrist_yaw", 1, &data->l_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "l_hand", 1, &data->l_hand);
  add_fieldinfo(IFT_FLOAT, "l_hip_yaw_pitch", 1, &data->l_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "l_hip_roll", 1, &data->l_hip_roll);
  add_fieldinfo(IFT_FLOAT, "l_hip_pitch", 1, &data->l_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "l_knee_pitch", 1, &data->l_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_pitch", 1, &data->l_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_roll", 1, &data->l_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "r_wrist_yaw", 1, &data->r_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "r_hand", 1, &data->r_hand);
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
}

/** Destructor */
NaoJointPositionInterface::GotoAnglesMessage::~GotoAnglesMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoJointPositionInterface::GotoAnglesMessage::GotoAnglesMessage(const GotoAnglesMessage *m) : Message("GotoAnglesMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GotoAnglesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get time_sec value.
 * 
      Time in seconds when to reach the desired position.
    
 * @return time_sec value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * 
      Time in seconds when to reach the desired position.
    
 * @param new_time_sec new time_sec value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Get head_yaw value.
 * Head yaw
 * @return head_yaw value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::head_yaw() const
{
  return data->head_yaw;
}

/** Get maximum length of head_yaw value.
 * @return length of head_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_head_yaw() const
{
  return 1;
}

/** Set head_yaw value.
 * Head yaw
 * @param new_head_yaw new head_yaw value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_head_yaw(const float new_head_yaw)
{
  data->head_yaw = new_head_yaw;
}

/** Get head_pitch value.
 * Head pitch
 * @return head_pitch value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::head_pitch() const
{
  return data->head_pitch;
}

/** Get maximum length of head_pitch value.
 * @return length of head_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_head_pitch() const
{
  return 1;
}

/** Set head_pitch value.
 * Head pitch
 * @param new_head_pitch new head_pitch value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_head_pitch(const float new_head_pitch)
{
  data->head_pitch = new_head_pitch;
}

/** Get l_shoulder_pitch value.
 * Left shoulder pitch
 * @return l_shoulder_pitch value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_shoulder_pitch() const
{
  return data->l_shoulder_pitch;
}

/** Get maximum length of l_shoulder_pitch value.
 * @return length of l_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_shoulder_pitch() const
{
  return 1;
}

/** Set l_shoulder_pitch value.
 * Left shoulder pitch
 * @param new_l_shoulder_pitch new l_shoulder_pitch value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_shoulder_pitch(const float new_l_shoulder_pitch)
{
  data->l_shoulder_pitch = new_l_shoulder_pitch;
}

/** Get l_shoulder_roll value.
 * Left shoulder roll
 * @return l_shoulder_roll value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_shoulder_roll() const
{
  return data->l_shoulder_roll;
}

/** Get maximum length of l_shoulder_roll value.
 * @return length of l_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_shoulder_roll() const
{
  return 1;
}

/** Set l_shoulder_roll value.
 * Left shoulder roll
 * @param new_l_shoulder_roll new l_shoulder_roll value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_shoulder_roll(const float new_l_shoulder_roll)
{
  data->l_shoulder_roll = new_l_shoulder_roll;
}

/** Get l_elbow_yaw value.
 * Left elbow yaw
 * @return l_elbow_yaw value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_elbow_yaw() const
{
  return data->l_elbow_yaw;
}

/** Get maximum length of l_elbow_yaw value.
 * @return length of l_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_elbow_yaw() const
{
  return 1;
}

/** Set l_elbow_yaw value.
 * Left elbow yaw
 * @param new_l_elbow_yaw new l_elbow_yaw value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_elbow_yaw(const float new_l_elbow_yaw)
{
  data->l_elbow_yaw = new_l_elbow_yaw;
}

/** Get l_elbow_roll value.
 * Left elbow roll
 * @return l_elbow_roll value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_elbow_roll() const
{
  return data->l_elbow_roll;
}

/** Get maximum length of l_elbow_roll value.
 * @return length of l_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_elbow_roll() const
{
  return 1;
}

/** Set l_elbow_roll value.
 * Left elbow roll
 * @param new_l_elbow_roll new l_elbow_roll value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_elbow_roll(const float new_l_elbow_roll)
{
  data->l_elbow_roll = new_l_elbow_roll;
}

/** Get l_wrist_yaw value.
 * Left wrist yaw
 * @return l_wrist_yaw value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_wrist_yaw() const
{
  return data->l_wrist_yaw;
}

/** Get maximum length of l_wrist_yaw value.
 * @return length of l_wrist_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_wrist_yaw() const
{
  return 1;
}

/** Set l_wrist_yaw value.
 * Left wrist yaw
 * @param new_l_wrist_yaw new l_wrist_yaw value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_wrist_yaw(const float new_l_wrist_yaw)
{
  data->l_wrist_yaw = new_l_wrist_yaw;
}

/** Get l_hand value.
 * Left hand
 * @return l_hand value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_hand() const
{
  return data->l_hand;
}

/** Get maximum length of l_hand value.
 * @return length of l_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_hand() const
{
  return 1;
}

/** Set l_hand value.
 * Left hand
 * @param new_l_hand new l_hand value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_hand(const float new_l_hand)
{
  data->l_hand = new_l_hand;
}

/** Get l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @return l_hip_yaw_pitch value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_hip_yaw_pitch() const
{
  return data->l_hip_yaw_pitch;
}

/** Get maximum length of l_hip_yaw_pitch value.
 * @return length of l_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_hip_yaw_pitch() const
{
  return 1;
}

/** Set l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @param new_l_hip_yaw_pitch new l_hip_yaw_pitch value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_hip_yaw_pitch(const float new_l_hip_yaw_pitch)
{
  data->l_hip_yaw_pitch = new_l_hip_yaw_pitch;
}

/** Get l_hip_roll value.
 * Left hip roll
 * @return l_hip_roll value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_hip_roll() const
{
  return data->l_hip_roll;
}

/** Get maximum length of l_hip_roll value.
 * @return length of l_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_hip_roll() const
{
  return 1;
}

/** Set l_hip_roll value.
 * Left hip roll
 * @param new_l_hip_roll new l_hip_roll value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_hip_roll(const float new_l_hip_roll)
{
  data->l_hip_roll = new_l_hip_roll;
}

/** Get l_hip_pitch value.
 * Left hip pitch
 * @return l_hip_pitch value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_hip_pitch() const
{
  return data->l_hip_pitch;
}

/** Get maximum length of l_hip_pitch value.
 * @return length of l_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_hip_pitch() const
{
  return 1;
}

/** Set l_hip_pitch value.
 * Left hip pitch
 * @param new_l_hip_pitch new l_hip_pitch value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_hip_pitch(const float new_l_hip_pitch)
{
  data->l_hip_pitch = new_l_hip_pitch;
}

/** Get l_knee_pitch value.
 * Left knee pitch
 * @return l_knee_pitch value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_knee_pitch() const
{
  return data->l_knee_pitch;
}

/** Get maximum length of l_knee_pitch value.
 * @return length of l_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_knee_pitch() const
{
  return 1;
}

/** Set l_knee_pitch value.
 * Left knee pitch
 * @param new_l_knee_pitch new l_knee_pitch value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_knee_pitch(const float new_l_knee_pitch)
{
  data->l_knee_pitch = new_l_knee_pitch;
}

/** Get l_ankle_pitch value.
 * Left ankle pitch
 * @return l_ankle_pitch value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_ankle_pitch() const
{
  return data->l_ankle_pitch;
}

/** Get maximum length of l_ankle_pitch value.
 * @return length of l_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_ankle_pitch() const
{
  return 1;
}

/** Set l_ankle_pitch value.
 * Left ankle pitch
 * @param new_l_ankle_pitch new l_ankle_pitch value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_ankle_pitch(const float new_l_ankle_pitch)
{
  data->l_ankle_pitch = new_l_ankle_pitch;
}

/** Get l_ankle_roll value.
 * Left ankle roll
 * @return l_ankle_roll value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::l_ankle_roll() const
{
  return data->l_ankle_roll;
}

/** Get maximum length of l_ankle_roll value.
 * @return length of l_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_l_ankle_roll() const
{
  return 1;
}

/** Set l_ankle_roll value.
 * Left ankle roll
 * @param new_l_ankle_roll new l_ankle_roll value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_l_ankle_roll(const float new_l_ankle_roll)
{
  data->l_ankle_roll = new_l_ankle_roll;
}

/** Get r_shoulder_pitch value.
 * Right shoulder pitch
 * @return r_shoulder_pitch value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_shoulder_pitch() const
{
  return data->r_shoulder_pitch;
}

/** Get maximum length of r_shoulder_pitch value.
 * @return length of r_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_shoulder_pitch() const
{
  return 1;
}

/** Set r_shoulder_pitch value.
 * Right shoulder pitch
 * @param new_r_shoulder_pitch new r_shoulder_pitch value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_shoulder_pitch(const float new_r_shoulder_pitch)
{
  data->r_shoulder_pitch = new_r_shoulder_pitch;
}

/** Get r_shoulder_roll value.
 * Right shoulder roll
 * @return r_shoulder_roll value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_shoulder_roll() const
{
  return data->r_shoulder_roll;
}

/** Get maximum length of r_shoulder_roll value.
 * @return length of r_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_shoulder_roll() const
{
  return 1;
}

/** Set r_shoulder_roll value.
 * Right shoulder roll
 * @param new_r_shoulder_roll new r_shoulder_roll value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_shoulder_roll(const float new_r_shoulder_roll)
{
  data->r_shoulder_roll = new_r_shoulder_roll;
}

/** Get r_elbow_yaw value.
 * Right elbow yaw
 * @return r_elbow_yaw value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_elbow_yaw() const
{
  return data->r_elbow_yaw;
}

/** Get maximum length of r_elbow_yaw value.
 * @return length of r_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_elbow_yaw() const
{
  return 1;
}

/** Set r_elbow_yaw value.
 * Right elbow yaw
 * @param new_r_elbow_yaw new r_elbow_yaw value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_elbow_yaw(const float new_r_elbow_yaw)
{
  data->r_elbow_yaw = new_r_elbow_yaw;
}

/** Get r_elbow_roll value.
 * Right elbow roll
 * @return r_elbow_roll value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_elbow_roll() const
{
  return data->r_elbow_roll;
}

/** Get maximum length of r_elbow_roll value.
 * @return length of r_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_elbow_roll() const
{
  return 1;
}

/** Set r_elbow_roll value.
 * Right elbow roll
 * @param new_r_elbow_roll new r_elbow_roll value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_elbow_roll(const float new_r_elbow_roll)
{
  data->r_elbow_roll = new_r_elbow_roll;
}

/** Get r_wrist_yaw value.
 * Right wrist yaw
 * @return r_wrist_yaw value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_wrist_yaw() const
{
  return data->r_wrist_yaw;
}

/** Get maximum length of r_wrist_yaw value.
 * @return length of r_wrist_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_wrist_yaw() const
{
  return 1;
}

/** Set r_wrist_yaw value.
 * Right wrist yaw
 * @param new_r_wrist_yaw new r_wrist_yaw value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_wrist_yaw(const float new_r_wrist_yaw)
{
  data->r_wrist_yaw = new_r_wrist_yaw;
}

/** Get r_hand value.
 * Right hand
 * @return r_hand value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_hand() const
{
  return data->r_hand;
}

/** Get maximum length of r_hand value.
 * @return length of r_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_hand() const
{
  return 1;
}

/** Set r_hand value.
 * Right hand
 * @param new_r_hand new r_hand value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_hand(const float new_r_hand)
{
  data->r_hand = new_r_hand;
}

/** Get r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @return r_hip_yaw_pitch value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_hip_yaw_pitch() const
{
  return data->r_hip_yaw_pitch;
}

/** Get maximum length of r_hip_yaw_pitch value.
 * @return length of r_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_hip_yaw_pitch() const
{
  return 1;
}

/** Set r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @param new_r_hip_yaw_pitch new r_hip_yaw_pitch value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_hip_yaw_pitch(const float new_r_hip_yaw_pitch)
{
  data->r_hip_yaw_pitch = new_r_hip_yaw_pitch;
}

/** Get r_hip_roll value.
 * Right hip roll
 * @return r_hip_roll value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_hip_roll() const
{
  return data->r_hip_roll;
}

/** Get maximum length of r_hip_roll value.
 * @return length of r_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_hip_roll() const
{
  return 1;
}

/** Set r_hip_roll value.
 * Right hip roll
 * @param new_r_hip_roll new r_hip_roll value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_hip_roll(const float new_r_hip_roll)
{
  data->r_hip_roll = new_r_hip_roll;
}

/** Get r_hip_pitch value.
 * Right hip pitch
 * @return r_hip_pitch value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_hip_pitch() const
{
  return data->r_hip_pitch;
}

/** Get maximum length of r_hip_pitch value.
 * @return length of r_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_hip_pitch() const
{
  return 1;
}

/** Set r_hip_pitch value.
 * Right hip pitch
 * @param new_r_hip_pitch new r_hip_pitch value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_hip_pitch(const float new_r_hip_pitch)
{
  data->r_hip_pitch = new_r_hip_pitch;
}

/** Get r_knee_pitch value.
 * Right knee pitch
 * @return r_knee_pitch value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_knee_pitch() const
{
  return data->r_knee_pitch;
}

/** Get maximum length of r_knee_pitch value.
 * @return length of r_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_knee_pitch() const
{
  return 1;
}

/** Set r_knee_pitch value.
 * Right knee pitch
 * @param new_r_knee_pitch new r_knee_pitch value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_knee_pitch(const float new_r_knee_pitch)
{
  data->r_knee_pitch = new_r_knee_pitch;
}

/** Get r_ankle_pitch value.
 * Right ankle pitch
 * @return r_ankle_pitch value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_ankle_pitch() const
{
  return data->r_ankle_pitch;
}

/** Get maximum length of r_ankle_pitch value.
 * @return length of r_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_ankle_pitch() const
{
  return 1;
}

/** Set r_ankle_pitch value.
 * Right ankle pitch
 * @param new_r_ankle_pitch new r_ankle_pitch value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_ankle_pitch(const float new_r_ankle_pitch)
{
  data->r_ankle_pitch = new_r_ankle_pitch;
}

/** Get r_ankle_roll value.
 * Right ankle roll
 * @return r_ankle_roll value
 */
float
NaoJointPositionInterface::GotoAnglesMessage::r_ankle_roll() const
{
  return data->r_ankle_roll;
}

/** Get maximum length of r_ankle_roll value.
 * @return length of r_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAnglesMessage::maxlenof_r_ankle_roll() const
{
  return 1;
}

/** Set r_ankle_roll value.
 * Right ankle roll
 * @param new_r_ankle_roll new r_ankle_roll value
 */
void
NaoJointPositionInterface::GotoAnglesMessage::set_r_ankle_roll(const float new_r_ankle_roll)
{
  data->r_ankle_roll = new_r_ankle_roll;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoJointPositionInterface::GotoAnglesMessage::clone() const
{
  return new NaoJointPositionInterface::GotoAnglesMessage(this);
}
/** @class NaoJointPositionInterface::GotoAngleWithSpeedMessage <interfaces/NaoJointPositionInterface.h>
 * GotoAngleWithSpeedMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_servo initial value for servo
 * @param ini_value initial value for value
 * @param ini_speed initial value for speed
 */
NaoJointPositionInterface::GotoAngleWithSpeedMessage::GotoAngleWithSpeedMessage(const uint32_t ini_servo, const float ini_value, const uint16_t ini_speed) : Message("GotoAngleWithSpeedMessage")
{
  data_size = sizeof(GotoAngleWithSpeedMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAngleWithSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->servo = ini_servo;
  data->value = ini_value;
  data->speed = ini_speed;
  add_fieldinfo(IFT_UINT32, "servo", 1, &data->servo);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_UINT16, "speed", 1, &data->speed);
}
/** Constructor */
NaoJointPositionInterface::GotoAngleWithSpeedMessage::GotoAngleWithSpeedMessage() : Message("GotoAngleWithSpeedMessage")
{
  data_size = sizeof(GotoAngleWithSpeedMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAngleWithSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "servo", 1, &data->servo);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_UINT16, "speed", 1, &data->speed);
}

/** Destructor */
NaoJointPositionInterface::GotoAngleWithSpeedMessage::~GotoAngleWithSpeedMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoJointPositionInterface::GotoAngleWithSpeedMessage::GotoAngleWithSpeedMessage(const GotoAngleWithSpeedMessage *m) : Message("GotoAngleWithSpeedMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GotoAngleWithSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get servo value.
 * A concatenated list of SERVO_* constants to
      define the servos that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
 * @return servo value
 */
uint32_t
NaoJointPositionInterface::GotoAngleWithSpeedMessage::servo() const
{
  return data->servo;
}

/** Get maximum length of servo value.
 * @return length of servo value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAngleWithSpeedMessage::maxlenof_servo() const
{
  return 1;
}

/** Set servo value.
 * A concatenated list of SERVO_* constants to
      define the servos that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
 * @param new_servo new servo value
 */
void
NaoJointPositionInterface::GotoAngleWithSpeedMessage::set_servo(const uint32_t new_servo)
{
  data->servo = new_servo;
}

/** Get value value.
 * Servo value to set for servos.
 * @return value value
 */
float
NaoJointPositionInterface::GotoAngleWithSpeedMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAngleWithSpeedMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * Servo value to set for servos.
 * @param new_value new value value
 */
void
NaoJointPositionInterface::GotoAngleWithSpeedMessage::set_value(const float new_value)
{
  data->value = new_value;
}

/** Get speed value.
 * Percentage of the servo speed (1-100).
 * @return speed value
 */
uint16_t
NaoJointPositionInterface::GotoAngleWithSpeedMessage::speed() const
{
  return data->speed;
}

/** Get maximum length of speed value.
 * @return length of speed value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointPositionInterface::GotoAngleWithSpeedMessage::maxlenof_speed() const
{
  return 1;
}

/** Set speed value.
 * Percentage of the servo speed (1-100).
 * @param new_speed new speed value
 */
void
NaoJointPositionInterface::GotoAngleWithSpeedMessage::set_speed(const uint16_t new_speed)
{
  data->speed = new_speed;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoJointPositionInterface::GotoAngleWithSpeedMessage::clone() const
{
  return new NaoJointPositionInterface::GotoAngleWithSpeedMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
NaoJointPositionInterface::message_valid(const Message *message) const
{
  const SetServoMessage *m0 = dynamic_cast<const SetServoMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetServosMessage *m1 = dynamic_cast<const SetServosMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const GotoAngleMessage *m2 = dynamic_cast<const GotoAngleMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const GotoAnglesMessage *m3 = dynamic_cast<const GotoAnglesMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const GotoAngleWithSpeedMessage *m4 = dynamic_cast<const GotoAngleWithSpeedMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NaoJointPositionInterface)
/// @endcond


} // end namespace fawkes
