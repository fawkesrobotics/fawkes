
/***************************************************************************
 *  NaoHardwareInterface.cpp - Fawkes BlackBoard Interface - NaoHardwareInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
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

#include <interfaces/NaoHardwareInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class NaoHardwareInterface <interfaces/NaoHardwareInterface.h>
 * NaoHardwareInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to the hardware related data of a Nao robot.
    
 * @ingroup FawkesInterfaces
 */


/** SERVO_head_yaw constant */
const uint32_t NaoHardwareInterface::SERVO_head_yaw = 1u;
/** SERVO_head_pitch constant */
const uint32_t NaoHardwareInterface::SERVO_head_pitch = 2u;
/** SERVO_l_shoulder_pitch constant */
const uint32_t NaoHardwareInterface::SERVO_l_shoulder_pitch = 4u;
/** SERVO_l_shoulder_roll constant */
const uint32_t NaoHardwareInterface::SERVO_l_shoulder_roll = 8u;
/** SERVO_l_elbow_yaw constant */
const uint32_t NaoHardwareInterface::SERVO_l_elbow_yaw = 16u;
/** SERVO_l_elbow_roll constant */
const uint32_t NaoHardwareInterface::SERVO_l_elbow_roll = 32u;
/** SERVO_l_hip_yaw_pitch constant */
const uint32_t NaoHardwareInterface::SERVO_l_hip_yaw_pitch = 64u;
/** SERVO_l_hip_roll constant */
const uint32_t NaoHardwareInterface::SERVO_l_hip_roll = 128u;
/** SERVO_l_hip_pitch constant */
const uint32_t NaoHardwareInterface::SERVO_l_hip_pitch = 256u;
/** SERVO_l_knee_pitch constant */
const uint32_t NaoHardwareInterface::SERVO_l_knee_pitch = 512u;
/** SERVO_l_ankle_pitch constant */
const uint32_t NaoHardwareInterface::SERVO_l_ankle_pitch = 1024u;
/** SERVO_l_ankle_roll constant */
const uint32_t NaoHardwareInterface::SERVO_l_ankle_roll = 2048u;
/** SERVO_r_hip_yaw_pitch constant */
const uint32_t NaoHardwareInterface::SERVO_r_hip_yaw_pitch = 4096u;
/** SERVO_r_hip_roll constant */
const uint32_t NaoHardwareInterface::SERVO_r_hip_roll = 8192u;
/** SERVO_r_hip_pitch constant */
const uint32_t NaoHardwareInterface::SERVO_r_hip_pitch = 16384u;
/** SERVO_r_knee_pitch constant */
const uint32_t NaoHardwareInterface::SERVO_r_knee_pitch = 32768u;
/** SERVO_r_ankle_pitch constant */
const uint32_t NaoHardwareInterface::SERVO_r_ankle_pitch = 65536u;
/** SERVO_r_ankle_roll constant */
const uint32_t NaoHardwareInterface::SERVO_r_ankle_roll = 131072u;
/** SERVO_r_shoulder_pitch constant */
const uint32_t NaoHardwareInterface::SERVO_r_shoulder_pitch = 262144u;
/** SERVO_r_shoulder_roll constant */
const uint32_t NaoHardwareInterface::SERVO_r_shoulder_roll = 524288u;
/** SERVO_r_elbow_yaw constant */
const uint32_t NaoHardwareInterface::SERVO_r_elbow_yaw = 1048576u;
/** SERVO_r_elbow_roll constant */
const uint32_t NaoHardwareInterface::SERVO_r_elbow_roll = 2097152u;
/** SERVO_min constant */
const uint32_t NaoHardwareInterface::SERVO_min = 1u;
/** SERVO_max constant */
const uint32_t NaoHardwareInterface::SERVO_max = 2097152u;
/** USD_left_left constant */
const float NaoHardwareInterface::USD_left_left = 0.0;
/** USD_left_right constant */
const float NaoHardwareInterface::USD_left_right = 1.0;
/** USD_right_left constant */
const float NaoHardwareInterface::USD_right_left = 2.0;
/** USD_right_right constant */
const float NaoHardwareInterface::USD_right_right = 3.0;

/** Constructor */
NaoHardwareInterface::NaoHardwareInterface() : Interface()
{
  data_size = sizeof(NaoHardwareInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (NaoHardwareInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_BOOL, "servo_enabled", 1, &data->servo_enabled);
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
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "accel_x", 1, &data->accel_x);
  add_fieldinfo(IFT_FLOAT, "accel_y", 1, &data->accel_y);
  add_fieldinfo(IFT_FLOAT, "accel_z", 1, &data->accel_z);
  add_fieldinfo(IFT_FLOAT, "gyro_x", 1, &data->gyro_x);
  add_fieldinfo(IFT_FLOAT, "gyro_y", 1, &data->gyro_y);
  add_fieldinfo(IFT_FLOAT, "gyro_ref", 1, &data->gyro_ref);
  add_fieldinfo(IFT_FLOAT, "angle_x", 1, &data->angle_x);
  add_fieldinfo(IFT_FLOAT, "angle_y", 1, &data->angle_y);
  add_fieldinfo(IFT_FLOAT, "l_fsr_fl", 1, &data->l_fsr_fl);
  add_fieldinfo(IFT_FLOAT, "l_fsr_fr", 1, &data->l_fsr_fr);
  add_fieldinfo(IFT_FLOAT, "l_fsr_rl", 1, &data->l_fsr_rl);
  add_fieldinfo(IFT_FLOAT, "l_fsr_rr", 1, &data->l_fsr_rr);
  add_fieldinfo(IFT_FLOAT, "r_fsr_fl", 1, &data->r_fsr_fl);
  add_fieldinfo(IFT_FLOAT, "r_fsr_fr", 1, &data->r_fsr_fr);
  add_fieldinfo(IFT_FLOAT, "r_fsr_rl", 1, &data->r_fsr_rl);
  add_fieldinfo(IFT_FLOAT, "r_fsr_rr", 1, &data->r_fsr_rr);
  add_fieldinfo(IFT_FLOAT, "ultrasonic_distance", 1, &data->ultrasonic_distance);
  add_fieldinfo(IFT_FLOAT, "ultrasonic_direction", 1, &data->ultrasonic_direction);
  add_fieldinfo(IFT_FLOAT, "l_bumper_l", 1, &data->l_bumper_l);
  add_fieldinfo(IFT_FLOAT, "l_bumper_r", 1, &data->l_bumper_r);
  add_fieldinfo(IFT_FLOAT, "r_bumper_l", 1, &data->r_bumper_l);
  add_fieldinfo(IFT_FLOAT, "r_bumper_r", 1, &data->r_bumper_r);
  add_fieldinfo(IFT_FLOAT, "chest_button", 1, &data->chest_button);
  add_fieldinfo(IFT_FLOAT, "battery_charge", 1, &data->battery_charge);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
  add_messageinfo("SetServosMessage");
  add_messageinfo("GotoAnglesMessage");
  add_messageinfo("GotoAngleMessage");
  add_messageinfo("GotoAngleWithSpeedMessage");
  add_messageinfo("SetServoMessage");
  add_messageinfo("EnableServosMessage");
  add_messageinfo("DisableServosMessage");
  add_messageinfo("SetGlobalStiffnessMessage");
  add_messageinfo("EmitUltrasonicWaveMessage");
  unsigned char tmp_hash[] = {0x57, 0xd9, 0x69, 0xfd, 0x44, 0x7, 0x37, 0x62, 0xa0, 0x30, 0x47, 0xc3, 0xb5, 0xc9, 0xe4, 0xc0};
  set_hash(tmp_hash);
}

/** Destructor */
NaoHardwareInterface::~NaoHardwareInterface()
{
  free(data_ptr);
}
/** Convert InterpolationType constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
NaoHardwareInterface::tostring_InterpolationType(InterpolationType value) const
{
  switch (value) {
  case INTERPOLATION_LINEAR: return "INTERPOLATION_LINEAR";
  case INTERPOLATION_SMOOTH: return "INTERPOLATION_SMOOTH";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get servo_enabled value.
 * True if servos are enabled, false otherwise
 * @return servo_enabled value
 */
bool
NaoHardwareInterface::is_servo_enabled() const
{
  return data->servo_enabled;
}

/** Get maximum length of servo_enabled value.
 * @return length of servo_enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_servo_enabled() const
{
  return 1;
}

/** Set servo_enabled value.
 * True if servos are enabled, false otherwise
 * @param new_servo_enabled new servo_enabled value
 */
void
NaoHardwareInterface::set_servo_enabled(const bool new_servo_enabled)
{
  data->servo_enabled = new_servo_enabled;
  data_changed = true;
}

/** Get head_yaw value.
 * Head yaw
 * @return head_yaw value
 */
float
NaoHardwareInterface::head_yaw() const
{
  return data->head_yaw;
}

/** Get maximum length of head_yaw value.
 * @return length of head_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_head_yaw() const
{
  return 1;
}

/** Set head_yaw value.
 * Head yaw
 * @param new_head_yaw new head_yaw value
 */
void
NaoHardwareInterface::set_head_yaw(const float new_head_yaw)
{
  data->head_yaw = new_head_yaw;
  data_changed = true;
}

/** Get head_pitch value.
 * Head pitch
 * @return head_pitch value
 */
float
NaoHardwareInterface::head_pitch() const
{
  return data->head_pitch;
}

/** Get maximum length of head_pitch value.
 * @return length of head_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_head_pitch() const
{
  return 1;
}

/** Set head_pitch value.
 * Head pitch
 * @param new_head_pitch new head_pitch value
 */
void
NaoHardwareInterface::set_head_pitch(const float new_head_pitch)
{
  data->head_pitch = new_head_pitch;
  data_changed = true;
}

/** Get l_shoulder_pitch value.
 * Left shoulder pitch
 * @return l_shoulder_pitch value
 */
float
NaoHardwareInterface::l_shoulder_pitch() const
{
  return data->l_shoulder_pitch;
}

/** Get maximum length of l_shoulder_pitch value.
 * @return length of l_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_shoulder_pitch() const
{
  return 1;
}

/** Set l_shoulder_pitch value.
 * Left shoulder pitch
 * @param new_l_shoulder_pitch new l_shoulder_pitch value
 */
void
NaoHardwareInterface::set_l_shoulder_pitch(const float new_l_shoulder_pitch)
{
  data->l_shoulder_pitch = new_l_shoulder_pitch;
  data_changed = true;
}

/** Get l_shoulder_roll value.
 * Left shoulder roll
 * @return l_shoulder_roll value
 */
float
NaoHardwareInterface::l_shoulder_roll() const
{
  return data->l_shoulder_roll;
}

/** Get maximum length of l_shoulder_roll value.
 * @return length of l_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_shoulder_roll() const
{
  return 1;
}

/** Set l_shoulder_roll value.
 * Left shoulder roll
 * @param new_l_shoulder_roll new l_shoulder_roll value
 */
void
NaoHardwareInterface::set_l_shoulder_roll(const float new_l_shoulder_roll)
{
  data->l_shoulder_roll = new_l_shoulder_roll;
  data_changed = true;
}

/** Get l_elbow_yaw value.
 * Left elbow yaw
 * @return l_elbow_yaw value
 */
float
NaoHardwareInterface::l_elbow_yaw() const
{
  return data->l_elbow_yaw;
}

/** Get maximum length of l_elbow_yaw value.
 * @return length of l_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_elbow_yaw() const
{
  return 1;
}

/** Set l_elbow_yaw value.
 * Left elbow yaw
 * @param new_l_elbow_yaw new l_elbow_yaw value
 */
void
NaoHardwareInterface::set_l_elbow_yaw(const float new_l_elbow_yaw)
{
  data->l_elbow_yaw = new_l_elbow_yaw;
  data_changed = true;
}

/** Get l_elbow_roll value.
 * Left elbow roll
 * @return l_elbow_roll value
 */
float
NaoHardwareInterface::l_elbow_roll() const
{
  return data->l_elbow_roll;
}

/** Get maximum length of l_elbow_roll value.
 * @return length of l_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_elbow_roll() const
{
  return 1;
}

/** Set l_elbow_roll value.
 * Left elbow roll
 * @param new_l_elbow_roll new l_elbow_roll value
 */
void
NaoHardwareInterface::set_l_elbow_roll(const float new_l_elbow_roll)
{
  data->l_elbow_roll = new_l_elbow_roll;
  data_changed = true;
}

/** Get l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @return l_hip_yaw_pitch value
 */
float
NaoHardwareInterface::l_hip_yaw_pitch() const
{
  return data->l_hip_yaw_pitch;
}

/** Get maximum length of l_hip_yaw_pitch value.
 * @return length of l_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_hip_yaw_pitch() const
{
  return 1;
}

/** Set l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @param new_l_hip_yaw_pitch new l_hip_yaw_pitch value
 */
void
NaoHardwareInterface::set_l_hip_yaw_pitch(const float new_l_hip_yaw_pitch)
{
  data->l_hip_yaw_pitch = new_l_hip_yaw_pitch;
  data_changed = true;
}

/** Get l_hip_roll value.
 * Left hip roll
 * @return l_hip_roll value
 */
float
NaoHardwareInterface::l_hip_roll() const
{
  return data->l_hip_roll;
}

/** Get maximum length of l_hip_roll value.
 * @return length of l_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_hip_roll() const
{
  return 1;
}

/** Set l_hip_roll value.
 * Left hip roll
 * @param new_l_hip_roll new l_hip_roll value
 */
void
NaoHardwareInterface::set_l_hip_roll(const float new_l_hip_roll)
{
  data->l_hip_roll = new_l_hip_roll;
  data_changed = true;
}

/** Get l_hip_pitch value.
 * Left hip pitch
 * @return l_hip_pitch value
 */
float
NaoHardwareInterface::l_hip_pitch() const
{
  return data->l_hip_pitch;
}

/** Get maximum length of l_hip_pitch value.
 * @return length of l_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_hip_pitch() const
{
  return 1;
}

/** Set l_hip_pitch value.
 * Left hip pitch
 * @param new_l_hip_pitch new l_hip_pitch value
 */
void
NaoHardwareInterface::set_l_hip_pitch(const float new_l_hip_pitch)
{
  data->l_hip_pitch = new_l_hip_pitch;
  data_changed = true;
}

/** Get l_knee_pitch value.
 * Left knee pitch
 * @return l_knee_pitch value
 */
float
NaoHardwareInterface::l_knee_pitch() const
{
  return data->l_knee_pitch;
}

/** Get maximum length of l_knee_pitch value.
 * @return length of l_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_knee_pitch() const
{
  return 1;
}

/** Set l_knee_pitch value.
 * Left knee pitch
 * @param new_l_knee_pitch new l_knee_pitch value
 */
void
NaoHardwareInterface::set_l_knee_pitch(const float new_l_knee_pitch)
{
  data->l_knee_pitch = new_l_knee_pitch;
  data_changed = true;
}

/** Get l_ankle_pitch value.
 * Left ankle pitch
 * @return l_ankle_pitch value
 */
float
NaoHardwareInterface::l_ankle_pitch() const
{
  return data->l_ankle_pitch;
}

/** Get maximum length of l_ankle_pitch value.
 * @return length of l_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_ankle_pitch() const
{
  return 1;
}

/** Set l_ankle_pitch value.
 * Left ankle pitch
 * @param new_l_ankle_pitch new l_ankle_pitch value
 */
void
NaoHardwareInterface::set_l_ankle_pitch(const float new_l_ankle_pitch)
{
  data->l_ankle_pitch = new_l_ankle_pitch;
  data_changed = true;
}

/** Get l_ankle_roll value.
 * Left ankle roll
 * @return l_ankle_roll value
 */
float
NaoHardwareInterface::l_ankle_roll() const
{
  return data->l_ankle_roll;
}

/** Get maximum length of l_ankle_roll value.
 * @return length of l_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_ankle_roll() const
{
  return 1;
}

/** Set l_ankle_roll value.
 * Left ankle roll
 * @param new_l_ankle_roll new l_ankle_roll value
 */
void
NaoHardwareInterface::set_l_ankle_roll(const float new_l_ankle_roll)
{
  data->l_ankle_roll = new_l_ankle_roll;
  data_changed = true;
}

/** Get r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @return r_hip_yaw_pitch value
 */
float
NaoHardwareInterface::r_hip_yaw_pitch() const
{
  return data->r_hip_yaw_pitch;
}

/** Get maximum length of r_hip_yaw_pitch value.
 * @return length of r_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_hip_yaw_pitch() const
{
  return 1;
}

/** Set r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @param new_r_hip_yaw_pitch new r_hip_yaw_pitch value
 */
void
NaoHardwareInterface::set_r_hip_yaw_pitch(const float new_r_hip_yaw_pitch)
{
  data->r_hip_yaw_pitch = new_r_hip_yaw_pitch;
  data_changed = true;
}

/** Get r_hip_roll value.
 * Right hip roll
 * @return r_hip_roll value
 */
float
NaoHardwareInterface::r_hip_roll() const
{
  return data->r_hip_roll;
}

/** Get maximum length of r_hip_roll value.
 * @return length of r_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_hip_roll() const
{
  return 1;
}

/** Set r_hip_roll value.
 * Right hip roll
 * @param new_r_hip_roll new r_hip_roll value
 */
void
NaoHardwareInterface::set_r_hip_roll(const float new_r_hip_roll)
{
  data->r_hip_roll = new_r_hip_roll;
  data_changed = true;
}

/** Get r_hip_pitch value.
 * Right hip pitch
 * @return r_hip_pitch value
 */
float
NaoHardwareInterface::r_hip_pitch() const
{
  return data->r_hip_pitch;
}

/** Get maximum length of r_hip_pitch value.
 * @return length of r_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_hip_pitch() const
{
  return 1;
}

/** Set r_hip_pitch value.
 * Right hip pitch
 * @param new_r_hip_pitch new r_hip_pitch value
 */
void
NaoHardwareInterface::set_r_hip_pitch(const float new_r_hip_pitch)
{
  data->r_hip_pitch = new_r_hip_pitch;
  data_changed = true;
}

/** Get r_knee_pitch value.
 * Right knee pitch
 * @return r_knee_pitch value
 */
float
NaoHardwareInterface::r_knee_pitch() const
{
  return data->r_knee_pitch;
}

/** Get maximum length of r_knee_pitch value.
 * @return length of r_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_knee_pitch() const
{
  return 1;
}

/** Set r_knee_pitch value.
 * Right knee pitch
 * @param new_r_knee_pitch new r_knee_pitch value
 */
void
NaoHardwareInterface::set_r_knee_pitch(const float new_r_knee_pitch)
{
  data->r_knee_pitch = new_r_knee_pitch;
  data_changed = true;
}

/** Get r_ankle_pitch value.
 * Right ankle pitch
 * @return r_ankle_pitch value
 */
float
NaoHardwareInterface::r_ankle_pitch() const
{
  return data->r_ankle_pitch;
}

/** Get maximum length of r_ankle_pitch value.
 * @return length of r_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_ankle_pitch() const
{
  return 1;
}

/** Set r_ankle_pitch value.
 * Right ankle pitch
 * @param new_r_ankle_pitch new r_ankle_pitch value
 */
void
NaoHardwareInterface::set_r_ankle_pitch(const float new_r_ankle_pitch)
{
  data->r_ankle_pitch = new_r_ankle_pitch;
  data_changed = true;
}

/** Get r_ankle_roll value.
 * Right ankle roll
 * @return r_ankle_roll value
 */
float
NaoHardwareInterface::r_ankle_roll() const
{
  return data->r_ankle_roll;
}

/** Get maximum length of r_ankle_roll value.
 * @return length of r_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_ankle_roll() const
{
  return 1;
}

/** Set r_ankle_roll value.
 * Right ankle roll
 * @param new_r_ankle_roll new r_ankle_roll value
 */
void
NaoHardwareInterface::set_r_ankle_roll(const float new_r_ankle_roll)
{
  data->r_ankle_roll = new_r_ankle_roll;
  data_changed = true;
}

/** Get r_shoulder_pitch value.
 * Right shoulder pitch
 * @return r_shoulder_pitch value
 */
float
NaoHardwareInterface::r_shoulder_pitch() const
{
  return data->r_shoulder_pitch;
}

/** Get maximum length of r_shoulder_pitch value.
 * @return length of r_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_shoulder_pitch() const
{
  return 1;
}

/** Set r_shoulder_pitch value.
 * Right shoulder pitch
 * @param new_r_shoulder_pitch new r_shoulder_pitch value
 */
void
NaoHardwareInterface::set_r_shoulder_pitch(const float new_r_shoulder_pitch)
{
  data->r_shoulder_pitch = new_r_shoulder_pitch;
  data_changed = true;
}

/** Get r_shoulder_roll value.
 * Right shoulder roll
 * @return r_shoulder_roll value
 */
float
NaoHardwareInterface::r_shoulder_roll() const
{
  return data->r_shoulder_roll;
}

/** Get maximum length of r_shoulder_roll value.
 * @return length of r_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_shoulder_roll() const
{
  return 1;
}

/** Set r_shoulder_roll value.
 * Right shoulder roll
 * @param new_r_shoulder_roll new r_shoulder_roll value
 */
void
NaoHardwareInterface::set_r_shoulder_roll(const float new_r_shoulder_roll)
{
  data->r_shoulder_roll = new_r_shoulder_roll;
  data_changed = true;
}

/** Get r_elbow_yaw value.
 * Right elbow yaw
 * @return r_elbow_yaw value
 */
float
NaoHardwareInterface::r_elbow_yaw() const
{
  return data->r_elbow_yaw;
}

/** Get maximum length of r_elbow_yaw value.
 * @return length of r_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_elbow_yaw() const
{
  return 1;
}

/** Set r_elbow_yaw value.
 * Right elbow yaw
 * @param new_r_elbow_yaw new r_elbow_yaw value
 */
void
NaoHardwareInterface::set_r_elbow_yaw(const float new_r_elbow_yaw)
{
  data->r_elbow_yaw = new_r_elbow_yaw;
  data_changed = true;
}

/** Get r_elbow_roll value.
 * Right elbow roll
 * @return r_elbow_roll value
 */
float
NaoHardwareInterface::r_elbow_roll() const
{
  return data->r_elbow_roll;
}

/** Get maximum length of r_elbow_roll value.
 * @return length of r_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_elbow_roll() const
{
  return 1;
}

/** Set r_elbow_roll value.
 * Right elbow roll
 * @param new_r_elbow_roll new r_elbow_roll value
 */
void
NaoHardwareInterface::set_r_elbow_roll(const float new_r_elbow_roll)
{
  data->r_elbow_roll = new_r_elbow_roll;
  data_changed = true;
}

/** Get accel_x value.
 * Accelerometer x
 * @return accel_x value
 */
float
NaoHardwareInterface::accel_x() const
{
  return data->accel_x;
}

/** Get maximum length of accel_x value.
 * @return length of accel_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_accel_x() const
{
  return 1;
}

/** Set accel_x value.
 * Accelerometer x
 * @param new_accel_x new accel_x value
 */
void
NaoHardwareInterface::set_accel_x(const float new_accel_x)
{
  data->accel_x = new_accel_x;
  data_changed = true;
}

/** Get accel_y value.
 * Accelerometer y
 * @return accel_y value
 */
float
NaoHardwareInterface::accel_y() const
{
  return data->accel_y;
}

/** Get maximum length of accel_y value.
 * @return length of accel_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_accel_y() const
{
  return 1;
}

/** Set accel_y value.
 * Accelerometer y
 * @param new_accel_y new accel_y value
 */
void
NaoHardwareInterface::set_accel_y(const float new_accel_y)
{
  data->accel_y = new_accel_y;
  data_changed = true;
}

/** Get accel_z value.
 * Accelerometer z
 * @return accel_z value
 */
float
NaoHardwareInterface::accel_z() const
{
  return data->accel_z;
}

/** Get maximum length of accel_z value.
 * @return length of accel_z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_accel_z() const
{
  return 1;
}

/** Set accel_z value.
 * Accelerometer z
 * @param new_accel_z new accel_z value
 */
void
NaoHardwareInterface::set_accel_z(const float new_accel_z)
{
  data->accel_z = new_accel_z;
  data_changed = true;
}

/** Get gyro_x value.
 * Gyrometer x
 * @return gyro_x value
 */
float
NaoHardwareInterface::gyro_x() const
{
  return data->gyro_x;
}

/** Get maximum length of gyro_x value.
 * @return length of gyro_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_gyro_x() const
{
  return 1;
}

/** Set gyro_x value.
 * Gyrometer x
 * @param new_gyro_x new gyro_x value
 */
void
NaoHardwareInterface::set_gyro_x(const float new_gyro_x)
{
  data->gyro_x = new_gyro_x;
  data_changed = true;
}

/** Get gyro_y value.
 * Gyrometer y
 * @return gyro_y value
 */
float
NaoHardwareInterface::gyro_y() const
{
  return data->gyro_y;
}

/** Get maximum length of gyro_y value.
 * @return length of gyro_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_gyro_y() const
{
  return 1;
}

/** Set gyro_y value.
 * Gyrometer y
 * @param new_gyro_y new gyro_y value
 */
void
NaoHardwareInterface::set_gyro_y(const float new_gyro_y)
{
  data->gyro_y = new_gyro_y;
  data_changed = true;
}

/** Get gyro_ref value.
 * Gyrometer reference
 * @return gyro_ref value
 */
float
NaoHardwareInterface::gyro_ref() const
{
  return data->gyro_ref;
}

/** Get maximum length of gyro_ref value.
 * @return length of gyro_ref value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_gyro_ref() const
{
  return 1;
}

/** Set gyro_ref value.
 * Gyrometer reference
 * @param new_gyro_ref new gyro_ref value
 */
void
NaoHardwareInterface::set_gyro_ref(const float new_gyro_ref)
{
  data->gyro_ref = new_gyro_ref;
  data_changed = true;
}

/** Get angle_x value.
 * Angle x
 * @return angle_x value
 */
float
NaoHardwareInterface::angle_x() const
{
  return data->angle_x;
}

/** Get maximum length of angle_x value.
 * @return length of angle_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_angle_x() const
{
  return 1;
}

/** Set angle_x value.
 * Angle x
 * @param new_angle_x new angle_x value
 */
void
NaoHardwareInterface::set_angle_x(const float new_angle_x)
{
  data->angle_x = new_angle_x;
  data_changed = true;
}

/** Get angle_y value.
 * Angle y
 * @return angle_y value
 */
float
NaoHardwareInterface::angle_y() const
{
  return data->angle_y;
}

/** Get maximum length of angle_y value.
 * @return length of angle_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_angle_y() const
{
  return 1;
}

/** Set angle_y value.
 * Angle y
 * @param new_angle_y new angle_y value
 */
void
NaoHardwareInterface::set_angle_y(const float new_angle_y)
{
  data->angle_y = new_angle_y;
  data_changed = true;
}

/** Get l_fsr_fl value.
 * Left FSR front left
 * @return l_fsr_fl value
 */
float
NaoHardwareInterface::l_fsr_fl() const
{
  return data->l_fsr_fl;
}

/** Get maximum length of l_fsr_fl value.
 * @return length of l_fsr_fl value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_fsr_fl() const
{
  return 1;
}

/** Set l_fsr_fl value.
 * Left FSR front left
 * @param new_l_fsr_fl new l_fsr_fl value
 */
void
NaoHardwareInterface::set_l_fsr_fl(const float new_l_fsr_fl)
{
  data->l_fsr_fl = new_l_fsr_fl;
  data_changed = true;
}

/** Get l_fsr_fr value.
 * Left FSR front right
 * @return l_fsr_fr value
 */
float
NaoHardwareInterface::l_fsr_fr() const
{
  return data->l_fsr_fr;
}

/** Get maximum length of l_fsr_fr value.
 * @return length of l_fsr_fr value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_fsr_fr() const
{
  return 1;
}

/** Set l_fsr_fr value.
 * Left FSR front right
 * @param new_l_fsr_fr new l_fsr_fr value
 */
void
NaoHardwareInterface::set_l_fsr_fr(const float new_l_fsr_fr)
{
  data->l_fsr_fr = new_l_fsr_fr;
  data_changed = true;
}

/** Get l_fsr_rl value.
 * Left FSR rear left
 * @return l_fsr_rl value
 */
float
NaoHardwareInterface::l_fsr_rl() const
{
  return data->l_fsr_rl;
}

/** Get maximum length of l_fsr_rl value.
 * @return length of l_fsr_rl value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_fsr_rl() const
{
  return 1;
}

/** Set l_fsr_rl value.
 * Left FSR rear left
 * @param new_l_fsr_rl new l_fsr_rl value
 */
void
NaoHardwareInterface::set_l_fsr_rl(const float new_l_fsr_rl)
{
  data->l_fsr_rl = new_l_fsr_rl;
  data_changed = true;
}

/** Get l_fsr_rr value.
 * Left FSR rear right
 * @return l_fsr_rr value
 */
float
NaoHardwareInterface::l_fsr_rr() const
{
  return data->l_fsr_rr;
}

/** Get maximum length of l_fsr_rr value.
 * @return length of l_fsr_rr value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_fsr_rr() const
{
  return 1;
}

/** Set l_fsr_rr value.
 * Left FSR rear right
 * @param new_l_fsr_rr new l_fsr_rr value
 */
void
NaoHardwareInterface::set_l_fsr_rr(const float new_l_fsr_rr)
{
  data->l_fsr_rr = new_l_fsr_rr;
  data_changed = true;
}

/** Get r_fsr_fl value.
 * Right FSR front left
 * @return r_fsr_fl value
 */
float
NaoHardwareInterface::r_fsr_fl() const
{
  return data->r_fsr_fl;
}

/** Get maximum length of r_fsr_fl value.
 * @return length of r_fsr_fl value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_fsr_fl() const
{
  return 1;
}

/** Set r_fsr_fl value.
 * Right FSR front left
 * @param new_r_fsr_fl new r_fsr_fl value
 */
void
NaoHardwareInterface::set_r_fsr_fl(const float new_r_fsr_fl)
{
  data->r_fsr_fl = new_r_fsr_fl;
  data_changed = true;
}

/** Get r_fsr_fr value.
 * Right FSR front right
 * @return r_fsr_fr value
 */
float
NaoHardwareInterface::r_fsr_fr() const
{
  return data->r_fsr_fr;
}

/** Get maximum length of r_fsr_fr value.
 * @return length of r_fsr_fr value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_fsr_fr() const
{
  return 1;
}

/** Set r_fsr_fr value.
 * Right FSR front right
 * @param new_r_fsr_fr new r_fsr_fr value
 */
void
NaoHardwareInterface::set_r_fsr_fr(const float new_r_fsr_fr)
{
  data->r_fsr_fr = new_r_fsr_fr;
  data_changed = true;
}

/** Get r_fsr_rl value.
 * Right FSR rear left
 * @return r_fsr_rl value
 */
float
NaoHardwareInterface::r_fsr_rl() const
{
  return data->r_fsr_rl;
}

/** Get maximum length of r_fsr_rl value.
 * @return length of r_fsr_rl value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_fsr_rl() const
{
  return 1;
}

/** Set r_fsr_rl value.
 * Right FSR rear left
 * @param new_r_fsr_rl new r_fsr_rl value
 */
void
NaoHardwareInterface::set_r_fsr_rl(const float new_r_fsr_rl)
{
  data->r_fsr_rl = new_r_fsr_rl;
  data_changed = true;
}

/** Get r_fsr_rr value.
 * Right FSR rear right
 * @return r_fsr_rr value
 */
float
NaoHardwareInterface::r_fsr_rr() const
{
  return data->r_fsr_rr;
}

/** Get maximum length of r_fsr_rr value.
 * @return length of r_fsr_rr value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_fsr_rr() const
{
  return 1;
}

/** Set r_fsr_rr value.
 * Right FSR rear right
 * @param new_r_fsr_rr new r_fsr_rr value
 */
void
NaoHardwareInterface::set_r_fsr_rr(const float new_r_fsr_rr)
{
  data->r_fsr_rr = new_r_fsr_rr;
  data_changed = true;
}

/** Get ultrasonic_distance value.
 * Ultrasonic sensor reading
 * @return ultrasonic_distance value
 */
float
NaoHardwareInterface::ultrasonic_distance() const
{
  return data->ultrasonic_distance;
}

/** Get maximum length of ultrasonic_distance value.
 * @return length of ultrasonic_distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_ultrasonic_distance() const
{
  return 1;
}

/** Set ultrasonic_distance value.
 * Ultrasonic sensor reading
 * @param new_ultrasonic_distance new ultrasonic_distance value
 */
void
NaoHardwareInterface::set_ultrasonic_distance(const float new_ultrasonic_distance)
{
  data->ultrasonic_distance = new_ultrasonic_distance;
  data_changed = true;
}

/** Get ultrasonic_direction value.
 * Direction that was used to gather the ultrasonic reading.
 * @return ultrasonic_direction value
 */
float
NaoHardwareInterface::ultrasonic_direction() const
{
  return data->ultrasonic_direction;
}

/** Get maximum length of ultrasonic_direction value.
 * @return length of ultrasonic_direction value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_ultrasonic_direction() const
{
  return 1;
}

/** Set ultrasonic_direction value.
 * Direction that was used to gather the ultrasonic reading.
 * @param new_ultrasonic_direction new ultrasonic_direction value
 */
void
NaoHardwareInterface::set_ultrasonic_direction(const float new_ultrasonic_direction)
{
  data->ultrasonic_direction = new_ultrasonic_direction;
  data_changed = true;
}

/** Get l_bumper_l value.
 * Left foot bumper left side
 * @return l_bumper_l value
 */
float
NaoHardwareInterface::l_bumper_l() const
{
  return data->l_bumper_l;
}

/** Get maximum length of l_bumper_l value.
 * @return length of l_bumper_l value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_bumper_l() const
{
  return 1;
}

/** Set l_bumper_l value.
 * Left foot bumper left side
 * @param new_l_bumper_l new l_bumper_l value
 */
void
NaoHardwareInterface::set_l_bumper_l(const float new_l_bumper_l)
{
  data->l_bumper_l = new_l_bumper_l;
  data_changed = true;
}

/** Get l_bumper_r value.
 * Left foot bumper right side
 * @return l_bumper_r value
 */
float
NaoHardwareInterface::l_bumper_r() const
{
  return data->l_bumper_r;
}

/** Get maximum length of l_bumper_r value.
 * @return length of l_bumper_r value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_l_bumper_r() const
{
  return 1;
}

/** Set l_bumper_r value.
 * Left foot bumper right side
 * @param new_l_bumper_r new l_bumper_r value
 */
void
NaoHardwareInterface::set_l_bumper_r(const float new_l_bumper_r)
{
  data->l_bumper_r = new_l_bumper_r;
  data_changed = true;
}

/** Get r_bumper_l value.
 * Right foot bumper left side
 * @return r_bumper_l value
 */
float
NaoHardwareInterface::r_bumper_l() const
{
  return data->r_bumper_l;
}

/** Get maximum length of r_bumper_l value.
 * @return length of r_bumper_l value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_bumper_l() const
{
  return 1;
}

/** Set r_bumper_l value.
 * Right foot bumper left side
 * @param new_r_bumper_l new r_bumper_l value
 */
void
NaoHardwareInterface::set_r_bumper_l(const float new_r_bumper_l)
{
  data->r_bumper_l = new_r_bumper_l;
  data_changed = true;
}

/** Get r_bumper_r value.
 * Right foot bumper right side
 * @return r_bumper_r value
 */
float
NaoHardwareInterface::r_bumper_r() const
{
  return data->r_bumper_r;
}

/** Get maximum length of r_bumper_r value.
 * @return length of r_bumper_r value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_r_bumper_r() const
{
  return 1;
}

/** Set r_bumper_r value.
 * Right foot bumper right side
 * @param new_r_bumper_r new r_bumper_r value
 */
void
NaoHardwareInterface::set_r_bumper_r(const float new_r_bumper_r)
{
  data->r_bumper_r = new_r_bumper_r;
  data_changed = true;
}

/** Get chest_button value.
 * Chest button state
 * @return chest_button value
 */
float
NaoHardwareInterface::chest_button() const
{
  return data->chest_button;
}

/** Get maximum length of chest_button value.
 * @return length of chest_button value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_chest_button() const
{
  return 1;
}

/** Set chest_button value.
 * Chest button state
 * @param new_chest_button new chest_button value
 */
void
NaoHardwareInterface::set_chest_button(const float new_chest_button)
{
  data->chest_button = new_chest_button;
  data_changed = true;
}

/** Get battery_charge value.
 * Battery charge
 * @return battery_charge value
 */
float
NaoHardwareInterface::battery_charge() const
{
  return data->battery_charge;
}

/** Get maximum length of battery_charge value.
 * @return length of battery_charge value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_battery_charge() const
{
  return 1;
}

/** Set battery_charge value.
 * Battery charge
 * @param new_battery_charge new battery_charge value
 */
void
NaoHardwareInterface::set_battery_charge(const float new_battery_charge)
{
  data->battery_charge = new_battery_charge;
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
NaoHardwareInterface::time() const
{
  return data->time;
}

/** Get maximum length of time value.
 * @return length of time value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::maxlenof_time() const
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
NaoHardwareInterface::set_time(const int32_t new_time)
{
  data->time = new_time;
  data_changed = true;
}

/** Get servo_value value.
 * 
      
 * @param key key of the value
 * @return servo_value value
 */
float
NaoHardwareInterface::servo_value(const uint32_t key) const
{
  if (key == 1) {
    return data->head_yaw;
  } else if (key == 2) {
    return data->head_pitch;
  } else if (key == 4) {
    return data->l_shoulder_pitch;
  } else if (key == 8) {
    return data->l_shoulder_roll;
  } else if (key == 16) {
    return data->l_elbow_yaw;
  } else if (key == 32) {
    return data->l_elbow_roll;
  } else if (key == 64) {
    return data->l_hip_yaw_pitch;
  } else if (key == 128) {
    return data->l_hip_roll;
  } else if (key == 256) {
    return data->l_hip_pitch;
  } else if (key == 512) {
    return data->l_knee_pitch;
  } else if (key == 1024) {
    return data->l_ankle_pitch;
  } else if (key == 2048) {
    return data->l_ankle_roll;
  } else if (key == 4096) {
    return data->r_hip_yaw_pitch;
  } else if (key == 8192) {
    return data->r_hip_roll;
  } else if (key == 16384) {
    return data->r_hip_pitch;
  } else if (key == 32768) {
    return data->r_knee_pitch;
  } else if (key == 65536) {
    return data->r_ankle_pitch;
  } else if (key == 131072) {
    return data->r_ankle_roll;
  } else if (key == 262144) {
    return data->r_shoulder_pitch;
  } else if (key == 524288) {
    return data->r_shoulder_roll;
  } else if (key == 1048576) {
    return data->r_elbow_yaw;
  } else if (key == 2097152) {
    return data->r_elbow_roll;
  } else {
    throw Exception("Invalid key, cannot retrieve value");
  }
}

/** Set servo_value value.
 * 
      
 * @param key key of the value
 * @param new_value new value
 */
void
NaoHardwareInterface::set_servo_value(const uint32_t key, const float new_value)
{
  if (key == 1) {
    data->head_yaw = new_value;
  } else if (key == 2) {
    data->head_pitch = new_value;
  } else if (key == 4) {
    data->l_shoulder_pitch = new_value;
  } else if (key == 8) {
    data->l_shoulder_roll = new_value;
  } else if (key == 16) {
    data->l_elbow_yaw = new_value;
  } else if (key == 32) {
    data->l_elbow_roll = new_value;
  } else if (key == 64) {
    data->l_hip_yaw_pitch = new_value;
  } else if (key == 128) {
    data->l_hip_roll = new_value;
  } else if (key == 256) {
    data->l_hip_pitch = new_value;
  } else if (key == 512) {
    data->l_knee_pitch = new_value;
  } else if (key == 1024) {
    data->l_ankle_pitch = new_value;
  } else if (key == 2048) {
    data->l_ankle_roll = new_value;
  } else if (key == 4096) {
    data->r_hip_yaw_pitch = new_value;
  } else if (key == 8192) {
    data->r_hip_roll = new_value;
  } else if (key == 16384) {
    data->r_hip_pitch = new_value;
  } else if (key == 32768) {
    data->r_knee_pitch = new_value;
  } else if (key == 65536) {
    data->r_ankle_pitch = new_value;
  } else if (key == 131072) {
    data->r_ankle_roll = new_value;
  } else if (key == 262144) {
    data->r_shoulder_pitch = new_value;
  } else if (key == 524288) {
    data->r_shoulder_roll = new_value;
  } else if (key == 1048576) {
    data->r_elbow_yaw = new_value;
  } else if (key == 2097152) {
    data->r_elbow_roll = new_value;
  }
}

/* =========== message create =========== */
Message *
NaoHardwareInterface::create_message(const char *type) const
{
  if ( strncmp("SetServosMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetServosMessage();
  } else if ( strncmp("GotoAnglesMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GotoAnglesMessage();
  } else if ( strncmp("GotoAngleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GotoAngleMessage();
  } else if ( strncmp("GotoAngleWithSpeedMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GotoAngleWithSpeedMessage();
  } else if ( strncmp("SetServoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetServoMessage();
  } else if ( strncmp("EnableServosMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new EnableServosMessage();
  } else if ( strncmp("DisableServosMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DisableServosMessage();
  } else if ( strncmp("SetGlobalStiffnessMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetGlobalStiffnessMessage();
  } else if ( strncmp("EmitUltrasonicWaveMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new EmitUltrasonicWaveMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
NaoHardwareInterface::copy_values(const Interface *other)
{
  const NaoHardwareInterface *oi = dynamic_cast<const NaoHardwareInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(NaoHardwareInterface_data_t));
}

const char *
NaoHardwareInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "InterpolationType") == 0) {
    return tostring_InterpolationType((InterpolationType)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class NaoHardwareInterface::SetServosMessage <interfaces/NaoHardwareInterface.h>
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
 * @param ini_r_hip_yaw_pitch initial value for r_hip_yaw_pitch
 * @param ini_r_hip_roll initial value for r_hip_roll
 * @param ini_r_hip_pitch initial value for r_hip_pitch
 * @param ini_r_knee_pitch initial value for r_knee_pitch
 * @param ini_r_ankle_pitch initial value for r_ankle_pitch
 * @param ini_r_ankle_roll initial value for r_ankle_roll
 * @param ini_r_shoulder_pitch initial value for r_shoulder_pitch
 * @param ini_r_shoulder_roll initial value for r_shoulder_roll
 * @param ini_r_elbow_yaw initial value for r_elbow_yaw
 * @param ini_r_elbow_roll initial value for r_elbow_roll
 * @param ini_time initial value for time
 */
NaoHardwareInterface::SetServosMessage::SetServosMessage(const float ini_head_yaw, const float ini_head_pitch, const float ini_l_shoulder_pitch, const float ini_l_shoulder_roll, const float ini_l_elbow_yaw, const float ini_l_elbow_roll, const float ini_l_hip_yaw_pitch, const float ini_l_hip_roll, const float ini_l_hip_pitch, const float ini_l_knee_pitch, const float ini_l_ankle_pitch, const float ini_l_ankle_roll, const float ini_r_hip_yaw_pitch, const float ini_r_hip_roll, const float ini_r_hip_pitch, const float ini_r_knee_pitch, const float ini_r_ankle_pitch, const float ini_r_ankle_roll, const float ini_r_shoulder_pitch, const float ini_r_shoulder_roll, const float ini_r_elbow_yaw, const float ini_r_elbow_roll, const int32_t ini_time) : Message("SetServosMessage")
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
  data->r_hip_yaw_pitch = ini_r_hip_yaw_pitch;
  data->r_hip_roll = ini_r_hip_roll;
  data->r_hip_pitch = ini_r_hip_pitch;
  data->r_knee_pitch = ini_r_knee_pitch;
  data->r_ankle_pitch = ini_r_ankle_pitch;
  data->r_ankle_roll = ini_r_ankle_roll;
  data->r_shoulder_pitch = ini_r_shoulder_pitch;
  data->r_shoulder_roll = ini_r_shoulder_roll;
  data->r_elbow_yaw = ini_r_elbow_yaw;
  data->r_elbow_roll = ini_r_elbow_roll;
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
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}
/** Constructor */
NaoHardwareInterface::SetServosMessage::SetServosMessage() : Message("SetServosMessage")
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
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}

/** Destructor */
NaoHardwareInterface::SetServosMessage::~SetServosMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoHardwareInterface::SetServosMessage::SetServosMessage(const SetServosMessage *m) : Message("SetServosMessage")
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
NaoHardwareInterface::SetServosMessage::head_yaw() const
{
  return data->head_yaw;
}

/** Get maximum length of head_yaw value.
 * @return length of head_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_head_yaw() const
{
  return 1;
}

/** Set head_yaw value.
 * Head yaw
 * @param new_head_yaw new head_yaw value
 */
void
NaoHardwareInterface::SetServosMessage::set_head_yaw(const float new_head_yaw)
{
  data->head_yaw = new_head_yaw;
}

/** Get head_pitch value.
 * Head pitch
 * @return head_pitch value
 */
float
NaoHardwareInterface::SetServosMessage::head_pitch() const
{
  return data->head_pitch;
}

/** Get maximum length of head_pitch value.
 * @return length of head_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_head_pitch() const
{
  return 1;
}

/** Set head_pitch value.
 * Head pitch
 * @param new_head_pitch new head_pitch value
 */
void
NaoHardwareInterface::SetServosMessage::set_head_pitch(const float new_head_pitch)
{
  data->head_pitch = new_head_pitch;
}

/** Get l_shoulder_pitch value.
 * Left shoulder pitch
 * @return l_shoulder_pitch value
 */
float
NaoHardwareInterface::SetServosMessage::l_shoulder_pitch() const
{
  return data->l_shoulder_pitch;
}

/** Get maximum length of l_shoulder_pitch value.
 * @return length of l_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_l_shoulder_pitch() const
{
  return 1;
}

/** Set l_shoulder_pitch value.
 * Left shoulder pitch
 * @param new_l_shoulder_pitch new l_shoulder_pitch value
 */
void
NaoHardwareInterface::SetServosMessage::set_l_shoulder_pitch(const float new_l_shoulder_pitch)
{
  data->l_shoulder_pitch = new_l_shoulder_pitch;
}

/** Get l_shoulder_roll value.
 * Left shoulder roll
 * @return l_shoulder_roll value
 */
float
NaoHardwareInterface::SetServosMessage::l_shoulder_roll() const
{
  return data->l_shoulder_roll;
}

/** Get maximum length of l_shoulder_roll value.
 * @return length of l_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_l_shoulder_roll() const
{
  return 1;
}

/** Set l_shoulder_roll value.
 * Left shoulder roll
 * @param new_l_shoulder_roll new l_shoulder_roll value
 */
void
NaoHardwareInterface::SetServosMessage::set_l_shoulder_roll(const float new_l_shoulder_roll)
{
  data->l_shoulder_roll = new_l_shoulder_roll;
}

/** Get l_elbow_yaw value.
 * Left elbow yaw
 * @return l_elbow_yaw value
 */
float
NaoHardwareInterface::SetServosMessage::l_elbow_yaw() const
{
  return data->l_elbow_yaw;
}

/** Get maximum length of l_elbow_yaw value.
 * @return length of l_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_l_elbow_yaw() const
{
  return 1;
}

/** Set l_elbow_yaw value.
 * Left elbow yaw
 * @param new_l_elbow_yaw new l_elbow_yaw value
 */
void
NaoHardwareInterface::SetServosMessage::set_l_elbow_yaw(const float new_l_elbow_yaw)
{
  data->l_elbow_yaw = new_l_elbow_yaw;
}

/** Get l_elbow_roll value.
 * Left elbow roll
 * @return l_elbow_roll value
 */
float
NaoHardwareInterface::SetServosMessage::l_elbow_roll() const
{
  return data->l_elbow_roll;
}

/** Get maximum length of l_elbow_roll value.
 * @return length of l_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_l_elbow_roll() const
{
  return 1;
}

/** Set l_elbow_roll value.
 * Left elbow roll
 * @param new_l_elbow_roll new l_elbow_roll value
 */
void
NaoHardwareInterface::SetServosMessage::set_l_elbow_roll(const float new_l_elbow_roll)
{
  data->l_elbow_roll = new_l_elbow_roll;
}

/** Get l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @return l_hip_yaw_pitch value
 */
float
NaoHardwareInterface::SetServosMessage::l_hip_yaw_pitch() const
{
  return data->l_hip_yaw_pitch;
}

/** Get maximum length of l_hip_yaw_pitch value.
 * @return length of l_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_l_hip_yaw_pitch() const
{
  return 1;
}

/** Set l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @param new_l_hip_yaw_pitch new l_hip_yaw_pitch value
 */
void
NaoHardwareInterface::SetServosMessage::set_l_hip_yaw_pitch(const float new_l_hip_yaw_pitch)
{
  data->l_hip_yaw_pitch = new_l_hip_yaw_pitch;
}

/** Get l_hip_roll value.
 * Left hip roll
 * @return l_hip_roll value
 */
float
NaoHardwareInterface::SetServosMessage::l_hip_roll() const
{
  return data->l_hip_roll;
}

/** Get maximum length of l_hip_roll value.
 * @return length of l_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_l_hip_roll() const
{
  return 1;
}

/** Set l_hip_roll value.
 * Left hip roll
 * @param new_l_hip_roll new l_hip_roll value
 */
void
NaoHardwareInterface::SetServosMessage::set_l_hip_roll(const float new_l_hip_roll)
{
  data->l_hip_roll = new_l_hip_roll;
}

/** Get l_hip_pitch value.
 * Left hip pitch
 * @return l_hip_pitch value
 */
float
NaoHardwareInterface::SetServosMessage::l_hip_pitch() const
{
  return data->l_hip_pitch;
}

/** Get maximum length of l_hip_pitch value.
 * @return length of l_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_l_hip_pitch() const
{
  return 1;
}

/** Set l_hip_pitch value.
 * Left hip pitch
 * @param new_l_hip_pitch new l_hip_pitch value
 */
void
NaoHardwareInterface::SetServosMessage::set_l_hip_pitch(const float new_l_hip_pitch)
{
  data->l_hip_pitch = new_l_hip_pitch;
}

/** Get l_knee_pitch value.
 * Left knee pitch
 * @return l_knee_pitch value
 */
float
NaoHardwareInterface::SetServosMessage::l_knee_pitch() const
{
  return data->l_knee_pitch;
}

/** Get maximum length of l_knee_pitch value.
 * @return length of l_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_l_knee_pitch() const
{
  return 1;
}

/** Set l_knee_pitch value.
 * Left knee pitch
 * @param new_l_knee_pitch new l_knee_pitch value
 */
void
NaoHardwareInterface::SetServosMessage::set_l_knee_pitch(const float new_l_knee_pitch)
{
  data->l_knee_pitch = new_l_knee_pitch;
}

/** Get l_ankle_pitch value.
 * Left ankle pitch
 * @return l_ankle_pitch value
 */
float
NaoHardwareInterface::SetServosMessage::l_ankle_pitch() const
{
  return data->l_ankle_pitch;
}

/** Get maximum length of l_ankle_pitch value.
 * @return length of l_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_l_ankle_pitch() const
{
  return 1;
}

/** Set l_ankle_pitch value.
 * Left ankle pitch
 * @param new_l_ankle_pitch new l_ankle_pitch value
 */
void
NaoHardwareInterface::SetServosMessage::set_l_ankle_pitch(const float new_l_ankle_pitch)
{
  data->l_ankle_pitch = new_l_ankle_pitch;
}

/** Get l_ankle_roll value.
 * Left ankle roll
 * @return l_ankle_roll value
 */
float
NaoHardwareInterface::SetServosMessage::l_ankle_roll() const
{
  return data->l_ankle_roll;
}

/** Get maximum length of l_ankle_roll value.
 * @return length of l_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_l_ankle_roll() const
{
  return 1;
}

/** Set l_ankle_roll value.
 * Left ankle roll
 * @param new_l_ankle_roll new l_ankle_roll value
 */
void
NaoHardwareInterface::SetServosMessage::set_l_ankle_roll(const float new_l_ankle_roll)
{
  data->l_ankle_roll = new_l_ankle_roll;
}

/** Get r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @return r_hip_yaw_pitch value
 */
float
NaoHardwareInterface::SetServosMessage::r_hip_yaw_pitch() const
{
  return data->r_hip_yaw_pitch;
}

/** Get maximum length of r_hip_yaw_pitch value.
 * @return length of r_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_r_hip_yaw_pitch() const
{
  return 1;
}

/** Set r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @param new_r_hip_yaw_pitch new r_hip_yaw_pitch value
 */
void
NaoHardwareInterface::SetServosMessage::set_r_hip_yaw_pitch(const float new_r_hip_yaw_pitch)
{
  data->r_hip_yaw_pitch = new_r_hip_yaw_pitch;
}

/** Get r_hip_roll value.
 * Right hip roll
 * @return r_hip_roll value
 */
float
NaoHardwareInterface::SetServosMessage::r_hip_roll() const
{
  return data->r_hip_roll;
}

/** Get maximum length of r_hip_roll value.
 * @return length of r_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_r_hip_roll() const
{
  return 1;
}

/** Set r_hip_roll value.
 * Right hip roll
 * @param new_r_hip_roll new r_hip_roll value
 */
void
NaoHardwareInterface::SetServosMessage::set_r_hip_roll(const float new_r_hip_roll)
{
  data->r_hip_roll = new_r_hip_roll;
}

/** Get r_hip_pitch value.
 * Right hip pitch
 * @return r_hip_pitch value
 */
float
NaoHardwareInterface::SetServosMessage::r_hip_pitch() const
{
  return data->r_hip_pitch;
}

/** Get maximum length of r_hip_pitch value.
 * @return length of r_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_r_hip_pitch() const
{
  return 1;
}

/** Set r_hip_pitch value.
 * Right hip pitch
 * @param new_r_hip_pitch new r_hip_pitch value
 */
void
NaoHardwareInterface::SetServosMessage::set_r_hip_pitch(const float new_r_hip_pitch)
{
  data->r_hip_pitch = new_r_hip_pitch;
}

/** Get r_knee_pitch value.
 * Right knee pitch
 * @return r_knee_pitch value
 */
float
NaoHardwareInterface::SetServosMessage::r_knee_pitch() const
{
  return data->r_knee_pitch;
}

/** Get maximum length of r_knee_pitch value.
 * @return length of r_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_r_knee_pitch() const
{
  return 1;
}

/** Set r_knee_pitch value.
 * Right knee pitch
 * @param new_r_knee_pitch new r_knee_pitch value
 */
void
NaoHardwareInterface::SetServosMessage::set_r_knee_pitch(const float new_r_knee_pitch)
{
  data->r_knee_pitch = new_r_knee_pitch;
}

/** Get r_ankle_pitch value.
 * Right ankle pitch
 * @return r_ankle_pitch value
 */
float
NaoHardwareInterface::SetServosMessage::r_ankle_pitch() const
{
  return data->r_ankle_pitch;
}

/** Get maximum length of r_ankle_pitch value.
 * @return length of r_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_r_ankle_pitch() const
{
  return 1;
}

/** Set r_ankle_pitch value.
 * Right ankle pitch
 * @param new_r_ankle_pitch new r_ankle_pitch value
 */
void
NaoHardwareInterface::SetServosMessage::set_r_ankle_pitch(const float new_r_ankle_pitch)
{
  data->r_ankle_pitch = new_r_ankle_pitch;
}

/** Get r_ankle_roll value.
 * Right ankle roll
 * @return r_ankle_roll value
 */
float
NaoHardwareInterface::SetServosMessage::r_ankle_roll() const
{
  return data->r_ankle_roll;
}

/** Get maximum length of r_ankle_roll value.
 * @return length of r_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_r_ankle_roll() const
{
  return 1;
}

/** Set r_ankle_roll value.
 * Right ankle roll
 * @param new_r_ankle_roll new r_ankle_roll value
 */
void
NaoHardwareInterface::SetServosMessage::set_r_ankle_roll(const float new_r_ankle_roll)
{
  data->r_ankle_roll = new_r_ankle_roll;
}

/** Get r_shoulder_pitch value.
 * Right shoulder pitch
 * @return r_shoulder_pitch value
 */
float
NaoHardwareInterface::SetServosMessage::r_shoulder_pitch() const
{
  return data->r_shoulder_pitch;
}

/** Get maximum length of r_shoulder_pitch value.
 * @return length of r_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_r_shoulder_pitch() const
{
  return 1;
}

/** Set r_shoulder_pitch value.
 * Right shoulder pitch
 * @param new_r_shoulder_pitch new r_shoulder_pitch value
 */
void
NaoHardwareInterface::SetServosMessage::set_r_shoulder_pitch(const float new_r_shoulder_pitch)
{
  data->r_shoulder_pitch = new_r_shoulder_pitch;
}

/** Get r_shoulder_roll value.
 * Right shoulder roll
 * @return r_shoulder_roll value
 */
float
NaoHardwareInterface::SetServosMessage::r_shoulder_roll() const
{
  return data->r_shoulder_roll;
}

/** Get maximum length of r_shoulder_roll value.
 * @return length of r_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_r_shoulder_roll() const
{
  return 1;
}

/** Set r_shoulder_roll value.
 * Right shoulder roll
 * @param new_r_shoulder_roll new r_shoulder_roll value
 */
void
NaoHardwareInterface::SetServosMessage::set_r_shoulder_roll(const float new_r_shoulder_roll)
{
  data->r_shoulder_roll = new_r_shoulder_roll;
}

/** Get r_elbow_yaw value.
 * Right elbow yaw
 * @return r_elbow_yaw value
 */
float
NaoHardwareInterface::SetServosMessage::r_elbow_yaw() const
{
  return data->r_elbow_yaw;
}

/** Get maximum length of r_elbow_yaw value.
 * @return length of r_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_r_elbow_yaw() const
{
  return 1;
}

/** Set r_elbow_yaw value.
 * Right elbow yaw
 * @param new_r_elbow_yaw new r_elbow_yaw value
 */
void
NaoHardwareInterface::SetServosMessage::set_r_elbow_yaw(const float new_r_elbow_yaw)
{
  data->r_elbow_yaw = new_r_elbow_yaw;
}

/** Get r_elbow_roll value.
 * Right elbow roll
 * @return r_elbow_roll value
 */
float
NaoHardwareInterface::SetServosMessage::r_elbow_roll() const
{
  return data->r_elbow_roll;
}

/** Get maximum length of r_elbow_roll value.
 * @return length of r_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_r_elbow_roll() const
{
  return 1;
}

/** Set r_elbow_roll value.
 * Right elbow roll
 * @param new_r_elbow_roll new r_elbow_roll value
 */
void
NaoHardwareInterface::SetServosMessage::set_r_elbow_roll(const float new_r_elbow_roll)
{
  data->r_elbow_roll = new_r_elbow_roll;
}

/** Get time value.
 * 
      Current reference time in ms. For real hardware this is the DCM time.
      Times in messages are always offsets to the current time and the current
      time is added before executing the command.
    
 * @return time value
 */
int32_t
NaoHardwareInterface::SetServosMessage::time() const
{
  return data->time;
}

/** Get maximum length of time value.
 * @return length of time value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServosMessage::maxlenof_time() const
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
NaoHardwareInterface::SetServosMessage::set_time(const int32_t new_time)
{
  data->time = new_time;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoHardwareInterface::SetServosMessage::clone() const
{
  return new NaoHardwareInterface::SetServosMessage(this);
}
/** @class NaoHardwareInterface::GotoAnglesMessage <interfaces/NaoHardwareInterface.h>
 * GotoAnglesMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_time_sec initial value for time_sec
 * @param ini_interpolation initial value for interpolation
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
 * @param ini_r_hip_yaw_pitch initial value for r_hip_yaw_pitch
 * @param ini_r_hip_roll initial value for r_hip_roll
 * @param ini_r_hip_pitch initial value for r_hip_pitch
 * @param ini_r_knee_pitch initial value for r_knee_pitch
 * @param ini_r_ankle_pitch initial value for r_ankle_pitch
 * @param ini_r_ankle_roll initial value for r_ankle_roll
 * @param ini_r_shoulder_pitch initial value for r_shoulder_pitch
 * @param ini_r_shoulder_roll initial value for r_shoulder_roll
 * @param ini_r_elbow_yaw initial value for r_elbow_yaw
 * @param ini_r_elbow_roll initial value for r_elbow_roll
 */
NaoHardwareInterface::GotoAnglesMessage::GotoAnglesMessage(const float ini_time_sec, const InterpolationType ini_interpolation, const float ini_head_yaw, const float ini_head_pitch, const float ini_l_shoulder_pitch, const float ini_l_shoulder_roll, const float ini_l_elbow_yaw, const float ini_l_elbow_roll, const float ini_l_hip_yaw_pitch, const float ini_l_hip_roll, const float ini_l_hip_pitch, const float ini_l_knee_pitch, const float ini_l_ankle_pitch, const float ini_l_ankle_roll, const float ini_r_hip_yaw_pitch, const float ini_r_hip_roll, const float ini_r_hip_pitch, const float ini_r_knee_pitch, const float ini_r_ankle_pitch, const float ini_r_ankle_roll, const float ini_r_shoulder_pitch, const float ini_r_shoulder_roll, const float ini_r_elbow_yaw, const float ini_r_elbow_roll) : Message("GotoAnglesMessage")
{
  data_size = sizeof(GotoAnglesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAnglesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->time_sec = ini_time_sec;
  data->interpolation = ini_interpolation;
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
  data->r_hip_yaw_pitch = ini_r_hip_yaw_pitch;
  data->r_hip_roll = ini_r_hip_roll;
  data->r_hip_pitch = ini_r_hip_pitch;
  data->r_knee_pitch = ini_r_knee_pitch;
  data->r_ankle_pitch = ini_r_ankle_pitch;
  data->r_ankle_roll = ini_r_ankle_roll;
  data->r_shoulder_pitch = ini_r_shoulder_pitch;
  data->r_shoulder_roll = ini_r_shoulder_roll;
  data->r_elbow_yaw = ini_r_elbow_yaw;
  data->r_elbow_roll = ini_r_elbow_roll;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_ENUM, "interpolation", 1, &data->interpolation, "InterpolationType");
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
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
}
/** Constructor */
NaoHardwareInterface::GotoAnglesMessage::GotoAnglesMessage() : Message("GotoAnglesMessage")
{
  data_size = sizeof(GotoAnglesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAnglesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_ENUM, "interpolation", 1, &data->interpolation, "InterpolationType");
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
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
}

/** Destructor */
NaoHardwareInterface::GotoAnglesMessage::~GotoAnglesMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoHardwareInterface::GotoAnglesMessage::GotoAnglesMessage(const GotoAnglesMessage *m) : Message("GotoAnglesMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GotoAnglesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get time_sec value.
 * Time in seconds when to reach the desired position
 * @return time_sec value
 */
float
NaoHardwareInterface::GotoAnglesMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time in seconds when to reach the desired position
 * @param new_time_sec new time_sec value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Get interpolation value.
 * Type of interpolation
 * @return interpolation value
 */
NaoHardwareInterface::InterpolationType
NaoHardwareInterface::GotoAnglesMessage::interpolation() const
{
  return (NaoHardwareInterface::InterpolationType)data->interpolation;
}

/** Get maximum length of interpolation value.
 * @return length of interpolation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_interpolation() const
{
  return 1;
}

/** Set interpolation value.
 * Type of interpolation
 * @param new_interpolation new interpolation value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_interpolation(const InterpolationType new_interpolation)
{
  data->interpolation = new_interpolation;
}

/** Get head_yaw value.
 * Head yaw
 * @return head_yaw value
 */
float
NaoHardwareInterface::GotoAnglesMessage::head_yaw() const
{
  return data->head_yaw;
}

/** Get maximum length of head_yaw value.
 * @return length of head_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_head_yaw() const
{
  return 1;
}

/** Set head_yaw value.
 * Head yaw
 * @param new_head_yaw new head_yaw value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_head_yaw(const float new_head_yaw)
{
  data->head_yaw = new_head_yaw;
}

/** Get head_pitch value.
 * Head pitch
 * @return head_pitch value
 */
float
NaoHardwareInterface::GotoAnglesMessage::head_pitch() const
{
  return data->head_pitch;
}

/** Get maximum length of head_pitch value.
 * @return length of head_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_head_pitch() const
{
  return 1;
}

/** Set head_pitch value.
 * Head pitch
 * @param new_head_pitch new head_pitch value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_head_pitch(const float new_head_pitch)
{
  data->head_pitch = new_head_pitch;
}

/** Get l_shoulder_pitch value.
 * Left shoulder pitch
 * @return l_shoulder_pitch value
 */
float
NaoHardwareInterface::GotoAnglesMessage::l_shoulder_pitch() const
{
  return data->l_shoulder_pitch;
}

/** Get maximum length of l_shoulder_pitch value.
 * @return length of l_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_l_shoulder_pitch() const
{
  return 1;
}

/** Set l_shoulder_pitch value.
 * Left shoulder pitch
 * @param new_l_shoulder_pitch new l_shoulder_pitch value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_l_shoulder_pitch(const float new_l_shoulder_pitch)
{
  data->l_shoulder_pitch = new_l_shoulder_pitch;
}

/** Get l_shoulder_roll value.
 * Left shoulder roll
 * @return l_shoulder_roll value
 */
float
NaoHardwareInterface::GotoAnglesMessage::l_shoulder_roll() const
{
  return data->l_shoulder_roll;
}

/** Get maximum length of l_shoulder_roll value.
 * @return length of l_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_l_shoulder_roll() const
{
  return 1;
}

/** Set l_shoulder_roll value.
 * Left shoulder roll
 * @param new_l_shoulder_roll new l_shoulder_roll value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_l_shoulder_roll(const float new_l_shoulder_roll)
{
  data->l_shoulder_roll = new_l_shoulder_roll;
}

/** Get l_elbow_yaw value.
 * Left elbow yaw
 * @return l_elbow_yaw value
 */
float
NaoHardwareInterface::GotoAnglesMessage::l_elbow_yaw() const
{
  return data->l_elbow_yaw;
}

/** Get maximum length of l_elbow_yaw value.
 * @return length of l_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_l_elbow_yaw() const
{
  return 1;
}

/** Set l_elbow_yaw value.
 * Left elbow yaw
 * @param new_l_elbow_yaw new l_elbow_yaw value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_l_elbow_yaw(const float new_l_elbow_yaw)
{
  data->l_elbow_yaw = new_l_elbow_yaw;
}

/** Get l_elbow_roll value.
 * Left elbow roll
 * @return l_elbow_roll value
 */
float
NaoHardwareInterface::GotoAnglesMessage::l_elbow_roll() const
{
  return data->l_elbow_roll;
}

/** Get maximum length of l_elbow_roll value.
 * @return length of l_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_l_elbow_roll() const
{
  return 1;
}

/** Set l_elbow_roll value.
 * Left elbow roll
 * @param new_l_elbow_roll new l_elbow_roll value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_l_elbow_roll(const float new_l_elbow_roll)
{
  data->l_elbow_roll = new_l_elbow_roll;
}

/** Get l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @return l_hip_yaw_pitch value
 */
float
NaoHardwareInterface::GotoAnglesMessage::l_hip_yaw_pitch() const
{
  return data->l_hip_yaw_pitch;
}

/** Get maximum length of l_hip_yaw_pitch value.
 * @return length of l_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_l_hip_yaw_pitch() const
{
  return 1;
}

/** Set l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @param new_l_hip_yaw_pitch new l_hip_yaw_pitch value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_l_hip_yaw_pitch(const float new_l_hip_yaw_pitch)
{
  data->l_hip_yaw_pitch = new_l_hip_yaw_pitch;
}

/** Get l_hip_roll value.
 * Left hip roll
 * @return l_hip_roll value
 */
float
NaoHardwareInterface::GotoAnglesMessage::l_hip_roll() const
{
  return data->l_hip_roll;
}

/** Get maximum length of l_hip_roll value.
 * @return length of l_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_l_hip_roll() const
{
  return 1;
}

/** Set l_hip_roll value.
 * Left hip roll
 * @param new_l_hip_roll new l_hip_roll value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_l_hip_roll(const float new_l_hip_roll)
{
  data->l_hip_roll = new_l_hip_roll;
}

/** Get l_hip_pitch value.
 * Left hip pitch
 * @return l_hip_pitch value
 */
float
NaoHardwareInterface::GotoAnglesMessage::l_hip_pitch() const
{
  return data->l_hip_pitch;
}

/** Get maximum length of l_hip_pitch value.
 * @return length of l_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_l_hip_pitch() const
{
  return 1;
}

/** Set l_hip_pitch value.
 * Left hip pitch
 * @param new_l_hip_pitch new l_hip_pitch value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_l_hip_pitch(const float new_l_hip_pitch)
{
  data->l_hip_pitch = new_l_hip_pitch;
}

/** Get l_knee_pitch value.
 * Left knee pitch
 * @return l_knee_pitch value
 */
float
NaoHardwareInterface::GotoAnglesMessage::l_knee_pitch() const
{
  return data->l_knee_pitch;
}

/** Get maximum length of l_knee_pitch value.
 * @return length of l_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_l_knee_pitch() const
{
  return 1;
}

/** Set l_knee_pitch value.
 * Left knee pitch
 * @param new_l_knee_pitch new l_knee_pitch value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_l_knee_pitch(const float new_l_knee_pitch)
{
  data->l_knee_pitch = new_l_knee_pitch;
}

/** Get l_ankle_pitch value.
 * Left ankle pitch
 * @return l_ankle_pitch value
 */
float
NaoHardwareInterface::GotoAnglesMessage::l_ankle_pitch() const
{
  return data->l_ankle_pitch;
}

/** Get maximum length of l_ankle_pitch value.
 * @return length of l_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_l_ankle_pitch() const
{
  return 1;
}

/** Set l_ankle_pitch value.
 * Left ankle pitch
 * @param new_l_ankle_pitch new l_ankle_pitch value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_l_ankle_pitch(const float new_l_ankle_pitch)
{
  data->l_ankle_pitch = new_l_ankle_pitch;
}

/** Get l_ankle_roll value.
 * Left ankle roll
 * @return l_ankle_roll value
 */
float
NaoHardwareInterface::GotoAnglesMessage::l_ankle_roll() const
{
  return data->l_ankle_roll;
}

/** Get maximum length of l_ankle_roll value.
 * @return length of l_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_l_ankle_roll() const
{
  return 1;
}

/** Set l_ankle_roll value.
 * Left ankle roll
 * @param new_l_ankle_roll new l_ankle_roll value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_l_ankle_roll(const float new_l_ankle_roll)
{
  data->l_ankle_roll = new_l_ankle_roll;
}

/** Get r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @return r_hip_yaw_pitch value
 */
float
NaoHardwareInterface::GotoAnglesMessage::r_hip_yaw_pitch() const
{
  return data->r_hip_yaw_pitch;
}

/** Get maximum length of r_hip_yaw_pitch value.
 * @return length of r_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_r_hip_yaw_pitch() const
{
  return 1;
}

/** Set r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @param new_r_hip_yaw_pitch new r_hip_yaw_pitch value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_r_hip_yaw_pitch(const float new_r_hip_yaw_pitch)
{
  data->r_hip_yaw_pitch = new_r_hip_yaw_pitch;
}

/** Get r_hip_roll value.
 * Right hip roll
 * @return r_hip_roll value
 */
float
NaoHardwareInterface::GotoAnglesMessage::r_hip_roll() const
{
  return data->r_hip_roll;
}

/** Get maximum length of r_hip_roll value.
 * @return length of r_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_r_hip_roll() const
{
  return 1;
}

/** Set r_hip_roll value.
 * Right hip roll
 * @param new_r_hip_roll new r_hip_roll value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_r_hip_roll(const float new_r_hip_roll)
{
  data->r_hip_roll = new_r_hip_roll;
}

/** Get r_hip_pitch value.
 * Right hip pitch
 * @return r_hip_pitch value
 */
float
NaoHardwareInterface::GotoAnglesMessage::r_hip_pitch() const
{
  return data->r_hip_pitch;
}

/** Get maximum length of r_hip_pitch value.
 * @return length of r_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_r_hip_pitch() const
{
  return 1;
}

/** Set r_hip_pitch value.
 * Right hip pitch
 * @param new_r_hip_pitch new r_hip_pitch value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_r_hip_pitch(const float new_r_hip_pitch)
{
  data->r_hip_pitch = new_r_hip_pitch;
}

/** Get r_knee_pitch value.
 * Right knee pitch
 * @return r_knee_pitch value
 */
float
NaoHardwareInterface::GotoAnglesMessage::r_knee_pitch() const
{
  return data->r_knee_pitch;
}

/** Get maximum length of r_knee_pitch value.
 * @return length of r_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_r_knee_pitch() const
{
  return 1;
}

/** Set r_knee_pitch value.
 * Right knee pitch
 * @param new_r_knee_pitch new r_knee_pitch value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_r_knee_pitch(const float new_r_knee_pitch)
{
  data->r_knee_pitch = new_r_knee_pitch;
}

/** Get r_ankle_pitch value.
 * Right ankle pitch
 * @return r_ankle_pitch value
 */
float
NaoHardwareInterface::GotoAnglesMessage::r_ankle_pitch() const
{
  return data->r_ankle_pitch;
}

/** Get maximum length of r_ankle_pitch value.
 * @return length of r_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_r_ankle_pitch() const
{
  return 1;
}

/** Set r_ankle_pitch value.
 * Right ankle pitch
 * @param new_r_ankle_pitch new r_ankle_pitch value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_r_ankle_pitch(const float new_r_ankle_pitch)
{
  data->r_ankle_pitch = new_r_ankle_pitch;
}

/** Get r_ankle_roll value.
 * Right ankle roll
 * @return r_ankle_roll value
 */
float
NaoHardwareInterface::GotoAnglesMessage::r_ankle_roll() const
{
  return data->r_ankle_roll;
}

/** Get maximum length of r_ankle_roll value.
 * @return length of r_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_r_ankle_roll() const
{
  return 1;
}

/** Set r_ankle_roll value.
 * Right ankle roll
 * @param new_r_ankle_roll new r_ankle_roll value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_r_ankle_roll(const float new_r_ankle_roll)
{
  data->r_ankle_roll = new_r_ankle_roll;
}

/** Get r_shoulder_pitch value.
 * Right shoulder pitch
 * @return r_shoulder_pitch value
 */
float
NaoHardwareInterface::GotoAnglesMessage::r_shoulder_pitch() const
{
  return data->r_shoulder_pitch;
}

/** Get maximum length of r_shoulder_pitch value.
 * @return length of r_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_r_shoulder_pitch() const
{
  return 1;
}

/** Set r_shoulder_pitch value.
 * Right shoulder pitch
 * @param new_r_shoulder_pitch new r_shoulder_pitch value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_r_shoulder_pitch(const float new_r_shoulder_pitch)
{
  data->r_shoulder_pitch = new_r_shoulder_pitch;
}

/** Get r_shoulder_roll value.
 * Right shoulder roll
 * @return r_shoulder_roll value
 */
float
NaoHardwareInterface::GotoAnglesMessage::r_shoulder_roll() const
{
  return data->r_shoulder_roll;
}

/** Get maximum length of r_shoulder_roll value.
 * @return length of r_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_r_shoulder_roll() const
{
  return 1;
}

/** Set r_shoulder_roll value.
 * Right shoulder roll
 * @param new_r_shoulder_roll new r_shoulder_roll value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_r_shoulder_roll(const float new_r_shoulder_roll)
{
  data->r_shoulder_roll = new_r_shoulder_roll;
}

/** Get r_elbow_yaw value.
 * Right elbow yaw
 * @return r_elbow_yaw value
 */
float
NaoHardwareInterface::GotoAnglesMessage::r_elbow_yaw() const
{
  return data->r_elbow_yaw;
}

/** Get maximum length of r_elbow_yaw value.
 * @return length of r_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_r_elbow_yaw() const
{
  return 1;
}

/** Set r_elbow_yaw value.
 * Right elbow yaw
 * @param new_r_elbow_yaw new r_elbow_yaw value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_r_elbow_yaw(const float new_r_elbow_yaw)
{
  data->r_elbow_yaw = new_r_elbow_yaw;
}

/** Get r_elbow_roll value.
 * Right elbow roll
 * @return r_elbow_roll value
 */
float
NaoHardwareInterface::GotoAnglesMessage::r_elbow_roll() const
{
  return data->r_elbow_roll;
}

/** Get maximum length of r_elbow_roll value.
 * @return length of r_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAnglesMessage::maxlenof_r_elbow_roll() const
{
  return 1;
}

/** Set r_elbow_roll value.
 * Right elbow roll
 * @param new_r_elbow_roll new r_elbow_roll value
 */
void
NaoHardwareInterface::GotoAnglesMessage::set_r_elbow_roll(const float new_r_elbow_roll)
{
  data->r_elbow_roll = new_r_elbow_roll;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoHardwareInterface::GotoAnglesMessage::clone() const
{
  return new NaoHardwareInterface::GotoAnglesMessage(this);
}
/** @class NaoHardwareInterface::GotoAngleMessage <interfaces/NaoHardwareInterface.h>
 * GotoAngleMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_servos initial value for servos
 * @param ini_value initial value for value
 * @param ini_time_sec initial value for time_sec
 * @param ini_interpolation initial value for interpolation
 */
NaoHardwareInterface::GotoAngleMessage::GotoAngleMessage(const uint32_t ini_servos, const float ini_value, const float ini_time_sec, const InterpolationType ini_interpolation) : Message("GotoAngleMessage")
{
  data_size = sizeof(GotoAngleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAngleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->servos = ini_servos;
  data->value = ini_value;
  data->time_sec = ini_time_sec;
  data->interpolation = ini_interpolation;
  add_fieldinfo(IFT_UINT32, "servos", 1, &data->servos);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_ENUM, "interpolation", 1, &data->interpolation, "InterpolationType");
}
/** Constructor */
NaoHardwareInterface::GotoAngleMessage::GotoAngleMessage() : Message("GotoAngleMessage")
{
  data_size = sizeof(GotoAngleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAngleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "servos", 1, &data->servos);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_ENUM, "interpolation", 1, &data->interpolation, "InterpolationType");
}

/** Destructor */
NaoHardwareInterface::GotoAngleMessage::~GotoAngleMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoHardwareInterface::GotoAngleMessage::GotoAngleMessage(const GotoAngleMessage *m) : Message("GotoAngleMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GotoAngleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get servos value.
 * A concatenated list of SERVO_* constants to
      define the servos that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
 * @return servos value
 */
uint32_t
NaoHardwareInterface::GotoAngleMessage::servos() const
{
  return data->servos;
}

/** Get maximum length of servos value.
 * @return length of servos value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAngleMessage::maxlenof_servos() const
{
  return 1;
}

/** Set servos value.
 * A concatenated list of SERVO_* constants to
      define the servos that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
 * @param new_servos new servos value
 */
void
NaoHardwareInterface::GotoAngleMessage::set_servos(const uint32_t new_servos)
{
  data->servos = new_servos;
}

/** Get value value.
 * Servo value to set for servos.
 * @return value value
 */
float
NaoHardwareInterface::GotoAngleMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAngleMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * Servo value to set for servos.
 * @param new_value new value value
 */
void
NaoHardwareInterface::GotoAngleMessage::set_value(const float new_value)
{
  data->value = new_value;
}

/** Get time_sec value.
 * Time in seconds when to reach the desired position
 * @return time_sec value
 */
float
NaoHardwareInterface::GotoAngleMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAngleMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time in seconds when to reach the desired position
 * @param new_time_sec new time_sec value
 */
void
NaoHardwareInterface::GotoAngleMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Get interpolation value.
 * Type of interpolation
 * @return interpolation value
 */
NaoHardwareInterface::InterpolationType
NaoHardwareInterface::GotoAngleMessage::interpolation() const
{
  return (NaoHardwareInterface::InterpolationType)data->interpolation;
}

/** Get maximum length of interpolation value.
 * @return length of interpolation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAngleMessage::maxlenof_interpolation() const
{
  return 1;
}

/** Set interpolation value.
 * Type of interpolation
 * @param new_interpolation new interpolation value
 */
void
NaoHardwareInterface::GotoAngleMessage::set_interpolation(const InterpolationType new_interpolation)
{
  data->interpolation = new_interpolation;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoHardwareInterface::GotoAngleMessage::clone() const
{
  return new NaoHardwareInterface::GotoAngleMessage(this);
}
/** @class NaoHardwareInterface::GotoAngleWithSpeedMessage <interfaces/NaoHardwareInterface.h>
 * GotoAngleWithSpeedMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_servos initial value for servos
 * @param ini_value initial value for value
 * @param ini_speed initial value for speed
 */
NaoHardwareInterface::GotoAngleWithSpeedMessage::GotoAngleWithSpeedMessage(const uint32_t ini_servos, const float ini_value, const uint16_t ini_speed) : Message("GotoAngleWithSpeedMessage")
{
  data_size = sizeof(GotoAngleWithSpeedMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAngleWithSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->servos = ini_servos;
  data->value = ini_value;
  data->speed = ini_speed;
  add_fieldinfo(IFT_UINT32, "servos", 1, &data->servos);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_UINT16, "speed", 1, &data->speed);
}
/** Constructor */
NaoHardwareInterface::GotoAngleWithSpeedMessage::GotoAngleWithSpeedMessage() : Message("GotoAngleWithSpeedMessage")
{
  data_size = sizeof(GotoAngleWithSpeedMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoAngleWithSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "servos", 1, &data->servos);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_UINT16, "speed", 1, &data->speed);
}

/** Destructor */
NaoHardwareInterface::GotoAngleWithSpeedMessage::~GotoAngleWithSpeedMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoHardwareInterface::GotoAngleWithSpeedMessage::GotoAngleWithSpeedMessage(const GotoAngleWithSpeedMessage *m) : Message("GotoAngleWithSpeedMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GotoAngleWithSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get servos value.
 * A concatenated list of SERVO_* constants to
      define the servos that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
 * @return servos value
 */
uint32_t
NaoHardwareInterface::GotoAngleWithSpeedMessage::servos() const
{
  return data->servos;
}

/** Get maximum length of servos value.
 * @return length of servos value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAngleWithSpeedMessage::maxlenof_servos() const
{
  return 1;
}

/** Set servos value.
 * A concatenated list of SERVO_* constants to
      define the servos that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
 * @param new_servos new servos value
 */
void
NaoHardwareInterface::GotoAngleWithSpeedMessage::set_servos(const uint32_t new_servos)
{
  data->servos = new_servos;
}

/** Get value value.
 * Servo value to set for servos.
 * @return value value
 */
float
NaoHardwareInterface::GotoAngleWithSpeedMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAngleWithSpeedMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * Servo value to set for servos.
 * @param new_value new value value
 */
void
NaoHardwareInterface::GotoAngleWithSpeedMessage::set_value(const float new_value)
{
  data->value = new_value;
}

/** Get speed value.
 * Percentage of the servo speed (1-100).
 * @return speed value
 */
uint16_t
NaoHardwareInterface::GotoAngleWithSpeedMessage::speed() const
{
  return data->speed;
}

/** Get maximum length of speed value.
 * @return length of speed value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::GotoAngleWithSpeedMessage::maxlenof_speed() const
{
  return 1;
}

/** Set speed value.
 * Percentage of the servo speed (1-100).
 * @param new_speed new speed value
 */
void
NaoHardwareInterface::GotoAngleWithSpeedMessage::set_speed(const uint16_t new_speed)
{
  data->speed = new_speed;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoHardwareInterface::GotoAngleWithSpeedMessage::clone() const
{
  return new NaoHardwareInterface::GotoAngleWithSpeedMessage(this);
}
/** @class NaoHardwareInterface::SetServoMessage <interfaces/NaoHardwareInterface.h>
 * SetServoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_servos initial value for servos
 * @param ini_value initial value for value
 * @param ini_time initial value for time
 */
NaoHardwareInterface::SetServoMessage::SetServoMessage(const uint32_t ini_servos, const float ini_value, const int32_t ini_time) : Message("SetServoMessage")
{
  data_size = sizeof(SetServoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetServoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->servos = ini_servos;
  data->value = ini_value;
  data->time = ini_time;
  add_fieldinfo(IFT_UINT32, "servos", 1, &data->servos);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}
/** Constructor */
NaoHardwareInterface::SetServoMessage::SetServoMessage() : Message("SetServoMessage")
{
  data_size = sizeof(SetServoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetServoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "servos", 1, &data->servos);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}

/** Destructor */
NaoHardwareInterface::SetServoMessage::~SetServoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoHardwareInterface::SetServoMessage::SetServoMessage(const SetServoMessage *m) : Message("SetServoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetServoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get servos value.
 * A concatenated list of SERVO_* constants to
      define the servos that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
 * @return servos value
 */
uint32_t
NaoHardwareInterface::SetServoMessage::servos() const
{
  return data->servos;
}

/** Get maximum length of servos value.
 * @return length of servos value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServoMessage::maxlenof_servos() const
{
  return 1;
}

/** Set servos value.
 * A concatenated list of SERVO_* constants to
      define the servos that should execute the movement. The list shall consist of
      binary or'ed SERVO_* constants.
 * @param new_servos new servos value
 */
void
NaoHardwareInterface::SetServoMessage::set_servos(const uint32_t new_servos)
{
  data->servos = new_servos;
}

/** Get value value.
 * Servo value to set for servos.
 * @return value value
 */
float
NaoHardwareInterface::SetServoMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServoMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * Servo value to set for servos.
 * @param new_value new value value
 */
void
NaoHardwareInterface::SetServoMessage::set_value(const float new_value)
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
NaoHardwareInterface::SetServoMessage::time() const
{
  return data->time;
}

/** Get maximum length of time value.
 * @return length of time value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetServoMessage::maxlenof_time() const
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
NaoHardwareInterface::SetServoMessage::set_time(const int32_t new_time)
{
  data->time = new_time;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoHardwareInterface::SetServoMessage::clone() const
{
  return new NaoHardwareInterface::SetServoMessage(this);
}
/** @class NaoHardwareInterface::EnableServosMessage <interfaces/NaoHardwareInterface.h>
 * EnableServosMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
NaoHardwareInterface::EnableServosMessage::EnableServosMessage() : Message("EnableServosMessage")
{
  data_size = sizeof(EnableServosMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (EnableServosMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
NaoHardwareInterface::EnableServosMessage::~EnableServosMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoHardwareInterface::EnableServosMessage::EnableServosMessage(const EnableServosMessage *m) : Message("EnableServosMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (EnableServosMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoHardwareInterface::EnableServosMessage::clone() const
{
  return new NaoHardwareInterface::EnableServosMessage(this);
}
/** @class NaoHardwareInterface::DisableServosMessage <interfaces/NaoHardwareInterface.h>
 * DisableServosMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
NaoHardwareInterface::DisableServosMessage::DisableServosMessage() : Message("DisableServosMessage")
{
  data_size = sizeof(DisableServosMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DisableServosMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
NaoHardwareInterface::DisableServosMessage::~DisableServosMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoHardwareInterface::DisableServosMessage::DisableServosMessage(const DisableServosMessage *m) : Message("DisableServosMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (DisableServosMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoHardwareInterface::DisableServosMessage::clone() const
{
  return new NaoHardwareInterface::DisableServosMessage(this);
}
/** @class NaoHardwareInterface::SetGlobalStiffnessMessage <interfaces/NaoHardwareInterface.h>
 * SetGlobalStiffnessMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_value initial value for value
 * @param ini_time initial value for time
 */
NaoHardwareInterface::SetGlobalStiffnessMessage::SetGlobalStiffnessMessage(const float ini_value, const int32_t ini_time) : Message("SetGlobalStiffnessMessage")
{
  data_size = sizeof(SetGlobalStiffnessMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGlobalStiffnessMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->value = ini_value;
  data->time = ini_time;
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}
/** Constructor */
NaoHardwareInterface::SetGlobalStiffnessMessage::SetGlobalStiffnessMessage() : Message("SetGlobalStiffnessMessage")
{
  data_size = sizeof(SetGlobalStiffnessMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGlobalStiffnessMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}

/** Destructor */
NaoHardwareInterface::SetGlobalStiffnessMessage::~SetGlobalStiffnessMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoHardwareInterface::SetGlobalStiffnessMessage::SetGlobalStiffnessMessage(const SetGlobalStiffnessMessage *m) : Message("SetGlobalStiffnessMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetGlobalStiffnessMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get value value.
 * Stiffness value to set for all joints.(0-1)
 * @return value value
 */
float
NaoHardwareInterface::SetGlobalStiffnessMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetGlobalStiffnessMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * Stiffness value to set for all joints.(0-1)
 * @param new_value new value value
 */
void
NaoHardwareInterface::SetGlobalStiffnessMessage::set_value(const float new_value)
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
NaoHardwareInterface::SetGlobalStiffnessMessage::time() const
{
  return data->time;
}

/** Get maximum length of time value.
 * @return length of time value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::SetGlobalStiffnessMessage::maxlenof_time() const
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
NaoHardwareInterface::SetGlobalStiffnessMessage::set_time(const int32_t new_time)
{
  data->time = new_time;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoHardwareInterface::SetGlobalStiffnessMessage::clone() const
{
  return new NaoHardwareInterface::SetGlobalStiffnessMessage(this);
}
/** @class NaoHardwareInterface::EmitUltrasonicWaveMessage <interfaces/NaoHardwareInterface.h>
 * EmitUltrasonicWaveMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_ultrasonic_direction initial value for ultrasonic_direction
 * @param ini_time initial value for time
 */
NaoHardwareInterface::EmitUltrasonicWaveMessage::EmitUltrasonicWaveMessage(const float ini_ultrasonic_direction, const int32_t ini_time) : Message("EmitUltrasonicWaveMessage")
{
  data_size = sizeof(EmitUltrasonicWaveMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (EmitUltrasonicWaveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->ultrasonic_direction = ini_ultrasonic_direction;
  data->time = ini_time;
  add_fieldinfo(IFT_FLOAT, "ultrasonic_direction", 1, &data->ultrasonic_direction);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}
/** Constructor */
NaoHardwareInterface::EmitUltrasonicWaveMessage::EmitUltrasonicWaveMessage() : Message("EmitUltrasonicWaveMessage")
{
  data_size = sizeof(EmitUltrasonicWaveMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (EmitUltrasonicWaveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "ultrasonic_direction", 1, &data->ultrasonic_direction);
  add_fieldinfo(IFT_INT32, "time", 1, &data->time);
}

/** Destructor */
NaoHardwareInterface::EmitUltrasonicWaveMessage::~EmitUltrasonicWaveMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoHardwareInterface::EmitUltrasonicWaveMessage::EmitUltrasonicWaveMessage(const EmitUltrasonicWaveMessage *m) : Message("EmitUltrasonicWaveMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (EmitUltrasonicWaveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get ultrasonic_direction value.
 * Direction that was used to gather the ultrasonic reading.
 * @return ultrasonic_direction value
 */
float
NaoHardwareInterface::EmitUltrasonicWaveMessage::ultrasonic_direction() const
{
  return data->ultrasonic_direction;
}

/** Get maximum length of ultrasonic_direction value.
 * @return length of ultrasonic_direction value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::EmitUltrasonicWaveMessage::maxlenof_ultrasonic_direction() const
{
  return 1;
}

/** Set ultrasonic_direction value.
 * Direction that was used to gather the ultrasonic reading.
 * @param new_ultrasonic_direction new ultrasonic_direction value
 */
void
NaoHardwareInterface::EmitUltrasonicWaveMessage::set_ultrasonic_direction(const float new_ultrasonic_direction)
{
  data->ultrasonic_direction = new_ultrasonic_direction;
}

/** Get time value.
 * 
      Current reference time in ms. For real hardware this is the DCM time.
      Times in messages are always offsets to the current time and the current
      time is added before executing the command.
    
 * @return time value
 */
int32_t
NaoHardwareInterface::EmitUltrasonicWaveMessage::time() const
{
  return data->time;
}

/** Get maximum length of time value.
 * @return length of time value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoHardwareInterface::EmitUltrasonicWaveMessage::maxlenof_time() const
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
NaoHardwareInterface::EmitUltrasonicWaveMessage::set_time(const int32_t new_time)
{
  data->time = new_time;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoHardwareInterface::EmitUltrasonicWaveMessage::clone() const
{
  return new NaoHardwareInterface::EmitUltrasonicWaveMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
NaoHardwareInterface::message_valid(const Message *message) const
{
  const SetServosMessage *m0 = dynamic_cast<const SetServosMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const GotoAnglesMessage *m1 = dynamic_cast<const GotoAnglesMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const GotoAngleMessage *m2 = dynamic_cast<const GotoAngleMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const GotoAngleWithSpeedMessage *m3 = dynamic_cast<const GotoAngleWithSpeedMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const SetServoMessage *m4 = dynamic_cast<const SetServoMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const EnableServosMessage *m5 = dynamic_cast<const EnableServosMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const DisableServosMessage *m6 = dynamic_cast<const DisableServosMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const SetGlobalStiffnessMessage *m7 = dynamic_cast<const SetGlobalStiffnessMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const EmitUltrasonicWaveMessage *m8 = dynamic_cast<const EmitUltrasonicWaveMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NaoHardwareInterface)
/// @endcond


} // end namespace fawkes
