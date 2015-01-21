
/***************************************************************************
 *  NaoSensorInterface.cpp - Fawkes BlackBoard Interface - NaoSensorInterface
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

#include <interfaces/NaoSensorInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class NaoSensorInterface <interfaces/NaoSensorInterface.h>
 * NaoSensorInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to Nao sensors.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
NaoSensorInterface::NaoSensorInterface() : Interface()
{
  data_size = sizeof(NaoSensorInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (NaoSensorInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_UltrasonicDirection[(int)USD_NONE] = "USD_NONE";
  enum_map_UltrasonicDirection[(int)USD_LEFT_LEFT] = "USD_LEFT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_LEFT_RIGHT] = "USD_LEFT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_RIGHT] = "USD_RIGHT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_LEFT] = "USD_RIGHT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_BOTH_BOTH] = "USD_BOTH_BOTH";
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
  add_fieldinfo(IFT_FLOAT, "l_total_weight", 1, &data->l_total_weight);
  add_fieldinfo(IFT_FLOAT, "r_total_weight", 1, &data->r_total_weight);
  add_fieldinfo(IFT_FLOAT, "l_cop_x", 1, &data->l_cop_x);
  add_fieldinfo(IFT_FLOAT, "l_cop_y", 1, &data->l_cop_y);
  add_fieldinfo(IFT_FLOAT, "r_cop_x", 1, &data->r_cop_x);
  add_fieldinfo(IFT_FLOAT, "r_cop_y", 1, &data->r_cop_y);
  add_fieldinfo(IFT_FLOAT, "ultrasonic_distance_left", 4, &data->ultrasonic_distance_left);
  add_fieldinfo(IFT_FLOAT, "ultrasonic_distance_right", 4, &data->ultrasonic_distance_right);
  add_fieldinfo(IFT_ENUM, "ultrasonic_direction", 1, &data->ultrasonic_direction, "UltrasonicDirection", &enum_map_UltrasonicDirection);
  add_fieldinfo(IFT_UINT8, "l_foot_bumper_l", 1, &data->l_foot_bumper_l);
  add_fieldinfo(IFT_UINT8, "l_foot_bumper_r", 1, &data->l_foot_bumper_r);
  add_fieldinfo(IFT_UINT8, "r_foot_bumper_l", 1, &data->r_foot_bumper_l);
  add_fieldinfo(IFT_UINT8, "r_foot_bumper_r", 1, &data->r_foot_bumper_r);
  add_fieldinfo(IFT_UINT8, "head_touch_front", 1, &data->head_touch_front);
  add_fieldinfo(IFT_UINT8, "head_touch_middle", 1, &data->head_touch_middle);
  add_fieldinfo(IFT_UINT8, "head_touch_rear", 1, &data->head_touch_rear);
  add_fieldinfo(IFT_UINT8, "chest_button", 1, &data->chest_button);
  add_fieldinfo(IFT_FLOAT, "battery_charge", 1, &data->battery_charge);
  add_messageinfo("EmitUltrasonicWaveMessage");
  add_messageinfo("StartUltrasonicMessage");
  add_messageinfo("StopUltrasonicMessage");
  unsigned char tmp_hash[] = {0x41, 0x41, 0x54, 0x94, 0xca, 0xe8, 0x3d, 0x7a, 0xb8, 0xaa, 0xc2, 0x4e, 0x2c, 0xac, 0xcb, 0x15};
  set_hash(tmp_hash);
}

/** Destructor */
NaoSensorInterface::~NaoSensorInterface()
{
  free(data_ptr);
}
/** Convert UltrasonicDirection constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
NaoSensorInterface::tostring_UltrasonicDirection(UltrasonicDirection value) const
{
  switch (value) {
  case USD_NONE: return "USD_NONE";
  case USD_LEFT_LEFT: return "USD_LEFT_LEFT";
  case USD_LEFT_RIGHT: return "USD_LEFT_RIGHT";
  case USD_RIGHT_RIGHT: return "USD_RIGHT_RIGHT";
  case USD_RIGHT_LEFT: return "USD_RIGHT_LEFT";
  case USD_BOTH_BOTH: return "USD_BOTH_BOTH";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get accel_x value.
 * Accelerometer x
 * @return accel_x value
 */
float
NaoSensorInterface::accel_x() const
{
  return data->accel_x;
}

/** Get maximum length of accel_x value.
 * @return length of accel_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_accel_x() const
{
  return 1;
}

/** Set accel_x value.
 * Accelerometer x
 * @param new_accel_x new accel_x value
 */
void
NaoSensorInterface::set_accel_x(const float new_accel_x)
{
  data->accel_x = new_accel_x;
  data_changed = true;
}

/** Get accel_y value.
 * Accelerometer y
 * @return accel_y value
 */
float
NaoSensorInterface::accel_y() const
{
  return data->accel_y;
}

/** Get maximum length of accel_y value.
 * @return length of accel_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_accel_y() const
{
  return 1;
}

/** Set accel_y value.
 * Accelerometer y
 * @param new_accel_y new accel_y value
 */
void
NaoSensorInterface::set_accel_y(const float new_accel_y)
{
  data->accel_y = new_accel_y;
  data_changed = true;
}

/** Get accel_z value.
 * Accelerometer z
 * @return accel_z value
 */
float
NaoSensorInterface::accel_z() const
{
  return data->accel_z;
}

/** Get maximum length of accel_z value.
 * @return length of accel_z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_accel_z() const
{
  return 1;
}

/** Set accel_z value.
 * Accelerometer z
 * @param new_accel_z new accel_z value
 */
void
NaoSensorInterface::set_accel_z(const float new_accel_z)
{
  data->accel_z = new_accel_z;
  data_changed = true;
}

/** Get gyro_x value.
 * Gyrometer x
 * @return gyro_x value
 */
float
NaoSensorInterface::gyro_x() const
{
  return data->gyro_x;
}

/** Get maximum length of gyro_x value.
 * @return length of gyro_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_gyro_x() const
{
  return 1;
}

/** Set gyro_x value.
 * Gyrometer x
 * @param new_gyro_x new gyro_x value
 */
void
NaoSensorInterface::set_gyro_x(const float new_gyro_x)
{
  data->gyro_x = new_gyro_x;
  data_changed = true;
}

/** Get gyro_y value.
 * Gyrometer y
 * @return gyro_y value
 */
float
NaoSensorInterface::gyro_y() const
{
  return data->gyro_y;
}

/** Get maximum length of gyro_y value.
 * @return length of gyro_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_gyro_y() const
{
  return 1;
}

/** Set gyro_y value.
 * Gyrometer y
 * @param new_gyro_y new gyro_y value
 */
void
NaoSensorInterface::set_gyro_y(const float new_gyro_y)
{
  data->gyro_y = new_gyro_y;
  data_changed = true;
}

/** Get gyro_ref value.
 * Gyrometer reference
 * @return gyro_ref value
 */
float
NaoSensorInterface::gyro_ref() const
{
  return data->gyro_ref;
}

/** Get maximum length of gyro_ref value.
 * @return length of gyro_ref value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_gyro_ref() const
{
  return 1;
}

/** Set gyro_ref value.
 * Gyrometer reference
 * @param new_gyro_ref new gyro_ref value
 */
void
NaoSensorInterface::set_gyro_ref(const float new_gyro_ref)
{
  data->gyro_ref = new_gyro_ref;
  data_changed = true;
}

/** Get angle_x value.
 * Angle x
 * @return angle_x value
 */
float
NaoSensorInterface::angle_x() const
{
  return data->angle_x;
}

/** Get maximum length of angle_x value.
 * @return length of angle_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_angle_x() const
{
  return 1;
}

/** Set angle_x value.
 * Angle x
 * @param new_angle_x new angle_x value
 */
void
NaoSensorInterface::set_angle_x(const float new_angle_x)
{
  data->angle_x = new_angle_x;
  data_changed = true;
}

/** Get angle_y value.
 * Angle y
 * @return angle_y value
 */
float
NaoSensorInterface::angle_y() const
{
  return data->angle_y;
}

/** Get maximum length of angle_y value.
 * @return length of angle_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_angle_y() const
{
  return 1;
}

/** Set angle_y value.
 * Angle y
 * @param new_angle_y new angle_y value
 */
void
NaoSensorInterface::set_angle_y(const float new_angle_y)
{
  data->angle_y = new_angle_y;
  data_changed = true;
}

/** Get l_fsr_fl value.
 * Left FSR front left
 * @return l_fsr_fl value
 */
float
NaoSensorInterface::l_fsr_fl() const
{
  return data->l_fsr_fl;
}

/** Get maximum length of l_fsr_fl value.
 * @return length of l_fsr_fl value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_l_fsr_fl() const
{
  return 1;
}

/** Set l_fsr_fl value.
 * Left FSR front left
 * @param new_l_fsr_fl new l_fsr_fl value
 */
void
NaoSensorInterface::set_l_fsr_fl(const float new_l_fsr_fl)
{
  data->l_fsr_fl = new_l_fsr_fl;
  data_changed = true;
}

/** Get l_fsr_fr value.
 * Left FSR front right
 * @return l_fsr_fr value
 */
float
NaoSensorInterface::l_fsr_fr() const
{
  return data->l_fsr_fr;
}

/** Get maximum length of l_fsr_fr value.
 * @return length of l_fsr_fr value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_l_fsr_fr() const
{
  return 1;
}

/** Set l_fsr_fr value.
 * Left FSR front right
 * @param new_l_fsr_fr new l_fsr_fr value
 */
void
NaoSensorInterface::set_l_fsr_fr(const float new_l_fsr_fr)
{
  data->l_fsr_fr = new_l_fsr_fr;
  data_changed = true;
}

/** Get l_fsr_rl value.
 * Left FSR rear left
 * @return l_fsr_rl value
 */
float
NaoSensorInterface::l_fsr_rl() const
{
  return data->l_fsr_rl;
}

/** Get maximum length of l_fsr_rl value.
 * @return length of l_fsr_rl value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_l_fsr_rl() const
{
  return 1;
}

/** Set l_fsr_rl value.
 * Left FSR rear left
 * @param new_l_fsr_rl new l_fsr_rl value
 */
void
NaoSensorInterface::set_l_fsr_rl(const float new_l_fsr_rl)
{
  data->l_fsr_rl = new_l_fsr_rl;
  data_changed = true;
}

/** Get l_fsr_rr value.
 * Left FSR rear right
 * @return l_fsr_rr value
 */
float
NaoSensorInterface::l_fsr_rr() const
{
  return data->l_fsr_rr;
}

/** Get maximum length of l_fsr_rr value.
 * @return length of l_fsr_rr value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_l_fsr_rr() const
{
  return 1;
}

/** Set l_fsr_rr value.
 * Left FSR rear right
 * @param new_l_fsr_rr new l_fsr_rr value
 */
void
NaoSensorInterface::set_l_fsr_rr(const float new_l_fsr_rr)
{
  data->l_fsr_rr = new_l_fsr_rr;
  data_changed = true;
}

/** Get r_fsr_fl value.
 * Right FSR front left
 * @return r_fsr_fl value
 */
float
NaoSensorInterface::r_fsr_fl() const
{
  return data->r_fsr_fl;
}

/** Get maximum length of r_fsr_fl value.
 * @return length of r_fsr_fl value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_r_fsr_fl() const
{
  return 1;
}

/** Set r_fsr_fl value.
 * Right FSR front left
 * @param new_r_fsr_fl new r_fsr_fl value
 */
void
NaoSensorInterface::set_r_fsr_fl(const float new_r_fsr_fl)
{
  data->r_fsr_fl = new_r_fsr_fl;
  data_changed = true;
}

/** Get r_fsr_fr value.
 * Right FSR front right
 * @return r_fsr_fr value
 */
float
NaoSensorInterface::r_fsr_fr() const
{
  return data->r_fsr_fr;
}

/** Get maximum length of r_fsr_fr value.
 * @return length of r_fsr_fr value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_r_fsr_fr() const
{
  return 1;
}

/** Set r_fsr_fr value.
 * Right FSR front right
 * @param new_r_fsr_fr new r_fsr_fr value
 */
void
NaoSensorInterface::set_r_fsr_fr(const float new_r_fsr_fr)
{
  data->r_fsr_fr = new_r_fsr_fr;
  data_changed = true;
}

/** Get r_fsr_rl value.
 * Right FSR rear left
 * @return r_fsr_rl value
 */
float
NaoSensorInterface::r_fsr_rl() const
{
  return data->r_fsr_rl;
}

/** Get maximum length of r_fsr_rl value.
 * @return length of r_fsr_rl value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_r_fsr_rl() const
{
  return 1;
}

/** Set r_fsr_rl value.
 * Right FSR rear left
 * @param new_r_fsr_rl new r_fsr_rl value
 */
void
NaoSensorInterface::set_r_fsr_rl(const float new_r_fsr_rl)
{
  data->r_fsr_rl = new_r_fsr_rl;
  data_changed = true;
}

/** Get r_fsr_rr value.
 * Right FSR rear right
 * @return r_fsr_rr value
 */
float
NaoSensorInterface::r_fsr_rr() const
{
  return data->r_fsr_rr;
}

/** Get maximum length of r_fsr_rr value.
 * @return length of r_fsr_rr value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_r_fsr_rr() const
{
  return 1;
}

/** Set r_fsr_rr value.
 * Right FSR rear right
 * @param new_r_fsr_rr new r_fsr_rr value
 */
void
NaoSensorInterface::set_r_fsr_rr(const float new_r_fsr_rr)
{
  data->r_fsr_rr = new_r_fsr_rr;
  data_changed = true;
}

/** Get l_total_weight value.
 * Total weight on left foot
 * @return l_total_weight value
 */
float
NaoSensorInterface::l_total_weight() const
{
  return data->l_total_weight;
}

/** Get maximum length of l_total_weight value.
 * @return length of l_total_weight value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_l_total_weight() const
{
  return 1;
}

/** Set l_total_weight value.
 * Total weight on left foot
 * @param new_l_total_weight new l_total_weight value
 */
void
NaoSensorInterface::set_l_total_weight(const float new_l_total_weight)
{
  data->l_total_weight = new_l_total_weight;
  data_changed = true;
}

/** Get r_total_weight value.
 * Total weight on right foot
 * @return r_total_weight value
 */
float
NaoSensorInterface::r_total_weight() const
{
  return data->r_total_weight;
}

/** Get maximum length of r_total_weight value.
 * @return length of r_total_weight value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_r_total_weight() const
{
  return 1;
}

/** Set r_total_weight value.
 * Total weight on right foot
 * @param new_r_total_weight new r_total_weight value
 */
void
NaoSensorInterface::set_r_total_weight(const float new_r_total_weight)
{
  data->r_total_weight = new_r_total_weight;
  data_changed = true;
}

/** Get l_cop_x value.
 * Center of pressure X for left foot.
 * @return l_cop_x value
 */
float
NaoSensorInterface::l_cop_x() const
{
  return data->l_cop_x;
}

/** Get maximum length of l_cop_x value.
 * @return length of l_cop_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_l_cop_x() const
{
  return 1;
}

/** Set l_cop_x value.
 * Center of pressure X for left foot.
 * @param new_l_cop_x new l_cop_x value
 */
void
NaoSensorInterface::set_l_cop_x(const float new_l_cop_x)
{
  data->l_cop_x = new_l_cop_x;
  data_changed = true;
}

/** Get l_cop_y value.
 * Center of pressure Y for left foot.
 * @return l_cop_y value
 */
float
NaoSensorInterface::l_cop_y() const
{
  return data->l_cop_y;
}

/** Get maximum length of l_cop_y value.
 * @return length of l_cop_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_l_cop_y() const
{
  return 1;
}

/** Set l_cop_y value.
 * Center of pressure Y for left foot.
 * @param new_l_cop_y new l_cop_y value
 */
void
NaoSensorInterface::set_l_cop_y(const float new_l_cop_y)
{
  data->l_cop_y = new_l_cop_y;
  data_changed = true;
}

/** Get r_cop_x value.
 * Center of pressure X for right foot.
 * @return r_cop_x value
 */
float
NaoSensorInterface::r_cop_x() const
{
  return data->r_cop_x;
}

/** Get maximum length of r_cop_x value.
 * @return length of r_cop_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_r_cop_x() const
{
  return 1;
}

/** Set r_cop_x value.
 * Center of pressure X for right foot.
 * @param new_r_cop_x new r_cop_x value
 */
void
NaoSensorInterface::set_r_cop_x(const float new_r_cop_x)
{
  data->r_cop_x = new_r_cop_x;
  data_changed = true;
}

/** Get r_cop_y value.
 * Center of pressure Y for right foot.
 * @return r_cop_y value
 */
float
NaoSensorInterface::r_cop_y() const
{
  return data->r_cop_y;
}

/** Get maximum length of r_cop_y value.
 * @return length of r_cop_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_r_cop_y() const
{
  return 1;
}

/** Set r_cop_y value.
 * Center of pressure Y for right foot.
 * @param new_r_cop_y new r_cop_y value
 */
void
NaoSensorInterface::set_r_cop_y(const float new_r_cop_y)
{
  data->r_cop_y = new_r_cop_y;
  data_changed = true;
}

/** Get ultrasonic_distance_left value.
 * 
      First four ultrasonic sensor readings from left receiver. Distance
      to detected object is in meters.
    
 * @return ultrasonic_distance_left value
 */
float *
NaoSensorInterface::ultrasonic_distance_left() const
{
  return data->ultrasonic_distance_left;
}

/** Get ultrasonic_distance_left value at given index.
 * 
      First four ultrasonic sensor readings from left receiver. Distance
      to detected object is in meters.
    
 * @param index index of value
 * @return ultrasonic_distance_left value
 * @exception Exception thrown if index is out of bounds
 */
float
NaoSensorInterface::ultrasonic_distance_left(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->ultrasonic_distance_left[index];
}

/** Get maximum length of ultrasonic_distance_left value.
 * @return length of ultrasonic_distance_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_ultrasonic_distance_left() const
{
  return 4;
}

/** Set ultrasonic_distance_left value.
 * 
      First four ultrasonic sensor readings from left receiver. Distance
      to detected object is in meters.
    
 * @param new_ultrasonic_distance_left new ultrasonic_distance_left value
 */
void
NaoSensorInterface::set_ultrasonic_distance_left(const float * new_ultrasonic_distance_left)
{
  memcpy(data->ultrasonic_distance_left, new_ultrasonic_distance_left, sizeof(float) * 4);
  data_changed = true;
}

/** Set ultrasonic_distance_left value at given index.
 * 
      First four ultrasonic sensor readings from left receiver. Distance
      to detected object is in meters.
    
 * @param new_ultrasonic_distance_left new ultrasonic_distance_left value
 * @param index index for of the value
 */
void
NaoSensorInterface::set_ultrasonic_distance_left(unsigned int index, const float new_ultrasonic_distance_left)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->ultrasonic_distance_left[index] = new_ultrasonic_distance_left;
  data_changed = true;
}
/** Get ultrasonic_distance_right value.
 * 
      First four ultrasonic sensor readings from right receiver. Distance
      to detected object is in meters.
    
 * @return ultrasonic_distance_right value
 */
float *
NaoSensorInterface::ultrasonic_distance_right() const
{
  return data->ultrasonic_distance_right;
}

/** Get ultrasonic_distance_right value at given index.
 * 
      First four ultrasonic sensor readings from right receiver. Distance
      to detected object is in meters.
    
 * @param index index of value
 * @return ultrasonic_distance_right value
 * @exception Exception thrown if index is out of bounds
 */
float
NaoSensorInterface::ultrasonic_distance_right(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->ultrasonic_distance_right[index];
}

/** Get maximum length of ultrasonic_distance_right value.
 * @return length of ultrasonic_distance_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_ultrasonic_distance_right() const
{
  return 4;
}

/** Set ultrasonic_distance_right value.
 * 
      First four ultrasonic sensor readings from right receiver. Distance
      to detected object is in meters.
    
 * @param new_ultrasonic_distance_right new ultrasonic_distance_right value
 */
void
NaoSensorInterface::set_ultrasonic_distance_right(const float * new_ultrasonic_distance_right)
{
  memcpy(data->ultrasonic_distance_right, new_ultrasonic_distance_right, sizeof(float) * 4);
  data_changed = true;
}

/** Set ultrasonic_distance_right value at given index.
 * 
      First four ultrasonic sensor readings from right receiver. Distance
      to detected object is in meters.
    
 * @param new_ultrasonic_distance_right new ultrasonic_distance_right value
 * @param index index for of the value
 */
void
NaoSensorInterface::set_ultrasonic_distance_right(unsigned int index, const float new_ultrasonic_distance_right)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->ultrasonic_distance_right[index] = new_ultrasonic_distance_right;
  data_changed = true;
}
/** Get ultrasonic_direction value.
 * 
      Direction that was used to gather the ultrasonic readings.
    
 * @return ultrasonic_direction value
 */
NaoSensorInterface::UltrasonicDirection
NaoSensorInterface::ultrasonic_direction() const
{
  return (NaoSensorInterface::UltrasonicDirection)data->ultrasonic_direction;
}

/** Get maximum length of ultrasonic_direction value.
 * @return length of ultrasonic_direction value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_ultrasonic_direction() const
{
  return 1;
}

/** Set ultrasonic_direction value.
 * 
      Direction that was used to gather the ultrasonic readings.
    
 * @param new_ultrasonic_direction new ultrasonic_direction value
 */
void
NaoSensorInterface::set_ultrasonic_direction(const UltrasonicDirection new_ultrasonic_direction)
{
  data->ultrasonic_direction = new_ultrasonic_direction;
  data_changed = true;
}

/** Get l_foot_bumper_l value.
 * Left foot bumper left side
 * @return l_foot_bumper_l value
 */
uint8_t
NaoSensorInterface::l_foot_bumper_l() const
{
  return data->l_foot_bumper_l;
}

/** Get maximum length of l_foot_bumper_l value.
 * @return length of l_foot_bumper_l value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_l_foot_bumper_l() const
{
  return 1;
}

/** Set l_foot_bumper_l value.
 * Left foot bumper left side
 * @param new_l_foot_bumper_l new l_foot_bumper_l value
 */
void
NaoSensorInterface::set_l_foot_bumper_l(const uint8_t new_l_foot_bumper_l)
{
  data->l_foot_bumper_l = new_l_foot_bumper_l;
  data_changed = true;
}

/** Get l_foot_bumper_r value.
 * Left foot bumper right side
 * @return l_foot_bumper_r value
 */
uint8_t
NaoSensorInterface::l_foot_bumper_r() const
{
  return data->l_foot_bumper_r;
}

/** Get maximum length of l_foot_bumper_r value.
 * @return length of l_foot_bumper_r value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_l_foot_bumper_r() const
{
  return 1;
}

/** Set l_foot_bumper_r value.
 * Left foot bumper right side
 * @param new_l_foot_bumper_r new l_foot_bumper_r value
 */
void
NaoSensorInterface::set_l_foot_bumper_r(const uint8_t new_l_foot_bumper_r)
{
  data->l_foot_bumper_r = new_l_foot_bumper_r;
  data_changed = true;
}

/** Get r_foot_bumper_l value.
 * Right foot bumper left side
 * @return r_foot_bumper_l value
 */
uint8_t
NaoSensorInterface::r_foot_bumper_l() const
{
  return data->r_foot_bumper_l;
}

/** Get maximum length of r_foot_bumper_l value.
 * @return length of r_foot_bumper_l value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_r_foot_bumper_l() const
{
  return 1;
}

/** Set r_foot_bumper_l value.
 * Right foot bumper left side
 * @param new_r_foot_bumper_l new r_foot_bumper_l value
 */
void
NaoSensorInterface::set_r_foot_bumper_l(const uint8_t new_r_foot_bumper_l)
{
  data->r_foot_bumper_l = new_r_foot_bumper_l;
  data_changed = true;
}

/** Get r_foot_bumper_r value.
 * Right foot bumper right side
 * @return r_foot_bumper_r value
 */
uint8_t
NaoSensorInterface::r_foot_bumper_r() const
{
  return data->r_foot_bumper_r;
}

/** Get maximum length of r_foot_bumper_r value.
 * @return length of r_foot_bumper_r value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_r_foot_bumper_r() const
{
  return 1;
}

/** Set r_foot_bumper_r value.
 * Right foot bumper right side
 * @param new_r_foot_bumper_r new r_foot_bumper_r value
 */
void
NaoSensorInterface::set_r_foot_bumper_r(const uint8_t new_r_foot_bumper_r)
{
  data->r_foot_bumper_r = new_r_foot_bumper_r;
  data_changed = true;
}

/** Get head_touch_front value.
 * Front part of head touch sensor (only Academics robot)
 * @return head_touch_front value
 */
uint8_t
NaoSensorInterface::head_touch_front() const
{
  return data->head_touch_front;
}

/** Get maximum length of head_touch_front value.
 * @return length of head_touch_front value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_head_touch_front() const
{
  return 1;
}

/** Set head_touch_front value.
 * Front part of head touch sensor (only Academics robot)
 * @param new_head_touch_front new head_touch_front value
 */
void
NaoSensorInterface::set_head_touch_front(const uint8_t new_head_touch_front)
{
  data->head_touch_front = new_head_touch_front;
  data_changed = true;
}

/** Get head_touch_middle value.
 * Middle part of head touch sensor (only Academics robot)
 * @return head_touch_middle value
 */
uint8_t
NaoSensorInterface::head_touch_middle() const
{
  return data->head_touch_middle;
}

/** Get maximum length of head_touch_middle value.
 * @return length of head_touch_middle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_head_touch_middle() const
{
  return 1;
}

/** Set head_touch_middle value.
 * Middle part of head touch sensor (only Academics robot)
 * @param new_head_touch_middle new head_touch_middle value
 */
void
NaoSensorInterface::set_head_touch_middle(const uint8_t new_head_touch_middle)
{
  data->head_touch_middle = new_head_touch_middle;
  data_changed = true;
}

/** Get head_touch_rear value.
 * Rear part of head touch sensor (only Academics robot)
 * @return head_touch_rear value
 */
uint8_t
NaoSensorInterface::head_touch_rear() const
{
  return data->head_touch_rear;
}

/** Get maximum length of head_touch_rear value.
 * @return length of head_touch_rear value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_head_touch_rear() const
{
  return 1;
}

/** Set head_touch_rear value.
 * Rear part of head touch sensor (only Academics robot)
 * @param new_head_touch_rear new head_touch_rear value
 */
void
NaoSensorInterface::set_head_touch_rear(const uint8_t new_head_touch_rear)
{
  data->head_touch_rear = new_head_touch_rear;
  data_changed = true;
}

/** Get chest_button value.
 * Chest button state
 * @return chest_button value
 */
uint8_t
NaoSensorInterface::chest_button() const
{
  return data->chest_button;
}

/** Get maximum length of chest_button value.
 * @return length of chest_button value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_chest_button() const
{
  return 1;
}

/** Set chest_button value.
 * Chest button state
 * @param new_chest_button new chest_button value
 */
void
NaoSensorInterface::set_chest_button(const uint8_t new_chest_button)
{
  data->chest_button = new_chest_button;
  data_changed = true;
}

/** Get battery_charge value.
 * Battery charge
 * @return battery_charge value
 */
float
NaoSensorInterface::battery_charge() const
{
  return data->battery_charge;
}

/** Get maximum length of battery_charge value.
 * @return length of battery_charge value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::maxlenof_battery_charge() const
{
  return 1;
}

/** Set battery_charge value.
 * Battery charge
 * @param new_battery_charge new battery_charge value
 */
void
NaoSensorInterface::set_battery_charge(const float new_battery_charge)
{
  data->battery_charge = new_battery_charge;
  data_changed = true;
}

/* =========== message create =========== */
Message *
NaoSensorInterface::create_message(const char *type) const
{
  if ( strncmp("EmitUltrasonicWaveMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new EmitUltrasonicWaveMessage();
  } else if ( strncmp("StartUltrasonicMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StartUltrasonicMessage();
  } else if ( strncmp("StopUltrasonicMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopUltrasonicMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
NaoSensorInterface::copy_values(const Interface *other)
{
  const NaoSensorInterface *oi = dynamic_cast<const NaoSensorInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(NaoSensorInterface_data_t));
}

const char *
NaoSensorInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "UltrasonicDirection") == 0) {
    return tostring_UltrasonicDirection((UltrasonicDirection)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class NaoSensorInterface::EmitUltrasonicWaveMessage <interfaces/NaoSensorInterface.h>
 * EmitUltrasonicWaveMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_ultrasonic_direction initial value for ultrasonic_direction
 */
NaoSensorInterface::EmitUltrasonicWaveMessage::EmitUltrasonicWaveMessage(const UltrasonicDirection ini_ultrasonic_direction) : Message("EmitUltrasonicWaveMessage")
{
  data_size = sizeof(EmitUltrasonicWaveMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (EmitUltrasonicWaveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->ultrasonic_direction = ini_ultrasonic_direction;
  enum_map_UltrasonicDirection[(int)USD_NONE] = "USD_NONE";
  enum_map_UltrasonicDirection[(int)USD_LEFT_LEFT] = "USD_LEFT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_LEFT_RIGHT] = "USD_LEFT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_RIGHT] = "USD_RIGHT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_LEFT] = "USD_RIGHT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_BOTH_BOTH] = "USD_BOTH_BOTH";
  add_fieldinfo(IFT_ENUM, "ultrasonic_direction", 1, &data->ultrasonic_direction, "UltrasonicDirection", &enum_map_UltrasonicDirection);
}
/** Constructor */
NaoSensorInterface::EmitUltrasonicWaveMessage::EmitUltrasonicWaveMessage() : Message("EmitUltrasonicWaveMessage")
{
  data_size = sizeof(EmitUltrasonicWaveMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (EmitUltrasonicWaveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_UltrasonicDirection[(int)USD_NONE] = "USD_NONE";
  enum_map_UltrasonicDirection[(int)USD_LEFT_LEFT] = "USD_LEFT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_LEFT_RIGHT] = "USD_LEFT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_RIGHT] = "USD_RIGHT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_LEFT] = "USD_RIGHT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_BOTH_BOTH] = "USD_BOTH_BOTH";
  add_fieldinfo(IFT_ENUM, "ultrasonic_direction", 1, &data->ultrasonic_direction, "UltrasonicDirection", &enum_map_UltrasonicDirection);
}

/** Destructor */
NaoSensorInterface::EmitUltrasonicWaveMessage::~EmitUltrasonicWaveMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoSensorInterface::EmitUltrasonicWaveMessage::EmitUltrasonicWaveMessage(const EmitUltrasonicWaveMessage *m) : Message("EmitUltrasonicWaveMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (EmitUltrasonicWaveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get ultrasonic_direction value.
 * 
      Direction that was used to gather the ultrasonic readings.
    
 * @return ultrasonic_direction value
 */
NaoSensorInterface::UltrasonicDirection
NaoSensorInterface::EmitUltrasonicWaveMessage::ultrasonic_direction() const
{
  return (NaoSensorInterface::UltrasonicDirection)data->ultrasonic_direction;
}

/** Get maximum length of ultrasonic_direction value.
 * @return length of ultrasonic_direction value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::EmitUltrasonicWaveMessage::maxlenof_ultrasonic_direction() const
{
  return 1;
}

/** Set ultrasonic_direction value.
 * 
      Direction that was used to gather the ultrasonic readings.
    
 * @param new_ultrasonic_direction new ultrasonic_direction value
 */
void
NaoSensorInterface::EmitUltrasonicWaveMessage::set_ultrasonic_direction(const UltrasonicDirection new_ultrasonic_direction)
{
  data->ultrasonic_direction = new_ultrasonic_direction;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoSensorInterface::EmitUltrasonicWaveMessage::clone() const
{
  return new NaoSensorInterface::EmitUltrasonicWaveMessage(this);
}
/** @class NaoSensorInterface::StartUltrasonicMessage <interfaces/NaoSensorInterface.h>
 * StartUltrasonicMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_ultrasonic_direction initial value for ultrasonic_direction
 */
NaoSensorInterface::StartUltrasonicMessage::StartUltrasonicMessage(const UltrasonicDirection ini_ultrasonic_direction) : Message("StartUltrasonicMessage")
{
  data_size = sizeof(StartUltrasonicMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StartUltrasonicMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->ultrasonic_direction = ini_ultrasonic_direction;
  enum_map_UltrasonicDirection[(int)USD_NONE] = "USD_NONE";
  enum_map_UltrasonicDirection[(int)USD_LEFT_LEFT] = "USD_LEFT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_LEFT_RIGHT] = "USD_LEFT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_RIGHT] = "USD_RIGHT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_LEFT] = "USD_RIGHT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_BOTH_BOTH] = "USD_BOTH_BOTH";
  add_fieldinfo(IFT_ENUM, "ultrasonic_direction", 1, &data->ultrasonic_direction, "UltrasonicDirection", &enum_map_UltrasonicDirection);
}
/** Constructor */
NaoSensorInterface::StartUltrasonicMessage::StartUltrasonicMessage() : Message("StartUltrasonicMessage")
{
  data_size = sizeof(StartUltrasonicMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StartUltrasonicMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_UltrasonicDirection[(int)USD_NONE] = "USD_NONE";
  enum_map_UltrasonicDirection[(int)USD_LEFT_LEFT] = "USD_LEFT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_LEFT_RIGHT] = "USD_LEFT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_RIGHT] = "USD_RIGHT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_LEFT] = "USD_RIGHT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_BOTH_BOTH] = "USD_BOTH_BOTH";
  add_fieldinfo(IFT_ENUM, "ultrasonic_direction", 1, &data->ultrasonic_direction, "UltrasonicDirection", &enum_map_UltrasonicDirection);
}

/** Destructor */
NaoSensorInterface::StartUltrasonicMessage::~StartUltrasonicMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoSensorInterface::StartUltrasonicMessage::StartUltrasonicMessage(const StartUltrasonicMessage *m) : Message("StartUltrasonicMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StartUltrasonicMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get ultrasonic_direction value.
 * 
      Direction that was used to gather the ultrasonic readings.
    
 * @return ultrasonic_direction value
 */
NaoSensorInterface::UltrasonicDirection
NaoSensorInterface::StartUltrasonicMessage::ultrasonic_direction() const
{
  return (NaoSensorInterface::UltrasonicDirection)data->ultrasonic_direction;
}

/** Get maximum length of ultrasonic_direction value.
 * @return length of ultrasonic_direction value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoSensorInterface::StartUltrasonicMessage::maxlenof_ultrasonic_direction() const
{
  return 1;
}

/** Set ultrasonic_direction value.
 * 
      Direction that was used to gather the ultrasonic readings.
    
 * @param new_ultrasonic_direction new ultrasonic_direction value
 */
void
NaoSensorInterface::StartUltrasonicMessage::set_ultrasonic_direction(const UltrasonicDirection new_ultrasonic_direction)
{
  data->ultrasonic_direction = new_ultrasonic_direction;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoSensorInterface::StartUltrasonicMessage::clone() const
{
  return new NaoSensorInterface::StartUltrasonicMessage(this);
}
/** @class NaoSensorInterface::StopUltrasonicMessage <interfaces/NaoSensorInterface.h>
 * StopUltrasonicMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
NaoSensorInterface::StopUltrasonicMessage::StopUltrasonicMessage() : Message("StopUltrasonicMessage")
{
  data_size = sizeof(StopUltrasonicMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopUltrasonicMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_UltrasonicDirection[(int)USD_NONE] = "USD_NONE";
  enum_map_UltrasonicDirection[(int)USD_LEFT_LEFT] = "USD_LEFT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_LEFT_RIGHT] = "USD_LEFT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_RIGHT] = "USD_RIGHT_RIGHT";
  enum_map_UltrasonicDirection[(int)USD_RIGHT_LEFT] = "USD_RIGHT_LEFT";
  enum_map_UltrasonicDirection[(int)USD_BOTH_BOTH] = "USD_BOTH_BOTH";
}

/** Destructor */
NaoSensorInterface::StopUltrasonicMessage::~StopUltrasonicMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoSensorInterface::StopUltrasonicMessage::StopUltrasonicMessage(const StopUltrasonicMessage *m) : Message("StopUltrasonicMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StopUltrasonicMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoSensorInterface::StopUltrasonicMessage::clone() const
{
  return new NaoSensorInterface::StopUltrasonicMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
NaoSensorInterface::message_valid(const Message *message) const
{
  const EmitUltrasonicWaveMessage *m0 = dynamic_cast<const EmitUltrasonicWaveMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const StartUltrasonicMessage *m1 = dynamic_cast<const StartUltrasonicMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const StopUltrasonicMessage *m2 = dynamic_cast<const StopUltrasonicMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NaoSensorInterface)
/// @endcond


} // end namespace fawkes
