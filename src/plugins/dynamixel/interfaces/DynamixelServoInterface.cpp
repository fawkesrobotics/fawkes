
/***************************************************************************
 *  DynamixelServoInterface.cpp - Fawkes BlackBoard Interface - DynamixelServoInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2015  Tim Niemueller, Nicolas Limpert
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

#include <interfaces/DynamixelServoInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class DynamixelServoInterface <interfaces/DynamixelServoInterface.h>
 * DynamixelServoInterface Fawkes BlackBoard Interface.
 * 
      Interface to access Robotis Dynamixel Servos.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
DynamixelServoInterface::DynamixelServoInterface() : Interface()
{
  data_size = sizeof(DynamixelServoInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (DynamixelServoInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_STRING, "model", 8, data->model);
  add_fieldinfo(IFT_UINT32, "model_number", 1, &data->model_number);
  add_fieldinfo(IFT_UINT32, "cw_angle_limit", 1, &data->cw_angle_limit);
  add_fieldinfo(IFT_UINT32, "ccw_angle_limit", 1, &data->ccw_angle_limit);
  add_fieldinfo(IFT_UINT8, "temperature_limit", 1, &data->temperature_limit);
  add_fieldinfo(IFT_UINT32, "max_torque", 1, &data->max_torque);
  add_fieldinfo(IFT_UINT8, "cw_margin", 1, &data->cw_margin);
  add_fieldinfo(IFT_UINT8, "ccw_margin", 1, &data->ccw_margin);
  add_fieldinfo(IFT_UINT8, "cw_slope", 1, &data->cw_slope);
  add_fieldinfo(IFT_UINT8, "ccw_slope", 1, &data->ccw_slope);
  add_fieldinfo(IFT_UINT32, "goal_position", 1, &data->goal_position);
  add_fieldinfo(IFT_UINT32, "goal_speed", 1, &data->goal_speed);
  add_fieldinfo(IFT_UINT32, "torque_limit", 1, &data->torque_limit);
  add_fieldinfo(IFT_UINT32, "position", 1, &data->position);
  add_fieldinfo(IFT_UINT32, "speed", 1, &data->speed);
  add_fieldinfo(IFT_UINT32, "load", 1, &data->load);
  add_fieldinfo(IFT_UINT8, "voltage", 1, &data->voltage);
  add_fieldinfo(IFT_UINT8, "temperature", 1, &data->temperature);
  add_fieldinfo(IFT_UINT32, "punch", 1, &data->punch);
  add_fieldinfo(IFT_UINT8, "alarm_shutdown", 1, &data->alarm_shutdown);
  add_fieldinfo(IFT_UINT8, "error", 1, &data->error);
  add_fieldinfo(IFT_BOOL, "enable_prevent_alarm_shutdown", 1, &data->enable_prevent_alarm_shutdown);
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
  add_fieldinfo(IFT_FLOAT, "min_angle", 1, &data->min_angle);
  add_fieldinfo(IFT_FLOAT, "max_angle", 1, &data->max_angle);
  add_fieldinfo(IFT_FLOAT, "max_velocity", 1, &data->max_velocity);
  add_fieldinfo(IFT_FLOAT, "velocity", 1, &data->velocity);
  add_fieldinfo(IFT_STRING, "mode", 5, data->mode);
  add_fieldinfo(IFT_FLOAT, "angle_margin", 1, &data->angle_margin);
  add_fieldinfo(IFT_BOOL, "autorecover_enabled", 1, &data->autorecover_enabled);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_ENUM, "error_code", 1, &data->error_code, "ErrorCode", &enum_map_ErrorCode);
  add_messageinfo("StopMessage");
  add_messageinfo("FlushMessage");
  add_messageinfo("GotoMessage");
  add_messageinfo("TimedGotoMessage");
  add_messageinfo("SetModeMessage");
  add_messageinfo("SetSpeedMessage");
  add_messageinfo("SetEnabledMessage");
  add_messageinfo("SetVelocityMessage");
  add_messageinfo("SetMarginMessage");
  add_messageinfo("SetComplianceValuesMessage");
  add_messageinfo("SetGoalSpeedMessage");
  add_messageinfo("SetTorqueLimitMessage");
  add_messageinfo("SetPunchMessage");
  add_messageinfo("GotoPositionMessage");
  add_messageinfo("SetAngleLimitsMessage");
  add_messageinfo("ResetRawErrorMessage");
  add_messageinfo("SetPreventAlarmShutdownMessage");
  add_messageinfo("SetAutorecoverEnabledMessage");
  add_messageinfo("RecoverMessage");
  unsigned char tmp_hash[] = {0x18, 0x72, 0x74, 0xa6, 0x80, 0xfa, 0x62, 0xa2, 0x56, 0x91, 0x21, 0xfc, 0x48, 0xd5, 0xe0, 0x5f};
  set_hash(tmp_hash);
}

/** Destructor */
DynamixelServoInterface::~DynamixelServoInterface()
{
  free(data_ptr);
}
/** Convert ErrorCode constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
DynamixelServoInterface::tostring_ErrorCode(ErrorCode value) const
{
  switch (value) {
  case ERROR_NONE: return "ERROR_NONE";
  case ERROR_UNSPECIFIC: return "ERROR_UNSPECIFIC";
  case ERROR_COMMUNICATION: return "ERROR_COMMUNICATION";
  case ERROR_ANGLE_OUTOFRANGE: return "ERROR_ANGLE_OUTOFRANGE";
  default: return "UNKNOWN";
  }
}
/** Convert WorkingMode constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
DynamixelServoInterface::tostring_WorkingMode(WorkingMode value) const
{
  switch (value) {
  case JOINT: return "JOINT";
  case WHEEL: return "WHEEL";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get model value.
 * Model if known
 * @return model value
 */
char *
DynamixelServoInterface::model() const
{
  return data->model;
}

/** Get maximum length of model value.
 * @return length of model value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_model() const
{
  return 8;
}

/** Set model value.
 * Model if known
 * @param new_model new model value
 */
void
DynamixelServoInterface::set_model(const char * new_model)
{
  strncpy(data->model, new_model, sizeof(data->model)-1);
  data->model[sizeof(data->model)-1] = 0;
  data_changed = true;
}

/** Get model_number value.
 * Model number
 * @return model_number value
 */
uint32_t
DynamixelServoInterface::model_number() const
{
  return data->model_number;
}

/** Get maximum length of model_number value.
 * @return length of model_number value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_model_number() const
{
  return 1;
}

/** Set model_number value.
 * Model number
 * @param new_model_number new model_number value
 */
void
DynamixelServoInterface::set_model_number(const uint32_t new_model_number)
{
  data->model_number = new_model_number;
  data_changed = true;
}

/** Get cw_angle_limit value.
 * Clockwise angle limit
 * @return cw_angle_limit value
 */
uint32_t
DynamixelServoInterface::cw_angle_limit() const
{
  return data->cw_angle_limit;
}

/** Get maximum length of cw_angle_limit value.
 * @return length of cw_angle_limit value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_cw_angle_limit() const
{
  return 1;
}

/** Set cw_angle_limit value.
 * Clockwise angle limit
 * @param new_cw_angle_limit new cw_angle_limit value
 */
void
DynamixelServoInterface::set_cw_angle_limit(const uint32_t new_cw_angle_limit)
{
  data->cw_angle_limit = new_cw_angle_limit;
  data_changed = true;
}

/** Get ccw_angle_limit value.
 * Counter-clockwise angle limit
 * @return ccw_angle_limit value
 */
uint32_t
DynamixelServoInterface::ccw_angle_limit() const
{
  return data->ccw_angle_limit;
}

/** Get maximum length of ccw_angle_limit value.
 * @return length of ccw_angle_limit value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_ccw_angle_limit() const
{
  return 1;
}

/** Set ccw_angle_limit value.
 * Counter-clockwise angle limit
 * @param new_ccw_angle_limit new ccw_angle_limit value
 */
void
DynamixelServoInterface::set_ccw_angle_limit(const uint32_t new_ccw_angle_limit)
{
  data->ccw_angle_limit = new_ccw_angle_limit;
  data_changed = true;
}

/** Get temperature_limit value.
 * Temperature limit
 * @return temperature_limit value
 */
uint8_t
DynamixelServoInterface::temperature_limit() const
{
  return data->temperature_limit;
}

/** Get maximum length of temperature_limit value.
 * @return length of temperature_limit value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_temperature_limit() const
{
  return 1;
}

/** Set temperature_limit value.
 * Temperature limit
 * @param new_temperature_limit new temperature_limit value
 */
void
DynamixelServoInterface::set_temperature_limit(const uint8_t new_temperature_limit)
{
  data->temperature_limit = new_temperature_limit;
  data_changed = true;
}

/** Get max_torque value.
 * Max torque
 * @return max_torque value
 */
uint32_t
DynamixelServoInterface::max_torque() const
{
  return data->max_torque;
}

/** Get maximum length of max_torque value.
 * @return length of max_torque value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_max_torque() const
{
  return 1;
}

/** Set max_torque value.
 * Max torque
 * @param new_max_torque new max_torque value
 */
void
DynamixelServoInterface::set_max_torque(const uint32_t new_max_torque)
{
  data->max_torque = new_max_torque;
  data_changed = true;
}

/** Get cw_margin value.
 * CW Compliance Margin
 * @return cw_margin value
 */
uint8_t
DynamixelServoInterface::cw_margin() const
{
  return data->cw_margin;
}

/** Get maximum length of cw_margin value.
 * @return length of cw_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_cw_margin() const
{
  return 1;
}

/** Set cw_margin value.
 * CW Compliance Margin
 * @param new_cw_margin new cw_margin value
 */
void
DynamixelServoInterface::set_cw_margin(const uint8_t new_cw_margin)
{
  data->cw_margin = new_cw_margin;
  data_changed = true;
}

/** Get ccw_margin value.
 * CCW Compliance Margin
 * @return ccw_margin value
 */
uint8_t
DynamixelServoInterface::ccw_margin() const
{
  return data->ccw_margin;
}

/** Get maximum length of ccw_margin value.
 * @return length of ccw_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_ccw_margin() const
{
  return 1;
}

/** Set ccw_margin value.
 * CCW Compliance Margin
 * @param new_ccw_margin new ccw_margin value
 */
void
DynamixelServoInterface::set_ccw_margin(const uint8_t new_ccw_margin)
{
  data->ccw_margin = new_ccw_margin;
  data_changed = true;
}

/** Get cw_slope value.
 * CW Compliance Slope
 * @return cw_slope value
 */
uint8_t
DynamixelServoInterface::cw_slope() const
{
  return data->cw_slope;
}

/** Get maximum length of cw_slope value.
 * @return length of cw_slope value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_cw_slope() const
{
  return 1;
}

/** Set cw_slope value.
 * CW Compliance Slope
 * @param new_cw_slope new cw_slope value
 */
void
DynamixelServoInterface::set_cw_slope(const uint8_t new_cw_slope)
{
  data->cw_slope = new_cw_slope;
  data_changed = true;
}

/** Get ccw_slope value.
 * CCW Compliance Slope
 * @return ccw_slope value
 */
uint8_t
DynamixelServoInterface::ccw_slope() const
{
  return data->ccw_slope;
}

/** Get maximum length of ccw_slope value.
 * @return length of ccw_slope value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_ccw_slope() const
{
  return 1;
}

/** Set ccw_slope value.
 * CCW Compliance Slope
 * @param new_ccw_slope new ccw_slope value
 */
void
DynamixelServoInterface::set_ccw_slope(const uint8_t new_ccw_slope)
{
  data->ccw_slope = new_ccw_slope;
  data_changed = true;
}

/** Get goal_position value.
 * Goal position
 * @return goal_position value
 */
uint32_t
DynamixelServoInterface::goal_position() const
{
  return data->goal_position;
}

/** Get maximum length of goal_position value.
 * @return length of goal_position value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_goal_position() const
{
  return 1;
}

/** Set goal_position value.
 * Goal position
 * @param new_goal_position new goal_position value
 */
void
DynamixelServoInterface::set_goal_position(const uint32_t new_goal_position)
{
  data->goal_position = new_goal_position;
  data_changed = true;
}

/** Get goal_speed value.
 * Goal speed
 * @return goal_speed value
 */
uint32_t
DynamixelServoInterface::goal_speed() const
{
  return data->goal_speed;
}

/** Get maximum length of goal_speed value.
 * @return length of goal_speed value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_goal_speed() const
{
  return 1;
}

/** Set goal_speed value.
 * Goal speed
 * @param new_goal_speed new goal_speed value
 */
void
DynamixelServoInterface::set_goal_speed(const uint32_t new_goal_speed)
{
  data->goal_speed = new_goal_speed;
  data_changed = true;
}

/** Get torque_limit value.
 * Torque limit
 * @return torque_limit value
 */
uint32_t
DynamixelServoInterface::torque_limit() const
{
  return data->torque_limit;
}

/** Get maximum length of torque_limit value.
 * @return length of torque_limit value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_torque_limit() const
{
  return 1;
}

/** Set torque_limit value.
 * Torque limit
 * @param new_torque_limit new torque_limit value
 */
void
DynamixelServoInterface::set_torque_limit(const uint32_t new_torque_limit)
{
  data->torque_limit = new_torque_limit;
  data_changed = true;
}

/** Get position value.
 * Present position
 * @return position value
 */
uint32_t
DynamixelServoInterface::position() const
{
  return data->position;
}

/** Get maximum length of position value.
 * @return length of position value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_position() const
{
  return 1;
}

/** Set position value.
 * Present position
 * @param new_position new position value
 */
void
DynamixelServoInterface::set_position(const uint32_t new_position)
{
  data->position = new_position;
  data_changed = true;
}

/** Get speed value.
 * Present speed
 * @return speed value
 */
uint32_t
DynamixelServoInterface::speed() const
{
  return data->speed;
}

/** Get maximum length of speed value.
 * @return length of speed value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_speed() const
{
  return 1;
}

/** Set speed value.
 * Present speed
 * @param new_speed new speed value
 */
void
DynamixelServoInterface::set_speed(const uint32_t new_speed)
{
  data->speed = new_speed;
  data_changed = true;
}

/** Get load value.
 * Present load
 * @return load value
 */
uint32_t
DynamixelServoInterface::load() const
{
  return data->load;
}

/** Get maximum length of load value.
 * @return length of load value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_load() const
{
  return 1;
}

/** Set load value.
 * Present load
 * @param new_load new load value
 */
void
DynamixelServoInterface::set_load(const uint32_t new_load)
{
  data->load = new_load;
  data_changed = true;
}

/** Get voltage value.
 * Present voltage
 * @return voltage value
 */
uint8_t
DynamixelServoInterface::voltage() const
{
  return data->voltage;
}

/** Get maximum length of voltage value.
 * @return length of voltage value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_voltage() const
{
  return 1;
}

/** Set voltage value.
 * Present voltage
 * @param new_voltage new voltage value
 */
void
DynamixelServoInterface::set_voltage(const uint8_t new_voltage)
{
  data->voltage = new_voltage;
  data_changed = true;
}

/** Get temperature value.
 * Present temperature
 * @return temperature value
 */
uint8_t
DynamixelServoInterface::temperature() const
{
  return data->temperature;
}

/** Get maximum length of temperature value.
 * @return length of temperature value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_temperature() const
{
  return 1;
}

/** Set temperature value.
 * Present temperature
 * @param new_temperature new temperature value
 */
void
DynamixelServoInterface::set_temperature(const uint8_t new_temperature)
{
  data->temperature = new_temperature;
  data_changed = true;
}

/** Get punch value.
 * Punch
 * @return punch value
 */
uint32_t
DynamixelServoInterface::punch() const
{
  return data->punch;
}

/** Get maximum length of punch value.
 * @return length of punch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_punch() const
{
  return 1;
}

/** Set punch value.
 * Punch
 * @param new_punch new punch value
 */
void
DynamixelServoInterface::set_punch(const uint32_t new_punch)
{
  data->punch = new_punch;
  data_changed = true;
}

/** Get alarm_shutdown value.
 * Alarm Shutdown.
      The bitmask is set as follows (taken from Trossen Robotics PDF "AX-12(English).pdf"):
      Bit 7: 0
      Bit 6: If set to 1, torque off when an Instruction Error occurs
      Bit 5: If set to 1, torque off when an Overload Error occurs
      Bit 4: If set to 1, torque off when a Checksum Error occurs
      Bit 3: If set to 1, torque off when a Range Error occurs
      Bit 2: If set to 1, torque off when an Overheating Error occurs
      Bit 1: If set to 1, torque off when an Angle Limit Error occurs
      Bit 0: If set to 1, torque off when an Input Voltage Error occurs
    
 * @return alarm_shutdown value
 */
uint8_t
DynamixelServoInterface::alarm_shutdown() const
{
  return data->alarm_shutdown;
}

/** Get maximum length of alarm_shutdown value.
 * @return length of alarm_shutdown value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_alarm_shutdown() const
{
  return 1;
}

/** Set alarm_shutdown value.
 * Alarm Shutdown.
      The bitmask is set as follows (taken from Trossen Robotics PDF "AX-12(English).pdf"):
      Bit 7: 0
      Bit 6: If set to 1, torque off when an Instruction Error occurs
      Bit 5: If set to 1, torque off when an Overload Error occurs
      Bit 4: If set to 1, torque off when a Checksum Error occurs
      Bit 3: If set to 1, torque off when a Range Error occurs
      Bit 2: If set to 1, torque off when an Overheating Error occurs
      Bit 1: If set to 1, torque off when an Angle Limit Error occurs
      Bit 0: If set to 1, torque off when an Input Voltage Error occurs
    
 * @param new_alarm_shutdown new alarm_shutdown value
 */
void
DynamixelServoInterface::set_alarm_shutdown(const uint8_t new_alarm_shutdown)
{
  data->alarm_shutdown = new_alarm_shutdown;
  data_changed = true;
}

/** Get error value.
 * Raw error code from servo.
      The bitmask is set as follows (taken from Trossen Robotics PDF "AX-12(English).pdf"):
      Bit 7: 0
      Bit 6: Set to 1 if an undefined instruction is sent or an action instruction is sent without a Reg_Write instruction.
      Bit 5: Set to 1 if the specified maximum torque can't control the applied load.
      Bit 4: Set to 1 if the checksum of the instruction packet is incorrect.
      Bit 3: Set to 1 if the instruction sent is out of the defined range.
      Bit 2: Set to 1 if the internal temperature of the Dynamixel unit is above the operating temperature range as defined in the control table.
      Bit 1: Set as 1 if the Goal Position is set outside of the range between CW Angle Limit and CCW Angle Limit.
      Bit 0: Set to 1 if the voltage is out of the operating voltage range as defined in the control table.
    
 * @return error value
 */
uint8_t
DynamixelServoInterface::error() const
{
  return data->error;
}

/** Get maximum length of error value.
 * @return length of error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_error() const
{
  return 1;
}

/** Set error value.
 * Raw error code from servo.
      The bitmask is set as follows (taken from Trossen Robotics PDF "AX-12(English).pdf"):
      Bit 7: 0
      Bit 6: Set to 1 if an undefined instruction is sent or an action instruction is sent without a Reg_Write instruction.
      Bit 5: Set to 1 if the specified maximum torque can't control the applied load.
      Bit 4: Set to 1 if the checksum of the instruction packet is incorrect.
      Bit 3: Set to 1 if the instruction sent is out of the defined range.
      Bit 2: Set to 1 if the internal temperature of the Dynamixel unit is above the operating temperature range as defined in the control table.
      Bit 1: Set as 1 if the Goal Position is set outside of the range between CW Angle Limit and CCW Angle Limit.
      Bit 0: Set to 1 if the voltage is out of the operating voltage range as defined in the control table.
    
 * @param new_error new error value
 */
void
DynamixelServoInterface::set_error(const uint8_t new_error)
{
  data->error = new_error;
  data_changed = true;
}

/** Get enable_prevent_alarm_shutdown value.
 * Enable alarm shutdown
 * @return enable_prevent_alarm_shutdown value
 */
bool
DynamixelServoInterface::is_enable_prevent_alarm_shutdown() const
{
  return data->enable_prevent_alarm_shutdown;
}

/** Get maximum length of enable_prevent_alarm_shutdown value.
 * @return length of enable_prevent_alarm_shutdown value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_enable_prevent_alarm_shutdown() const
{
  return 1;
}

/** Set enable_prevent_alarm_shutdown value.
 * Enable alarm shutdown
 * @param new_enable_prevent_alarm_shutdown new enable_prevent_alarm_shutdown value
 */
void
DynamixelServoInterface::set_enable_prevent_alarm_shutdown(const bool new_enable_prevent_alarm_shutdown)
{
  data->enable_prevent_alarm_shutdown = new_enable_prevent_alarm_shutdown;
  data_changed = true;
}

/** Get angle value.
 * Current angle.
 * @return angle value
 */
float
DynamixelServoInterface::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Current angle.
 * @param new_angle new angle value
 */
void
DynamixelServoInterface::set_angle(const float new_angle)
{
  data->angle = new_angle;
  data_changed = true;
}

/** Get enabled value.
 * Is the servo enabled?
 * @return enabled value
 */
bool
DynamixelServoInterface::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * Is the servo enabled?
 * @param new_enabled new enabled value
 */
void
DynamixelServoInterface::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
  data_changed = true;
}

/** Get min_angle value.
 * Minimum angle allowed.
 * @return min_angle value
 */
float
DynamixelServoInterface::min_angle() const
{
  return data->min_angle;
}

/** Get maximum length of min_angle value.
 * @return length of min_angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_min_angle() const
{
  return 1;
}

/** Set min_angle value.
 * Minimum angle allowed.
 * @param new_min_angle new min_angle value
 */
void
DynamixelServoInterface::set_min_angle(const float new_min_angle)
{
  data->min_angle = new_min_angle;
  data_changed = true;
}

/** Get max_angle value.
 * Maximum angle allowed.
 * @return max_angle value
 */
float
DynamixelServoInterface::max_angle() const
{
  return data->max_angle;
}

/** Get maximum length of max_angle value.
 * @return length of max_angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_max_angle() const
{
  return 1;
}

/** Set max_angle value.
 * Maximum angle allowed.
 * @param new_max_angle new max_angle value
 */
void
DynamixelServoInterface::set_max_angle(const float new_max_angle)
{
  data->max_angle = new_max_angle;
  data_changed = true;
}

/** Get max_velocity value.
 * Maximum supported velocity.
 * @return max_velocity value
 */
float
DynamixelServoInterface::max_velocity() const
{
  return data->max_velocity;
}

/** Get maximum length of max_velocity value.
 * @return length of max_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_max_velocity() const
{
  return 1;
}

/** Set max_velocity value.
 * Maximum supported velocity.
 * @param new_max_velocity new max_velocity value
 */
void
DynamixelServoInterface::set_max_velocity(const float new_max_velocity)
{
  data->max_velocity = new_max_velocity;
  data_changed = true;
}

/** Get velocity value.
 * Maximum servo velocity currently reached.
 * @return velocity value
 */
float
DynamixelServoInterface::velocity() const
{
  return data->velocity;
}

/** Get maximum length of velocity value.
 * @return length of velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_velocity() const
{
  return 1;
}

/** Set velocity value.
 * Maximum servo velocity currently reached.
 * @param new_velocity new velocity value
 */
void
DynamixelServoInterface::set_velocity(const float new_velocity)
{
  data->velocity = new_velocity;
  data_changed = true;
}

/** Get mode value.
 * Working mode, can either be JOINT or WHEEL
 * @return mode value
 */
char *
DynamixelServoInterface::mode() const
{
  return data->mode;
}

/** Get maximum length of mode value.
 * @return length of mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_mode() const
{
  return 5;
}

/** Set mode value.
 * Working mode, can either be JOINT or WHEEL
 * @param new_mode new mode value
 */
void
DynamixelServoInterface::set_mode(const char * new_mode)
{
  strncpy(data->mode, new_mode, sizeof(data->mode)-1);
  data->mode[sizeof(data->mode)-1] = 0;
  data_changed = true;
}

/** Get angle_margin value.
 * 
      Margin in radians around a target servo value to consider the
      motion as final.
    
 * @return angle_margin value
 */
float
DynamixelServoInterface::angle_margin() const
{
  return data->angle_margin;
}

/** Get maximum length of angle_margin value.
 * @return length of angle_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_angle_margin() const
{
  return 1;
}

/** Set angle_margin value.
 * 
      Margin in radians around a target servo value to consider the
      motion as final.
    
 * @param new_angle_margin new angle_margin value
 */
void
DynamixelServoInterface::set_angle_margin(const float new_angle_margin)
{
  data->angle_margin = new_angle_margin;
  data_changed = true;
}

/** Get autorecover_enabled value.
 * Automatically recover on alarm shutdown
 * @return autorecover_enabled value
 */
bool
DynamixelServoInterface::is_autorecover_enabled() const
{
  return data->autorecover_enabled;
}

/** Get maximum length of autorecover_enabled value.
 * @return length of autorecover_enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_autorecover_enabled() const
{
  return 1;
}

/** Set autorecover_enabled value.
 * Automatically recover on alarm shutdown
 * @param new_autorecover_enabled new autorecover_enabled value
 */
void
DynamixelServoInterface::set_autorecover_enabled(const bool new_autorecover_enabled)
{
  data->autorecover_enabled = new_autorecover_enabled;
  data_changed = true;
}

/** Get msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @return msgid value
 */
uint32_t
DynamixelServoInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @param new_msgid new msgid value
 */
void
DynamixelServoInterface::set_msgid(const uint32_t new_msgid)
{
  data->msgid = new_msgid;
  data_changed = true;
}

/** Get final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @return final value
 */
bool
DynamixelServoInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @param new_final new final value
 */
void
DynamixelServoInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/** Get error_code value.
 * Failure code set if
    final is true. ERROR_NONE if no error occured.
 * @return error_code value
 */
DynamixelServoInterface::ErrorCode
DynamixelServoInterface::error_code() const
{
  return (DynamixelServoInterface::ErrorCode)data->error_code;
}

/** Get maximum length of error_code value.
 * @return length of error_code value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::maxlenof_error_code() const
{
  return 1;
}

/** Set error_code value.
 * Failure code set if
    final is true. ERROR_NONE if no error occured.
 * @param new_error_code new error_code value
 */
void
DynamixelServoInterface::set_error_code(const ErrorCode new_error_code)
{
  data->error_code = new_error_code;
  data_changed = true;
}

/* =========== message create =========== */
Message *
DynamixelServoInterface::create_message(const char *type) const
{
  if ( strncmp("StopMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopMessage();
  } else if ( strncmp("FlushMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new FlushMessage();
  } else if ( strncmp("GotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GotoMessage();
  } else if ( strncmp("TimedGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TimedGotoMessage();
  } else if ( strncmp("SetModeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetModeMessage();
  } else if ( strncmp("SetSpeedMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetSpeedMessage();
  } else if ( strncmp("SetEnabledMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetEnabledMessage();
  } else if ( strncmp("SetVelocityMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetVelocityMessage();
  } else if ( strncmp("SetMarginMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMarginMessage();
  } else if ( strncmp("SetComplianceValuesMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetComplianceValuesMessage();
  } else if ( strncmp("SetGoalSpeedMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetGoalSpeedMessage();
  } else if ( strncmp("SetTorqueLimitMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetTorqueLimitMessage();
  } else if ( strncmp("SetPunchMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetPunchMessage();
  } else if ( strncmp("GotoPositionMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GotoPositionMessage();
  } else if ( strncmp("SetAngleLimitsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetAngleLimitsMessage();
  } else if ( strncmp("ResetRawErrorMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ResetRawErrorMessage();
  } else if ( strncmp("SetPreventAlarmShutdownMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetPreventAlarmShutdownMessage();
  } else if ( strncmp("SetAutorecoverEnabledMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetAutorecoverEnabledMessage();
  } else if ( strncmp("RecoverMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RecoverMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
DynamixelServoInterface::copy_values(const Interface *other)
{
  const DynamixelServoInterface *oi = dynamic_cast<const DynamixelServoInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(DynamixelServoInterface_data_t));
}

const char *
DynamixelServoInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "ErrorCode") == 0) {
    return tostring_ErrorCode((ErrorCode)val);
  }
  if (strcmp(enumtype, "WorkingMode") == 0) {
    return tostring_WorkingMode((WorkingMode)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class DynamixelServoInterface::StopMessage <interfaces/DynamixelServoInterface.h>
 * StopMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
DynamixelServoInterface::StopMessage::StopMessage() : Message("StopMessage")
{
  data_size = sizeof(StopMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
}

/** Destructor */
DynamixelServoInterface::StopMessage::~StopMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::StopMessage::StopMessage(const StopMessage *m) : Message("StopMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::StopMessage::clone() const
{
  return new DynamixelServoInterface::StopMessage(this);
}
/** @class DynamixelServoInterface::FlushMessage <interfaces/DynamixelServoInterface.h>
 * FlushMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
DynamixelServoInterface::FlushMessage::FlushMessage() : Message("FlushMessage")
{
  data_size = sizeof(FlushMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (FlushMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
}

/** Destructor */
DynamixelServoInterface::FlushMessage::~FlushMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::FlushMessage::FlushMessage(const FlushMessage *m) : Message("FlushMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (FlushMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::FlushMessage::clone() const
{
  return new DynamixelServoInterface::FlushMessage(this);
}
/** @class DynamixelServoInterface::GotoMessage <interfaces/DynamixelServoInterface.h>
 * GotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_angle initial value for angle
 */
DynamixelServoInterface::GotoMessage::GotoMessage(const float ini_angle) : Message("GotoMessage")
{
  data_size = sizeof(GotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->angle = ini_angle;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
}
/** Constructor */
DynamixelServoInterface::GotoMessage::GotoMessage() : Message("GotoMessage")
{
  data_size = sizeof(GotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
}

/** Destructor */
DynamixelServoInterface::GotoMessage::~GotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::GotoMessage::GotoMessage(const GotoMessage *m) : Message("GotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get angle value.
 * Current angle.
 * @return angle value
 */
float
DynamixelServoInterface::GotoMessage::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::GotoMessage::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Current angle.
 * @param new_angle new angle value
 */
void
DynamixelServoInterface::GotoMessage::set_angle(const float new_angle)
{
  data->angle = new_angle;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::GotoMessage::clone() const
{
  return new DynamixelServoInterface::GotoMessage(this);
}
/** @class DynamixelServoInterface::TimedGotoMessage <interfaces/DynamixelServoInterface.h>
 * TimedGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_time_sec initial value for time_sec
 * @param ini_angle initial value for angle
 */
DynamixelServoInterface::TimedGotoMessage::TimedGotoMessage(const float ini_time_sec, const float ini_angle) : Message("TimedGotoMessage")
{
  data_size = sizeof(TimedGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TimedGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->time_sec = ini_time_sec;
  data->angle = ini_angle;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
}
/** Constructor */
DynamixelServoInterface::TimedGotoMessage::TimedGotoMessage() : Message("TimedGotoMessage")
{
  data_size = sizeof(TimedGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TimedGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
}

/** Destructor */
DynamixelServoInterface::TimedGotoMessage::~TimedGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::TimedGotoMessage::TimedGotoMessage(const TimedGotoMessage *m) : Message("TimedGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TimedGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get time_sec value.
 * Time in seconds when to reach
    the final position.
 * @return time_sec value
 */
float
DynamixelServoInterface::TimedGotoMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::TimedGotoMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time in seconds when to reach
    the final position.
 * @param new_time_sec new time_sec value
 */
void
DynamixelServoInterface::TimedGotoMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Get angle value.
 * Current angle.
 * @return angle value
 */
float
DynamixelServoInterface::TimedGotoMessage::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::TimedGotoMessage::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Current angle.
 * @param new_angle new angle value
 */
void
DynamixelServoInterface::TimedGotoMessage::set_angle(const float new_angle)
{
  data->angle = new_angle;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::TimedGotoMessage::clone() const
{
  return new DynamixelServoInterface::TimedGotoMessage(this);
}
/** @class DynamixelServoInterface::SetModeMessage <interfaces/DynamixelServoInterface.h>
 * SetModeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_mode initial value for mode
 */
DynamixelServoInterface::SetModeMessage::SetModeMessage(const uint8_t ini_mode) : Message("SetModeMessage")
{
  data_size = sizeof(SetModeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->mode = ini_mode;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT8, "mode", 1, &data->mode);
}
/** Constructor */
DynamixelServoInterface::SetModeMessage::SetModeMessage() : Message("SetModeMessage")
{
  data_size = sizeof(SetModeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT8, "mode", 1, &data->mode);
}

/** Destructor */
DynamixelServoInterface::SetModeMessage::~SetModeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetModeMessage::SetModeMessage(const SetModeMessage *m) : Message("SetModeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get mode value.
 * New mode, see the enum WorkingMode in this interface
 * @return mode value
 */
uint8_t
DynamixelServoInterface::SetModeMessage::mode() const
{
  return data->mode;
}

/** Get maximum length of mode value.
 * @return length of mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetModeMessage::maxlenof_mode() const
{
  return 1;
}

/** Set mode value.
 * New mode, see the enum WorkingMode in this interface
 * @param new_mode new mode value
 */
void
DynamixelServoInterface::SetModeMessage::set_mode(const uint8_t new_mode)
{
  data->mode = new_mode;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetModeMessage::clone() const
{
  return new DynamixelServoInterface::SetModeMessage(this);
}
/** @class DynamixelServoInterface::SetSpeedMessage <interfaces/DynamixelServoInterface.h>
 * SetSpeedMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_speed initial value for speed
 */
DynamixelServoInterface::SetSpeedMessage::SetSpeedMessage(const uint16_t ini_speed) : Message("SetSpeedMessage")
{
  data_size = sizeof(SetSpeedMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->speed = ini_speed;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT16, "speed", 1, &data->speed);
}
/** Constructor */
DynamixelServoInterface::SetSpeedMessage::SetSpeedMessage() : Message("SetSpeedMessage")
{
  data_size = sizeof(SetSpeedMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT16, "speed", 1, &data->speed);
}

/** Destructor */
DynamixelServoInterface::SetSpeedMessage::~SetSpeedMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetSpeedMessage::SetSpeedMessage(const SetSpeedMessage *m) : Message("SetSpeedMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get speed value.
 * New speed. Used when the servo is in wheel mode. Bits 0-9 determine the rotation speed and bit 10 determines the rotation direction (0 for ccw and 1 for cw)
 * @return speed value
 */
uint16_t
DynamixelServoInterface::SetSpeedMessage::speed() const
{
  return data->speed;
}

/** Get maximum length of speed value.
 * @return length of speed value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetSpeedMessage::maxlenof_speed() const
{
  return 1;
}

/** Set speed value.
 * New speed. Used when the servo is in wheel mode. Bits 0-9 determine the rotation speed and bit 10 determines the rotation direction (0 for ccw and 1 for cw)
 * @param new_speed new speed value
 */
void
DynamixelServoInterface::SetSpeedMessage::set_speed(const uint16_t new_speed)
{
  data->speed = new_speed;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetSpeedMessage::clone() const
{
  return new DynamixelServoInterface::SetSpeedMessage(this);
}
/** @class DynamixelServoInterface::SetEnabledMessage <interfaces/DynamixelServoInterface.h>
 * SetEnabledMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_enabled initial value for enabled
 */
DynamixelServoInterface::SetEnabledMessage::SetEnabledMessage(const bool ini_enabled) : Message("SetEnabledMessage")
{
  data_size = sizeof(SetEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->enabled = ini_enabled;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
}
/** Constructor */
DynamixelServoInterface::SetEnabledMessage::SetEnabledMessage() : Message("SetEnabledMessage")
{
  data_size = sizeof(SetEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
}

/** Destructor */
DynamixelServoInterface::SetEnabledMessage::~SetEnabledMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetEnabledMessage::SetEnabledMessage(const SetEnabledMessage *m) : Message("SetEnabledMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get enabled value.
 * Is the servo enabled?
 * @return enabled value
 */
bool
DynamixelServoInterface::SetEnabledMessage::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetEnabledMessage::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * Is the servo enabled?
 * @param new_enabled new enabled value
 */
void
DynamixelServoInterface::SetEnabledMessage::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetEnabledMessage::clone() const
{
  return new DynamixelServoInterface::SetEnabledMessage(this);
}
/** @class DynamixelServoInterface::SetVelocityMessage <interfaces/DynamixelServoInterface.h>
 * SetVelocityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_velocity initial value for velocity
 */
DynamixelServoInterface::SetVelocityMessage::SetVelocityMessage(const float ini_velocity) : Message("SetVelocityMessage")
{
  data_size = sizeof(SetVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->velocity = ini_velocity;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_FLOAT, "velocity", 1, &data->velocity);
}
/** Constructor */
DynamixelServoInterface::SetVelocityMessage::SetVelocityMessage() : Message("SetVelocityMessage")
{
  data_size = sizeof(SetVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_FLOAT, "velocity", 1, &data->velocity);
}

/** Destructor */
DynamixelServoInterface::SetVelocityMessage::~SetVelocityMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetVelocityMessage::SetVelocityMessage(const SetVelocityMessage *m) : Message("SetVelocityMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get velocity value.
 * Maximum servo velocity currently reached.
 * @return velocity value
 */
float
DynamixelServoInterface::SetVelocityMessage::velocity() const
{
  return data->velocity;
}

/** Get maximum length of velocity value.
 * @return length of velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetVelocityMessage::maxlenof_velocity() const
{
  return 1;
}

/** Set velocity value.
 * Maximum servo velocity currently reached.
 * @param new_velocity new velocity value
 */
void
DynamixelServoInterface::SetVelocityMessage::set_velocity(const float new_velocity)
{
  data->velocity = new_velocity;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetVelocityMessage::clone() const
{
  return new DynamixelServoInterface::SetVelocityMessage(this);
}
/** @class DynamixelServoInterface::SetMarginMessage <interfaces/DynamixelServoInterface.h>
 * SetMarginMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_angle_margin initial value for angle_margin
 */
DynamixelServoInterface::SetMarginMessage::SetMarginMessage(const float ini_angle_margin) : Message("SetMarginMessage")
{
  data_size = sizeof(SetMarginMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMarginMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->angle_margin = ini_angle_margin;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_FLOAT, "angle_margin", 1, &data->angle_margin);
}
/** Constructor */
DynamixelServoInterface::SetMarginMessage::SetMarginMessage() : Message("SetMarginMessage")
{
  data_size = sizeof(SetMarginMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMarginMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_FLOAT, "angle_margin", 1, &data->angle_margin);
}

/** Destructor */
DynamixelServoInterface::SetMarginMessage::~SetMarginMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetMarginMessage::SetMarginMessage(const SetMarginMessage *m) : Message("SetMarginMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetMarginMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get angle_margin value.
 * 
      Margin in radians around a target servo value to consider the
      motion as final.
    
 * @return angle_margin value
 */
float
DynamixelServoInterface::SetMarginMessage::angle_margin() const
{
  return data->angle_margin;
}

/** Get maximum length of angle_margin value.
 * @return length of angle_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetMarginMessage::maxlenof_angle_margin() const
{
  return 1;
}

/** Set angle_margin value.
 * 
      Margin in radians around a target servo value to consider the
      motion as final.
    
 * @param new_angle_margin new angle_margin value
 */
void
DynamixelServoInterface::SetMarginMessage::set_angle_margin(const float new_angle_margin)
{
  data->angle_margin = new_angle_margin;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetMarginMessage::clone() const
{
  return new DynamixelServoInterface::SetMarginMessage(this);
}
/** @class DynamixelServoInterface::SetComplianceValuesMessage <interfaces/DynamixelServoInterface.h>
 * SetComplianceValuesMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_cw_margin initial value for cw_margin
 * @param ini_ccw_margin initial value for ccw_margin
 * @param ini_cw_slope initial value for cw_slope
 * @param ini_ccw_slope initial value for ccw_slope
 */
DynamixelServoInterface::SetComplianceValuesMessage::SetComplianceValuesMessage(const uint8_t ini_cw_margin, const uint8_t ini_ccw_margin, const uint8_t ini_cw_slope, const uint8_t ini_ccw_slope) : Message("SetComplianceValuesMessage")
{
  data_size = sizeof(SetComplianceValuesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetComplianceValuesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->cw_margin = ini_cw_margin;
  data->ccw_margin = ini_ccw_margin;
  data->cw_slope = ini_cw_slope;
  data->ccw_slope = ini_ccw_slope;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT8, "cw_margin", 1, &data->cw_margin);
  add_fieldinfo(IFT_UINT8, "ccw_margin", 1, &data->ccw_margin);
  add_fieldinfo(IFT_UINT8, "cw_slope", 1, &data->cw_slope);
  add_fieldinfo(IFT_UINT8, "ccw_slope", 1, &data->ccw_slope);
}
/** Constructor */
DynamixelServoInterface::SetComplianceValuesMessage::SetComplianceValuesMessage() : Message("SetComplianceValuesMessage")
{
  data_size = sizeof(SetComplianceValuesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetComplianceValuesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT8, "cw_margin", 1, &data->cw_margin);
  add_fieldinfo(IFT_UINT8, "ccw_margin", 1, &data->ccw_margin);
  add_fieldinfo(IFT_UINT8, "cw_slope", 1, &data->cw_slope);
  add_fieldinfo(IFT_UINT8, "ccw_slope", 1, &data->ccw_slope);
}

/** Destructor */
DynamixelServoInterface::SetComplianceValuesMessage::~SetComplianceValuesMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetComplianceValuesMessage::SetComplianceValuesMessage(const SetComplianceValuesMessage *m) : Message("SetComplianceValuesMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetComplianceValuesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get cw_margin value.
 * New CW compliance margin
 * @return cw_margin value
 */
uint8_t
DynamixelServoInterface::SetComplianceValuesMessage::cw_margin() const
{
  return data->cw_margin;
}

/** Get maximum length of cw_margin value.
 * @return length of cw_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetComplianceValuesMessage::maxlenof_cw_margin() const
{
  return 1;
}

/** Set cw_margin value.
 * New CW compliance margin
 * @param new_cw_margin new cw_margin value
 */
void
DynamixelServoInterface::SetComplianceValuesMessage::set_cw_margin(const uint8_t new_cw_margin)
{
  data->cw_margin = new_cw_margin;
}

/** Get ccw_margin value.
 * New CCW compliance margin
 * @return ccw_margin value
 */
uint8_t
DynamixelServoInterface::SetComplianceValuesMessage::ccw_margin() const
{
  return data->ccw_margin;
}

/** Get maximum length of ccw_margin value.
 * @return length of ccw_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetComplianceValuesMessage::maxlenof_ccw_margin() const
{
  return 1;
}

/** Set ccw_margin value.
 * New CCW compliance margin
 * @param new_ccw_margin new ccw_margin value
 */
void
DynamixelServoInterface::SetComplianceValuesMessage::set_ccw_margin(const uint8_t new_ccw_margin)
{
  data->ccw_margin = new_ccw_margin;
}

/** Get cw_slope value.
 * New CW compliance slope
 * @return cw_slope value
 */
uint8_t
DynamixelServoInterface::SetComplianceValuesMessage::cw_slope() const
{
  return data->cw_slope;
}

/** Get maximum length of cw_slope value.
 * @return length of cw_slope value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetComplianceValuesMessage::maxlenof_cw_slope() const
{
  return 1;
}

/** Set cw_slope value.
 * New CW compliance slope
 * @param new_cw_slope new cw_slope value
 */
void
DynamixelServoInterface::SetComplianceValuesMessage::set_cw_slope(const uint8_t new_cw_slope)
{
  data->cw_slope = new_cw_slope;
}

/** Get ccw_slope value.
 * New LED enabled value
 * @return ccw_slope value
 */
uint8_t
DynamixelServoInterface::SetComplianceValuesMessage::ccw_slope() const
{
  return data->ccw_slope;
}

/** Get maximum length of ccw_slope value.
 * @return length of ccw_slope value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetComplianceValuesMessage::maxlenof_ccw_slope() const
{
  return 1;
}

/** Set ccw_slope value.
 * New LED enabled value
 * @param new_ccw_slope new ccw_slope value
 */
void
DynamixelServoInterface::SetComplianceValuesMessage::set_ccw_slope(const uint8_t new_ccw_slope)
{
  data->ccw_slope = new_ccw_slope;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetComplianceValuesMessage::clone() const
{
  return new DynamixelServoInterface::SetComplianceValuesMessage(this);
}
/** @class DynamixelServoInterface::SetGoalSpeedMessage <interfaces/DynamixelServoInterface.h>
 * SetGoalSpeedMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_goal_speed initial value for goal_speed
 */
DynamixelServoInterface::SetGoalSpeedMessage::SetGoalSpeedMessage(const uint32_t ini_goal_speed) : Message("SetGoalSpeedMessage")
{
  data_size = sizeof(SetGoalSpeedMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGoalSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->goal_speed = ini_goal_speed;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT32, "goal_speed", 1, &data->goal_speed);
}
/** Constructor */
DynamixelServoInterface::SetGoalSpeedMessage::SetGoalSpeedMessage() : Message("SetGoalSpeedMessage")
{
  data_size = sizeof(SetGoalSpeedMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGoalSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT32, "goal_speed", 1, &data->goal_speed);
}

/** Destructor */
DynamixelServoInterface::SetGoalSpeedMessage::~SetGoalSpeedMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetGoalSpeedMessage::SetGoalSpeedMessage(const SetGoalSpeedMessage *m) : Message("SetGoalSpeedMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetGoalSpeedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get goal_speed value.
 * New goal speed
 * @return goal_speed value
 */
uint32_t
DynamixelServoInterface::SetGoalSpeedMessage::goal_speed() const
{
  return data->goal_speed;
}

/** Get maximum length of goal_speed value.
 * @return length of goal_speed value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetGoalSpeedMessage::maxlenof_goal_speed() const
{
  return 1;
}

/** Set goal_speed value.
 * New goal speed
 * @param new_goal_speed new goal_speed value
 */
void
DynamixelServoInterface::SetGoalSpeedMessage::set_goal_speed(const uint32_t new_goal_speed)
{
  data->goal_speed = new_goal_speed;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetGoalSpeedMessage::clone() const
{
  return new DynamixelServoInterface::SetGoalSpeedMessage(this);
}
/** @class DynamixelServoInterface::SetTorqueLimitMessage <interfaces/DynamixelServoInterface.h>
 * SetTorqueLimitMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_torque_limit initial value for torque_limit
 */
DynamixelServoInterface::SetTorqueLimitMessage::SetTorqueLimitMessage(const uint32_t ini_torque_limit) : Message("SetTorqueLimitMessage")
{
  data_size = sizeof(SetTorqueLimitMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTorqueLimitMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->torque_limit = ini_torque_limit;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT32, "torque_limit", 1, &data->torque_limit);
}
/** Constructor */
DynamixelServoInterface::SetTorqueLimitMessage::SetTorqueLimitMessage() : Message("SetTorqueLimitMessage")
{
  data_size = sizeof(SetTorqueLimitMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTorqueLimitMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT32, "torque_limit", 1, &data->torque_limit);
}

/** Destructor */
DynamixelServoInterface::SetTorqueLimitMessage::~SetTorqueLimitMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetTorqueLimitMessage::SetTorqueLimitMessage(const SetTorqueLimitMessage *m) : Message("SetTorqueLimitMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetTorqueLimitMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get torque_limit value.
 * New torque limit
 * @return torque_limit value
 */
uint32_t
DynamixelServoInterface::SetTorqueLimitMessage::torque_limit() const
{
  return data->torque_limit;
}

/** Get maximum length of torque_limit value.
 * @return length of torque_limit value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetTorqueLimitMessage::maxlenof_torque_limit() const
{
  return 1;
}

/** Set torque_limit value.
 * New torque limit
 * @param new_torque_limit new torque_limit value
 */
void
DynamixelServoInterface::SetTorqueLimitMessage::set_torque_limit(const uint32_t new_torque_limit)
{
  data->torque_limit = new_torque_limit;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetTorqueLimitMessage::clone() const
{
  return new DynamixelServoInterface::SetTorqueLimitMessage(this);
}
/** @class DynamixelServoInterface::SetPunchMessage <interfaces/DynamixelServoInterface.h>
 * SetPunchMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_punch initial value for punch
 */
DynamixelServoInterface::SetPunchMessage::SetPunchMessage(const uint32_t ini_punch) : Message("SetPunchMessage")
{
  data_size = sizeof(SetPunchMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPunchMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->punch = ini_punch;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT32, "punch", 1, &data->punch);
}
/** Constructor */
DynamixelServoInterface::SetPunchMessage::SetPunchMessage() : Message("SetPunchMessage")
{
  data_size = sizeof(SetPunchMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPunchMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT32, "punch", 1, &data->punch);
}

/** Destructor */
DynamixelServoInterface::SetPunchMessage::~SetPunchMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetPunchMessage::SetPunchMessage(const SetPunchMessage *m) : Message("SetPunchMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetPunchMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get punch value.
 * New punch
 * @return punch value
 */
uint32_t
DynamixelServoInterface::SetPunchMessage::punch() const
{
  return data->punch;
}

/** Get maximum length of punch value.
 * @return length of punch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetPunchMessage::maxlenof_punch() const
{
  return 1;
}

/** Set punch value.
 * New punch
 * @param new_punch new punch value
 */
void
DynamixelServoInterface::SetPunchMessage::set_punch(const uint32_t new_punch)
{
  data->punch = new_punch;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetPunchMessage::clone() const
{
  return new DynamixelServoInterface::SetPunchMessage(this);
}
/** @class DynamixelServoInterface::GotoPositionMessage <interfaces/DynamixelServoInterface.h>
 * GotoPositionMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_position initial value for position
 */
DynamixelServoInterface::GotoPositionMessage::GotoPositionMessage(const uint32_t ini_position) : Message("GotoPositionMessage")
{
  data_size = sizeof(GotoPositionMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoPositionMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->position = ini_position;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT32, "position", 1, &data->position);
}
/** Constructor */
DynamixelServoInterface::GotoPositionMessage::GotoPositionMessage() : Message("GotoPositionMessage")
{
  data_size = sizeof(GotoPositionMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoPositionMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT32, "position", 1, &data->position);
}

/** Destructor */
DynamixelServoInterface::GotoPositionMessage::~GotoPositionMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::GotoPositionMessage::GotoPositionMessage(const GotoPositionMessage *m) : Message("GotoPositionMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GotoPositionMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get position value.
 * New position
 * @return position value
 */
uint32_t
DynamixelServoInterface::GotoPositionMessage::position() const
{
  return data->position;
}

/** Get maximum length of position value.
 * @return length of position value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::GotoPositionMessage::maxlenof_position() const
{
  return 1;
}

/** Set position value.
 * New position
 * @param new_position new position value
 */
void
DynamixelServoInterface::GotoPositionMessage::set_position(const uint32_t new_position)
{
  data->position = new_position;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::GotoPositionMessage::clone() const
{
  return new DynamixelServoInterface::GotoPositionMessage(this);
}
/** @class DynamixelServoInterface::SetAngleLimitsMessage <interfaces/DynamixelServoInterface.h>
 * SetAngleLimitsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_angle_limit_cw initial value for angle_limit_cw
 * @param ini_angle_limit_ccw initial value for angle_limit_ccw
 */
DynamixelServoInterface::SetAngleLimitsMessage::SetAngleLimitsMessage(const uint32_t ini_angle_limit_cw, const uint32_t ini_angle_limit_ccw) : Message("SetAngleLimitsMessage")
{
  data_size = sizeof(SetAngleLimitsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetAngleLimitsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->angle_limit_cw = ini_angle_limit_cw;
  data->angle_limit_ccw = ini_angle_limit_ccw;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT32, "angle_limit_cw", 1, &data->angle_limit_cw);
  add_fieldinfo(IFT_UINT32, "angle_limit_ccw", 1, &data->angle_limit_ccw);
}
/** Constructor */
DynamixelServoInterface::SetAngleLimitsMessage::SetAngleLimitsMessage() : Message("SetAngleLimitsMessage")
{
  data_size = sizeof(SetAngleLimitsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetAngleLimitsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_UINT32, "angle_limit_cw", 1, &data->angle_limit_cw);
  add_fieldinfo(IFT_UINT32, "angle_limit_ccw", 1, &data->angle_limit_ccw);
}

/** Destructor */
DynamixelServoInterface::SetAngleLimitsMessage::~SetAngleLimitsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetAngleLimitsMessage::SetAngleLimitsMessage(const SetAngleLimitsMessage *m) : Message("SetAngleLimitsMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetAngleLimitsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get angle_limit_cw value.
 * New cw angle limit
 * @return angle_limit_cw value
 */
uint32_t
DynamixelServoInterface::SetAngleLimitsMessage::angle_limit_cw() const
{
  return data->angle_limit_cw;
}

/** Get maximum length of angle_limit_cw value.
 * @return length of angle_limit_cw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetAngleLimitsMessage::maxlenof_angle_limit_cw() const
{
  return 1;
}

/** Set angle_limit_cw value.
 * New cw angle limit
 * @param new_angle_limit_cw new angle_limit_cw value
 */
void
DynamixelServoInterface::SetAngleLimitsMessage::set_angle_limit_cw(const uint32_t new_angle_limit_cw)
{
  data->angle_limit_cw = new_angle_limit_cw;
}

/** Get angle_limit_ccw value.
 * New ccw angle limit
 * @return angle_limit_ccw value
 */
uint32_t
DynamixelServoInterface::SetAngleLimitsMessage::angle_limit_ccw() const
{
  return data->angle_limit_ccw;
}

/** Get maximum length of angle_limit_ccw value.
 * @return length of angle_limit_ccw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetAngleLimitsMessage::maxlenof_angle_limit_ccw() const
{
  return 1;
}

/** Set angle_limit_ccw value.
 * New ccw angle limit
 * @param new_angle_limit_ccw new angle_limit_ccw value
 */
void
DynamixelServoInterface::SetAngleLimitsMessage::set_angle_limit_ccw(const uint32_t new_angle_limit_ccw)
{
  data->angle_limit_ccw = new_angle_limit_ccw;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetAngleLimitsMessage::clone() const
{
  return new DynamixelServoInterface::SetAngleLimitsMessage(this);
}
/** @class DynamixelServoInterface::ResetRawErrorMessage <interfaces/DynamixelServoInterface.h>
 * ResetRawErrorMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
DynamixelServoInterface::ResetRawErrorMessage::ResetRawErrorMessage() : Message("ResetRawErrorMessage")
{
  data_size = sizeof(ResetRawErrorMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ResetRawErrorMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
}

/** Destructor */
DynamixelServoInterface::ResetRawErrorMessage::~ResetRawErrorMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::ResetRawErrorMessage::ResetRawErrorMessage(const ResetRawErrorMessage *m) : Message("ResetRawErrorMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ResetRawErrorMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::ResetRawErrorMessage::clone() const
{
  return new DynamixelServoInterface::ResetRawErrorMessage(this);
}
/** @class DynamixelServoInterface::SetPreventAlarmShutdownMessage <interfaces/DynamixelServoInterface.h>
 * SetPreventAlarmShutdownMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_enable_prevent_alarm_shutdown initial value for enable_prevent_alarm_shutdown
 */
DynamixelServoInterface::SetPreventAlarmShutdownMessage::SetPreventAlarmShutdownMessage(const bool ini_enable_prevent_alarm_shutdown) : Message("SetPreventAlarmShutdownMessage")
{
  data_size = sizeof(SetPreventAlarmShutdownMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPreventAlarmShutdownMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->enable_prevent_alarm_shutdown = ini_enable_prevent_alarm_shutdown;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_BOOL, "enable_prevent_alarm_shutdown", 1, &data->enable_prevent_alarm_shutdown);
}
/** Constructor */
DynamixelServoInterface::SetPreventAlarmShutdownMessage::SetPreventAlarmShutdownMessage() : Message("SetPreventAlarmShutdownMessage")
{
  data_size = sizeof(SetPreventAlarmShutdownMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPreventAlarmShutdownMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_BOOL, "enable_prevent_alarm_shutdown", 1, &data->enable_prevent_alarm_shutdown);
}

/** Destructor */
DynamixelServoInterface::SetPreventAlarmShutdownMessage::~SetPreventAlarmShutdownMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetPreventAlarmShutdownMessage::SetPreventAlarmShutdownMessage(const SetPreventAlarmShutdownMessage *m) : Message("SetPreventAlarmShutdownMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetPreventAlarmShutdownMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get enable_prevent_alarm_shutdown value.
 * Enable alarm shutdown
 * @return enable_prevent_alarm_shutdown value
 */
bool
DynamixelServoInterface::SetPreventAlarmShutdownMessage::is_enable_prevent_alarm_shutdown() const
{
  return data->enable_prevent_alarm_shutdown;
}

/** Get maximum length of enable_prevent_alarm_shutdown value.
 * @return length of enable_prevent_alarm_shutdown value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetPreventAlarmShutdownMessage::maxlenof_enable_prevent_alarm_shutdown() const
{
  return 1;
}

/** Set enable_prevent_alarm_shutdown value.
 * Enable alarm shutdown
 * @param new_enable_prevent_alarm_shutdown new enable_prevent_alarm_shutdown value
 */
void
DynamixelServoInterface::SetPreventAlarmShutdownMessage::set_enable_prevent_alarm_shutdown(const bool new_enable_prevent_alarm_shutdown)
{
  data->enable_prevent_alarm_shutdown = new_enable_prevent_alarm_shutdown;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetPreventAlarmShutdownMessage::clone() const
{
  return new DynamixelServoInterface::SetPreventAlarmShutdownMessage(this);
}
/** @class DynamixelServoInterface::SetAutorecoverEnabledMessage <interfaces/DynamixelServoInterface.h>
 * SetAutorecoverEnabledMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_autorecover_enabled initial value for autorecover_enabled
 */
DynamixelServoInterface::SetAutorecoverEnabledMessage::SetAutorecoverEnabledMessage(const bool ini_autorecover_enabled) : Message("SetAutorecoverEnabledMessage")
{
  data_size = sizeof(SetAutorecoverEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetAutorecoverEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->autorecover_enabled = ini_autorecover_enabled;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_BOOL, "autorecover_enabled", 1, &data->autorecover_enabled);
}
/** Constructor */
DynamixelServoInterface::SetAutorecoverEnabledMessage::SetAutorecoverEnabledMessage() : Message("SetAutorecoverEnabledMessage")
{
  data_size = sizeof(SetAutorecoverEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetAutorecoverEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
  add_fieldinfo(IFT_BOOL, "autorecover_enabled", 1, &data->autorecover_enabled);
}

/** Destructor */
DynamixelServoInterface::SetAutorecoverEnabledMessage::~SetAutorecoverEnabledMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::SetAutorecoverEnabledMessage::SetAutorecoverEnabledMessage(const SetAutorecoverEnabledMessage *m) : Message("SetAutorecoverEnabledMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetAutorecoverEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get autorecover_enabled value.
 * Automatically recover on alarm shutdown
 * @return autorecover_enabled value
 */
bool
DynamixelServoInterface::SetAutorecoverEnabledMessage::is_autorecover_enabled() const
{
  return data->autorecover_enabled;
}

/** Get maximum length of autorecover_enabled value.
 * @return length of autorecover_enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
DynamixelServoInterface::SetAutorecoverEnabledMessage::maxlenof_autorecover_enabled() const
{
  return 1;
}

/** Set autorecover_enabled value.
 * Automatically recover on alarm shutdown
 * @param new_autorecover_enabled new autorecover_enabled value
 */
void
DynamixelServoInterface::SetAutorecoverEnabledMessage::set_autorecover_enabled(const bool new_autorecover_enabled)
{
  data->autorecover_enabled = new_autorecover_enabled;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::SetAutorecoverEnabledMessage::clone() const
{
  return new DynamixelServoInterface::SetAutorecoverEnabledMessage(this);
}
/** @class DynamixelServoInterface::RecoverMessage <interfaces/DynamixelServoInterface.h>
 * RecoverMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
DynamixelServoInterface::RecoverMessage::RecoverMessage() : Message("RecoverMessage")
{
  data_size = sizeof(RecoverMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RecoverMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ErrorCode[(int)ERROR_NONE] = "ERROR_NONE";
  enum_map_ErrorCode[(int)ERROR_UNSPECIFIC] = "ERROR_UNSPECIFIC";
  enum_map_ErrorCode[(int)ERROR_COMMUNICATION] = "ERROR_COMMUNICATION";
  enum_map_ErrorCode[(int)ERROR_ANGLE_OUTOFRANGE] = "ERROR_ANGLE_OUTOFRANGE";
  enum_map_WorkingMode[(int)JOINT] = "JOINT";
  enum_map_WorkingMode[(int)WHEEL] = "WHEEL";
}

/** Destructor */
DynamixelServoInterface::RecoverMessage::~RecoverMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
DynamixelServoInterface::RecoverMessage::RecoverMessage(const RecoverMessage *m) : Message("RecoverMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (RecoverMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
DynamixelServoInterface::RecoverMessage::clone() const
{
  return new DynamixelServoInterface::RecoverMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
DynamixelServoInterface::message_valid(const Message *message) const
{
  const StopMessage *m0 = dynamic_cast<const StopMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const FlushMessage *m1 = dynamic_cast<const FlushMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const GotoMessage *m2 = dynamic_cast<const GotoMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const TimedGotoMessage *m3 = dynamic_cast<const TimedGotoMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const SetModeMessage *m4 = dynamic_cast<const SetModeMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const SetSpeedMessage *m5 = dynamic_cast<const SetSpeedMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const SetEnabledMessage *m6 = dynamic_cast<const SetEnabledMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const SetVelocityMessage *m7 = dynamic_cast<const SetVelocityMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const SetMarginMessage *m8 = dynamic_cast<const SetMarginMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  const SetComplianceValuesMessage *m9 = dynamic_cast<const SetComplianceValuesMessage *>(message);
  if ( m9 != NULL ) {
    return true;
  }
  const SetGoalSpeedMessage *m10 = dynamic_cast<const SetGoalSpeedMessage *>(message);
  if ( m10 != NULL ) {
    return true;
  }
  const SetTorqueLimitMessage *m11 = dynamic_cast<const SetTorqueLimitMessage *>(message);
  if ( m11 != NULL ) {
    return true;
  }
  const SetPunchMessage *m12 = dynamic_cast<const SetPunchMessage *>(message);
  if ( m12 != NULL ) {
    return true;
  }
  const GotoPositionMessage *m13 = dynamic_cast<const GotoPositionMessage *>(message);
  if ( m13 != NULL ) {
    return true;
  }
  const SetAngleLimitsMessage *m14 = dynamic_cast<const SetAngleLimitsMessage *>(message);
  if ( m14 != NULL ) {
    return true;
  }
  const ResetRawErrorMessage *m15 = dynamic_cast<const ResetRawErrorMessage *>(message);
  if ( m15 != NULL ) {
    return true;
  }
  const SetPreventAlarmShutdownMessage *m16 = dynamic_cast<const SetPreventAlarmShutdownMessage *>(message);
  if ( m16 != NULL ) {
    return true;
  }
  const SetAutorecoverEnabledMessage *m17 = dynamic_cast<const SetAutorecoverEnabledMessage *>(message);
  if ( m17 != NULL ) {
    return true;
  }
  const RecoverMessage *m18 = dynamic_cast<const RecoverMessage *>(message);
  if ( m18 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(DynamixelServoInterface)
/// @endcond


} // end namespace fawkes
