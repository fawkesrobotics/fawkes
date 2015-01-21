
/***************************************************************************
 *  Roomba500Interface.cpp - Fawkes BlackBoard Interface - Roomba500Interface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2011  Tim Niemueller
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

#include <interfaces/Roomba500Interface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class Roomba500Interface <interfaces/Roomba500Interface.h>
 * Roomba500Interface Fawkes BlackBoard Interface.
 * 
      Roomba 500 hardware interface.      
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
Roomba500Interface::Roomba500Interface() : Interface()
{
  data_size = sizeof(Roomba500Interface_data_t);
  data_ptr  = malloc(data_size);
  data      = (Roomba500Interface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_Mode[(int)MODE_OFF] = "MODE_OFF";
  enum_map_Mode[(int)MODE_PASSIVE] = "MODE_PASSIVE";
  enum_map_Mode[(int)MODE_SAFE] = "MODE_SAFE";
  enum_map_Mode[(int)MODE_FULL] = "MODE_FULL";
  enum_map_InfraredCharacter[(int)IR_NONE] = "IR_NONE";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LEFT] = "IR_REMOTE_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_FORWARD] = "IR_REMOTE_FORWARD";
  enum_map_InfraredCharacter[(int)IR_REMOTE_RIGHT] = "IR_REMOTE_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SPOT] = "IR_REMOTE_SPOT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MAX] = "IR_REMOTE_MAX";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SMALL] = "IR_REMOTE_SMALL";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MEDIUM] = "IR_REMOTE_MEDIUM";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LARGE_CLEAN] = "IR_REMOTE_LARGE_CLEAN";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP] = "IR_REMOTE_STOP";
  enum_map_InfraredCharacter[(int)IR_REMOTE_POWER] = "IR_REMOTE_POWER";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_LEFT] = "IR_REMOTE_ARC_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_RIGHT] = "IR_REMOTE_ARC_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP2] = "IR_REMOTE_STOP2";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_DOWNLOAD] = "IR_SCHED_REMOTE_DOWNLOAD";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_SEEK_DOCK] = "IR_SCHED_REMOTE_SEEK_DOCK";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RESERVED] = "IR_DISC_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY] = "IR_DISC_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY] = "IR_DISC_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_FORCE_FIELD] = "IR_DISC_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY] = "IR_DISC_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RESERVED] = "IR_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY] = "IR_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY] = "IR_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_FORCE_FIELD] = "IR_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY] = "IR_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_VIRTUAL_WALL] = "IR_VIRTUAL_WALL";
  enum_map_ChargingState[(int)CHARGING_NO] = "CHARGING_NO";
  enum_map_ChargingState[(int)CHARGING_RECONDITIONING] = "CHARGING_RECONDITIONING";
  enum_map_ChargingState[(int)CHARGING_FULL] = "CHARGING_FULL";
  enum_map_ChargingState[(int)CHARGING_TRICKLE] = "CHARGING_TRICKLE";
  enum_map_ChargingState[(int)CHARGING_WAITING] = "CHARGING_WAITING";
  enum_map_ChargingState[(int)CHARGING_ERROR] = "CHARGING_ERROR";
  enum_map_BrushState[(int)BRUSHSTATE_OFF] = "BRUSHSTATE_OFF";
  enum_map_BrushState[(int)BRUSHSTATE_FORWARD] = "BRUSHSTATE_FORWARD";
  enum_map_BrushState[(int)BRUSHSTATE_BACKWARD] = "BRUSHSTATE_BACKWARD";
  add_fieldinfo(IFT_ENUM, "mode", 1, &data->mode, "Mode", &enum_map_Mode);
  add_fieldinfo(IFT_BOOL, "wheel_drop_left", 1, &data->wheel_drop_left);
  add_fieldinfo(IFT_BOOL, "wheel_drop_right", 1, &data->wheel_drop_right);
  add_fieldinfo(IFT_BOOL, "bump_left", 1, &data->bump_left);
  add_fieldinfo(IFT_BOOL, "bump_right", 1, &data->bump_right);
  add_fieldinfo(IFT_BOOL, "wall", 1, &data->wall);
  add_fieldinfo(IFT_BOOL, "cliff_left", 1, &data->cliff_left);
  add_fieldinfo(IFT_BOOL, "cliff_front_left", 1, &data->cliff_front_left);
  add_fieldinfo(IFT_BOOL, "cliff_front_right", 1, &data->cliff_front_right);
  add_fieldinfo(IFT_BOOL, "cliff_right", 1, &data->cliff_right);
  add_fieldinfo(IFT_BOOL, "virtual_wall", 1, &data->virtual_wall);
  add_fieldinfo(IFT_BOOL, "overcurrent_side_brush", 1, &data->overcurrent_side_brush);
  add_fieldinfo(IFT_BOOL, "overcurrent_main_brush", 1, &data->overcurrent_main_brush);
  add_fieldinfo(IFT_BOOL, "overcurrent_left_wheel", 1, &data->overcurrent_left_wheel);
  add_fieldinfo(IFT_BOOL, "overcurrent_right_wheel", 1, &data->overcurrent_right_wheel);
  add_fieldinfo(IFT_BOOL, "dirt_detect", 1, &data->dirt_detect);
  add_fieldinfo(IFT_ENUM, "ir_opcode_omni", 1, &data->ir_opcode_omni, "InfraredCharacter", &enum_map_InfraredCharacter);
  add_fieldinfo(IFT_BOOL, "button_clean", 1, &data->button_clean);
  add_fieldinfo(IFT_BOOL, "button_spot", 1, &data->button_spot);
  add_fieldinfo(IFT_BOOL, "button_dock", 1, &data->button_dock);
  add_fieldinfo(IFT_BOOL, "button_minute", 1, &data->button_minute);
  add_fieldinfo(IFT_BOOL, "button_hour", 1, &data->button_hour);
  add_fieldinfo(IFT_BOOL, "button_day", 1, &data->button_day);
  add_fieldinfo(IFT_BOOL, "button_schedule", 1, &data->button_schedule);
  add_fieldinfo(IFT_BOOL, "button_clock", 1, &data->button_clock);
  add_fieldinfo(IFT_INT16, "distance", 1, &data->distance);
  add_fieldinfo(IFT_INT16, "angle", 1, &data->angle);
  add_fieldinfo(IFT_ENUM, "charging_state", 1, &data->charging_state, "ChargingState", &enum_map_ChargingState);
  add_fieldinfo(IFT_UINT16, "voltage", 1, &data->voltage);
  add_fieldinfo(IFT_INT16, "current", 1, &data->current);
  add_fieldinfo(IFT_INT8, "temperature", 1, &data->temperature);
  add_fieldinfo(IFT_UINT16, "battery_charge", 1, &data->battery_charge);
  add_fieldinfo(IFT_UINT16, "battery_capacity", 1, &data->battery_capacity);
  add_fieldinfo(IFT_UINT16, "wall_signal", 1, &data->wall_signal);
  add_fieldinfo(IFT_UINT16, "cliff_left_signal", 1, &data->cliff_left_signal);
  add_fieldinfo(IFT_UINT16, "cliff_front_left_signal", 1, &data->cliff_front_left_signal);
  add_fieldinfo(IFT_UINT16, "cliff_front_right_signal", 1, &data->cliff_front_right_signal);
  add_fieldinfo(IFT_UINT16, "cliff_right_signal", 1, &data->cliff_right_signal);
  add_fieldinfo(IFT_BOOL, "home_base_charger_available", 1, &data->home_base_charger_available);
  add_fieldinfo(IFT_BOOL, "internal_charger_available", 1, &data->internal_charger_available);
  add_fieldinfo(IFT_UINT8, "song_number", 1, &data->song_number);
  add_fieldinfo(IFT_BOOL, "song_playing", 1, &data->song_playing);
  add_fieldinfo(IFT_INT16, "velocity", 1, &data->velocity);
  add_fieldinfo(IFT_INT16, "radius", 1, &data->radius);
  add_fieldinfo(IFT_INT16, "velocity_right", 1, &data->velocity_right);
  add_fieldinfo(IFT_INT16, "velocity_left", 1, &data->velocity_left);
  add_fieldinfo(IFT_UINT16, "encoder_counts_left", 1, &data->encoder_counts_left);
  add_fieldinfo(IFT_UINT16, "encoder_counts_right", 1, &data->encoder_counts_right);
  add_fieldinfo(IFT_BOOL, "bumper_left", 1, &data->bumper_left);
  add_fieldinfo(IFT_BOOL, "bumper_front_left", 1, &data->bumper_front_left);
  add_fieldinfo(IFT_BOOL, "bumper_center_left", 1, &data->bumper_center_left);
  add_fieldinfo(IFT_BOOL, "bumper_center_right", 1, &data->bumper_center_right);
  add_fieldinfo(IFT_BOOL, "bumper_front_right", 1, &data->bumper_front_right);
  add_fieldinfo(IFT_BOOL, "bumper_right", 1, &data->bumper_right);
  add_fieldinfo(IFT_UINT16, "light_bump_left", 1, &data->light_bump_left);
  add_fieldinfo(IFT_UINT16, "light_bump_front_left", 1, &data->light_bump_front_left);
  add_fieldinfo(IFT_UINT16, "light_bump_center_left", 1, &data->light_bump_center_left);
  add_fieldinfo(IFT_UINT16, "light_bump_center_right", 1, &data->light_bump_center_right);
  add_fieldinfo(IFT_UINT16, "light_bump_front_right", 1, &data->light_bump_front_right);
  add_fieldinfo(IFT_UINT16, "light_bump_right", 1, &data->light_bump_right);
  add_fieldinfo(IFT_ENUM, "ir_opcode_left", 1, &data->ir_opcode_left, "InfraredCharacter", &enum_map_InfraredCharacter);
  add_fieldinfo(IFT_ENUM, "ir_opcode_right", 1, &data->ir_opcode_right, "InfraredCharacter", &enum_map_InfraredCharacter);
  add_fieldinfo(IFT_INT16, "left_motor_current", 1, &data->left_motor_current);
  add_fieldinfo(IFT_INT16, "right_motor_current", 1, &data->right_motor_current);
  add_fieldinfo(IFT_INT16, "main_brush_current", 1, &data->main_brush_current);
  add_fieldinfo(IFT_INT16, "side_brush_current", 1, &data->side_brush_current);
  add_fieldinfo(IFT_BOOL, "caster_stasis", 1, &data->caster_stasis);
  add_messageinfo("StopMessage");
  add_messageinfo("DockMessage");
  add_messageinfo("SetModeMessage");
  add_messageinfo("DriveStraightMessage");
  add_messageinfo("DriveMessage");
  add_messageinfo("SetMotorsMessage");
  unsigned char tmp_hash[] = {0x60, 0xa1, 0x1e, 0x86, 0x89, 0x2, 0x20, 0x34, 0x89, 0x32, 0xdb, 0x69, 0xee, 0x12, 0x86, 0x43};
  set_hash(tmp_hash);
}

/** Destructor */
Roomba500Interface::~Roomba500Interface()
{
  free(data_ptr);
}
/** Convert Mode constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
Roomba500Interface::tostring_Mode(Mode value) const
{
  switch (value) {
  case MODE_OFF: return "MODE_OFF";
  case MODE_PASSIVE: return "MODE_PASSIVE";
  case MODE_SAFE: return "MODE_SAFE";
  case MODE_FULL: return "MODE_FULL";
  default: return "UNKNOWN";
  }
}
/** Convert InfraredCharacter constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
Roomba500Interface::tostring_InfraredCharacter(InfraredCharacter value) const
{
  switch (value) {
  case IR_NONE: return "IR_NONE";
  case IR_REMOTE_LEFT: return "IR_REMOTE_LEFT";
  case IR_REMOTE_FORWARD: return "IR_REMOTE_FORWARD";
  case IR_REMOTE_RIGHT: return "IR_REMOTE_RIGHT";
  case IR_REMOTE_SPOT: return "IR_REMOTE_SPOT";
  case IR_REMOTE_MAX: return "IR_REMOTE_MAX";
  case IR_REMOTE_SMALL: return "IR_REMOTE_SMALL";
  case IR_REMOTE_MEDIUM: return "IR_REMOTE_MEDIUM";
  case IR_REMOTE_LARGE_CLEAN: return "IR_REMOTE_LARGE_CLEAN";
  case IR_REMOTE_STOP: return "IR_REMOTE_STOP";
  case IR_REMOTE_POWER: return "IR_REMOTE_POWER";
  case IR_REMOTE_ARC_LEFT: return "IR_REMOTE_ARC_LEFT";
  case IR_REMOTE_ARC_RIGHT: return "IR_REMOTE_ARC_RIGHT";
  case IR_REMOTE_STOP2: return "IR_REMOTE_STOP2";
  case IR_SCHED_REMOTE_DOWNLOAD: return "IR_SCHED_REMOTE_DOWNLOAD";
  case IR_SCHED_REMOTE_SEEK_DOCK: return "IR_SCHED_REMOTE_SEEK_DOCK";
  case IR_DISC_DOCK_RESERVED: return "IR_DISC_DOCK_RESERVED";
  case IR_DISC_DOCK_RED_BUOY: return "IR_DISC_DOCK_RED_BUOY";
  case IR_DISC_DOCK_GREEN_BUOY: return "IR_DISC_DOCK_GREEN_BUOY";
  case IR_DISC_DOCK_FORCE_FIELD: return "IR_DISC_DOCK_FORCE_FIELD";
  case IR_DISC_DOCK_RED_GREEN_BUOY: return "IR_DISC_DOCK_RED_GREEN_BUOY";
  case IR_DISC_DOCK_RED_BUOY_FORCE_FIELD: return "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  case IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD: return "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  case IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD: return "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  case IR_DOCK_RESERVED: return "IR_DOCK_RESERVED";
  case IR_DOCK_RED_BUOY: return "IR_DOCK_RED_BUOY";
  case IR_DOCK_GREEN_BUOY: return "IR_DOCK_GREEN_BUOY";
  case IR_DOCK_FORCE_FIELD: return "IR_DOCK_FORCE_FIELD";
  case IR_DOCK_RED_GREEN_BUOY: return "IR_DOCK_RED_GREEN_BUOY";
  case IR_DOCK_RED_BUOY_FORCE_FIELD: return "IR_DOCK_RED_BUOY_FORCE_FIELD";
  case IR_DOCK_GREEN_BUOY_FORCE_FIELD: return "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  case IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD: return "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  case IR_VIRTUAL_WALL: return "IR_VIRTUAL_WALL";
  default: return "UNKNOWN";
  }
}
/** Convert ChargingState constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
Roomba500Interface::tostring_ChargingState(ChargingState value) const
{
  switch (value) {
  case CHARGING_NO: return "CHARGING_NO";
  case CHARGING_RECONDITIONING: return "CHARGING_RECONDITIONING";
  case CHARGING_FULL: return "CHARGING_FULL";
  case CHARGING_TRICKLE: return "CHARGING_TRICKLE";
  case CHARGING_WAITING: return "CHARGING_WAITING";
  case CHARGING_ERROR: return "CHARGING_ERROR";
  default: return "UNKNOWN";
  }
}
/** Convert BrushState constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
Roomba500Interface::tostring_BrushState(BrushState value) const
{
  switch (value) {
  case BRUSHSTATE_OFF: return "BRUSHSTATE_OFF";
  case BRUSHSTATE_FORWARD: return "BRUSHSTATE_FORWARD";
  case BRUSHSTATE_BACKWARD: return "BRUSHSTATE_BACKWARD";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get mode value.
 * Open Interface mode.
 * @return mode value
 */
Roomba500Interface::Mode
Roomba500Interface::mode() const
{
  return (Roomba500Interface::Mode)data->mode;
}

/** Get maximum length of mode value.
 * @return length of mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_mode() const
{
  return 1;
}

/** Set mode value.
 * Open Interface mode.
 * @param new_mode new mode value
 */
void
Roomba500Interface::set_mode(const Mode new_mode)
{
  data->mode = new_mode;
  data_changed = true;
}

/** Get wheel_drop_left value.
 * Left wheel drop sensor.
 * @return wheel_drop_left value
 */
bool
Roomba500Interface::is_wheel_drop_left() const
{
  return data->wheel_drop_left;
}

/** Get maximum length of wheel_drop_left value.
 * @return length of wheel_drop_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_wheel_drop_left() const
{
  return 1;
}

/** Set wheel_drop_left value.
 * Left wheel drop sensor.
 * @param new_wheel_drop_left new wheel_drop_left value
 */
void
Roomba500Interface::set_wheel_drop_left(const bool new_wheel_drop_left)
{
  data->wheel_drop_left = new_wheel_drop_left;
  data_changed = true;
}

/** Get wheel_drop_right value.
 * Right wheel drop sensor.
 * @return wheel_drop_right value
 */
bool
Roomba500Interface::is_wheel_drop_right() const
{
  return data->wheel_drop_right;
}

/** Get maximum length of wheel_drop_right value.
 * @return length of wheel_drop_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_wheel_drop_right() const
{
  return 1;
}

/** Set wheel_drop_right value.
 * Right wheel drop sensor.
 * @param new_wheel_drop_right new wheel_drop_right value
 */
void
Roomba500Interface::set_wheel_drop_right(const bool new_wheel_drop_right)
{
  data->wheel_drop_right = new_wheel_drop_right;
  data_changed = true;
}

/** Get bump_left value.
 * Bump on left.
 * @return bump_left value
 */
bool
Roomba500Interface::is_bump_left() const
{
  return data->bump_left;
}

/** Get maximum length of bump_left value.
 * @return length of bump_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_bump_left() const
{
  return 1;
}

/** Set bump_left value.
 * Bump on left.
 * @param new_bump_left new bump_left value
 */
void
Roomba500Interface::set_bump_left(const bool new_bump_left)
{
  data->bump_left = new_bump_left;
  data_changed = true;
}

/** Get bump_right value.
 * Bump on right.
 * @return bump_right value
 */
bool
Roomba500Interface::is_bump_right() const
{
  return data->bump_right;
}

/** Get maximum length of bump_right value.
 * @return length of bump_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_bump_right() const
{
  return 1;
}

/** Set bump_right value.
 * Bump on right.
 * @param new_bump_right new bump_right value
 */
void
Roomba500Interface::set_bump_right(const bool new_bump_right)
{
  data->bump_right = new_bump_right;
  data_changed = true;
}

/** Get wall value.
 * Wall sensor.
 * @return wall value
 */
bool
Roomba500Interface::is_wall() const
{
  return data->wall;
}

/** Get maximum length of wall value.
 * @return length of wall value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_wall() const
{
  return 1;
}

/** Set wall value.
 * Wall sensor.
 * @param new_wall new wall value
 */
void
Roomba500Interface::set_wall(const bool new_wall)
{
  data->wall = new_wall;
  data_changed = true;
}

/** Get cliff_left value.
 * Cliff detected left.
 * @return cliff_left value
 */
bool
Roomba500Interface::is_cliff_left() const
{
  return data->cliff_left;
}

/** Get maximum length of cliff_left value.
 * @return length of cliff_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_cliff_left() const
{
  return 1;
}

/** Set cliff_left value.
 * Cliff detected left.
 * @param new_cliff_left new cliff_left value
 */
void
Roomba500Interface::set_cliff_left(const bool new_cliff_left)
{
  data->cliff_left = new_cliff_left;
  data_changed = true;
}

/** Get cliff_front_left value.
 * Cliff detected front left.
 * @return cliff_front_left value
 */
bool
Roomba500Interface::is_cliff_front_left() const
{
  return data->cliff_front_left;
}

/** Get maximum length of cliff_front_left value.
 * @return length of cliff_front_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_cliff_front_left() const
{
  return 1;
}

/** Set cliff_front_left value.
 * Cliff detected front left.
 * @param new_cliff_front_left new cliff_front_left value
 */
void
Roomba500Interface::set_cliff_front_left(const bool new_cliff_front_left)
{
  data->cliff_front_left = new_cliff_front_left;
  data_changed = true;
}

/** Get cliff_front_right value.
 * Cliff detected front right.
 * @return cliff_front_right value
 */
bool
Roomba500Interface::is_cliff_front_right() const
{
  return data->cliff_front_right;
}

/** Get maximum length of cliff_front_right value.
 * @return length of cliff_front_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_cliff_front_right() const
{
  return 1;
}

/** Set cliff_front_right value.
 * Cliff detected front right.
 * @param new_cliff_front_right new cliff_front_right value
 */
void
Roomba500Interface::set_cliff_front_right(const bool new_cliff_front_right)
{
  data->cliff_front_right = new_cliff_front_right;
  data_changed = true;
}

/** Get cliff_right value.
 * Cliff detected right.
 * @return cliff_right value
 */
bool
Roomba500Interface::is_cliff_right() const
{
  return data->cliff_right;
}

/** Get maximum length of cliff_right value.
 * @return length of cliff_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_cliff_right() const
{
  return 1;
}

/** Set cliff_right value.
 * Cliff detected right.
 * @param new_cliff_right new cliff_right value
 */
void
Roomba500Interface::set_cliff_right(const bool new_cliff_right)
{
  data->cliff_right = new_cliff_right;
  data_changed = true;
}

/** Get virtual_wall value.
 * Virtual wall detected.
 * @return virtual_wall value
 */
bool
Roomba500Interface::is_virtual_wall() const
{
  return data->virtual_wall;
}

/** Get maximum length of virtual_wall value.
 * @return length of virtual_wall value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_virtual_wall() const
{
  return 1;
}

/** Set virtual_wall value.
 * Virtual wall detected.
 * @param new_virtual_wall new virtual_wall value
 */
void
Roomba500Interface::set_virtual_wall(const bool new_virtual_wall)
{
  data->virtual_wall = new_virtual_wall;
  data_changed = true;
}

/** Get overcurrent_side_brush value.
 * Overcurrent on side brush.
 * @return overcurrent_side_brush value
 */
bool
Roomba500Interface::is_overcurrent_side_brush() const
{
  return data->overcurrent_side_brush;
}

/** Get maximum length of overcurrent_side_brush value.
 * @return length of overcurrent_side_brush value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_overcurrent_side_brush() const
{
  return 1;
}

/** Set overcurrent_side_brush value.
 * Overcurrent on side brush.
 * @param new_overcurrent_side_brush new overcurrent_side_brush value
 */
void
Roomba500Interface::set_overcurrent_side_brush(const bool new_overcurrent_side_brush)
{
  data->overcurrent_side_brush = new_overcurrent_side_brush;
  data_changed = true;
}

/** Get overcurrent_main_brush value.
 * Overcurrent on main brush.
 * @return overcurrent_main_brush value
 */
bool
Roomba500Interface::is_overcurrent_main_brush() const
{
  return data->overcurrent_main_brush;
}

/** Get maximum length of overcurrent_main_brush value.
 * @return length of overcurrent_main_brush value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_overcurrent_main_brush() const
{
  return 1;
}

/** Set overcurrent_main_brush value.
 * Overcurrent on main brush.
 * @param new_overcurrent_main_brush new overcurrent_main_brush value
 */
void
Roomba500Interface::set_overcurrent_main_brush(const bool new_overcurrent_main_brush)
{
  data->overcurrent_main_brush = new_overcurrent_main_brush;
  data_changed = true;
}

/** Get overcurrent_left_wheel value.
 * Overcurrent on left wheel.
 * @return overcurrent_left_wheel value
 */
bool
Roomba500Interface::is_overcurrent_left_wheel() const
{
  return data->overcurrent_left_wheel;
}

/** Get maximum length of overcurrent_left_wheel value.
 * @return length of overcurrent_left_wheel value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_overcurrent_left_wheel() const
{
  return 1;
}

/** Set overcurrent_left_wheel value.
 * Overcurrent on left wheel.
 * @param new_overcurrent_left_wheel new overcurrent_left_wheel value
 */
void
Roomba500Interface::set_overcurrent_left_wheel(const bool new_overcurrent_left_wheel)
{
  data->overcurrent_left_wheel = new_overcurrent_left_wheel;
  data_changed = true;
}

/** Get overcurrent_right_wheel value.
 * Overcurrent on right wheel.
 * @return overcurrent_right_wheel value
 */
bool
Roomba500Interface::is_overcurrent_right_wheel() const
{
  return data->overcurrent_right_wheel;
}

/** Get maximum length of overcurrent_right_wheel value.
 * @return length of overcurrent_right_wheel value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_overcurrent_right_wheel() const
{
  return 1;
}

/** Set overcurrent_right_wheel value.
 * Overcurrent on right wheel.
 * @param new_overcurrent_right_wheel new overcurrent_right_wheel value
 */
void
Roomba500Interface::set_overcurrent_right_wheel(const bool new_overcurrent_right_wheel)
{
  data->overcurrent_right_wheel = new_overcurrent_right_wheel;
  data_changed = true;
}

/** Get dirt_detect value.
 * Dirt detected?
 * @return dirt_detect value
 */
bool
Roomba500Interface::is_dirt_detect() const
{
  return data->dirt_detect;
}

/** Get maximum length of dirt_detect value.
 * @return length of dirt_detect value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_dirt_detect() const
{
  return 1;
}

/** Set dirt_detect value.
 * Dirt detected?
 * @param new_dirt_detect new dirt_detect value
 */
void
Roomba500Interface::set_dirt_detect(const bool new_dirt_detect)
{
  data->dirt_detect = new_dirt_detect;
  data_changed = true;
}

/** Get ir_opcode_omni value.
 * Omni IR receiver code.
 * @return ir_opcode_omni value
 */
Roomba500Interface::InfraredCharacter
Roomba500Interface::ir_opcode_omni() const
{
  return (Roomba500Interface::InfraredCharacter)data->ir_opcode_omni;
}

/** Get maximum length of ir_opcode_omni value.
 * @return length of ir_opcode_omni value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_ir_opcode_omni() const
{
  return 1;
}

/** Set ir_opcode_omni value.
 * Omni IR receiver code.
 * @param new_ir_opcode_omni new ir_opcode_omni value
 */
void
Roomba500Interface::set_ir_opcode_omni(const InfraredCharacter new_ir_opcode_omni)
{
  data->ir_opcode_omni = new_ir_opcode_omni;
  data_changed = true;
}

/** Get button_clean value.
 * Clean button pressed.
 * @return button_clean value
 */
bool
Roomba500Interface::is_button_clean() const
{
  return data->button_clean;
}

/** Get maximum length of button_clean value.
 * @return length of button_clean value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_button_clean() const
{
  return 1;
}

/** Set button_clean value.
 * Clean button pressed.
 * @param new_button_clean new button_clean value
 */
void
Roomba500Interface::set_button_clean(const bool new_button_clean)
{
  data->button_clean = new_button_clean;
  data_changed = true;
}

/** Get button_spot value.
 * Spot button pressed.
 * @return button_spot value
 */
bool
Roomba500Interface::is_button_spot() const
{
  return data->button_spot;
}

/** Get maximum length of button_spot value.
 * @return length of button_spot value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_button_spot() const
{
  return 1;
}

/** Set button_spot value.
 * Spot button pressed.
 * @param new_button_spot new button_spot value
 */
void
Roomba500Interface::set_button_spot(const bool new_button_spot)
{
  data->button_spot = new_button_spot;
  data_changed = true;
}

/** Get button_dock value.
 * Dock button pressed.
 * @return button_dock value
 */
bool
Roomba500Interface::is_button_dock() const
{
  return data->button_dock;
}

/** Get maximum length of button_dock value.
 * @return length of button_dock value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_button_dock() const
{
  return 1;
}

/** Set button_dock value.
 * Dock button pressed.
 * @param new_button_dock new button_dock value
 */
void
Roomba500Interface::set_button_dock(const bool new_button_dock)
{
  data->button_dock = new_button_dock;
  data_changed = true;
}

/** Get button_minute value.
 * Minute button pressed.
 * @return button_minute value
 */
bool
Roomba500Interface::is_button_minute() const
{
  return data->button_minute;
}

/** Get maximum length of button_minute value.
 * @return length of button_minute value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_button_minute() const
{
  return 1;
}

/** Set button_minute value.
 * Minute button pressed.
 * @param new_button_minute new button_minute value
 */
void
Roomba500Interface::set_button_minute(const bool new_button_minute)
{
  data->button_minute = new_button_minute;
  data_changed = true;
}

/** Get button_hour value.
 * Hour button pressed.
 * @return button_hour value
 */
bool
Roomba500Interface::is_button_hour() const
{
  return data->button_hour;
}

/** Get maximum length of button_hour value.
 * @return length of button_hour value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_button_hour() const
{
  return 1;
}

/** Set button_hour value.
 * Hour button pressed.
 * @param new_button_hour new button_hour value
 */
void
Roomba500Interface::set_button_hour(const bool new_button_hour)
{
  data->button_hour = new_button_hour;
  data_changed = true;
}

/** Get button_day value.
 * Day button pressed.
 * @return button_day value
 */
bool
Roomba500Interface::is_button_day() const
{
  return data->button_day;
}

/** Get maximum length of button_day value.
 * @return length of button_day value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_button_day() const
{
  return 1;
}

/** Set button_day value.
 * Day button pressed.
 * @param new_button_day new button_day value
 */
void
Roomba500Interface::set_button_day(const bool new_button_day)
{
  data->button_day = new_button_day;
  data_changed = true;
}

/** Get button_schedule value.
 * Schedule button pressed.
 * @return button_schedule value
 */
bool
Roomba500Interface::is_button_schedule() const
{
  return data->button_schedule;
}

/** Get maximum length of button_schedule value.
 * @return length of button_schedule value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_button_schedule() const
{
  return 1;
}

/** Set button_schedule value.
 * Schedule button pressed.
 * @param new_button_schedule new button_schedule value
 */
void
Roomba500Interface::set_button_schedule(const bool new_button_schedule)
{
  data->button_schedule = new_button_schedule;
  data_changed = true;
}

/** Get button_clock value.
 * Clock button pressed.
 * @return button_clock value
 */
bool
Roomba500Interface::is_button_clock() const
{
  return data->button_clock;
}

/** Get maximum length of button_clock value.
 * @return length of button_clock value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_button_clock() const
{
  return 1;
}

/** Set button_clock value.
 * Clock button pressed.
 * @param new_button_clock new button_clock value
 */
void
Roomba500Interface::set_button_clock(const bool new_button_clock)
{
  data->button_clock = new_button_clock;
  data_changed = true;
}

/** Get distance value.
 * Travelled distance in m.
 * @return distance value
 */
int16_t
Roomba500Interface::distance() const
{
  return data->distance;
}

/** Get maximum length of distance value.
 * @return length of distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_distance() const
{
  return 1;
}

/** Set distance value.
 * Travelled distance in m.
 * @param new_distance new distance value
 */
void
Roomba500Interface::set_distance(const int16_t new_distance)
{
  data->distance = new_distance;
  data_changed = true;
}

/** Get angle value.
 * Turned angle in radians.
 * @return angle value
 */
int16_t
Roomba500Interface::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Turned angle in radians.
 * @param new_angle new angle value
 */
void
Roomba500Interface::set_angle(const int16_t new_angle)
{
  data->angle = new_angle;
  data_changed = true;
}

/** Get charging_state value.
 * Charging state.
 * @return charging_state value
 */
Roomba500Interface::ChargingState
Roomba500Interface::charging_state() const
{
  return (Roomba500Interface::ChargingState)data->charging_state;
}

/** Get maximum length of charging_state value.
 * @return length of charging_state value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_charging_state() const
{
  return 1;
}

/** Set charging_state value.
 * Charging state.
 * @param new_charging_state new charging_state value
 */
void
Roomba500Interface::set_charging_state(const ChargingState new_charging_state)
{
  data->charging_state = new_charging_state;
  data_changed = true;
}

/** Get voltage value.
 * Voltage in mV.
 * @return voltage value
 */
uint16_t
Roomba500Interface::voltage() const
{
  return data->voltage;
}

/** Get maximum length of voltage value.
 * @return length of voltage value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_voltage() const
{
  return 1;
}

/** Set voltage value.
 * Voltage in mV.
 * @param new_voltage new voltage value
 */
void
Roomba500Interface::set_voltage(const uint16_t new_voltage)
{
  data->voltage = new_voltage;
  data_changed = true;
}

/** Get current value.
 * Current in mA.
 * @return current value
 */
int16_t
Roomba500Interface::current() const
{
  return data->current;
}

/** Get maximum length of current value.
 * @return length of current value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_current() const
{
  return 1;
}

/** Set current value.
 * Current in mA.
 * @param new_current new current value
 */
void
Roomba500Interface::set_current(const int16_t new_current)
{
  data->current = new_current;
  data_changed = true;
}

/** Get temperature value.
 * Temperature in degree Celsius.
 * @return temperature value
 */
int8_t
Roomba500Interface::temperature() const
{
  return data->temperature;
}

/** Get maximum length of temperature value.
 * @return length of temperature value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_temperature() const
{
  return 1;
}

/** Set temperature value.
 * Temperature in degree Celsius.
 * @param new_temperature new temperature value
 */
void
Roomba500Interface::set_temperature(const int8_t new_temperature)
{
  data->temperature = new_temperature;
  data_changed = true;
}

/** Get battery_charge value.
 * Battery charge in mAh.
 * @return battery_charge value
 */
uint16_t
Roomba500Interface::battery_charge() const
{
  return data->battery_charge;
}

/** Get maximum length of battery_charge value.
 * @return length of battery_charge value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_battery_charge() const
{
  return 1;
}

/** Set battery_charge value.
 * Battery charge in mAh.
 * @param new_battery_charge new battery_charge value
 */
void
Roomba500Interface::set_battery_charge(const uint16_t new_battery_charge)
{
  data->battery_charge = new_battery_charge;
  data_changed = true;
}

/** Get battery_capacity value.
 * Battery capacity in mAh.
 * @return battery_capacity value
 */
uint16_t
Roomba500Interface::battery_capacity() const
{
  return data->battery_capacity;
}

/** Get maximum length of battery_capacity value.
 * @return length of battery_capacity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_battery_capacity() const
{
  return 1;
}

/** Set battery_capacity value.
 * Battery capacity in mAh.
 * @param new_battery_capacity new battery_capacity value
 */
void
Roomba500Interface::set_battery_capacity(const uint16_t new_battery_capacity)
{
  data->battery_capacity = new_battery_capacity;
  data_changed = true;
}

/** Get wall_signal value.
 * Raw wall signal
 * @return wall_signal value
 */
uint16_t
Roomba500Interface::wall_signal() const
{
  return data->wall_signal;
}

/** Get maximum length of wall_signal value.
 * @return length of wall_signal value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_wall_signal() const
{
  return 1;
}

/** Set wall_signal value.
 * Raw wall signal
 * @param new_wall_signal new wall_signal value
 */
void
Roomba500Interface::set_wall_signal(const uint16_t new_wall_signal)
{
  data->wall_signal = new_wall_signal;
  data_changed = true;
}

/** Get cliff_left_signal value.
 * Raw left cliff signal.
 * @return cliff_left_signal value
 */
uint16_t
Roomba500Interface::cliff_left_signal() const
{
  return data->cliff_left_signal;
}

/** Get maximum length of cliff_left_signal value.
 * @return length of cliff_left_signal value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_cliff_left_signal() const
{
  return 1;
}

/** Set cliff_left_signal value.
 * Raw left cliff signal.
 * @param new_cliff_left_signal new cliff_left_signal value
 */
void
Roomba500Interface::set_cliff_left_signal(const uint16_t new_cliff_left_signal)
{
  data->cliff_left_signal = new_cliff_left_signal;
  data_changed = true;
}

/** Get cliff_front_left_signal value.
 * Raw front left
      cliff signal.
 * @return cliff_front_left_signal value
 */
uint16_t
Roomba500Interface::cliff_front_left_signal() const
{
  return data->cliff_front_left_signal;
}

/** Get maximum length of cliff_front_left_signal value.
 * @return length of cliff_front_left_signal value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_cliff_front_left_signal() const
{
  return 1;
}

/** Set cliff_front_left_signal value.
 * Raw front left
      cliff signal.
 * @param new_cliff_front_left_signal new cliff_front_left_signal value
 */
void
Roomba500Interface::set_cliff_front_left_signal(const uint16_t new_cliff_front_left_signal)
{
  data->cliff_front_left_signal = new_cliff_front_left_signal;
  data_changed = true;
}

/** Get cliff_front_right_signal value.
 * Raw front right
      cliff signal.
 * @return cliff_front_right_signal value
 */
uint16_t
Roomba500Interface::cliff_front_right_signal() const
{
  return data->cliff_front_right_signal;
}

/** Get maximum length of cliff_front_right_signal value.
 * @return length of cliff_front_right_signal value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_cliff_front_right_signal() const
{
  return 1;
}

/** Set cliff_front_right_signal value.
 * Raw front right
      cliff signal.
 * @param new_cliff_front_right_signal new cliff_front_right_signal value
 */
void
Roomba500Interface::set_cliff_front_right_signal(const uint16_t new_cliff_front_right_signal)
{
  data->cliff_front_right_signal = new_cliff_front_right_signal;
  data_changed = true;
}

/** Get cliff_right_signal value.
 * Raw right cliff signal.
 * @return cliff_right_signal value
 */
uint16_t
Roomba500Interface::cliff_right_signal() const
{
  return data->cliff_right_signal;
}

/** Get maximum length of cliff_right_signal value.
 * @return length of cliff_right_signal value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_cliff_right_signal() const
{
  return 1;
}

/** Set cliff_right_signal value.
 * Raw right cliff signal.
 * @param new_cliff_right_signal new cliff_right_signal value
 */
void
Roomba500Interface::set_cliff_right_signal(const uint16_t new_cliff_right_signal)
{
  data->cliff_right_signal = new_cliff_right_signal;
  data_changed = true;
}

/** Get home_base_charger_available value.
 * 
      Home base charger available?
 * @return home_base_charger_available value
 */
bool
Roomba500Interface::is_home_base_charger_available() const
{
  return data->home_base_charger_available;
}

/** Get maximum length of home_base_charger_available value.
 * @return length of home_base_charger_available value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_home_base_charger_available() const
{
  return 1;
}

/** Set home_base_charger_available value.
 * 
      Home base charger available?
 * @param new_home_base_charger_available new home_base_charger_available value
 */
void
Roomba500Interface::set_home_base_charger_available(const bool new_home_base_charger_available)
{
  data->home_base_charger_available = new_home_base_charger_available;
  data_changed = true;
}

/** Get internal_charger_available value.
 * 
      Internal charger available?
 * @return internal_charger_available value
 */
bool
Roomba500Interface::is_internal_charger_available() const
{
  return data->internal_charger_available;
}

/** Get maximum length of internal_charger_available value.
 * @return length of internal_charger_available value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_internal_charger_available() const
{
  return 1;
}

/** Set internal_charger_available value.
 * 
      Internal charger available?
 * @param new_internal_charger_available new internal_charger_available value
 */
void
Roomba500Interface::set_internal_charger_available(const bool new_internal_charger_available)
{
  data->internal_charger_available = new_internal_charger_available;
  data_changed = true;
}

/** Get song_number value.
 * Song number.
 * @return song_number value
 */
uint8_t
Roomba500Interface::song_number() const
{
  return data->song_number;
}

/** Get maximum length of song_number value.
 * @return length of song_number value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_song_number() const
{
  return 1;
}

/** Set song_number value.
 * Song number.
 * @param new_song_number new song_number value
 */
void
Roomba500Interface::set_song_number(const uint8_t new_song_number)
{
  data->song_number = new_song_number;
  data_changed = true;
}

/** Get song_playing value.
 * Song playing?
 * @return song_playing value
 */
bool
Roomba500Interface::is_song_playing() const
{
  return data->song_playing;
}

/** Get maximum length of song_playing value.
 * @return length of song_playing value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_song_playing() const
{
  return 1;
}

/** Set song_playing value.
 * Song playing?
 * @param new_song_playing new song_playing value
 */
void
Roomba500Interface::set_song_playing(const bool new_song_playing)
{
  data->song_playing = new_song_playing;
  data_changed = true;
}

/** Get velocity value.
 * Requested velocity in mm/s.
 * @return velocity value
 */
int16_t
Roomba500Interface::velocity() const
{
  return data->velocity;
}

/** Get maximum length of velocity value.
 * @return length of velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_velocity() const
{
  return 1;
}

/** Set velocity value.
 * Requested velocity in mm/s.
 * @param new_velocity new velocity value
 */
void
Roomba500Interface::set_velocity(const int16_t new_velocity)
{
  data->velocity = new_velocity;
  data_changed = true;
}

/** Get radius value.
 * Requested radius in mm.
 * @return radius value
 */
int16_t
Roomba500Interface::radius() const
{
  return data->radius;
}

/** Get maximum length of radius value.
 * @return length of radius value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_radius() const
{
  return 1;
}

/** Set radius value.
 * Requested radius in mm.
 * @param new_radius new radius value
 */
void
Roomba500Interface::set_radius(const int16_t new_radius)
{
  data->radius = new_radius;
  data_changed = true;
}

/** Get velocity_right value.
 * Requested left velocity in mm/s.
 * @return velocity_right value
 */
int16_t
Roomba500Interface::velocity_right() const
{
  return data->velocity_right;
}

/** Get maximum length of velocity_right value.
 * @return length of velocity_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_velocity_right() const
{
  return 1;
}

/** Set velocity_right value.
 * Requested left velocity in mm/s.
 * @param new_velocity_right new velocity_right value
 */
void
Roomba500Interface::set_velocity_right(const int16_t new_velocity_right)
{
  data->velocity_right = new_velocity_right;
  data_changed = true;
}

/** Get velocity_left value.
 * Requested right velocity in mm/s.
 * @return velocity_left value
 */
int16_t
Roomba500Interface::velocity_left() const
{
  return data->velocity_left;
}

/** Get maximum length of velocity_left value.
 * @return length of velocity_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_velocity_left() const
{
  return 1;
}

/** Set velocity_left value.
 * Requested right velocity in mm/s.
 * @param new_velocity_left new velocity_left value
 */
void
Roomba500Interface::set_velocity_left(const int16_t new_velocity_left)
{
  data->velocity_left = new_velocity_left;
  data_changed = true;
}

/** Get encoder_counts_left value.
 * Encoder count left.
 * @return encoder_counts_left value
 */
uint16_t
Roomba500Interface::encoder_counts_left() const
{
  return data->encoder_counts_left;
}

/** Get maximum length of encoder_counts_left value.
 * @return length of encoder_counts_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_encoder_counts_left() const
{
  return 1;
}

/** Set encoder_counts_left value.
 * Encoder count left.
 * @param new_encoder_counts_left new encoder_counts_left value
 */
void
Roomba500Interface::set_encoder_counts_left(const uint16_t new_encoder_counts_left)
{
  data->encoder_counts_left = new_encoder_counts_left;
  data_changed = true;
}

/** Get encoder_counts_right value.
 * Encoder count right.
 * @return encoder_counts_right value
 */
uint16_t
Roomba500Interface::encoder_counts_right() const
{
  return data->encoder_counts_right;
}

/** Get maximum length of encoder_counts_right value.
 * @return length of encoder_counts_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_encoder_counts_right() const
{
  return 1;
}

/** Set encoder_counts_right value.
 * Encoder count right.
 * @param new_encoder_counts_right new encoder_counts_right value
 */
void
Roomba500Interface::set_encoder_counts_right(const uint16_t new_encoder_counts_right)
{
  data->encoder_counts_right = new_encoder_counts_right;
  data_changed = true;
}

/** Get bumper_left value.
 * Left bumper active?
 * @return bumper_left value
 */
bool
Roomba500Interface::is_bumper_left() const
{
  return data->bumper_left;
}

/** Get maximum length of bumper_left value.
 * @return length of bumper_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_bumper_left() const
{
  return 1;
}

/** Set bumper_left value.
 * Left bumper active?
 * @param new_bumper_left new bumper_left value
 */
void
Roomba500Interface::set_bumper_left(const bool new_bumper_left)
{
  data->bumper_left = new_bumper_left;
  data_changed = true;
}

/** Get bumper_front_left value.
 * Front left bumper active?
 * @return bumper_front_left value
 */
bool
Roomba500Interface::is_bumper_front_left() const
{
  return data->bumper_front_left;
}

/** Get maximum length of bumper_front_left value.
 * @return length of bumper_front_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_bumper_front_left() const
{
  return 1;
}

/** Set bumper_front_left value.
 * Front left bumper active?
 * @param new_bumper_front_left new bumper_front_left value
 */
void
Roomba500Interface::set_bumper_front_left(const bool new_bumper_front_left)
{
  data->bumper_front_left = new_bumper_front_left;
  data_changed = true;
}

/** Get bumper_center_left value.
 * Center left bumper active?
 * @return bumper_center_left value
 */
bool
Roomba500Interface::is_bumper_center_left() const
{
  return data->bumper_center_left;
}

/** Get maximum length of bumper_center_left value.
 * @return length of bumper_center_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_bumper_center_left() const
{
  return 1;
}

/** Set bumper_center_left value.
 * Center left bumper active?
 * @param new_bumper_center_left new bumper_center_left value
 */
void
Roomba500Interface::set_bumper_center_left(const bool new_bumper_center_left)
{
  data->bumper_center_left = new_bumper_center_left;
  data_changed = true;
}

/** Get bumper_center_right value.
 * Center right bumper active?
 * @return bumper_center_right value
 */
bool
Roomba500Interface::is_bumper_center_right() const
{
  return data->bumper_center_right;
}

/** Get maximum length of bumper_center_right value.
 * @return length of bumper_center_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_bumper_center_right() const
{
  return 1;
}

/** Set bumper_center_right value.
 * Center right bumper active?
 * @param new_bumper_center_right new bumper_center_right value
 */
void
Roomba500Interface::set_bumper_center_right(const bool new_bumper_center_right)
{
  data->bumper_center_right = new_bumper_center_right;
  data_changed = true;
}

/** Get bumper_front_right value.
 * Front right bumper active?
 * @return bumper_front_right value
 */
bool
Roomba500Interface::is_bumper_front_right() const
{
  return data->bumper_front_right;
}

/** Get maximum length of bumper_front_right value.
 * @return length of bumper_front_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_bumper_front_right() const
{
  return 1;
}

/** Set bumper_front_right value.
 * Front right bumper active?
 * @param new_bumper_front_right new bumper_front_right value
 */
void
Roomba500Interface::set_bumper_front_right(const bool new_bumper_front_right)
{
  data->bumper_front_right = new_bumper_front_right;
  data_changed = true;
}

/** Get bumper_right value.
 * Right bumper active?
 * @return bumper_right value
 */
bool
Roomba500Interface::is_bumper_right() const
{
  return data->bumper_right;
}

/** Get maximum length of bumper_right value.
 * @return length of bumper_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_bumper_right() const
{
  return 1;
}

/** Set bumper_right value.
 * Right bumper active?
 * @param new_bumper_right new bumper_right value
 */
void
Roomba500Interface::set_bumper_right(const bool new_bumper_right)
{
  data->bumper_right = new_bumper_right;
  data_changed = true;
}

/** Get light_bump_left value.
 * Raw left bumper signal.
 * @return light_bump_left value
 */
uint16_t
Roomba500Interface::light_bump_left() const
{
  return data->light_bump_left;
}

/** Get maximum length of light_bump_left value.
 * @return length of light_bump_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_light_bump_left() const
{
  return 1;
}

/** Set light_bump_left value.
 * Raw left bumper signal.
 * @param new_light_bump_left new light_bump_left value
 */
void
Roomba500Interface::set_light_bump_left(const uint16_t new_light_bump_left)
{
  data->light_bump_left = new_light_bump_left;
  data_changed = true;
}

/** Get light_bump_front_left value.
 * Raw front left bumper
      signal.
 * @return light_bump_front_left value
 */
uint16_t
Roomba500Interface::light_bump_front_left() const
{
  return data->light_bump_front_left;
}

/** Get maximum length of light_bump_front_left value.
 * @return length of light_bump_front_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_light_bump_front_left() const
{
  return 1;
}

/** Set light_bump_front_left value.
 * Raw front left bumper
      signal.
 * @param new_light_bump_front_left new light_bump_front_left value
 */
void
Roomba500Interface::set_light_bump_front_left(const uint16_t new_light_bump_front_left)
{
  data->light_bump_front_left = new_light_bump_front_left;
  data_changed = true;
}

/** Get light_bump_center_left value.
 * Raw center left
      bumper signal.
 * @return light_bump_center_left value
 */
uint16_t
Roomba500Interface::light_bump_center_left() const
{
  return data->light_bump_center_left;
}

/** Get maximum length of light_bump_center_left value.
 * @return length of light_bump_center_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_light_bump_center_left() const
{
  return 1;
}

/** Set light_bump_center_left value.
 * Raw center left
      bumper signal.
 * @param new_light_bump_center_left new light_bump_center_left value
 */
void
Roomba500Interface::set_light_bump_center_left(const uint16_t new_light_bump_center_left)
{
  data->light_bump_center_left = new_light_bump_center_left;
  data_changed = true;
}

/** Get light_bump_center_right value.
 * Raw center right
      bumper signal.
 * @return light_bump_center_right value
 */
uint16_t
Roomba500Interface::light_bump_center_right() const
{
  return data->light_bump_center_right;
}

/** Get maximum length of light_bump_center_right value.
 * @return length of light_bump_center_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_light_bump_center_right() const
{
  return 1;
}

/** Set light_bump_center_right value.
 * Raw center right
      bumper signal.
 * @param new_light_bump_center_right new light_bump_center_right value
 */
void
Roomba500Interface::set_light_bump_center_right(const uint16_t new_light_bump_center_right)
{
  data->light_bump_center_right = new_light_bump_center_right;
  data_changed = true;
}

/** Get light_bump_front_right value.
 * Raw front right
      bumper signal.
 * @return light_bump_front_right value
 */
uint16_t
Roomba500Interface::light_bump_front_right() const
{
  return data->light_bump_front_right;
}

/** Get maximum length of light_bump_front_right value.
 * @return length of light_bump_front_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_light_bump_front_right() const
{
  return 1;
}

/** Set light_bump_front_right value.
 * Raw front right
      bumper signal.
 * @param new_light_bump_front_right new light_bump_front_right value
 */
void
Roomba500Interface::set_light_bump_front_right(const uint16_t new_light_bump_front_right)
{
  data->light_bump_front_right = new_light_bump_front_right;
  data_changed = true;
}

/** Get light_bump_right value.
 * Raw right bumper signal.
 * @return light_bump_right value
 */
uint16_t
Roomba500Interface::light_bump_right() const
{
  return data->light_bump_right;
}

/** Get maximum length of light_bump_right value.
 * @return length of light_bump_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_light_bump_right() const
{
  return 1;
}

/** Set light_bump_right value.
 * Raw right bumper signal.
 * @param new_light_bump_right new light_bump_right value
 */
void
Roomba500Interface::set_light_bump_right(const uint16_t new_light_bump_right)
{
  data->light_bump_right = new_light_bump_right;
  data_changed = true;
}

/** Get ir_opcode_left value.
 * 
      Left receiver opcode.
 * @return ir_opcode_left value
 */
Roomba500Interface::InfraredCharacter
Roomba500Interface::ir_opcode_left() const
{
  return (Roomba500Interface::InfraredCharacter)data->ir_opcode_left;
}

/** Get maximum length of ir_opcode_left value.
 * @return length of ir_opcode_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_ir_opcode_left() const
{
  return 1;
}

/** Set ir_opcode_left value.
 * 
      Left receiver opcode.
 * @param new_ir_opcode_left new ir_opcode_left value
 */
void
Roomba500Interface::set_ir_opcode_left(const InfraredCharacter new_ir_opcode_left)
{
  data->ir_opcode_left = new_ir_opcode_left;
  data_changed = true;
}

/** Get ir_opcode_right value.
 * 
      Right receiver opcode.
 * @return ir_opcode_right value
 */
Roomba500Interface::InfraredCharacter
Roomba500Interface::ir_opcode_right() const
{
  return (Roomba500Interface::InfraredCharacter)data->ir_opcode_right;
}

/** Get maximum length of ir_opcode_right value.
 * @return length of ir_opcode_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_ir_opcode_right() const
{
  return 1;
}

/** Set ir_opcode_right value.
 * 
      Right receiver opcode.
 * @param new_ir_opcode_right new ir_opcode_right value
 */
void
Roomba500Interface::set_ir_opcode_right(const InfraredCharacter new_ir_opcode_right)
{
  data->ir_opcode_right = new_ir_opcode_right;
  data_changed = true;
}

/** Get left_motor_current value.
 * Left motor current in mA.
 * @return left_motor_current value
 */
int16_t
Roomba500Interface::left_motor_current() const
{
  return data->left_motor_current;
}

/** Get maximum length of left_motor_current value.
 * @return length of left_motor_current value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_left_motor_current() const
{
  return 1;
}

/** Set left_motor_current value.
 * Left motor current in mA.
 * @param new_left_motor_current new left_motor_current value
 */
void
Roomba500Interface::set_left_motor_current(const int16_t new_left_motor_current)
{
  data->left_motor_current = new_left_motor_current;
  data_changed = true;
}

/** Get right_motor_current value.
 * Right motor current in mA.
 * @return right_motor_current value
 */
int16_t
Roomba500Interface::right_motor_current() const
{
  return data->right_motor_current;
}

/** Get maximum length of right_motor_current value.
 * @return length of right_motor_current value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_right_motor_current() const
{
  return 1;
}

/** Set right_motor_current value.
 * Right motor current in mA.
 * @param new_right_motor_current new right_motor_current value
 */
void
Roomba500Interface::set_right_motor_current(const int16_t new_right_motor_current)
{
  data->right_motor_current = new_right_motor_current;
  data_changed = true;
}

/** Get main_brush_current value.
 * Main brush current in mA.
 * @return main_brush_current value
 */
int16_t
Roomba500Interface::main_brush_current() const
{
  return data->main_brush_current;
}

/** Get maximum length of main_brush_current value.
 * @return length of main_brush_current value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_main_brush_current() const
{
  return 1;
}

/** Set main_brush_current value.
 * Main brush current in mA.
 * @param new_main_brush_current new main_brush_current value
 */
void
Roomba500Interface::set_main_brush_current(const int16_t new_main_brush_current)
{
  data->main_brush_current = new_main_brush_current;
  data_changed = true;
}

/** Get side_brush_current value.
 * Side brush current in mA.
 * @return side_brush_current value
 */
int16_t
Roomba500Interface::side_brush_current() const
{
  return data->side_brush_current;
}

/** Get maximum length of side_brush_current value.
 * @return length of side_brush_current value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_side_brush_current() const
{
  return 1;
}

/** Set side_brush_current value.
 * Side brush current in mA.
 * @param new_side_brush_current new side_brush_current value
 */
void
Roomba500Interface::set_side_brush_current(const int16_t new_side_brush_current)
{
  data->side_brush_current = new_side_brush_current;
  data_changed = true;
}

/** Get caster_stasis value.
 * Caster wheel stasis.
 * @return caster_stasis value
 */
bool
Roomba500Interface::is_caster_stasis() const
{
  return data->caster_stasis;
}

/** Get maximum length of caster_stasis value.
 * @return length of caster_stasis value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::maxlenof_caster_stasis() const
{
  return 1;
}

/** Set caster_stasis value.
 * Caster wheel stasis.
 * @param new_caster_stasis new caster_stasis value
 */
void
Roomba500Interface::set_caster_stasis(const bool new_caster_stasis)
{
  data->caster_stasis = new_caster_stasis;
  data_changed = true;
}

/* =========== message create =========== */
Message *
Roomba500Interface::create_message(const char *type) const
{
  if ( strncmp("StopMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopMessage();
  } else if ( strncmp("DockMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DockMessage();
  } else if ( strncmp("SetModeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetModeMessage();
  } else if ( strncmp("DriveStraightMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DriveStraightMessage();
  } else if ( strncmp("DriveMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DriveMessage();
  } else if ( strncmp("SetMotorsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMotorsMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
Roomba500Interface::copy_values(const Interface *other)
{
  const Roomba500Interface *oi = dynamic_cast<const Roomba500Interface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(Roomba500Interface_data_t));
}

const char *
Roomba500Interface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "Mode") == 0) {
    return tostring_Mode((Mode)val);
  }
  if (strcmp(enumtype, "InfraredCharacter") == 0) {
    return tostring_InfraredCharacter((InfraredCharacter)val);
  }
  if (strcmp(enumtype, "ChargingState") == 0) {
    return tostring_ChargingState((ChargingState)val);
  }
  if (strcmp(enumtype, "BrushState") == 0) {
    return tostring_BrushState((BrushState)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class Roomba500Interface::StopMessage <interfaces/Roomba500Interface.h>
 * StopMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
Roomba500Interface::StopMessage::StopMessage() : Message("StopMessage")
{
  data_size = sizeof(StopMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Mode[(int)MODE_OFF] = "MODE_OFF";
  enum_map_Mode[(int)MODE_PASSIVE] = "MODE_PASSIVE";
  enum_map_Mode[(int)MODE_SAFE] = "MODE_SAFE";
  enum_map_Mode[(int)MODE_FULL] = "MODE_FULL";
  enum_map_InfraredCharacter[(int)IR_NONE] = "IR_NONE";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LEFT] = "IR_REMOTE_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_FORWARD] = "IR_REMOTE_FORWARD";
  enum_map_InfraredCharacter[(int)IR_REMOTE_RIGHT] = "IR_REMOTE_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SPOT] = "IR_REMOTE_SPOT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MAX] = "IR_REMOTE_MAX";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SMALL] = "IR_REMOTE_SMALL";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MEDIUM] = "IR_REMOTE_MEDIUM";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LARGE_CLEAN] = "IR_REMOTE_LARGE_CLEAN";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP] = "IR_REMOTE_STOP";
  enum_map_InfraredCharacter[(int)IR_REMOTE_POWER] = "IR_REMOTE_POWER";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_LEFT] = "IR_REMOTE_ARC_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_RIGHT] = "IR_REMOTE_ARC_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP2] = "IR_REMOTE_STOP2";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_DOWNLOAD] = "IR_SCHED_REMOTE_DOWNLOAD";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_SEEK_DOCK] = "IR_SCHED_REMOTE_SEEK_DOCK";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RESERVED] = "IR_DISC_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY] = "IR_DISC_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY] = "IR_DISC_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_FORCE_FIELD] = "IR_DISC_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY] = "IR_DISC_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RESERVED] = "IR_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY] = "IR_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY] = "IR_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_FORCE_FIELD] = "IR_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY] = "IR_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_VIRTUAL_WALL] = "IR_VIRTUAL_WALL";
  enum_map_ChargingState[(int)CHARGING_NO] = "CHARGING_NO";
  enum_map_ChargingState[(int)CHARGING_RECONDITIONING] = "CHARGING_RECONDITIONING";
  enum_map_ChargingState[(int)CHARGING_FULL] = "CHARGING_FULL";
  enum_map_ChargingState[(int)CHARGING_TRICKLE] = "CHARGING_TRICKLE";
  enum_map_ChargingState[(int)CHARGING_WAITING] = "CHARGING_WAITING";
  enum_map_ChargingState[(int)CHARGING_ERROR] = "CHARGING_ERROR";
  enum_map_BrushState[(int)BRUSHSTATE_OFF] = "BRUSHSTATE_OFF";
  enum_map_BrushState[(int)BRUSHSTATE_FORWARD] = "BRUSHSTATE_FORWARD";
  enum_map_BrushState[(int)BRUSHSTATE_BACKWARD] = "BRUSHSTATE_BACKWARD";
}

/** Destructor */
Roomba500Interface::StopMessage::~StopMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
Roomba500Interface::StopMessage::StopMessage(const StopMessage *m) : Message("StopMessage")
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
Roomba500Interface::StopMessage::clone() const
{
  return new Roomba500Interface::StopMessage(this);
}
/** @class Roomba500Interface::DockMessage <interfaces/Roomba500Interface.h>
 * DockMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
Roomba500Interface::DockMessage::DockMessage() : Message("DockMessage")
{
  data_size = sizeof(DockMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DockMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Mode[(int)MODE_OFF] = "MODE_OFF";
  enum_map_Mode[(int)MODE_PASSIVE] = "MODE_PASSIVE";
  enum_map_Mode[(int)MODE_SAFE] = "MODE_SAFE";
  enum_map_Mode[(int)MODE_FULL] = "MODE_FULL";
  enum_map_InfraredCharacter[(int)IR_NONE] = "IR_NONE";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LEFT] = "IR_REMOTE_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_FORWARD] = "IR_REMOTE_FORWARD";
  enum_map_InfraredCharacter[(int)IR_REMOTE_RIGHT] = "IR_REMOTE_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SPOT] = "IR_REMOTE_SPOT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MAX] = "IR_REMOTE_MAX";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SMALL] = "IR_REMOTE_SMALL";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MEDIUM] = "IR_REMOTE_MEDIUM";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LARGE_CLEAN] = "IR_REMOTE_LARGE_CLEAN";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP] = "IR_REMOTE_STOP";
  enum_map_InfraredCharacter[(int)IR_REMOTE_POWER] = "IR_REMOTE_POWER";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_LEFT] = "IR_REMOTE_ARC_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_RIGHT] = "IR_REMOTE_ARC_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP2] = "IR_REMOTE_STOP2";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_DOWNLOAD] = "IR_SCHED_REMOTE_DOWNLOAD";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_SEEK_DOCK] = "IR_SCHED_REMOTE_SEEK_DOCK";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RESERVED] = "IR_DISC_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY] = "IR_DISC_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY] = "IR_DISC_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_FORCE_FIELD] = "IR_DISC_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY] = "IR_DISC_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RESERVED] = "IR_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY] = "IR_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY] = "IR_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_FORCE_FIELD] = "IR_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY] = "IR_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_VIRTUAL_WALL] = "IR_VIRTUAL_WALL";
  enum_map_ChargingState[(int)CHARGING_NO] = "CHARGING_NO";
  enum_map_ChargingState[(int)CHARGING_RECONDITIONING] = "CHARGING_RECONDITIONING";
  enum_map_ChargingState[(int)CHARGING_FULL] = "CHARGING_FULL";
  enum_map_ChargingState[(int)CHARGING_TRICKLE] = "CHARGING_TRICKLE";
  enum_map_ChargingState[(int)CHARGING_WAITING] = "CHARGING_WAITING";
  enum_map_ChargingState[(int)CHARGING_ERROR] = "CHARGING_ERROR";
  enum_map_BrushState[(int)BRUSHSTATE_OFF] = "BRUSHSTATE_OFF";
  enum_map_BrushState[(int)BRUSHSTATE_FORWARD] = "BRUSHSTATE_FORWARD";
  enum_map_BrushState[(int)BRUSHSTATE_BACKWARD] = "BRUSHSTATE_BACKWARD";
}

/** Destructor */
Roomba500Interface::DockMessage::~DockMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
Roomba500Interface::DockMessage::DockMessage(const DockMessage *m) : Message("DockMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (DockMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
Roomba500Interface::DockMessage::clone() const
{
  return new Roomba500Interface::DockMessage(this);
}
/** @class Roomba500Interface::SetModeMessage <interfaces/Roomba500Interface.h>
 * SetModeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_mode initial value for mode
 */
Roomba500Interface::SetModeMessage::SetModeMessage(const Mode ini_mode) : Message("SetModeMessage")
{
  data_size = sizeof(SetModeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->mode = ini_mode;
  enum_map_Mode[(int)MODE_OFF] = "MODE_OFF";
  enum_map_Mode[(int)MODE_PASSIVE] = "MODE_PASSIVE";
  enum_map_Mode[(int)MODE_SAFE] = "MODE_SAFE";
  enum_map_Mode[(int)MODE_FULL] = "MODE_FULL";
  enum_map_InfraredCharacter[(int)IR_NONE] = "IR_NONE";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LEFT] = "IR_REMOTE_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_FORWARD] = "IR_REMOTE_FORWARD";
  enum_map_InfraredCharacter[(int)IR_REMOTE_RIGHT] = "IR_REMOTE_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SPOT] = "IR_REMOTE_SPOT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MAX] = "IR_REMOTE_MAX";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SMALL] = "IR_REMOTE_SMALL";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MEDIUM] = "IR_REMOTE_MEDIUM";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LARGE_CLEAN] = "IR_REMOTE_LARGE_CLEAN";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP] = "IR_REMOTE_STOP";
  enum_map_InfraredCharacter[(int)IR_REMOTE_POWER] = "IR_REMOTE_POWER";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_LEFT] = "IR_REMOTE_ARC_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_RIGHT] = "IR_REMOTE_ARC_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP2] = "IR_REMOTE_STOP2";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_DOWNLOAD] = "IR_SCHED_REMOTE_DOWNLOAD";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_SEEK_DOCK] = "IR_SCHED_REMOTE_SEEK_DOCK";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RESERVED] = "IR_DISC_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY] = "IR_DISC_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY] = "IR_DISC_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_FORCE_FIELD] = "IR_DISC_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY] = "IR_DISC_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RESERVED] = "IR_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY] = "IR_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY] = "IR_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_FORCE_FIELD] = "IR_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY] = "IR_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_VIRTUAL_WALL] = "IR_VIRTUAL_WALL";
  enum_map_ChargingState[(int)CHARGING_NO] = "CHARGING_NO";
  enum_map_ChargingState[(int)CHARGING_RECONDITIONING] = "CHARGING_RECONDITIONING";
  enum_map_ChargingState[(int)CHARGING_FULL] = "CHARGING_FULL";
  enum_map_ChargingState[(int)CHARGING_TRICKLE] = "CHARGING_TRICKLE";
  enum_map_ChargingState[(int)CHARGING_WAITING] = "CHARGING_WAITING";
  enum_map_ChargingState[(int)CHARGING_ERROR] = "CHARGING_ERROR";
  enum_map_BrushState[(int)BRUSHSTATE_OFF] = "BRUSHSTATE_OFF";
  enum_map_BrushState[(int)BRUSHSTATE_FORWARD] = "BRUSHSTATE_FORWARD";
  enum_map_BrushState[(int)BRUSHSTATE_BACKWARD] = "BRUSHSTATE_BACKWARD";
  add_fieldinfo(IFT_ENUM, "mode", 1, &data->mode, "Mode", &enum_map_Mode);
}
/** Constructor */
Roomba500Interface::SetModeMessage::SetModeMessage() : Message("SetModeMessage")
{
  data_size = sizeof(SetModeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Mode[(int)MODE_OFF] = "MODE_OFF";
  enum_map_Mode[(int)MODE_PASSIVE] = "MODE_PASSIVE";
  enum_map_Mode[(int)MODE_SAFE] = "MODE_SAFE";
  enum_map_Mode[(int)MODE_FULL] = "MODE_FULL";
  enum_map_InfraredCharacter[(int)IR_NONE] = "IR_NONE";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LEFT] = "IR_REMOTE_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_FORWARD] = "IR_REMOTE_FORWARD";
  enum_map_InfraredCharacter[(int)IR_REMOTE_RIGHT] = "IR_REMOTE_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SPOT] = "IR_REMOTE_SPOT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MAX] = "IR_REMOTE_MAX";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SMALL] = "IR_REMOTE_SMALL";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MEDIUM] = "IR_REMOTE_MEDIUM";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LARGE_CLEAN] = "IR_REMOTE_LARGE_CLEAN";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP] = "IR_REMOTE_STOP";
  enum_map_InfraredCharacter[(int)IR_REMOTE_POWER] = "IR_REMOTE_POWER";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_LEFT] = "IR_REMOTE_ARC_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_RIGHT] = "IR_REMOTE_ARC_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP2] = "IR_REMOTE_STOP2";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_DOWNLOAD] = "IR_SCHED_REMOTE_DOWNLOAD";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_SEEK_DOCK] = "IR_SCHED_REMOTE_SEEK_DOCK";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RESERVED] = "IR_DISC_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY] = "IR_DISC_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY] = "IR_DISC_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_FORCE_FIELD] = "IR_DISC_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY] = "IR_DISC_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RESERVED] = "IR_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY] = "IR_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY] = "IR_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_FORCE_FIELD] = "IR_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY] = "IR_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_VIRTUAL_WALL] = "IR_VIRTUAL_WALL";
  enum_map_ChargingState[(int)CHARGING_NO] = "CHARGING_NO";
  enum_map_ChargingState[(int)CHARGING_RECONDITIONING] = "CHARGING_RECONDITIONING";
  enum_map_ChargingState[(int)CHARGING_FULL] = "CHARGING_FULL";
  enum_map_ChargingState[(int)CHARGING_TRICKLE] = "CHARGING_TRICKLE";
  enum_map_ChargingState[(int)CHARGING_WAITING] = "CHARGING_WAITING";
  enum_map_ChargingState[(int)CHARGING_ERROR] = "CHARGING_ERROR";
  enum_map_BrushState[(int)BRUSHSTATE_OFF] = "BRUSHSTATE_OFF";
  enum_map_BrushState[(int)BRUSHSTATE_FORWARD] = "BRUSHSTATE_FORWARD";
  enum_map_BrushState[(int)BRUSHSTATE_BACKWARD] = "BRUSHSTATE_BACKWARD";
  add_fieldinfo(IFT_ENUM, "mode", 1, &data->mode, "Mode", &enum_map_Mode);
}

/** Destructor */
Roomba500Interface::SetModeMessage::~SetModeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
Roomba500Interface::SetModeMessage::SetModeMessage(const SetModeMessage *m) : Message("SetModeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get mode value.
 * Open Interface mode.
 * @return mode value
 */
Roomba500Interface::Mode
Roomba500Interface::SetModeMessage::mode() const
{
  return (Roomba500Interface::Mode)data->mode;
}

/** Get maximum length of mode value.
 * @return length of mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::SetModeMessage::maxlenof_mode() const
{
  return 1;
}

/** Set mode value.
 * Open Interface mode.
 * @param new_mode new mode value
 */
void
Roomba500Interface::SetModeMessage::set_mode(const Mode new_mode)
{
  data->mode = new_mode;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
Roomba500Interface::SetModeMessage::clone() const
{
  return new Roomba500Interface::SetModeMessage(this);
}
/** @class Roomba500Interface::DriveStraightMessage <interfaces/Roomba500Interface.h>
 * DriveStraightMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_velocity initial value for velocity
 */
Roomba500Interface::DriveStraightMessage::DriveStraightMessage(const int16_t ini_velocity) : Message("DriveStraightMessage")
{
  data_size = sizeof(DriveStraightMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DriveStraightMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->velocity = ini_velocity;
  enum_map_Mode[(int)MODE_OFF] = "MODE_OFF";
  enum_map_Mode[(int)MODE_PASSIVE] = "MODE_PASSIVE";
  enum_map_Mode[(int)MODE_SAFE] = "MODE_SAFE";
  enum_map_Mode[(int)MODE_FULL] = "MODE_FULL";
  enum_map_InfraredCharacter[(int)IR_NONE] = "IR_NONE";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LEFT] = "IR_REMOTE_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_FORWARD] = "IR_REMOTE_FORWARD";
  enum_map_InfraredCharacter[(int)IR_REMOTE_RIGHT] = "IR_REMOTE_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SPOT] = "IR_REMOTE_SPOT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MAX] = "IR_REMOTE_MAX";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SMALL] = "IR_REMOTE_SMALL";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MEDIUM] = "IR_REMOTE_MEDIUM";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LARGE_CLEAN] = "IR_REMOTE_LARGE_CLEAN";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP] = "IR_REMOTE_STOP";
  enum_map_InfraredCharacter[(int)IR_REMOTE_POWER] = "IR_REMOTE_POWER";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_LEFT] = "IR_REMOTE_ARC_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_RIGHT] = "IR_REMOTE_ARC_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP2] = "IR_REMOTE_STOP2";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_DOWNLOAD] = "IR_SCHED_REMOTE_DOWNLOAD";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_SEEK_DOCK] = "IR_SCHED_REMOTE_SEEK_DOCK";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RESERVED] = "IR_DISC_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY] = "IR_DISC_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY] = "IR_DISC_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_FORCE_FIELD] = "IR_DISC_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY] = "IR_DISC_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RESERVED] = "IR_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY] = "IR_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY] = "IR_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_FORCE_FIELD] = "IR_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY] = "IR_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_VIRTUAL_WALL] = "IR_VIRTUAL_WALL";
  enum_map_ChargingState[(int)CHARGING_NO] = "CHARGING_NO";
  enum_map_ChargingState[(int)CHARGING_RECONDITIONING] = "CHARGING_RECONDITIONING";
  enum_map_ChargingState[(int)CHARGING_FULL] = "CHARGING_FULL";
  enum_map_ChargingState[(int)CHARGING_TRICKLE] = "CHARGING_TRICKLE";
  enum_map_ChargingState[(int)CHARGING_WAITING] = "CHARGING_WAITING";
  enum_map_ChargingState[(int)CHARGING_ERROR] = "CHARGING_ERROR";
  enum_map_BrushState[(int)BRUSHSTATE_OFF] = "BRUSHSTATE_OFF";
  enum_map_BrushState[(int)BRUSHSTATE_FORWARD] = "BRUSHSTATE_FORWARD";
  enum_map_BrushState[(int)BRUSHSTATE_BACKWARD] = "BRUSHSTATE_BACKWARD";
  add_fieldinfo(IFT_INT16, "velocity", 1, &data->velocity);
}
/** Constructor */
Roomba500Interface::DriveStraightMessage::DriveStraightMessage() : Message("DriveStraightMessage")
{
  data_size = sizeof(DriveStraightMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DriveStraightMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Mode[(int)MODE_OFF] = "MODE_OFF";
  enum_map_Mode[(int)MODE_PASSIVE] = "MODE_PASSIVE";
  enum_map_Mode[(int)MODE_SAFE] = "MODE_SAFE";
  enum_map_Mode[(int)MODE_FULL] = "MODE_FULL";
  enum_map_InfraredCharacter[(int)IR_NONE] = "IR_NONE";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LEFT] = "IR_REMOTE_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_FORWARD] = "IR_REMOTE_FORWARD";
  enum_map_InfraredCharacter[(int)IR_REMOTE_RIGHT] = "IR_REMOTE_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SPOT] = "IR_REMOTE_SPOT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MAX] = "IR_REMOTE_MAX";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SMALL] = "IR_REMOTE_SMALL";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MEDIUM] = "IR_REMOTE_MEDIUM";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LARGE_CLEAN] = "IR_REMOTE_LARGE_CLEAN";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP] = "IR_REMOTE_STOP";
  enum_map_InfraredCharacter[(int)IR_REMOTE_POWER] = "IR_REMOTE_POWER";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_LEFT] = "IR_REMOTE_ARC_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_RIGHT] = "IR_REMOTE_ARC_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP2] = "IR_REMOTE_STOP2";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_DOWNLOAD] = "IR_SCHED_REMOTE_DOWNLOAD";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_SEEK_DOCK] = "IR_SCHED_REMOTE_SEEK_DOCK";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RESERVED] = "IR_DISC_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY] = "IR_DISC_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY] = "IR_DISC_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_FORCE_FIELD] = "IR_DISC_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY] = "IR_DISC_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RESERVED] = "IR_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY] = "IR_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY] = "IR_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_FORCE_FIELD] = "IR_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY] = "IR_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_VIRTUAL_WALL] = "IR_VIRTUAL_WALL";
  enum_map_ChargingState[(int)CHARGING_NO] = "CHARGING_NO";
  enum_map_ChargingState[(int)CHARGING_RECONDITIONING] = "CHARGING_RECONDITIONING";
  enum_map_ChargingState[(int)CHARGING_FULL] = "CHARGING_FULL";
  enum_map_ChargingState[(int)CHARGING_TRICKLE] = "CHARGING_TRICKLE";
  enum_map_ChargingState[(int)CHARGING_WAITING] = "CHARGING_WAITING";
  enum_map_ChargingState[(int)CHARGING_ERROR] = "CHARGING_ERROR";
  enum_map_BrushState[(int)BRUSHSTATE_OFF] = "BRUSHSTATE_OFF";
  enum_map_BrushState[(int)BRUSHSTATE_FORWARD] = "BRUSHSTATE_FORWARD";
  enum_map_BrushState[(int)BRUSHSTATE_BACKWARD] = "BRUSHSTATE_BACKWARD";
  add_fieldinfo(IFT_INT16, "velocity", 1, &data->velocity);
}

/** Destructor */
Roomba500Interface::DriveStraightMessage::~DriveStraightMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
Roomba500Interface::DriveStraightMessage::DriveStraightMessage(const DriveStraightMessage *m) : Message("DriveStraightMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (DriveStraightMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get velocity value.
 * Requested velocity in mm/s.
 * @return velocity value
 */
int16_t
Roomba500Interface::DriveStraightMessage::velocity() const
{
  return data->velocity;
}

/** Get maximum length of velocity value.
 * @return length of velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::DriveStraightMessage::maxlenof_velocity() const
{
  return 1;
}

/** Set velocity value.
 * Requested velocity in mm/s.
 * @param new_velocity new velocity value
 */
void
Roomba500Interface::DriveStraightMessage::set_velocity(const int16_t new_velocity)
{
  data->velocity = new_velocity;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
Roomba500Interface::DriveStraightMessage::clone() const
{
  return new Roomba500Interface::DriveStraightMessage(this);
}
/** @class Roomba500Interface::DriveMessage <interfaces/Roomba500Interface.h>
 * DriveMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_velocity initial value for velocity
 * @param ini_radius initial value for radius
 */
Roomba500Interface::DriveMessage::DriveMessage(const int16_t ini_velocity, const int16_t ini_radius) : Message("DriveMessage")
{
  data_size = sizeof(DriveMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DriveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->velocity = ini_velocity;
  data->radius = ini_radius;
  enum_map_Mode[(int)MODE_OFF] = "MODE_OFF";
  enum_map_Mode[(int)MODE_PASSIVE] = "MODE_PASSIVE";
  enum_map_Mode[(int)MODE_SAFE] = "MODE_SAFE";
  enum_map_Mode[(int)MODE_FULL] = "MODE_FULL";
  enum_map_InfraredCharacter[(int)IR_NONE] = "IR_NONE";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LEFT] = "IR_REMOTE_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_FORWARD] = "IR_REMOTE_FORWARD";
  enum_map_InfraredCharacter[(int)IR_REMOTE_RIGHT] = "IR_REMOTE_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SPOT] = "IR_REMOTE_SPOT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MAX] = "IR_REMOTE_MAX";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SMALL] = "IR_REMOTE_SMALL";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MEDIUM] = "IR_REMOTE_MEDIUM";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LARGE_CLEAN] = "IR_REMOTE_LARGE_CLEAN";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP] = "IR_REMOTE_STOP";
  enum_map_InfraredCharacter[(int)IR_REMOTE_POWER] = "IR_REMOTE_POWER";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_LEFT] = "IR_REMOTE_ARC_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_RIGHT] = "IR_REMOTE_ARC_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP2] = "IR_REMOTE_STOP2";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_DOWNLOAD] = "IR_SCHED_REMOTE_DOWNLOAD";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_SEEK_DOCK] = "IR_SCHED_REMOTE_SEEK_DOCK";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RESERVED] = "IR_DISC_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY] = "IR_DISC_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY] = "IR_DISC_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_FORCE_FIELD] = "IR_DISC_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY] = "IR_DISC_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RESERVED] = "IR_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY] = "IR_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY] = "IR_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_FORCE_FIELD] = "IR_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY] = "IR_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_VIRTUAL_WALL] = "IR_VIRTUAL_WALL";
  enum_map_ChargingState[(int)CHARGING_NO] = "CHARGING_NO";
  enum_map_ChargingState[(int)CHARGING_RECONDITIONING] = "CHARGING_RECONDITIONING";
  enum_map_ChargingState[(int)CHARGING_FULL] = "CHARGING_FULL";
  enum_map_ChargingState[(int)CHARGING_TRICKLE] = "CHARGING_TRICKLE";
  enum_map_ChargingState[(int)CHARGING_WAITING] = "CHARGING_WAITING";
  enum_map_ChargingState[(int)CHARGING_ERROR] = "CHARGING_ERROR";
  enum_map_BrushState[(int)BRUSHSTATE_OFF] = "BRUSHSTATE_OFF";
  enum_map_BrushState[(int)BRUSHSTATE_FORWARD] = "BRUSHSTATE_FORWARD";
  enum_map_BrushState[(int)BRUSHSTATE_BACKWARD] = "BRUSHSTATE_BACKWARD";
  add_fieldinfo(IFT_INT16, "velocity", 1, &data->velocity);
  add_fieldinfo(IFT_INT16, "radius", 1, &data->radius);
}
/** Constructor */
Roomba500Interface::DriveMessage::DriveMessage() : Message("DriveMessage")
{
  data_size = sizeof(DriveMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DriveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Mode[(int)MODE_OFF] = "MODE_OFF";
  enum_map_Mode[(int)MODE_PASSIVE] = "MODE_PASSIVE";
  enum_map_Mode[(int)MODE_SAFE] = "MODE_SAFE";
  enum_map_Mode[(int)MODE_FULL] = "MODE_FULL";
  enum_map_InfraredCharacter[(int)IR_NONE] = "IR_NONE";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LEFT] = "IR_REMOTE_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_FORWARD] = "IR_REMOTE_FORWARD";
  enum_map_InfraredCharacter[(int)IR_REMOTE_RIGHT] = "IR_REMOTE_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SPOT] = "IR_REMOTE_SPOT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MAX] = "IR_REMOTE_MAX";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SMALL] = "IR_REMOTE_SMALL";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MEDIUM] = "IR_REMOTE_MEDIUM";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LARGE_CLEAN] = "IR_REMOTE_LARGE_CLEAN";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP] = "IR_REMOTE_STOP";
  enum_map_InfraredCharacter[(int)IR_REMOTE_POWER] = "IR_REMOTE_POWER";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_LEFT] = "IR_REMOTE_ARC_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_RIGHT] = "IR_REMOTE_ARC_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP2] = "IR_REMOTE_STOP2";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_DOWNLOAD] = "IR_SCHED_REMOTE_DOWNLOAD";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_SEEK_DOCK] = "IR_SCHED_REMOTE_SEEK_DOCK";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RESERVED] = "IR_DISC_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY] = "IR_DISC_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY] = "IR_DISC_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_FORCE_FIELD] = "IR_DISC_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY] = "IR_DISC_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RESERVED] = "IR_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY] = "IR_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY] = "IR_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_FORCE_FIELD] = "IR_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY] = "IR_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_VIRTUAL_WALL] = "IR_VIRTUAL_WALL";
  enum_map_ChargingState[(int)CHARGING_NO] = "CHARGING_NO";
  enum_map_ChargingState[(int)CHARGING_RECONDITIONING] = "CHARGING_RECONDITIONING";
  enum_map_ChargingState[(int)CHARGING_FULL] = "CHARGING_FULL";
  enum_map_ChargingState[(int)CHARGING_TRICKLE] = "CHARGING_TRICKLE";
  enum_map_ChargingState[(int)CHARGING_WAITING] = "CHARGING_WAITING";
  enum_map_ChargingState[(int)CHARGING_ERROR] = "CHARGING_ERROR";
  enum_map_BrushState[(int)BRUSHSTATE_OFF] = "BRUSHSTATE_OFF";
  enum_map_BrushState[(int)BRUSHSTATE_FORWARD] = "BRUSHSTATE_FORWARD";
  enum_map_BrushState[(int)BRUSHSTATE_BACKWARD] = "BRUSHSTATE_BACKWARD";
  add_fieldinfo(IFT_INT16, "velocity", 1, &data->velocity);
  add_fieldinfo(IFT_INT16, "radius", 1, &data->radius);
}

/** Destructor */
Roomba500Interface::DriveMessage::~DriveMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
Roomba500Interface::DriveMessage::DriveMessage(const DriveMessage *m) : Message("DriveMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (DriveMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get velocity value.
 * Requested velocity in mm/s.
 * @return velocity value
 */
int16_t
Roomba500Interface::DriveMessage::velocity() const
{
  return data->velocity;
}

/** Get maximum length of velocity value.
 * @return length of velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::DriveMessage::maxlenof_velocity() const
{
  return 1;
}

/** Set velocity value.
 * Requested velocity in mm/s.
 * @param new_velocity new velocity value
 */
void
Roomba500Interface::DriveMessage::set_velocity(const int16_t new_velocity)
{
  data->velocity = new_velocity;
}

/** Get radius value.
 * Requested radius in mm.
 * @return radius value
 */
int16_t
Roomba500Interface::DriveMessage::radius() const
{
  return data->radius;
}

/** Get maximum length of radius value.
 * @return length of radius value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::DriveMessage::maxlenof_radius() const
{
  return 1;
}

/** Set radius value.
 * Requested radius in mm.
 * @param new_radius new radius value
 */
void
Roomba500Interface::DriveMessage::set_radius(const int16_t new_radius)
{
  data->radius = new_radius;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
Roomba500Interface::DriveMessage::clone() const
{
  return new Roomba500Interface::DriveMessage(this);
}
/** @class Roomba500Interface::SetMotorsMessage <interfaces/Roomba500Interface.h>
 * SetMotorsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_vacuuming initial value for vacuuming
 * @param ini_main initial value for main
 * @param ini_side initial value for side
 */
Roomba500Interface::SetMotorsMessage::SetMotorsMessage(const bool ini_vacuuming, const BrushState ini_main, const BrushState ini_side) : Message("SetMotorsMessage")
{
  data_size = sizeof(SetMotorsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMotorsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->vacuuming = ini_vacuuming;
  data->main = ini_main;
  data->side = ini_side;
  enum_map_Mode[(int)MODE_OFF] = "MODE_OFF";
  enum_map_Mode[(int)MODE_PASSIVE] = "MODE_PASSIVE";
  enum_map_Mode[(int)MODE_SAFE] = "MODE_SAFE";
  enum_map_Mode[(int)MODE_FULL] = "MODE_FULL";
  enum_map_InfraredCharacter[(int)IR_NONE] = "IR_NONE";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LEFT] = "IR_REMOTE_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_FORWARD] = "IR_REMOTE_FORWARD";
  enum_map_InfraredCharacter[(int)IR_REMOTE_RIGHT] = "IR_REMOTE_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SPOT] = "IR_REMOTE_SPOT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MAX] = "IR_REMOTE_MAX";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SMALL] = "IR_REMOTE_SMALL";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MEDIUM] = "IR_REMOTE_MEDIUM";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LARGE_CLEAN] = "IR_REMOTE_LARGE_CLEAN";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP] = "IR_REMOTE_STOP";
  enum_map_InfraredCharacter[(int)IR_REMOTE_POWER] = "IR_REMOTE_POWER";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_LEFT] = "IR_REMOTE_ARC_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_RIGHT] = "IR_REMOTE_ARC_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP2] = "IR_REMOTE_STOP2";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_DOWNLOAD] = "IR_SCHED_REMOTE_DOWNLOAD";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_SEEK_DOCK] = "IR_SCHED_REMOTE_SEEK_DOCK";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RESERVED] = "IR_DISC_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY] = "IR_DISC_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY] = "IR_DISC_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_FORCE_FIELD] = "IR_DISC_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY] = "IR_DISC_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RESERVED] = "IR_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY] = "IR_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY] = "IR_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_FORCE_FIELD] = "IR_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY] = "IR_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_VIRTUAL_WALL] = "IR_VIRTUAL_WALL";
  enum_map_ChargingState[(int)CHARGING_NO] = "CHARGING_NO";
  enum_map_ChargingState[(int)CHARGING_RECONDITIONING] = "CHARGING_RECONDITIONING";
  enum_map_ChargingState[(int)CHARGING_FULL] = "CHARGING_FULL";
  enum_map_ChargingState[(int)CHARGING_TRICKLE] = "CHARGING_TRICKLE";
  enum_map_ChargingState[(int)CHARGING_WAITING] = "CHARGING_WAITING";
  enum_map_ChargingState[(int)CHARGING_ERROR] = "CHARGING_ERROR";
  enum_map_BrushState[(int)BRUSHSTATE_OFF] = "BRUSHSTATE_OFF";
  enum_map_BrushState[(int)BRUSHSTATE_FORWARD] = "BRUSHSTATE_FORWARD";
  enum_map_BrushState[(int)BRUSHSTATE_BACKWARD] = "BRUSHSTATE_BACKWARD";
  add_fieldinfo(IFT_BOOL, "vacuuming", 1, &data->vacuuming);
  add_fieldinfo(IFT_ENUM, "main", 1, &data->main, "BrushState", &enum_map_BrushState);
  add_fieldinfo(IFT_ENUM, "side", 1, &data->side, "BrushState", &enum_map_BrushState);
}
/** Constructor */
Roomba500Interface::SetMotorsMessage::SetMotorsMessage() : Message("SetMotorsMessage")
{
  data_size = sizeof(SetMotorsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMotorsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Mode[(int)MODE_OFF] = "MODE_OFF";
  enum_map_Mode[(int)MODE_PASSIVE] = "MODE_PASSIVE";
  enum_map_Mode[(int)MODE_SAFE] = "MODE_SAFE";
  enum_map_Mode[(int)MODE_FULL] = "MODE_FULL";
  enum_map_InfraredCharacter[(int)IR_NONE] = "IR_NONE";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LEFT] = "IR_REMOTE_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_FORWARD] = "IR_REMOTE_FORWARD";
  enum_map_InfraredCharacter[(int)IR_REMOTE_RIGHT] = "IR_REMOTE_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SPOT] = "IR_REMOTE_SPOT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MAX] = "IR_REMOTE_MAX";
  enum_map_InfraredCharacter[(int)IR_REMOTE_SMALL] = "IR_REMOTE_SMALL";
  enum_map_InfraredCharacter[(int)IR_REMOTE_MEDIUM] = "IR_REMOTE_MEDIUM";
  enum_map_InfraredCharacter[(int)IR_REMOTE_LARGE_CLEAN] = "IR_REMOTE_LARGE_CLEAN";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP] = "IR_REMOTE_STOP";
  enum_map_InfraredCharacter[(int)IR_REMOTE_POWER] = "IR_REMOTE_POWER";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_LEFT] = "IR_REMOTE_ARC_LEFT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_ARC_RIGHT] = "IR_REMOTE_ARC_RIGHT";
  enum_map_InfraredCharacter[(int)IR_REMOTE_STOP2] = "IR_REMOTE_STOP2";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_DOWNLOAD] = "IR_SCHED_REMOTE_DOWNLOAD";
  enum_map_InfraredCharacter[(int)IR_SCHED_REMOTE_SEEK_DOCK] = "IR_SCHED_REMOTE_SEEK_DOCK";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RESERVED] = "IR_DISC_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY] = "IR_DISC_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY] = "IR_DISC_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_FORCE_FIELD] = "IR_DISC_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY] = "IR_DISC_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RESERVED] = "IR_DOCK_RESERVED";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY] = "IR_DOCK_RED_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY] = "IR_DOCK_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_FORCE_FIELD] = "IR_DOCK_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY] = "IR_DOCK_RED_GREEN_BUOY";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_BUOY_FORCE_FIELD] = "IR_DOCK_RED_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD] = "IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD";
  enum_map_InfraredCharacter[(int)IR_VIRTUAL_WALL] = "IR_VIRTUAL_WALL";
  enum_map_ChargingState[(int)CHARGING_NO] = "CHARGING_NO";
  enum_map_ChargingState[(int)CHARGING_RECONDITIONING] = "CHARGING_RECONDITIONING";
  enum_map_ChargingState[(int)CHARGING_FULL] = "CHARGING_FULL";
  enum_map_ChargingState[(int)CHARGING_TRICKLE] = "CHARGING_TRICKLE";
  enum_map_ChargingState[(int)CHARGING_WAITING] = "CHARGING_WAITING";
  enum_map_ChargingState[(int)CHARGING_ERROR] = "CHARGING_ERROR";
  enum_map_BrushState[(int)BRUSHSTATE_OFF] = "BRUSHSTATE_OFF";
  enum_map_BrushState[(int)BRUSHSTATE_FORWARD] = "BRUSHSTATE_FORWARD";
  enum_map_BrushState[(int)BRUSHSTATE_BACKWARD] = "BRUSHSTATE_BACKWARD";
  add_fieldinfo(IFT_BOOL, "vacuuming", 1, &data->vacuuming);
  add_fieldinfo(IFT_ENUM, "main", 1, &data->main, "BrushState", &enum_map_BrushState);
  add_fieldinfo(IFT_ENUM, "side", 1, &data->side, "BrushState", &enum_map_BrushState);
}

/** Destructor */
Roomba500Interface::SetMotorsMessage::~SetMotorsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
Roomba500Interface::SetMotorsMessage::SetMotorsMessage(const SetMotorsMessage *m) : Message("SetMotorsMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetMotorsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get vacuuming value.
 * Enable vacuuming?
 * @return vacuuming value
 */
bool
Roomba500Interface::SetMotorsMessage::is_vacuuming() const
{
  return data->vacuuming;
}

/** Get maximum length of vacuuming value.
 * @return length of vacuuming value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::SetMotorsMessage::maxlenof_vacuuming() const
{
  return 1;
}

/** Set vacuuming value.
 * Enable vacuuming?
 * @param new_vacuuming new vacuuming value
 */
void
Roomba500Interface::SetMotorsMessage::set_vacuuming(const bool new_vacuuming)
{
  data->vacuuming = new_vacuuming;
}

/** Get main value.
 * Main brush state.
 * @return main value
 */
Roomba500Interface::BrushState
Roomba500Interface::SetMotorsMessage::main() const
{
  return (Roomba500Interface::BrushState)data->main;
}

/** Get maximum length of main value.
 * @return length of main value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::SetMotorsMessage::maxlenof_main() const
{
  return 1;
}

/** Set main value.
 * Main brush state.
 * @param new_main new main value
 */
void
Roomba500Interface::SetMotorsMessage::set_main(const BrushState new_main)
{
  data->main = new_main;
}

/** Get side value.
 * Side brush state.
 * @return side value
 */
Roomba500Interface::BrushState
Roomba500Interface::SetMotorsMessage::side() const
{
  return (Roomba500Interface::BrushState)data->side;
}

/** Get maximum length of side value.
 * @return length of side value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Roomba500Interface::SetMotorsMessage::maxlenof_side() const
{
  return 1;
}

/** Set side value.
 * Side brush state.
 * @param new_side new side value
 */
void
Roomba500Interface::SetMotorsMessage::set_side(const BrushState new_side)
{
  data->side = new_side;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
Roomba500Interface::SetMotorsMessage::clone() const
{
  return new Roomba500Interface::SetMotorsMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
Roomba500Interface::message_valid(const Message *message) const
{
  const StopMessage *m0 = dynamic_cast<const StopMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const DockMessage *m1 = dynamic_cast<const DockMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const SetModeMessage *m2 = dynamic_cast<const SetModeMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const DriveStraightMessage *m3 = dynamic_cast<const DriveStraightMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const DriveMessage *m4 = dynamic_cast<const DriveMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const SetMotorsMessage *m5 = dynamic_cast<const SetMotorsMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(Roomba500Interface)
/// @endcond


} // end namespace fawkes
