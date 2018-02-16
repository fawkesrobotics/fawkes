
/***************************************************************************
 *  NavigatorInterface.cpp - Fawkes BlackBoard Interface - NavigatorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007-2009  Martin Liebenberg, Daniel Beck, Tim Niemueller
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

#include <interfaces/NavigatorInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class NavigatorInterface <interfaces/NavigatorInterface.h>
 * NavigatorInterface Fawkes BlackBoard Interface.
 * 
      The navigator interface is used by the navigator to export information about
      the current status of the navigator and to define all messages by which the navigator
      can be instructed.

      There are three coordinate systems, the robot system which is a right-handed cartesian
      coordinate system with the robot in its origin, X axis pointing forward, Y pointing to
      the left and Z pointing upwards. The second coordinate system is the so-called
      navigator system. It is a coordinate system similar to the robot system, but the
      origin is defined on the initialization of the navigator. The last system is the
      odometry system. It is again a similar system, but the origin is reset from time
      to time and the robot's position in this system gives the odometry deltas.
    
 * @ingroup FawkesInterfaces
 */


/** ERROR_NONE constant */
const uint32_t NavigatorInterface::ERROR_NONE = 0u;
/** ERROR_MOTOR constant */
const uint32_t NavigatorInterface::ERROR_MOTOR = 1u;
/** ERROR_OBSTRUCTION constant */
const uint32_t NavigatorInterface::ERROR_OBSTRUCTION = 2u;
/** ERROR_UNKNOWN_PLACE constant */
const uint32_t NavigatorInterface::ERROR_UNKNOWN_PLACE = 4u;
/** ERROR_PATH_GEN_FAIL constant */
const uint32_t NavigatorInterface::ERROR_PATH_GEN_FAIL = 8u;
/** FLAG_NONE constant */
const uint32_t NavigatorInterface::FLAG_NONE = 0u;
/** FLAG_CART_GOTO constant */
const uint32_t NavigatorInterface::FLAG_CART_GOTO = 1u;
/** FLAG_POLAR_GOTO constant */
const uint32_t NavigatorInterface::FLAG_POLAR_GOTO = 2u;
/** FLAG_PLACE_GOTO constant */
const uint32_t NavigatorInterface::FLAG_PLACE_GOTO = 4u;
/** FLAG_UPDATES_DEST_DIST constant */
const uint32_t NavigatorInterface::FLAG_UPDATES_DEST_DIST = 8u;
/** FLAG_SECURITY_DISTANCE constant */
const uint32_t NavigatorInterface::FLAG_SECURITY_DISTANCE = 16u;
/** FLAG_ESCAPING constant */
const uint32_t NavigatorInterface::FLAG_ESCAPING = 32u;

/** Constructor */
NavigatorInterface::NavigatorInterface() : Interface()
{
  data_size = sizeof(NavigatorInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (NavigatorInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_UINT32, "flags", 1, &data->flags);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "dest_x", 1, &data->dest_x);
  add_fieldinfo(IFT_FLOAT, "dest_y", 1, &data->dest_y);
  add_fieldinfo(IFT_FLOAT, "dest_ori", 1, &data->dest_ori);
  add_fieldinfo(IFT_FLOAT, "dest_dist", 1, &data->dest_dist);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_UINT32, "error_code", 1, &data->error_code);
  add_fieldinfo(IFT_FLOAT, "max_velocity", 1, &data->max_velocity);
  add_fieldinfo(IFT_FLOAT, "max_rotation", 1, &data->max_rotation);
  add_fieldinfo(IFT_FLOAT, "security_distance", 1, &data->security_distance);
  add_fieldinfo(IFT_BOOL, "escaping_enabled", 1, &data->escaping_enabled);
  add_fieldinfo(IFT_ENUM, "drive_mode", 1, &data->drive_mode, "DriveMode", &enum_map_DriveMode);
  add_fieldinfo(IFT_BOOL, "auto_drive_mode", 1, &data->auto_drive_mode);
  add_fieldinfo(IFT_BOOL, "stop_at_target", 1, &data->stop_at_target);
  add_fieldinfo(IFT_ENUM, "orientation_mode", 1, &data->orientation_mode, "OrientationMode", &enum_map_OrientationMode);
  add_fieldinfo(IFT_STRING, "target_frame", 64, data->target_frame);
  add_messageinfo("StopMessage");
  add_messageinfo("TurnMessage");
  add_messageinfo("CartesianGotoMessage");
  add_messageinfo("CartesianGotoWithFrameMessage");
  add_messageinfo("PolarGotoMessage");
  add_messageinfo("PlaceGotoMessage");
  add_messageinfo("PlaceWithOriGotoMessage");
  add_messageinfo("ObstacleMessage");
  add_messageinfo("ResetOdometryMessage");
  add_messageinfo("SetMaxVelocityMessage");
  add_messageinfo("SetMaxRotationMessage");
  add_messageinfo("SetEscapingMessage");
  add_messageinfo("SetSecurityDistanceMessage");
  add_messageinfo("SetDriveModeMessage");
  add_messageinfo("SetStopAtTargetMessage");
  add_messageinfo("SetOrientationModeMessage");
  add_messageinfo("ResetParametersMessage");
  unsigned char tmp_hash[] = {0x43, 0xfc, 0x3a, 0xa9, 0x15, 0x62, 0x32, 0x4c, 0x34, 0x54, 0x7d, 0xd4, 0xf, 0x20, 0x43, 0xcc};
  set_hash(tmp_hash);
}

/** Destructor */
NavigatorInterface::~NavigatorInterface()
{
  free(data_ptr);
}
/** Convert DriveMode constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
NavigatorInterface::tostring_DriveMode(DriveMode value) const
{
  switch (value) {
  case MovingNotAllowed: return "MovingNotAllowed";
  case Forward: return "Forward";
  case AllowBackward: return "AllowBackward";
  case Backward: return "Backward";
  case ESCAPE: return "ESCAPE";
  default: return "UNKNOWN";
  }
}
/** Convert OrientationMode constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
NavigatorInterface::tostring_OrientationMode(OrientationMode value) const
{
  switch (value) {
  case OrientAtTarget: return "OrientAtTarget";
  case OrientDuringTravel: return "OrientDuringTravel";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get flags value.
 * Bit-wise combination of
    FLAG_* constants denoting navigator component features.
 * @return flags value
 */
uint32_t
NavigatorInterface::flags() const
{
  return data->flags;
}

/** Get maximum length of flags value.
 * @return length of flags value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_flags() const
{
  return 1;
}

/** Set flags value.
 * Bit-wise combination of
    FLAG_* constants denoting navigator component features.
 * @param new_flags new flags value
 */
void
NavigatorInterface::set_flags(const uint32_t new_flags)
{
  data->flags = new_flags;
  data_changed = true;
}

/** Get x value.
 * Current X-coordinate in the navigator coordinate system.
 * @return x value
 */
float
NavigatorInterface::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * Current X-coordinate in the navigator coordinate system.
 * @param new_x new x value
 */
void
NavigatorInterface::set_x(const float new_x)
{
  data->x = new_x;
  data_changed = true;
}

/** Get y value.
 * Current Y-coordinate in the navigator coordinate system.
 * @return y value
 */
float
NavigatorInterface::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Current Y-coordinate in the navigator coordinate system.
 * @param new_y new y value
 */
void
NavigatorInterface::set_y(const float new_y)
{
  data->y = new_y;
  data_changed = true;
}

/** Get dest_x value.
 * X-coordinate of the current destination, or 0.0 if no target has been set.
 * @return dest_x value
 */
float
NavigatorInterface::dest_x() const
{
  return data->dest_x;
}

/** Get maximum length of dest_x value.
 * @return length of dest_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_dest_x() const
{
  return 1;
}

/** Set dest_x value.
 * X-coordinate of the current destination, or 0.0 if no target has been set.
 * @param new_dest_x new dest_x value
 */
void
NavigatorInterface::set_dest_x(const float new_dest_x)
{
  data->dest_x = new_dest_x;
  data_changed = true;
}

/** Get dest_y value.
 * Y-coordinate of the current destination, or 0.0 if no target has been set.
 * @return dest_y value
 */
float
NavigatorInterface::dest_y() const
{
  return data->dest_y;
}

/** Get maximum length of dest_y value.
 * @return length of dest_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_dest_y() const
{
  return 1;
}

/** Set dest_y value.
 * Y-coordinate of the current destination, or 0.0 if no target has been set.
 * @param new_dest_y new dest_y value
 */
void
NavigatorInterface::set_dest_y(const float new_dest_y)
{
  data->dest_y = new_dest_y;
  data_changed = true;
}

/** Get dest_ori value.
 * Orientation of the current destination, or 0.0 if no target has been set.
 * @return dest_ori value
 */
float
NavigatorInterface::dest_ori() const
{
  return data->dest_ori;
}

/** Get maximum length of dest_ori value.
 * @return length of dest_ori value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_dest_ori() const
{
  return 1;
}

/** Set dest_ori value.
 * Orientation of the current destination, or 0.0 if no target has been set.
 * @param new_dest_ori new dest_ori value
 */
void
NavigatorInterface::set_dest_ori(const float new_dest_ori)
{
  data->dest_ori = new_dest_ori;
  data_changed = true;
}

/** Get dest_dist value.
 * Distance to destination in m.
 * @return dest_dist value
 */
float
NavigatorInterface::dest_dist() const
{
  return data->dest_dist;
}

/** Get maximum length of dest_dist value.
 * @return length of dest_dist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_dest_dist() const
{
  return 1;
}

/** Set dest_dist value.
 * Distance to destination in m.
 * @param new_dest_dist new dest_dist value
 */
void
NavigatorInterface::set_dest_dist(const float new_dest_dist)
{
  data->dest_dist = new_dest_dist;
  data_changed = true;
}

/** Get msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @return msgid value
 */
uint32_t
NavigatorInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @param new_msgid new msgid value
 */
void
NavigatorInterface::set_msgid(const uint32_t new_msgid)
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
NavigatorInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @param new_final new final value
 */
void
NavigatorInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/** Get error_code value.
 * Failure code set if
    final is true. 0 if no error occured, an error code from ERROR_*
    constants otherwise (or a bit-wise combination).
 * @return error_code value
 */
uint32_t
NavigatorInterface::error_code() const
{
  return data->error_code;
}

/** Get maximum length of error_code value.
 * @return length of error_code value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_error_code() const
{
  return 1;
}

/** Set error_code value.
 * Failure code set if
    final is true. 0 if no error occured, an error code from ERROR_*
    constants otherwise (or a bit-wise combination).
 * @param new_error_code new error_code value
 */
void
NavigatorInterface::set_error_code(const uint32_t new_error_code)
{
  data->error_code = new_error_code;
  data_changed = true;
}

/** Get max_velocity value.
 * Maximum velocity
 * @return max_velocity value
 */
float
NavigatorInterface::max_velocity() const
{
  return data->max_velocity;
}

/** Get maximum length of max_velocity value.
 * @return length of max_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_max_velocity() const
{
  return 1;
}

/** Set max_velocity value.
 * Maximum velocity
 * @param new_max_velocity new max_velocity value
 */
void
NavigatorInterface::set_max_velocity(const float new_max_velocity)
{
  data->max_velocity = new_max_velocity;
  data_changed = true;
}

/** Get max_rotation value.
 * Maximum rotation velocity
 * @return max_rotation value
 */
float
NavigatorInterface::max_rotation() const
{
  return data->max_rotation;
}

/** Get maximum length of max_rotation value.
 * @return length of max_rotation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_max_rotation() const
{
  return 1;
}

/** Set max_rotation value.
 * Maximum rotation velocity
 * @param new_max_rotation new max_rotation value
 */
void
NavigatorInterface::set_max_rotation(const float new_max_rotation)
{
  data->max_rotation = new_max_rotation;
  data_changed = true;
}

/** Get security_distance value.
 * Security distance to keep to obstacles
 * @return security_distance value
 */
float
NavigatorInterface::security_distance() const
{
  return data->security_distance;
}

/** Get maximum length of security_distance value.
 * @return length of security_distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_security_distance() const
{
  return 1;
}

/** Set security_distance value.
 * Security distance to keep to obstacles
 * @param new_security_distance new security_distance value
 */
void
NavigatorInterface::set_security_distance(const float new_security_distance)
{
  data->security_distance = new_security_distance;
  data_changed = true;
}

/** Get escaping_enabled value.
 * This is used for navigation components with integrated collision avoidance,
      to check whether the navigator should stop when an obstacle obstructs the path, or if it should escape.
 * @return escaping_enabled value
 */
bool
NavigatorInterface::is_escaping_enabled() const
{
  return data->escaping_enabled;
}

/** Get maximum length of escaping_enabled value.
 * @return length of escaping_enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_escaping_enabled() const
{
  return 1;
}

/** Set escaping_enabled value.
 * This is used for navigation components with integrated collision avoidance,
      to check whether the navigator should stop when an obstacle obstructs the path, or if it should escape.
 * @param new_escaping_enabled new escaping_enabled value
 */
void
NavigatorInterface::set_escaping_enabled(const bool new_escaping_enabled)
{
  data->escaping_enabled = new_escaping_enabled;
  data_changed = true;
}

/** Get drive_mode value.
 * Current drive mode
 * @return drive_mode value
 */
NavigatorInterface::DriveMode
NavigatorInterface::drive_mode() const
{
  return (NavigatorInterface::DriveMode)data->drive_mode;
}

/** Get maximum length of drive_mode value.
 * @return length of drive_mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_drive_mode() const
{
  return 1;
}

/** Set drive_mode value.
 * Current drive mode
 * @param new_drive_mode new drive_mode value
 */
void
NavigatorInterface::set_drive_mode(const DriveMode new_drive_mode)
{
  data->drive_mode = new_drive_mode;
  data_changed = true;
}

/** Get auto_drive_mode value.
 * True, if the drive mode should be automatically decided each time.
      False, if the drive mode should not automatically change, which is the case when sending
      a SetAutoDriveMode-message (otherwise the navigator might ignore that value).
 * @return auto_drive_mode value
 */
bool
NavigatorInterface::is_auto_drive_mode() const
{
  return data->auto_drive_mode;
}

/** Get maximum length of auto_drive_mode value.
 * @return length of auto_drive_mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_auto_drive_mode() const
{
  return 1;
}

/** Set auto_drive_mode value.
 * True, if the drive mode should be automatically decided each time.
      False, if the drive mode should not automatically change, which is the case when sending
      a SetAutoDriveMode-message (otherwise the navigator might ignore that value).
 * @param new_auto_drive_mode new auto_drive_mode value
 */
void
NavigatorInterface::set_auto_drive_mode(const bool new_auto_drive_mode)
{
  data->auto_drive_mode = new_auto_drive_mode;
  data_changed = true;
}

/** Get stop_at_target value.
 * Stop when target is reached?
 * @return stop_at_target value
 */
bool
NavigatorInterface::is_stop_at_target() const
{
  return data->stop_at_target;
}

/** Get maximum length of stop_at_target value.
 * @return length of stop_at_target value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_stop_at_target() const
{
  return 1;
}

/** Set stop_at_target value.
 * Stop when target is reached?
 * @param new_stop_at_target new stop_at_target value
 */
void
NavigatorInterface::set_stop_at_target(const bool new_stop_at_target)
{
  data->stop_at_target = new_stop_at_target;
  data_changed = true;
}

/** Get orientation_mode value.
 * Mode how/when to orientate if orientation is given
 * @return orientation_mode value
 */
NavigatorInterface::OrientationMode
NavigatorInterface::orientation_mode() const
{
  return (NavigatorInterface::OrientationMode)data->orientation_mode;
}

/** Get maximum length of orientation_mode value.
 * @return length of orientation_mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_orientation_mode() const
{
  return 1;
}

/** Set orientation_mode value.
 * Mode how/when to orientate if orientation is given
 * @param new_orientation_mode new orientation_mode value
 */
void
NavigatorInterface::set_orientation_mode(const OrientationMode new_orientation_mode)
{
  data->orientation_mode = new_orientation_mode;
  data_changed = true;
}

/** Get target_frame value.
 * The target frame to plan into
 * @return target_frame value
 */
char *
NavigatorInterface::target_frame() const
{
  return data->target_frame;
}

/** Get maximum length of target_frame value.
 * @return length of target_frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_target_frame() const
{
  return 64;
}

/** Set target_frame value.
 * The target frame to plan into
 * @param new_target_frame new target_frame value
 */
void
NavigatorInterface::set_target_frame(const char * new_target_frame)
{
  strncpy(data->target_frame, new_target_frame, sizeof(data->target_frame));
  data_changed = true;
}

/* =========== message create =========== */
Message *
NavigatorInterface::create_message(const char *type) const
{
  if ( strncmp("StopMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopMessage();
  } else if ( strncmp("TurnMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TurnMessage();
  } else if ( strncmp("CartesianGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CartesianGotoMessage();
  } else if ( strncmp("CartesianGotoWithFrameMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CartesianGotoWithFrameMessage();
  } else if ( strncmp("PolarGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new PolarGotoMessage();
  } else if ( strncmp("PlaceGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new PlaceGotoMessage();
  } else if ( strncmp("PlaceWithOriGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new PlaceWithOriGotoMessage();
  } else if ( strncmp("ObstacleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ObstacleMessage();
  } else if ( strncmp("ResetOdometryMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ResetOdometryMessage();
  } else if ( strncmp("SetMaxVelocityMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMaxVelocityMessage();
  } else if ( strncmp("SetMaxRotationMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMaxRotationMessage();
  } else if ( strncmp("SetEscapingMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetEscapingMessage();
  } else if ( strncmp("SetSecurityDistanceMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetSecurityDistanceMessage();
  } else if ( strncmp("SetDriveModeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetDriveModeMessage();
  } else if ( strncmp("SetStopAtTargetMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetStopAtTargetMessage();
  } else if ( strncmp("SetOrientationModeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetOrientationModeMessage();
  } else if ( strncmp("ResetParametersMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ResetParametersMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
NavigatorInterface::copy_values(const Interface *other)
{
  const NavigatorInterface *oi = dynamic_cast<const NavigatorInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(NavigatorInterface_data_t));
}

const char *
NavigatorInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "DriveMode") == 0) {
    return tostring_DriveMode((DriveMode)val);
  }
  if (strcmp(enumtype, "OrientationMode") == 0) {
    return tostring_OrientationMode((OrientationMode)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class NavigatorInterface::StopMessage <interfaces/NavigatorInterface.h>
 * StopMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_msgid initial value for msgid
 */
NavigatorInterface::StopMessage::StopMessage(const uint32_t ini_msgid) : Message("StopMessage")
{
  data_size = sizeof(StopMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->msgid = ini_msgid;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
}
/** Constructor */
NavigatorInterface::StopMessage::StopMessage() : Message("StopMessage")
{
  data_size = sizeof(StopMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
}

/** Destructor */
NavigatorInterface::StopMessage::~StopMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::StopMessage::StopMessage(const StopMessage *m) : Message("StopMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get msgid value.
 * 
	    If zero, stops any motion. If non-zero, the component shall only
	    stop the motion if the currently executed command was received
	    through a message with that specific ID.

	    Use the specific version whenever possible. It avoids a race
	    condition if one intstructing component sends a stop, and
	    another a new drive command at the same time.
    
 * @return msgid value
 */
uint32_t
NavigatorInterface::StopMessage::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::StopMessage::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * 
	    If zero, stops any motion. If non-zero, the component shall only
	    stop the motion if the currently executed command was received
	    through a message with that specific ID.

	    Use the specific version whenever possible. It avoids a race
	    condition if one intstructing component sends a stop, and
	    another a new drive command at the same time.
    
 * @param new_msgid new msgid value
 */
void
NavigatorInterface::StopMessage::set_msgid(const uint32_t new_msgid)
{
  data->msgid = new_msgid;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::StopMessage::clone() const
{
  return new NavigatorInterface::StopMessage(this);
}
/** @class NavigatorInterface::TurnMessage <interfaces/NavigatorInterface.h>
 * TurnMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_angle initial value for angle
 * @param ini_velocity initial value for velocity
 */
NavigatorInterface::TurnMessage::TurnMessage(const float ini_angle, const float ini_velocity) : Message("TurnMessage")
{
  data_size = sizeof(TurnMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->angle = ini_angle;
  data->velocity = ini_velocity;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
  add_fieldinfo(IFT_FLOAT, "velocity", 1, &data->velocity);
}
/** Constructor */
NavigatorInterface::TurnMessage::TurnMessage() : Message("TurnMessage")
{
  data_size = sizeof(TurnMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
  add_fieldinfo(IFT_FLOAT, "velocity", 1, &data->velocity);
}

/** Destructor */
NavigatorInterface::TurnMessage::~TurnMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::TurnMessage::TurnMessage(const TurnMessage *m) : Message("TurnMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get angle value.
 * Angle of the turn.
 * @return angle value
 */
float
NavigatorInterface::TurnMessage::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::TurnMessage::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Angle of the turn.
 * @param new_angle new angle value
 */
void
NavigatorInterface::TurnMessage::set_angle(const float new_angle)
{
  data->angle = new_angle;
}

/** Get velocity value.
 * The desired turning velocity in rad/s,
      set to zero to use default value.
 * @return velocity value
 */
float
NavigatorInterface::TurnMessage::velocity() const
{
  return data->velocity;
}

/** Get maximum length of velocity value.
 * @return length of velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::TurnMessage::maxlenof_velocity() const
{
  return 1;
}

/** Set velocity value.
 * The desired turning velocity in rad/s,
      set to zero to use default value.
 * @param new_velocity new velocity value
 */
void
NavigatorInterface::TurnMessage::set_velocity(const float new_velocity)
{
  data->velocity = new_velocity;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::TurnMessage::clone() const
{
  return new NavigatorInterface::TurnMessage(this);
}
/** @class NavigatorInterface::CartesianGotoMessage <interfaces/NavigatorInterface.h>
 * CartesianGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_orientation initial value for orientation
 */
NavigatorInterface::CartesianGotoMessage::CartesianGotoMessage(const float ini_x, const float ini_y, const float ini_orientation) : Message("CartesianGotoMessage")
{
  data_size = sizeof(CartesianGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->orientation = ini_orientation;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "orientation", 1, &data->orientation);
}
/** Constructor */
NavigatorInterface::CartesianGotoMessage::CartesianGotoMessage() : Message("CartesianGotoMessage")
{
  data_size = sizeof(CartesianGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "orientation", 1, &data->orientation);
}

/** Destructor */
NavigatorInterface::CartesianGotoMessage::~CartesianGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::CartesianGotoMessage::CartesianGotoMessage(const CartesianGotoMessage *m) : Message("CartesianGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X-coordinate of the target, in the robot's coordinate system.
 * @return x value
 */
float
NavigatorInterface::CartesianGotoMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::CartesianGotoMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X-coordinate of the target, in the robot's coordinate system.
 * @param new_x new x value
 */
void
NavigatorInterface::CartesianGotoMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y-coordinate of the target, in the robot's coordinate system.
 * @return y value
 */
float
NavigatorInterface::CartesianGotoMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::CartesianGotoMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y-coordinate of the target, in the robot's coordinate system.
 * @param new_y new y value
 */
void
NavigatorInterface::CartesianGotoMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get orientation value.
 * The desired orientation of the robot at the target.
 * @return orientation value
 */
float
NavigatorInterface::CartesianGotoMessage::orientation() const
{
  return data->orientation;
}

/** Get maximum length of orientation value.
 * @return length of orientation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::CartesianGotoMessage::maxlenof_orientation() const
{
  return 1;
}

/** Set orientation value.
 * The desired orientation of the robot at the target.
 * @param new_orientation new orientation value
 */
void
NavigatorInterface::CartesianGotoMessage::set_orientation(const float new_orientation)
{
  data->orientation = new_orientation;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::CartesianGotoMessage::clone() const
{
  return new NavigatorInterface::CartesianGotoMessage(this);
}
/** @class NavigatorInterface::CartesianGotoWithFrameMessage <interfaces/NavigatorInterface.h>
 * CartesianGotoWithFrameMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_orientation initial value for orientation
 * @param ini_target_frame initial value for target_frame
 */
NavigatorInterface::CartesianGotoWithFrameMessage::CartesianGotoWithFrameMessage(const float ini_x, const float ini_y, const float ini_orientation, const char * ini_target_frame) : Message("CartesianGotoWithFrameMessage")
{
  data_size = sizeof(CartesianGotoWithFrameMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CartesianGotoWithFrameMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->orientation = ini_orientation;
  strncpy(data->target_frame, ini_target_frame, 64);
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "orientation", 1, &data->orientation);
  add_fieldinfo(IFT_STRING, "target_frame", 64, data->target_frame);
}
/** Constructor */
NavigatorInterface::CartesianGotoWithFrameMessage::CartesianGotoWithFrameMessage() : Message("CartesianGotoWithFrameMessage")
{
  data_size = sizeof(CartesianGotoWithFrameMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CartesianGotoWithFrameMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "orientation", 1, &data->orientation);
  add_fieldinfo(IFT_STRING, "target_frame", 64, data->target_frame);
}

/** Destructor */
NavigatorInterface::CartesianGotoWithFrameMessage::~CartesianGotoWithFrameMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::CartesianGotoWithFrameMessage::CartesianGotoWithFrameMessage(const CartesianGotoWithFrameMessage *m) : Message("CartesianGotoWithFrameMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CartesianGotoWithFrameMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X-coordinate of the target, in the robot's coordinate system.
 * @return x value
 */
float
NavigatorInterface::CartesianGotoWithFrameMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::CartesianGotoWithFrameMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X-coordinate of the target, in the robot's coordinate system.
 * @param new_x new x value
 */
void
NavigatorInterface::CartesianGotoWithFrameMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y-coordinate of the target, in the robot's coordinate system.
 * @return y value
 */
float
NavigatorInterface::CartesianGotoWithFrameMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::CartesianGotoWithFrameMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y-coordinate of the target, in the robot's coordinate system.
 * @param new_y new y value
 */
void
NavigatorInterface::CartesianGotoWithFrameMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get orientation value.
 * The desired orientation of the robot at the target.
 * @return orientation value
 */
float
NavigatorInterface::CartesianGotoWithFrameMessage::orientation() const
{
  return data->orientation;
}

/** Get maximum length of orientation value.
 * @return length of orientation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::CartesianGotoWithFrameMessage::maxlenof_orientation() const
{
  return 1;
}

/** Set orientation value.
 * The desired orientation of the robot at the target.
 * @param new_orientation new orientation value
 */
void
NavigatorInterface::CartesianGotoWithFrameMessage::set_orientation(const float new_orientation)
{
  data->orientation = new_orientation;
}

/** Get target_frame value.
 * The target frame to plan in.
 * @return target_frame value
 */
char *
NavigatorInterface::CartesianGotoWithFrameMessage::target_frame() const
{
  return data->target_frame;
}

/** Get maximum length of target_frame value.
 * @return length of target_frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::CartesianGotoWithFrameMessage::maxlenof_target_frame() const
{
  return 64;
}

/** Set target_frame value.
 * The target frame to plan in.
 * @param new_target_frame new target_frame value
 */
void
NavigatorInterface::CartesianGotoWithFrameMessage::set_target_frame(const char * new_target_frame)
{
  strncpy(data->target_frame, new_target_frame, sizeof(data->target_frame));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::CartesianGotoWithFrameMessage::clone() const
{
  return new NavigatorInterface::CartesianGotoWithFrameMessage(this);
}
/** @class NavigatorInterface::PolarGotoMessage <interfaces/NavigatorInterface.h>
 * PolarGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_phi initial value for phi
 * @param ini_dist initial value for dist
 * @param ini_orientation initial value for orientation
 */
NavigatorInterface::PolarGotoMessage::PolarGotoMessage(const float ini_phi, const float ini_dist, const float ini_orientation) : Message("PolarGotoMessage")
{
  data_size = sizeof(PolarGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PolarGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->phi = ini_phi;
  data->dist = ini_dist;
  data->orientation = ini_orientation;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "phi", 1, &data->phi);
  add_fieldinfo(IFT_FLOAT, "dist", 1, &data->dist);
  add_fieldinfo(IFT_FLOAT, "orientation", 1, &data->orientation);
}
/** Constructor */
NavigatorInterface::PolarGotoMessage::PolarGotoMessage() : Message("PolarGotoMessage")
{
  data_size = sizeof(PolarGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PolarGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "phi", 1, &data->phi);
  add_fieldinfo(IFT_FLOAT, "dist", 1, &data->dist);
  add_fieldinfo(IFT_FLOAT, "orientation", 1, &data->orientation);
}

/** Destructor */
NavigatorInterface::PolarGotoMessage::~PolarGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::PolarGotoMessage::PolarGotoMessage(const PolarGotoMessage *m) : Message("PolarGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (PolarGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get phi value.
 * Angle between the robot's front and the target.
 * @return phi value
 */
float
NavigatorInterface::PolarGotoMessage::phi() const
{
  return data->phi;
}

/** Get maximum length of phi value.
 * @return length of phi value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::PolarGotoMessage::maxlenof_phi() const
{
  return 1;
}

/** Set phi value.
 * Angle between the robot's front and the target.
 * @param new_phi new phi value
 */
void
NavigatorInterface::PolarGotoMessage::set_phi(const float new_phi)
{
  data->phi = new_phi;
}

/** Get dist value.
 * Distance to the target.
 * @return dist value
 */
float
NavigatorInterface::PolarGotoMessage::dist() const
{
  return data->dist;
}

/** Get maximum length of dist value.
 * @return length of dist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::PolarGotoMessage::maxlenof_dist() const
{
  return 1;
}

/** Set dist value.
 * Distance to the target.
 * @param new_dist new dist value
 */
void
NavigatorInterface::PolarGotoMessage::set_dist(const float new_dist)
{
  data->dist = new_dist;
}

/** Get orientation value.
 * The desired orientation of the robot at the target.
 * @return orientation value
 */
float
NavigatorInterface::PolarGotoMessage::orientation() const
{
  return data->orientation;
}

/** Get maximum length of orientation value.
 * @return length of orientation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::PolarGotoMessage::maxlenof_orientation() const
{
  return 1;
}

/** Set orientation value.
 * The desired orientation of the robot at the target.
 * @param new_orientation new orientation value
 */
void
NavigatorInterface::PolarGotoMessage::set_orientation(const float new_orientation)
{
  data->orientation = new_orientation;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::PolarGotoMessage::clone() const
{
  return new NavigatorInterface::PolarGotoMessage(this);
}
/** @class NavigatorInterface::PlaceGotoMessage <interfaces/NavigatorInterface.h>
 * PlaceGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_place initial value for place
 */
NavigatorInterface::PlaceGotoMessage::PlaceGotoMessage(const char * ini_place) : Message("PlaceGotoMessage")
{
  data_size = sizeof(PlaceGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PlaceGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->place, ini_place, 64);
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_STRING, "place", 64, data->place);
}
/** Constructor */
NavigatorInterface::PlaceGotoMessage::PlaceGotoMessage() : Message("PlaceGotoMessage")
{
  data_size = sizeof(PlaceGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PlaceGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_STRING, "place", 64, data->place);
}

/** Destructor */
NavigatorInterface::PlaceGotoMessage::~PlaceGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::PlaceGotoMessage::PlaceGotoMessage(const PlaceGotoMessage *m) : Message("PlaceGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (PlaceGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get place value.
 * Place to go to.
 * @return place value
 */
char *
NavigatorInterface::PlaceGotoMessage::place() const
{
  return data->place;
}

/** Get maximum length of place value.
 * @return length of place value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::PlaceGotoMessage::maxlenof_place() const
{
  return 64;
}

/** Set place value.
 * Place to go to.
 * @param new_place new place value
 */
void
NavigatorInterface::PlaceGotoMessage::set_place(const char * new_place)
{
  strncpy(data->place, new_place, sizeof(data->place));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::PlaceGotoMessage::clone() const
{
  return new NavigatorInterface::PlaceGotoMessage(this);
}
/** @class NavigatorInterface::PlaceWithOriGotoMessage <interfaces/NavigatorInterface.h>
 * PlaceWithOriGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_place initial value for place
 * @param ini_orientation initial value for orientation
 */
NavigatorInterface::PlaceWithOriGotoMessage::PlaceWithOriGotoMessage(const char * ini_place, const float ini_orientation) : Message("PlaceWithOriGotoMessage")
{
  data_size = sizeof(PlaceWithOriGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PlaceWithOriGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->place, ini_place, 64);
  data->orientation = ini_orientation;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_STRING, "place", 64, data->place);
  add_fieldinfo(IFT_FLOAT, "orientation", 1, &data->orientation);
}
/** Constructor */
NavigatorInterface::PlaceWithOriGotoMessage::PlaceWithOriGotoMessage() : Message("PlaceWithOriGotoMessage")
{
  data_size = sizeof(PlaceWithOriGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PlaceWithOriGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_STRING, "place", 64, data->place);
  add_fieldinfo(IFT_FLOAT, "orientation", 1, &data->orientation);
}

/** Destructor */
NavigatorInterface::PlaceWithOriGotoMessage::~PlaceWithOriGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::PlaceWithOriGotoMessage::PlaceWithOriGotoMessage(const PlaceWithOriGotoMessage *m) : Message("PlaceWithOriGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (PlaceWithOriGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get place value.
 * Place to go to.
 * @return place value
 */
char *
NavigatorInterface::PlaceWithOriGotoMessage::place() const
{
  return data->place;
}

/** Get maximum length of place value.
 * @return length of place value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::PlaceWithOriGotoMessage::maxlenof_place() const
{
  return 64;
}

/** Set place value.
 * Place to go to.
 * @param new_place new place value
 */
void
NavigatorInterface::PlaceWithOriGotoMessage::set_place(const char * new_place)
{
  strncpy(data->place, new_place, sizeof(data->place));
}

/** Get orientation value.
 * The desired orientation of the robot at the target.
 * @return orientation value
 */
float
NavigatorInterface::PlaceWithOriGotoMessage::orientation() const
{
  return data->orientation;
}

/** Get maximum length of orientation value.
 * @return length of orientation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::PlaceWithOriGotoMessage::maxlenof_orientation() const
{
  return 1;
}

/** Set orientation value.
 * The desired orientation of the robot at the target.
 * @param new_orientation new orientation value
 */
void
NavigatorInterface::PlaceWithOriGotoMessage::set_orientation(const float new_orientation)
{
  data->orientation = new_orientation;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::PlaceWithOriGotoMessage::clone() const
{
  return new NavigatorInterface::PlaceWithOriGotoMessage(this);
}
/** @class NavigatorInterface::ObstacleMessage <interfaces/NavigatorInterface.h>
 * ObstacleMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_width initial value for width
 */
NavigatorInterface::ObstacleMessage::ObstacleMessage(const float ini_x, const float ini_y, const float ini_width) : Message("ObstacleMessage")
{
  data_size = sizeof(ObstacleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ObstacleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->width = ini_width;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "width", 1, &data->width);
}
/** Constructor */
NavigatorInterface::ObstacleMessage::ObstacleMessage() : Message("ObstacleMessage")
{
  data_size = sizeof(ObstacleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ObstacleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "width", 1, &data->width);
}

/** Destructor */
NavigatorInterface::ObstacleMessage::~ObstacleMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::ObstacleMessage::ObstacleMessage(const ObstacleMessage *m) : Message("ObstacleMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ObstacleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X-coordinate of the obstacle.
 * @return x value
 */
float
NavigatorInterface::ObstacleMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::ObstacleMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X-coordinate of the obstacle.
 * @param new_x new x value
 */
void
NavigatorInterface::ObstacleMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y-coordinate of the obstacle.
 * @return y value
 */
float
NavigatorInterface::ObstacleMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::ObstacleMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y-coordinate of the obstacle.
 * @param new_y new y value
 */
void
NavigatorInterface::ObstacleMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get width value.
 * Width of the obstacle.
 * @return width value
 */
float
NavigatorInterface::ObstacleMessage::width() const
{
  return data->width;
}

/** Get maximum length of width value.
 * @return length of width value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::ObstacleMessage::maxlenof_width() const
{
  return 1;
}

/** Set width value.
 * Width of the obstacle.
 * @param new_width new width value
 */
void
NavigatorInterface::ObstacleMessage::set_width(const float new_width)
{
  data->width = new_width;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::ObstacleMessage::clone() const
{
  return new NavigatorInterface::ObstacleMessage(this);
}
/** @class NavigatorInterface::ResetOdometryMessage <interfaces/NavigatorInterface.h>
 * ResetOdometryMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
NavigatorInterface::ResetOdometryMessage::ResetOdometryMessage() : Message("ResetOdometryMessage")
{
  data_size = sizeof(ResetOdometryMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ResetOdometryMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
}

/** Destructor */
NavigatorInterface::ResetOdometryMessage::~ResetOdometryMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::ResetOdometryMessage::ResetOdometryMessage(const ResetOdometryMessage *m) : Message("ResetOdometryMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ResetOdometryMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::ResetOdometryMessage::clone() const
{
  return new NavigatorInterface::ResetOdometryMessage(this);
}
/** @class NavigatorInterface::SetMaxVelocityMessage <interfaces/NavigatorInterface.h>
 * SetMaxVelocityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_max_velocity initial value for max_velocity
 */
NavigatorInterface::SetMaxVelocityMessage::SetMaxVelocityMessage(const float ini_max_velocity) : Message("SetMaxVelocityMessage")
{
  data_size = sizeof(SetMaxVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMaxVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->max_velocity = ini_max_velocity;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "max_velocity", 1, &data->max_velocity);
}
/** Constructor */
NavigatorInterface::SetMaxVelocityMessage::SetMaxVelocityMessage() : Message("SetMaxVelocityMessage")
{
  data_size = sizeof(SetMaxVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMaxVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "max_velocity", 1, &data->max_velocity);
}

/** Destructor */
NavigatorInterface::SetMaxVelocityMessage::~SetMaxVelocityMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::SetMaxVelocityMessage::SetMaxVelocityMessage(const SetMaxVelocityMessage *m) : Message("SetMaxVelocityMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetMaxVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get max_velocity value.
 * Maximum velocity
 * @return max_velocity value
 */
float
NavigatorInterface::SetMaxVelocityMessage::max_velocity() const
{
  return data->max_velocity;
}

/** Get maximum length of max_velocity value.
 * @return length of max_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::SetMaxVelocityMessage::maxlenof_max_velocity() const
{
  return 1;
}

/** Set max_velocity value.
 * Maximum velocity
 * @param new_max_velocity new max_velocity value
 */
void
NavigatorInterface::SetMaxVelocityMessage::set_max_velocity(const float new_max_velocity)
{
  data->max_velocity = new_max_velocity;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::SetMaxVelocityMessage::clone() const
{
  return new NavigatorInterface::SetMaxVelocityMessage(this);
}
/** @class NavigatorInterface::SetMaxRotationMessage <interfaces/NavigatorInterface.h>
 * SetMaxRotationMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_max_rotation initial value for max_rotation
 */
NavigatorInterface::SetMaxRotationMessage::SetMaxRotationMessage(const float ini_max_rotation) : Message("SetMaxRotationMessage")
{
  data_size = sizeof(SetMaxRotationMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMaxRotationMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->max_rotation = ini_max_rotation;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "max_rotation", 1, &data->max_rotation);
}
/** Constructor */
NavigatorInterface::SetMaxRotationMessage::SetMaxRotationMessage() : Message("SetMaxRotationMessage")
{
  data_size = sizeof(SetMaxRotationMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMaxRotationMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "max_rotation", 1, &data->max_rotation);
}

/** Destructor */
NavigatorInterface::SetMaxRotationMessage::~SetMaxRotationMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::SetMaxRotationMessage::SetMaxRotationMessage(const SetMaxRotationMessage *m) : Message("SetMaxRotationMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetMaxRotationMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get max_rotation value.
 * Maximum rotation velocity
 * @return max_rotation value
 */
float
NavigatorInterface::SetMaxRotationMessage::max_rotation() const
{
  return data->max_rotation;
}

/** Get maximum length of max_rotation value.
 * @return length of max_rotation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::SetMaxRotationMessage::maxlenof_max_rotation() const
{
  return 1;
}

/** Set max_rotation value.
 * Maximum rotation velocity
 * @param new_max_rotation new max_rotation value
 */
void
NavigatorInterface::SetMaxRotationMessage::set_max_rotation(const float new_max_rotation)
{
  data->max_rotation = new_max_rotation;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::SetMaxRotationMessage::clone() const
{
  return new NavigatorInterface::SetMaxRotationMessage(this);
}
/** @class NavigatorInterface::SetEscapingMessage <interfaces/NavigatorInterface.h>
 * SetEscapingMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_escaping_enabled initial value for escaping_enabled
 */
NavigatorInterface::SetEscapingMessage::SetEscapingMessage(const bool ini_escaping_enabled) : Message("SetEscapingMessage")
{
  data_size = sizeof(SetEscapingMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEscapingMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->escaping_enabled = ini_escaping_enabled;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_BOOL, "escaping_enabled", 1, &data->escaping_enabled);
}
/** Constructor */
NavigatorInterface::SetEscapingMessage::SetEscapingMessage() : Message("SetEscapingMessage")
{
  data_size = sizeof(SetEscapingMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEscapingMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_BOOL, "escaping_enabled", 1, &data->escaping_enabled);
}

/** Destructor */
NavigatorInterface::SetEscapingMessage::~SetEscapingMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::SetEscapingMessage::SetEscapingMessage(const SetEscapingMessage *m) : Message("SetEscapingMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetEscapingMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get escaping_enabled value.
 * This is used for navigation components with integrated collision avoidance,
      to check whether the navigator should stop when an obstacle obstructs the path, or if it should escape.
 * @return escaping_enabled value
 */
bool
NavigatorInterface::SetEscapingMessage::is_escaping_enabled() const
{
  return data->escaping_enabled;
}

/** Get maximum length of escaping_enabled value.
 * @return length of escaping_enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::SetEscapingMessage::maxlenof_escaping_enabled() const
{
  return 1;
}

/** Set escaping_enabled value.
 * This is used for navigation components with integrated collision avoidance,
      to check whether the navigator should stop when an obstacle obstructs the path, or if it should escape.
 * @param new_escaping_enabled new escaping_enabled value
 */
void
NavigatorInterface::SetEscapingMessage::set_escaping_enabled(const bool new_escaping_enabled)
{
  data->escaping_enabled = new_escaping_enabled;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::SetEscapingMessage::clone() const
{
  return new NavigatorInterface::SetEscapingMessage(this);
}
/** @class NavigatorInterface::SetSecurityDistanceMessage <interfaces/NavigatorInterface.h>
 * SetSecurityDistanceMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_security_distance initial value for security_distance
 */
NavigatorInterface::SetSecurityDistanceMessage::SetSecurityDistanceMessage(const float ini_security_distance) : Message("SetSecurityDistanceMessage")
{
  data_size = sizeof(SetSecurityDistanceMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetSecurityDistanceMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->security_distance = ini_security_distance;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "security_distance", 1, &data->security_distance);
}
/** Constructor */
NavigatorInterface::SetSecurityDistanceMessage::SetSecurityDistanceMessage() : Message("SetSecurityDistanceMessage")
{
  data_size = sizeof(SetSecurityDistanceMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetSecurityDistanceMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_FLOAT, "security_distance", 1, &data->security_distance);
}

/** Destructor */
NavigatorInterface::SetSecurityDistanceMessage::~SetSecurityDistanceMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::SetSecurityDistanceMessage::SetSecurityDistanceMessage(const SetSecurityDistanceMessage *m) : Message("SetSecurityDistanceMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetSecurityDistanceMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get security_distance value.
 * Security distance to keep to obstacles
 * @return security_distance value
 */
float
NavigatorInterface::SetSecurityDistanceMessage::security_distance() const
{
  return data->security_distance;
}

/** Get maximum length of security_distance value.
 * @return length of security_distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::SetSecurityDistanceMessage::maxlenof_security_distance() const
{
  return 1;
}

/** Set security_distance value.
 * Security distance to keep to obstacles
 * @param new_security_distance new security_distance value
 */
void
NavigatorInterface::SetSecurityDistanceMessage::set_security_distance(const float new_security_distance)
{
  data->security_distance = new_security_distance;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::SetSecurityDistanceMessage::clone() const
{
  return new NavigatorInterface::SetSecurityDistanceMessage(this);
}
/** @class NavigatorInterface::SetDriveModeMessage <interfaces/NavigatorInterface.h>
 * SetDriveModeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_drive_mode initial value for drive_mode
 */
NavigatorInterface::SetDriveModeMessage::SetDriveModeMessage(const DriveMode ini_drive_mode) : Message("SetDriveModeMessage")
{
  data_size = sizeof(SetDriveModeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetDriveModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->drive_mode = ini_drive_mode;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_ENUM, "drive_mode", 1, &data->drive_mode, "DriveMode", &enum_map_DriveMode);
}
/** Constructor */
NavigatorInterface::SetDriveModeMessage::SetDriveModeMessage() : Message("SetDriveModeMessage")
{
  data_size = sizeof(SetDriveModeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetDriveModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_ENUM, "drive_mode", 1, &data->drive_mode, "DriveMode", &enum_map_DriveMode);
}

/** Destructor */
NavigatorInterface::SetDriveModeMessage::~SetDriveModeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::SetDriveModeMessage::SetDriveModeMessage(const SetDriveModeMessage *m) : Message("SetDriveModeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetDriveModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get drive_mode value.
 * Current drive mode
 * @return drive_mode value
 */
NavigatorInterface::DriveMode
NavigatorInterface::SetDriveModeMessage::drive_mode() const
{
  return (NavigatorInterface::DriveMode)data->drive_mode;
}

/** Get maximum length of drive_mode value.
 * @return length of drive_mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::SetDriveModeMessage::maxlenof_drive_mode() const
{
  return 1;
}

/** Set drive_mode value.
 * Current drive mode
 * @param new_drive_mode new drive_mode value
 */
void
NavigatorInterface::SetDriveModeMessage::set_drive_mode(const DriveMode new_drive_mode)
{
  data->drive_mode = new_drive_mode;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::SetDriveModeMessage::clone() const
{
  return new NavigatorInterface::SetDriveModeMessage(this);
}
/** @class NavigatorInterface::SetStopAtTargetMessage <interfaces/NavigatorInterface.h>
 * SetStopAtTargetMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_stop_at_target initial value for stop_at_target
 */
NavigatorInterface::SetStopAtTargetMessage::SetStopAtTargetMessage(const bool ini_stop_at_target) : Message("SetStopAtTargetMessage")
{
  data_size = sizeof(SetStopAtTargetMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetStopAtTargetMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->stop_at_target = ini_stop_at_target;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_BOOL, "stop_at_target", 1, &data->stop_at_target);
}
/** Constructor */
NavigatorInterface::SetStopAtTargetMessage::SetStopAtTargetMessage() : Message("SetStopAtTargetMessage")
{
  data_size = sizeof(SetStopAtTargetMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetStopAtTargetMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_BOOL, "stop_at_target", 1, &data->stop_at_target);
}

/** Destructor */
NavigatorInterface::SetStopAtTargetMessage::~SetStopAtTargetMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::SetStopAtTargetMessage::SetStopAtTargetMessage(const SetStopAtTargetMessage *m) : Message("SetStopAtTargetMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetStopAtTargetMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get stop_at_target value.
 * Stop when target is reached?
 * @return stop_at_target value
 */
bool
NavigatorInterface::SetStopAtTargetMessage::is_stop_at_target() const
{
  return data->stop_at_target;
}

/** Get maximum length of stop_at_target value.
 * @return length of stop_at_target value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::SetStopAtTargetMessage::maxlenof_stop_at_target() const
{
  return 1;
}

/** Set stop_at_target value.
 * Stop when target is reached?
 * @param new_stop_at_target new stop_at_target value
 */
void
NavigatorInterface::SetStopAtTargetMessage::set_stop_at_target(const bool new_stop_at_target)
{
  data->stop_at_target = new_stop_at_target;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::SetStopAtTargetMessage::clone() const
{
  return new NavigatorInterface::SetStopAtTargetMessage(this);
}
/** @class NavigatorInterface::SetOrientationModeMessage <interfaces/NavigatorInterface.h>
 * SetOrientationModeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_orientation_mode initial value for orientation_mode
 */
NavigatorInterface::SetOrientationModeMessage::SetOrientationModeMessage(const OrientationMode ini_orientation_mode) : Message("SetOrientationModeMessage")
{
  data_size = sizeof(SetOrientationModeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetOrientationModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->orientation_mode = ini_orientation_mode;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_ENUM, "orientation_mode", 1, &data->orientation_mode, "OrientationMode", &enum_map_OrientationMode);
}
/** Constructor */
NavigatorInterface::SetOrientationModeMessage::SetOrientationModeMessage() : Message("SetOrientationModeMessage")
{
  data_size = sizeof(SetOrientationModeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetOrientationModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
  add_fieldinfo(IFT_ENUM, "orientation_mode", 1, &data->orientation_mode, "OrientationMode", &enum_map_OrientationMode);
}

/** Destructor */
NavigatorInterface::SetOrientationModeMessage::~SetOrientationModeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::SetOrientationModeMessage::SetOrientationModeMessage(const SetOrientationModeMessage *m) : Message("SetOrientationModeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetOrientationModeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get orientation_mode value.
 * Mode how/when to orientate if orientation is given
 * @return orientation_mode value
 */
NavigatorInterface::OrientationMode
NavigatorInterface::SetOrientationModeMessage::orientation_mode() const
{
  return (NavigatorInterface::OrientationMode)data->orientation_mode;
}

/** Get maximum length of orientation_mode value.
 * @return length of orientation_mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::SetOrientationModeMessage::maxlenof_orientation_mode() const
{
  return 1;
}

/** Set orientation_mode value.
 * Mode how/when to orientate if orientation is given
 * @param new_orientation_mode new orientation_mode value
 */
void
NavigatorInterface::SetOrientationModeMessage::set_orientation_mode(const OrientationMode new_orientation_mode)
{
  data->orientation_mode = new_orientation_mode;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::SetOrientationModeMessage::clone() const
{
  return new NavigatorInterface::SetOrientationModeMessage(this);
}
/** @class NavigatorInterface::ResetParametersMessage <interfaces/NavigatorInterface.h>
 * ResetParametersMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
NavigatorInterface::ResetParametersMessage::ResetParametersMessage() : Message("ResetParametersMessage")
{
  data_size = sizeof(ResetParametersMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ResetParametersMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_DriveMode[(int)MovingNotAllowed] = "MovingNotAllowed";
  enum_map_DriveMode[(int)Forward] = "Forward";
  enum_map_DriveMode[(int)AllowBackward] = "AllowBackward";
  enum_map_DriveMode[(int)Backward] = "Backward";
  enum_map_DriveMode[(int)ESCAPE] = "ESCAPE";
  enum_map_OrientationMode[(int)OrientAtTarget] = "OrientAtTarget";
  enum_map_OrientationMode[(int)OrientDuringTravel] = "OrientDuringTravel";
}

/** Destructor */
NavigatorInterface::ResetParametersMessage::~ResetParametersMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::ResetParametersMessage::ResetParametersMessage(const ResetParametersMessage *m) : Message("ResetParametersMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ResetParametersMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::ResetParametersMessage::clone() const
{
  return new NavigatorInterface::ResetParametersMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
NavigatorInterface::message_valid(const Message *message) const
{
  const StopMessage *m0 = dynamic_cast<const StopMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const TurnMessage *m1 = dynamic_cast<const TurnMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const CartesianGotoMessage *m2 = dynamic_cast<const CartesianGotoMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const CartesianGotoWithFrameMessage *m3 = dynamic_cast<const CartesianGotoWithFrameMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const PolarGotoMessage *m4 = dynamic_cast<const PolarGotoMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const PlaceGotoMessage *m5 = dynamic_cast<const PlaceGotoMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const PlaceWithOriGotoMessage *m6 = dynamic_cast<const PlaceWithOriGotoMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const ObstacleMessage *m7 = dynamic_cast<const ObstacleMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const ResetOdometryMessage *m8 = dynamic_cast<const ResetOdometryMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  const SetMaxVelocityMessage *m9 = dynamic_cast<const SetMaxVelocityMessage *>(message);
  if ( m9 != NULL ) {
    return true;
  }
  const SetMaxRotationMessage *m10 = dynamic_cast<const SetMaxRotationMessage *>(message);
  if ( m10 != NULL ) {
    return true;
  }
  const SetEscapingMessage *m11 = dynamic_cast<const SetEscapingMessage *>(message);
  if ( m11 != NULL ) {
    return true;
  }
  const SetSecurityDistanceMessage *m12 = dynamic_cast<const SetSecurityDistanceMessage *>(message);
  if ( m12 != NULL ) {
    return true;
  }
  const SetDriveModeMessage *m13 = dynamic_cast<const SetDriveModeMessage *>(message);
  if ( m13 != NULL ) {
    return true;
  }
  const SetStopAtTargetMessage *m14 = dynamic_cast<const SetStopAtTargetMessage *>(message);
  if ( m14 != NULL ) {
    return true;
  }
  const SetOrientationModeMessage *m15 = dynamic_cast<const SetOrientationModeMessage *>(message);
  if ( m15 != NULL ) {
    return true;
  }
  const ResetParametersMessage *m16 = dynamic_cast<const ResetParametersMessage *>(message);
  if ( m16 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NavigatorInterface)
/// @endcond


} // end namespace fawkes
