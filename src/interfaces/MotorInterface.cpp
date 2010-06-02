
/***************************************************************************
 *  MotorInterface.cpp - Fawkes BlackBoard Interface - MotorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Martin Liebenberg, Tim Niemueller
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

#include <interfaces/MotorInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class MotorInterface <interfaces/MotorInterface.h>
 * MotorInterface Fawkes BlackBoard Interface.
 * This interface is currently prepared best for a holonomic robot.
      It will need modifications or a split to support differential drives.
    
 * @ingroup FawkesInterfaces
 */


/** MOTOR_ENABLED constant */
const uint32_t MotorInterface::MOTOR_ENABLED = 0u;
/** MOTOR_DISABLED constant */
const uint32_t MotorInterface::MOTOR_DISABLED = 1u;
/** DRIVE_MODE_RPM constant */
const uint32_t MotorInterface::DRIVE_MODE_RPM = 1u;
/** DRIVE_MODE_TRANS constant */
const uint32_t MotorInterface::DRIVE_MODE_TRANS = 2u;
/** DRIVE_MODE_ROT constant */
const uint32_t MotorInterface::DRIVE_MODE_ROT = 3u;
/** DRIVE_MODE_TRANS_ROT constant */
const uint32_t MotorInterface::DRIVE_MODE_TRANS_ROT = 4u;
/** DRIVE_MODE_ORBIT constant */
const uint32_t MotorInterface::DRIVE_MODE_ORBIT = 5u;
/** DRIVE_MODE_LINE_TRANS_ROT constant */
const uint32_t MotorInterface::DRIVE_MODE_LINE_TRANS_ROT = 6u;

/** Constructor */
MotorInterface::MotorInterface() : Interface()
{
  data_size = sizeof(MotorInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (MotorInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "motor_state", 1, &data->motor_state);
  add_fieldinfo(IFT_UINT32, "drive_mode", 1, &data->drive_mode);
  add_fieldinfo(IFT_INT32, "right_rpm", 1, &data->right_rpm);
  add_fieldinfo(IFT_INT32, "rear_rpm", 1, &data->rear_rpm);
  add_fieldinfo(IFT_INT32, "left_rpm", 1, &data->left_rpm);
  add_fieldinfo(IFT_FLOAT, "odometry_path_length", 1, &data->odometry_path_length);
  add_fieldinfo(IFT_FLOAT, "odometry_position_x", 1, &data->odometry_position_x);
  add_fieldinfo(IFT_FLOAT, "odometry_position_y", 1, &data->odometry_position_y);
  add_fieldinfo(IFT_FLOAT, "odometry_orientation", 1, &data->odometry_orientation);
  add_fieldinfo(IFT_FLOAT, "vx", 1, &data->vx);
  add_fieldinfo(IFT_FLOAT, "vy", 1, &data->vy);
  add_fieldinfo(IFT_FLOAT, "omega", 1, &data->omega);
  add_fieldinfo(IFT_UINT32, "controller", 1, &data->controller);
  add_fieldinfo(IFT_STRING, "controller_thread_name", 64, data->controller_thread_name);
  add_messageinfo("SetMotorStateMessage");
  add_messageinfo("AcquireControlMessage");
  add_messageinfo("ResetOdometryMessage");
  add_messageinfo("DriveRPMMessage");
  add_messageinfo("GotoMessage");
  add_messageinfo("TransMessage");
  add_messageinfo("RotMessage");
  add_messageinfo("TransRotMessage");
  add_messageinfo("OrbitMessage");
  add_messageinfo("LinTransRotMessage");
  unsigned char tmp_hash[] = {0x4d, 0x9b, 0xb8, 0x1e, 0xb7, 0x9e, 0xc1, 0xea, 0xc6, 0xad, 0xf9, 0x6, 0xd7, 0x6d, 0xa4, 0xa5};
  set_hash(tmp_hash);
}

/** Destructor */
MotorInterface::~MotorInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get motor_state value.
 * 
      The current state of the motor.
    
 * @return motor_state value
 */
uint32_t
MotorInterface::motor_state() const
{
  return data->motor_state;
}

/** Get maximum length of motor_state value.
 * @return length of motor_state value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_motor_state() const
{
  return 1;
}

/** Set motor_state value.
 * 
      The current state of the motor.
    
 * @param new_motor_state new motor_state value
 */
void
MotorInterface::set_motor_state(const uint32_t new_motor_state)
{
  data->motor_state = new_motor_state;
  data_changed = true;
}

/** Get drive_mode value.
 * 
      The current drive mode of the motor.
    
 * @return drive_mode value
 */
uint32_t
MotorInterface::drive_mode() const
{
  return data->drive_mode;
}

/** Get maximum length of drive_mode value.
 * @return length of drive_mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_drive_mode() const
{
  return 1;
}

/** Set drive_mode value.
 * 
      The current drive mode of the motor.
    
 * @param new_drive_mode new drive_mode value
 */
void
MotorInterface::set_drive_mode(const uint32_t new_drive_mode)
{
  data->drive_mode = new_drive_mode;
  data_changed = true;
}

/** Get right_rpm value.
 * 
      RPM of the motor on the right front of the robot.
    
 * @return right_rpm value
 */
int32_t
MotorInterface::right_rpm() const
{
  return data->right_rpm;
}

/** Get maximum length of right_rpm value.
 * @return length of right_rpm value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_right_rpm() const
{
  return 1;
}

/** Set right_rpm value.
 * 
      RPM of the motor on the right front of the robot.
    
 * @param new_right_rpm new right_rpm value
 */
void
MotorInterface::set_right_rpm(const int32_t new_right_rpm)
{
  data->right_rpm = new_right_rpm;
  data_changed = true;
}

/** Get rear_rpm value.
 * 
      RPM of motor on the rear of the robot.
    
 * @return rear_rpm value
 */
int32_t
MotorInterface::rear_rpm() const
{
  return data->rear_rpm;
}

/** Get maximum length of rear_rpm value.
 * @return length of rear_rpm value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_rear_rpm() const
{
  return 1;
}

/** Set rear_rpm value.
 * 
      RPM of motor on the rear of the robot.
    
 * @param new_rear_rpm new rear_rpm value
 */
void
MotorInterface::set_rear_rpm(const int32_t new_rear_rpm)
{
  data->rear_rpm = new_rear_rpm;
  data_changed = true;
}

/** Get left_rpm value.
 * 
      RPM of the motor on the left front of the robot.
    
 * @return left_rpm value
 */
int32_t
MotorInterface::left_rpm() const
{
  return data->left_rpm;
}

/** Get maximum length of left_rpm value.
 * @return length of left_rpm value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_left_rpm() const
{
  return 1;
}

/** Set left_rpm value.
 * 
      RPM of the motor on the left front of the robot.
    
 * @param new_left_rpm new left_rpm value
 */
void
MotorInterface::set_left_rpm(const int32_t new_left_rpm)
{
  data->left_rpm = new_left_rpm;
  data_changed = true;
}

/** Get odometry_path_length value.
 * 
      The actual length of the robot's trajectory since the last ResetOdometry.
    
 * @return odometry_path_length value
 */
float
MotorInterface::odometry_path_length() const
{
  return data->odometry_path_length;
}

/** Get maximum length of odometry_path_length value.
 * @return length of odometry_path_length value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_odometry_path_length() const
{
  return 1;
}

/** Set odometry_path_length value.
 * 
      The actual length of the robot's trajectory since the last ResetOdometry.
    
 * @param new_odometry_path_length new odometry_path_length value
 */
void
MotorInterface::set_odometry_path_length(const float new_odometry_path_length)
{
  data->odometry_path_length = new_odometry_path_length;
  data_changed = true;
}

/** Get odometry_position_x value.
 * 
      The actual position of the robot relative to the position at the last ResetOdometry.
    
 * @return odometry_position_x value
 */
float
MotorInterface::odometry_position_x() const
{
  return data->odometry_position_x;
}

/** Get maximum length of odometry_position_x value.
 * @return length of odometry_position_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_odometry_position_x() const
{
  return 1;
}

/** Set odometry_position_x value.
 * 
      The actual position of the robot relative to the position at the last ResetOdometry.
    
 * @param new_odometry_position_x new odometry_position_x value
 */
void
MotorInterface::set_odometry_position_x(const float new_odometry_position_x)
{
  data->odometry_position_x = new_odometry_position_x;
  data_changed = true;
}

/** Get odometry_position_y value.
 * 
      The actual position of the robot relative to the position at the last ResetOdometry.
    
 * @return odometry_position_y value
 */
float
MotorInterface::odometry_position_y() const
{
  return data->odometry_position_y;
}

/** Get maximum length of odometry_position_y value.
 * @return length of odometry_position_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_odometry_position_y() const
{
  return 1;
}

/** Set odometry_position_y value.
 * 
      The actual position of the robot relative to the position at the last ResetOdometry.
    
 * @param new_odometry_position_y new odometry_position_y value
 */
void
MotorInterface::set_odometry_position_y(const float new_odometry_position_y)
{
  data->odometry_position_y = new_odometry_position_y;
  data_changed = true;
}

/** Get odometry_orientation value.
 * 
      The actual orientation of the robot relative to the orientation at the last ResetOdometry.
    
 * @return odometry_orientation value
 */
float
MotorInterface::odometry_orientation() const
{
  return data->odometry_orientation;
}

/** Get maximum length of odometry_orientation value.
 * @return length of odometry_orientation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_odometry_orientation() const
{
  return 1;
}

/** Set odometry_orientation value.
 * 
      The actual orientation of the robot relative to the orientation at the last ResetOdometry.
    
 * @param new_odometry_orientation new odometry_orientation value
 */
void
MotorInterface::set_odometry_orientation(const float new_odometry_orientation)
{
  data->odometry_orientation = new_odometry_orientation;
  data_changed = true;
}

/** Get vx value.
 * 
      VX of the robot in m/s. Forward.
    
 * @return vx value
 */
float
MotorInterface::vx() const
{
  return data->vx;
}

/** Get maximum length of vx value.
 * @return length of vx value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_vx() const
{
  return 1;
}

/** Set vx value.
 * 
      VX of the robot in m/s. Forward.
    
 * @param new_vx new vx value
 */
void
MotorInterface::set_vx(const float new_vx)
{
  data->vx = new_vx;
  data_changed = true;
}

/** Get vy value.
 * 
      VY of the robot in m/s. Left.
    
 * @return vy value
 */
float
MotorInterface::vy() const
{
  return data->vy;
}

/** Get maximum length of vy value.
 * @return length of vy value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_vy() const
{
  return 1;
}

/** Set vy value.
 * 
      VY of the robot in m/s. Left.
    
 * @param new_vy new vy value
 */
void
MotorInterface::set_vy(const float new_vy)
{
  data->vy = new_vy;
  data_changed = true;
}

/** Get omega value.
 * 
      Rotation speed of the robot in rad/s.
    
 * @return omega value
 */
float
MotorInterface::omega() const
{
  return data->omega;
}

/** Get maximum length of omega value.
 * @return length of omega value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_omega() const
{
  return 1;
}

/** Set omega value.
 * 
      Rotation speed of the robot in rad/s.
    
 * @param new_omega new omega value
 */
void
MotorInterface::set_omega(const float new_omega)
{
  data->omega = new_omega;
  data_changed = true;
}

/** Get controller value.
 * 
     The ID of the controller. The controller ID is the instance serial of the sending
     interface. Only from this interface instance command messages are accepted.
    
 * @return controller value
 */
uint32_t
MotorInterface::controller() const
{
  return data->controller;
}

/** Get maximum length of controller value.
 * @return length of controller value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_controller() const
{
  return 1;
}

/** Set controller value.
 * 
     The ID of the controller. The controller ID is the instance serial of the sending
     interface. Only from this interface instance command messages are accepted.
    
 * @param new_controller new controller value
 */
void
MotorInterface::set_controller(const uint32_t new_controller)
{
  data->controller = new_controller;
  data_changed = true;
}

/** Get controller_thread_name value.
 * 
     The name of the controlling thread, for easier debugging. This is informative only
     and actually two threads may share an interface instance (although this should be
     avoided since the interface locking has to be reproduced for these threads then).
  
 * @return controller_thread_name value
 */
char *
MotorInterface::controller_thread_name() const
{
  return data->controller_thread_name;
}

/** Get maximum length of controller_thread_name value.
 * @return length of controller_thread_name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::maxlenof_controller_thread_name() const
{
  return 64;
}

/** Set controller_thread_name value.
 * 
     The name of the controlling thread, for easier debugging. This is informative only
     and actually two threads may share an interface instance (although this should be
     avoided since the interface locking has to be reproduced for these threads then).
  
 * @param new_controller_thread_name new controller_thread_name value
 */
void
MotorInterface::set_controller_thread_name(const char * new_controller_thread_name)
{
  strncpy(data->controller_thread_name, new_controller_thread_name, sizeof(data->controller_thread_name));
  data_changed = true;
}

/* =========== message create =========== */
Message *
MotorInterface::create_message(const char *type) const
{
  if ( strncmp("SetMotorStateMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMotorStateMessage();
  } else if ( strncmp("AcquireControlMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AcquireControlMessage();
  } else if ( strncmp("ResetOdometryMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ResetOdometryMessage();
  } else if ( strncmp("DriveRPMMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DriveRPMMessage();
  } else if ( strncmp("GotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GotoMessage();
  } else if ( strncmp("TransMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TransMessage();
  } else if ( strncmp("RotMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RotMessage();
  } else if ( strncmp("TransRotMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TransRotMessage();
  } else if ( strncmp("OrbitMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new OrbitMessage();
  } else if ( strncmp("LinTransRotMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new LinTransRotMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
MotorInterface::copy_values(const Interface *other)
{
  const MotorInterface *oi = dynamic_cast<const MotorInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(MotorInterface_data_t));
}

const char *
MotorInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class MotorInterface::SetMotorStateMessage <interfaces/MotorInterface.h>
 * SetMotorStateMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_motor_state initial value for motor_state
 */
MotorInterface::SetMotorStateMessage::SetMotorStateMessage(const uint32_t ini_motor_state) : Message("SetMotorStateMessage")
{
  data_size = sizeof(SetMotorStateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMotorStateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->motor_state = ini_motor_state;
  add_fieldinfo(IFT_UINT32, "motor_state", 1, &data->motor_state);
}
/** Constructor */
MotorInterface::SetMotorStateMessage::SetMotorStateMessage() : Message("SetMotorStateMessage")
{
  data_size = sizeof(SetMotorStateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMotorStateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "motor_state", 1, &data->motor_state);
}

/** Destructor */
MotorInterface::SetMotorStateMessage::~SetMotorStateMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MotorInterface::SetMotorStateMessage::SetMotorStateMessage(const SetMotorStateMessage *m) : Message("SetMotorStateMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetMotorStateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get motor_state value.
 * 
      The new motor state to set. Use the MOTOR_* constants.
    
 * @return motor_state value
 */
uint32_t
MotorInterface::SetMotorStateMessage::motor_state() const
{
  return data->motor_state;
}

/** Get maximum length of motor_state value.
 * @return length of motor_state value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::SetMotorStateMessage::maxlenof_motor_state() const
{
  return 1;
}

/** Set motor_state value.
 * 
      The new motor state to set. Use the MOTOR_* constants.
    
 * @param new_motor_state new motor_state value
 */
void
MotorInterface::SetMotorStateMessage::set_motor_state(const uint32_t new_motor_state)
{
  data->motor_state = new_motor_state;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MotorInterface::SetMotorStateMessage::clone() const
{
  return new MotorInterface::SetMotorStateMessage(this);
}
/** @class MotorInterface::AcquireControlMessage <interfaces/MotorInterface.h>
 * AcquireControlMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_controller initial value for controller
 * @param ini_controller_thread_name initial value for controller_thread_name
 */
MotorInterface::AcquireControlMessage::AcquireControlMessage(const uint32_t ini_controller, const char * ini_controller_thread_name) : Message("AcquireControlMessage")
{
  data_size = sizeof(AcquireControlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AcquireControlMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->controller = ini_controller;
  strncpy(data->controller_thread_name, ini_controller_thread_name, 64);
  add_fieldinfo(IFT_UINT32, "controller", 1, &data->controller);
  add_fieldinfo(IFT_STRING, "controller_thread_name", 64, data->controller_thread_name);
}
/** Constructor */
MotorInterface::AcquireControlMessage::AcquireControlMessage() : Message("AcquireControlMessage")
{
  data_size = sizeof(AcquireControlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AcquireControlMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "controller", 1, &data->controller);
  add_fieldinfo(IFT_STRING, "controller_thread_name", 64, data->controller_thread_name);
}

/** Destructor */
MotorInterface::AcquireControlMessage::~AcquireControlMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MotorInterface::AcquireControlMessage::AcquireControlMessage(const AcquireControlMessage *m) : Message("AcquireControlMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AcquireControlMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get controller value.
 * 
     The ID of the controller. The controller ID is the instance serial of the sending
     interface. Only from this interface instance command messages are accepted.
    
 * @return controller value
 */
uint32_t
MotorInterface::AcquireControlMessage::controller() const
{
  return data->controller;
}

/** Get maximum length of controller value.
 * @return length of controller value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::AcquireControlMessage::maxlenof_controller() const
{
  return 1;
}

/** Set controller value.
 * 
     The ID of the controller. The controller ID is the instance serial of the sending
     interface. Only from this interface instance command messages are accepted.
    
 * @param new_controller new controller value
 */
void
MotorInterface::AcquireControlMessage::set_controller(const uint32_t new_controller)
{
  data->controller = new_controller;
}

/** Get controller_thread_name value.
 * 
     The name of the controlling thread, for easier debugging. This is informative only
     and actually two threads may share an interface instance (although this should be
     avoided since the interface locking has to be reproduced for these threads then).
  
 * @return controller_thread_name value
 */
char *
MotorInterface::AcquireControlMessage::controller_thread_name() const
{
  return data->controller_thread_name;
}

/** Get maximum length of controller_thread_name value.
 * @return length of controller_thread_name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::AcquireControlMessage::maxlenof_controller_thread_name() const
{
  return 64;
}

/** Set controller_thread_name value.
 * 
     The name of the controlling thread, for easier debugging. This is informative only
     and actually two threads may share an interface instance (although this should be
     avoided since the interface locking has to be reproduced for these threads then).
  
 * @param new_controller_thread_name new controller_thread_name value
 */
void
MotorInterface::AcquireControlMessage::set_controller_thread_name(const char * new_controller_thread_name)
{
  strncpy(data->controller_thread_name, new_controller_thread_name, sizeof(data->controller_thread_name));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MotorInterface::AcquireControlMessage::clone() const
{
  return new MotorInterface::AcquireControlMessage(this);
}
/** @class MotorInterface::ResetOdometryMessage <interfaces/MotorInterface.h>
 * ResetOdometryMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
MotorInterface::ResetOdometryMessage::ResetOdometryMessage() : Message("ResetOdometryMessage")
{
  data_size = sizeof(ResetOdometryMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ResetOdometryMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
MotorInterface::ResetOdometryMessage::~ResetOdometryMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MotorInterface::ResetOdometryMessage::ResetOdometryMessage(const ResetOdometryMessage *m) : Message("ResetOdometryMessage")
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
MotorInterface::ResetOdometryMessage::clone() const
{
  return new MotorInterface::ResetOdometryMessage(this);
}
/** @class MotorInterface::DriveRPMMessage <interfaces/MotorInterface.h>
 * DriveRPMMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_front_right initial value for front_right
 * @param ini_front_left initial value for front_left
 * @param ini_rear initial value for rear
 */
MotorInterface::DriveRPMMessage::DriveRPMMessage(const float ini_front_right, const float ini_front_left, const float ini_rear) : Message("DriveRPMMessage")
{
  data_size = sizeof(DriveRPMMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DriveRPMMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->front_right = ini_front_right;
  data->front_left = ini_front_left;
  data->rear = ini_rear;
  add_fieldinfo(IFT_FLOAT, "front_right", 1, &data->front_right);
  add_fieldinfo(IFT_FLOAT, "front_left", 1, &data->front_left);
  add_fieldinfo(IFT_FLOAT, "rear", 1, &data->rear);
}
/** Constructor */
MotorInterface::DriveRPMMessage::DriveRPMMessage() : Message("DriveRPMMessage")
{
  data_size = sizeof(DriveRPMMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DriveRPMMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "front_right", 1, &data->front_right);
  add_fieldinfo(IFT_FLOAT, "front_left", 1, &data->front_left);
  add_fieldinfo(IFT_FLOAT, "rear", 1, &data->rear);
}

/** Destructor */
MotorInterface::DriveRPMMessage::~DriveRPMMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MotorInterface::DriveRPMMessage::DriveRPMMessage(const DriveRPMMessage *m) : Message("DriveRPMMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (DriveRPMMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get front_right value.
 * Rotation in RPM of the right front wheel.
 * @return front_right value
 */
float
MotorInterface::DriveRPMMessage::front_right() const
{
  return data->front_right;
}

/** Get maximum length of front_right value.
 * @return length of front_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::DriveRPMMessage::maxlenof_front_right() const
{
  return 1;
}

/** Set front_right value.
 * Rotation in RPM of the right front wheel.
 * @param new_front_right new front_right value
 */
void
MotorInterface::DriveRPMMessage::set_front_right(const float new_front_right)
{
  data->front_right = new_front_right;
}

/** Get front_left value.
 * Rotation in RPM of the left front wheel.
 * @return front_left value
 */
float
MotorInterface::DriveRPMMessage::front_left() const
{
  return data->front_left;
}

/** Get maximum length of front_left value.
 * @return length of front_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::DriveRPMMessage::maxlenof_front_left() const
{
  return 1;
}

/** Set front_left value.
 * Rotation in RPM of the left front wheel.
 * @param new_front_left new front_left value
 */
void
MotorInterface::DriveRPMMessage::set_front_left(const float new_front_left)
{
  data->front_left = new_front_left;
}

/** Get rear value.
 * Rotation in RPM of the rear wheel.
 * @return rear value
 */
float
MotorInterface::DriveRPMMessage::rear() const
{
  return data->rear;
}

/** Get maximum length of rear value.
 * @return length of rear value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::DriveRPMMessage::maxlenof_rear() const
{
  return 1;
}

/** Set rear value.
 * Rotation in RPM of the rear wheel.
 * @param new_rear new rear value
 */
void
MotorInterface::DriveRPMMessage::set_rear(const float new_rear)
{
  data->rear = new_rear;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MotorInterface::DriveRPMMessage::clone() const
{
  return new MotorInterface::DriveRPMMessage(this);
}
/** @class MotorInterface::GotoMessage <interfaces/MotorInterface.h>
 * GotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_phi initial value for phi
 * @param ini_time_sec initial value for time_sec
 */
MotorInterface::GotoMessage::GotoMessage(const float ini_x, const float ini_y, const float ini_phi, const float ini_time_sec) : Message("GotoMessage")
{
  data_size = sizeof(GotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->phi = ini_phi;
  data->time_sec = ini_time_sec;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "phi", 1, &data->phi);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
}
/** Constructor */
MotorInterface::GotoMessage::GotoMessage() : Message("GotoMessage")
{
  data_size = sizeof(GotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "phi", 1, &data->phi);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
}

/** Destructor */
MotorInterface::GotoMessage::~GotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MotorInterface::GotoMessage::GotoMessage(const GotoMessage *m) : Message("GotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X distance in m.
 * @return x value
 */
float
MotorInterface::GotoMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::GotoMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X distance in m.
 * @param new_x new x value
 */
void
MotorInterface::GotoMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y distance in m.
 * @return y value
 */
float
MotorInterface::GotoMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::GotoMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y distance in m.
 * @param new_y new y value
 */
void
MotorInterface::GotoMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get phi value.
 * Angle relative to current angle in rad.
 * @return phi value
 */
float
MotorInterface::GotoMessage::phi() const
{
  return data->phi;
}

/** Get maximum length of phi value.
 * @return length of phi value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::GotoMessage::maxlenof_phi() const
{
  return 1;
}

/** Set phi value.
 * Angle relative to current angle in rad.
 * @param new_phi new phi value
 */
void
MotorInterface::GotoMessage::set_phi(const float new_phi)
{
  data->phi = new_phi;
}

/** Get time_sec value.
 * When to reach the desired location.
 * @return time_sec value
 */
float
MotorInterface::GotoMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::GotoMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * When to reach the desired location.
 * @param new_time_sec new time_sec value
 */
void
MotorInterface::GotoMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MotorInterface::GotoMessage::clone() const
{
  return new MotorInterface::GotoMessage(this);
}
/** @class MotorInterface::TransMessage <interfaces/MotorInterface.h>
 * TransMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_vx initial value for vx
 * @param ini_vy initial value for vy
 */
MotorInterface::TransMessage::TransMessage(const float ini_vx, const float ini_vy) : Message("TransMessage")
{
  data_size = sizeof(TransMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->vx = ini_vx;
  data->vy = ini_vy;
  add_fieldinfo(IFT_FLOAT, "vx", 1, &data->vx);
  add_fieldinfo(IFT_FLOAT, "vy", 1, &data->vy);
}
/** Constructor */
MotorInterface::TransMessage::TransMessage() : Message("TransMessage")
{
  data_size = sizeof(TransMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "vx", 1, &data->vx);
  add_fieldinfo(IFT_FLOAT, "vy", 1, &data->vy);
}

/** Destructor */
MotorInterface::TransMessage::~TransMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MotorInterface::TransMessage::TransMessage(const TransMessage *m) : Message("TransMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TransMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get vx value.
 * Speed in X direction in m/s.
 * @return vx value
 */
float
MotorInterface::TransMessage::vx() const
{
  return data->vx;
}

/** Get maximum length of vx value.
 * @return length of vx value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::TransMessage::maxlenof_vx() const
{
  return 1;
}

/** Set vx value.
 * Speed in X direction in m/s.
 * @param new_vx new vx value
 */
void
MotorInterface::TransMessage::set_vx(const float new_vx)
{
  data->vx = new_vx;
}

/** Get vy value.
 * Speed in Y direction in m/s.
 * @return vy value
 */
float
MotorInterface::TransMessage::vy() const
{
  return data->vy;
}

/** Get maximum length of vy value.
 * @return length of vy value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::TransMessage::maxlenof_vy() const
{
  return 1;
}

/** Set vy value.
 * Speed in Y direction in m/s.
 * @param new_vy new vy value
 */
void
MotorInterface::TransMessage::set_vy(const float new_vy)
{
  data->vy = new_vy;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MotorInterface::TransMessage::clone() const
{
  return new MotorInterface::TransMessage(this);
}
/** @class MotorInterface::RotMessage <interfaces/MotorInterface.h>
 * RotMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_omega initial value for omega
 */
MotorInterface::RotMessage::RotMessage(const float ini_omega) : Message("RotMessage")
{
  data_size = sizeof(RotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RotMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->omega = ini_omega;
  add_fieldinfo(IFT_FLOAT, "omega", 1, &data->omega);
}
/** Constructor */
MotorInterface::RotMessage::RotMessage() : Message("RotMessage")
{
  data_size = sizeof(RotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RotMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "omega", 1, &data->omega);
}

/** Destructor */
MotorInterface::RotMessage::~RotMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MotorInterface::RotMessage::RotMessage(const RotMessage *m) : Message("RotMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (RotMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get omega value.
 * Angle rotation in rad/s.
 * @return omega value
 */
float
MotorInterface::RotMessage::omega() const
{
  return data->omega;
}

/** Get maximum length of omega value.
 * @return length of omega value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::RotMessage::maxlenof_omega() const
{
  return 1;
}

/** Set omega value.
 * Angle rotation in rad/s.
 * @param new_omega new omega value
 */
void
MotorInterface::RotMessage::set_omega(const float new_omega)
{
  data->omega = new_omega;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MotorInterface::RotMessage::clone() const
{
  return new MotorInterface::RotMessage(this);
}
/** @class MotorInterface::TransRotMessage <interfaces/MotorInterface.h>
 * TransRotMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_vx initial value for vx
 * @param ini_vy initial value for vy
 * @param ini_omega initial value for omega
 */
MotorInterface::TransRotMessage::TransRotMessage(const float ini_vx, const float ini_vy, const float ini_omega) : Message("TransRotMessage")
{
  data_size = sizeof(TransRotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransRotMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->vx = ini_vx;
  data->vy = ini_vy;
  data->omega = ini_omega;
  add_fieldinfo(IFT_FLOAT, "vx", 1, &data->vx);
  add_fieldinfo(IFT_FLOAT, "vy", 1, &data->vy);
  add_fieldinfo(IFT_FLOAT, "omega", 1, &data->omega);
}
/** Constructor */
MotorInterface::TransRotMessage::TransRotMessage() : Message("TransRotMessage")
{
  data_size = sizeof(TransRotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransRotMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "vx", 1, &data->vx);
  add_fieldinfo(IFT_FLOAT, "vy", 1, &data->vy);
  add_fieldinfo(IFT_FLOAT, "omega", 1, &data->omega);
}

/** Destructor */
MotorInterface::TransRotMessage::~TransRotMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MotorInterface::TransRotMessage::TransRotMessage(const TransRotMessage *m) : Message("TransRotMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TransRotMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get vx value.
 * Speed in X direction in m/s.
 * @return vx value
 */
float
MotorInterface::TransRotMessage::vx() const
{
  return data->vx;
}

/** Get maximum length of vx value.
 * @return length of vx value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::TransRotMessage::maxlenof_vx() const
{
  return 1;
}

/** Set vx value.
 * Speed in X direction in m/s.
 * @param new_vx new vx value
 */
void
MotorInterface::TransRotMessage::set_vx(const float new_vx)
{
  data->vx = new_vx;
}

/** Get vy value.
 * Speed in Y direction in m/s.
 * @return vy value
 */
float
MotorInterface::TransRotMessage::vy() const
{
  return data->vy;
}

/** Get maximum length of vy value.
 * @return length of vy value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::TransRotMessage::maxlenof_vy() const
{
  return 1;
}

/** Set vy value.
 * Speed in Y direction in m/s.
 * @param new_vy new vy value
 */
void
MotorInterface::TransRotMessage::set_vy(const float new_vy)
{
  data->vy = new_vy;
}

/** Get omega value.
 * Angle rotation in rad/s.
 * @return omega value
 */
float
MotorInterface::TransRotMessage::omega() const
{
  return data->omega;
}

/** Get maximum length of omega value.
 * @return length of omega value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::TransRotMessage::maxlenof_omega() const
{
  return 1;
}

/** Set omega value.
 * Angle rotation in rad/s.
 * @param new_omega new omega value
 */
void
MotorInterface::TransRotMessage::set_omega(const float new_omega)
{
  data->omega = new_omega;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MotorInterface::TransRotMessage::clone() const
{
  return new MotorInterface::TransRotMessage(this);
}
/** @class MotorInterface::OrbitMessage <interfaces/MotorInterface.h>
 * OrbitMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_px initial value for px
 * @param ini_py initial value for py
 * @param ini_omega initial value for omega
 */
MotorInterface::OrbitMessage::OrbitMessage(const float ini_px, const float ini_py, const float ini_omega) : Message("OrbitMessage")
{
  data_size = sizeof(OrbitMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (OrbitMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->px = ini_px;
  data->py = ini_py;
  data->omega = ini_omega;
  add_fieldinfo(IFT_FLOAT, "px", 1, &data->px);
  add_fieldinfo(IFT_FLOAT, "py", 1, &data->py);
  add_fieldinfo(IFT_FLOAT, "omega", 1, &data->omega);
}
/** Constructor */
MotorInterface::OrbitMessage::OrbitMessage() : Message("OrbitMessage")
{
  data_size = sizeof(OrbitMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (OrbitMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "px", 1, &data->px);
  add_fieldinfo(IFT_FLOAT, "py", 1, &data->py);
  add_fieldinfo(IFT_FLOAT, "omega", 1, &data->omega);
}

/** Destructor */
MotorInterface::OrbitMessage::~OrbitMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MotorInterface::OrbitMessage::OrbitMessage(const OrbitMessage *m) : Message("OrbitMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (OrbitMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get px value.
 * Point's X coordinate to orbit.
 * @return px value
 */
float
MotorInterface::OrbitMessage::px() const
{
  return data->px;
}

/** Get maximum length of px value.
 * @return length of px value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::OrbitMessage::maxlenof_px() const
{
  return 1;
}

/** Set px value.
 * Point's X coordinate to orbit.
 * @param new_px new px value
 */
void
MotorInterface::OrbitMessage::set_px(const float new_px)
{
  data->px = new_px;
}

/** Get py value.
 * Point's Y coordinate to orbit.
 * @return py value
 */
float
MotorInterface::OrbitMessage::py() const
{
  return data->py;
}

/** Get maximum length of py value.
 * @return length of py value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::OrbitMessage::maxlenof_py() const
{
  return 1;
}

/** Set py value.
 * Point's Y coordinate to orbit.
 * @param new_py new py value
 */
void
MotorInterface::OrbitMessage::set_py(const float new_py)
{
  data->py = new_py;
}

/** Get omega value.
 * Angular speed around point in rad/s.
 * @return omega value
 */
float
MotorInterface::OrbitMessage::omega() const
{
  return data->omega;
}

/** Get maximum length of omega value.
 * @return length of omega value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::OrbitMessage::maxlenof_omega() const
{
  return 1;
}

/** Set omega value.
 * Angular speed around point in rad/s.
 * @param new_omega new omega value
 */
void
MotorInterface::OrbitMessage::set_omega(const float new_omega)
{
  data->omega = new_omega;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MotorInterface::OrbitMessage::clone() const
{
  return new MotorInterface::OrbitMessage(this);
}
/** @class MotorInterface::LinTransRotMessage <interfaces/MotorInterface.h>
 * LinTransRotMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_vx initial value for vx
 * @param ini_vy initial value for vy
 * @param ini_omega initial value for omega
 */
MotorInterface::LinTransRotMessage::LinTransRotMessage(const float ini_vx, const float ini_vy, const float ini_omega) : Message("LinTransRotMessage")
{
  data_size = sizeof(LinTransRotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (LinTransRotMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->vx = ini_vx;
  data->vy = ini_vy;
  data->omega = ini_omega;
  add_fieldinfo(IFT_FLOAT, "vx", 1, &data->vx);
  add_fieldinfo(IFT_FLOAT, "vy", 1, &data->vy);
  add_fieldinfo(IFT_FLOAT, "omega", 1, &data->omega);
}
/** Constructor */
MotorInterface::LinTransRotMessage::LinTransRotMessage() : Message("LinTransRotMessage")
{
  data_size = sizeof(LinTransRotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (LinTransRotMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "vx", 1, &data->vx);
  add_fieldinfo(IFT_FLOAT, "vy", 1, &data->vy);
  add_fieldinfo(IFT_FLOAT, "omega", 1, &data->omega);
}

/** Destructor */
MotorInterface::LinTransRotMessage::~LinTransRotMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MotorInterface::LinTransRotMessage::LinTransRotMessage(const LinTransRotMessage *m) : Message("LinTransRotMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (LinTransRotMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get vx value.
 * Speed for translation in X direction in m/s.
 * @return vx value
 */
float
MotorInterface::LinTransRotMessage::vx() const
{
  return data->vx;
}

/** Get maximum length of vx value.
 * @return length of vx value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::LinTransRotMessage::maxlenof_vx() const
{
  return 1;
}

/** Set vx value.
 * Speed for translation in X direction in m/s.
 * @param new_vx new vx value
 */
void
MotorInterface::LinTransRotMessage::set_vx(const float new_vx)
{
  data->vx = new_vx;
}

/** Get vy value.
 * Speed for translation in Y direction in m/s.
 * @return vy value
 */
float
MotorInterface::LinTransRotMessage::vy() const
{
  return data->vy;
}

/** Get maximum length of vy value.
 * @return length of vy value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::LinTransRotMessage::maxlenof_vy() const
{
  return 1;
}

/** Set vy value.
 * Speed for translation in Y direction in m/s.
 * @param new_vy new vy value
 */
void
MotorInterface::LinTransRotMessage::set_vy(const float new_vy)
{
  data->vy = new_vy;
}

/** Get omega value.
 * Rotational speed in rad/s.
 * @return omega value
 */
float
MotorInterface::LinTransRotMessage::omega() const
{
  return data->omega;
}

/** Get maximum length of omega value.
 * @return length of omega value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MotorInterface::LinTransRotMessage::maxlenof_omega() const
{
  return 1;
}

/** Set omega value.
 * Rotational speed in rad/s.
 * @param new_omega new omega value
 */
void
MotorInterface::LinTransRotMessage::set_omega(const float new_omega)
{
  data->omega = new_omega;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MotorInterface::LinTransRotMessage::clone() const
{
  return new MotorInterface::LinTransRotMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
MotorInterface::message_valid(const Message *message) const
{
  const SetMotorStateMessage *m0 = dynamic_cast<const SetMotorStateMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const AcquireControlMessage *m1 = dynamic_cast<const AcquireControlMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const ResetOdometryMessage *m2 = dynamic_cast<const ResetOdometryMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const DriveRPMMessage *m3 = dynamic_cast<const DriveRPMMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const GotoMessage *m4 = dynamic_cast<const GotoMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const TransMessage *m5 = dynamic_cast<const TransMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const RotMessage *m6 = dynamic_cast<const RotMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const TransRotMessage *m7 = dynamic_cast<const TransRotMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const OrbitMessage *m8 = dynamic_cast<const OrbitMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  const LinTransRotMessage *m9 = dynamic_cast<const LinTransRotMessage *>(message);
  if ( m9 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(MotorInterface)
/// @endcond


} // end namespace fawkes
