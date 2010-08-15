
/***************************************************************************
 *  KatanaInterface.cpp - Fawkes BlackBoard Interface - KatanaInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2009  Tim Niemueller
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

#include <interfaces/KatanaInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class KatanaInterface <interfaces/KatanaInterface.h>
 * KatanaInterface Fawkes BlackBoard Interface.
 * 
      Interface to access component providing access to a Neuronics
      Katana arm.
    
 * @ingroup FawkesInterfaces
 */


/** SENSOR_IR_RIGHT_INNER_MIDDLE constant */
const uint32_t KatanaInterface::SENSOR_IR_RIGHT_INNER_MIDDLE = 0u;
/** SENSOR_IR_RIGHT_INNER_FRONT constant */
const uint32_t KatanaInterface::SENSOR_IR_RIGHT_INNER_FRONT = 1u;
/** SENSOR_RESERVED_2 constant */
const uint32_t KatanaInterface::SENSOR_RESERVED_2 = 2u;
/** SENSOR_COND_BOTH constant */
const uint32_t KatanaInterface::SENSOR_COND_BOTH = 3u;
/** SENSOR_IR_RIGHT_OUTER_FRONT constant */
const uint32_t KatanaInterface::SENSOR_IR_RIGHT_OUTER_FRONT = 4u;
/** SENSOR_IR_RIGHT_BOTTOM_FRONT constant */
const uint32_t KatanaInterface::SENSOR_IR_RIGHT_BOTTOM_FRONT = 5u;
/** SENSOR_FORCE_RIGHT_REAR constant */
const uint32_t KatanaInterface::SENSOR_FORCE_RIGHT_REAR = 6u;
/** SENSOR_FORCE_RIGHT_FRONT constant */
const uint32_t KatanaInterface::SENSOR_FORCE_RIGHT_FRONT = 7u;
/** SENSOR_IR_LEFT_INNER_MIDDLE constant */
const uint32_t KatanaInterface::SENSOR_IR_LEFT_INNER_MIDDLE = 8u;
/** SENSOR_IR_LEFT_INNER_FRONT constant */
const uint32_t KatanaInterface::SENSOR_IR_LEFT_INNER_FRONT = 9u;
/** SENSOR_RESERVED_10 constant */
const uint32_t KatanaInterface::SENSOR_RESERVED_10 = 10u;
/** SENSOR_IR_CENTER_GRIPPER constant */
const uint32_t KatanaInterface::SENSOR_IR_CENTER_GRIPPER = 11u;
/** SENSOR_IR_LEFT_OUTER_FRONT constant */
const uint32_t KatanaInterface::SENSOR_IR_LEFT_OUTER_FRONT = 12u;
/** SENSOR_IR_LEFT_BOTTOM_FRONT constant */
const uint32_t KatanaInterface::SENSOR_IR_LEFT_BOTTOM_FRONT = 13u;
/** SENSOR_FORCE_LEFT_REAR constant */
const uint32_t KatanaInterface::SENSOR_FORCE_LEFT_REAR = 14u;
/** SENSOR_FORCE_LEFT_FRONT constant */
const uint32_t KatanaInterface::SENSOR_FORCE_LEFT_FRONT = 15u;
/** ERROR_NONE constant */
const uint32_t KatanaInterface::ERROR_NONE = 0u;
/** ERROR_UNSPECIFIC constant */
const uint32_t KatanaInterface::ERROR_UNSPECIFIC = 1u;
/** ERROR_CMD_START_FAILED constant */
const uint32_t KatanaInterface::ERROR_CMD_START_FAILED = 2u;
/** ERROR_NO_SOLUTION constant */
const uint32_t KatanaInterface::ERROR_NO_SOLUTION = 4u;
/** ERROR_COMMUNICATION constant */
const uint32_t KatanaInterface::ERROR_COMMUNICATION = 8u;
/** ERROR_MOTOR_CRASHED constant */
const uint32_t KatanaInterface::ERROR_MOTOR_CRASHED = 16u;

/** Constructor */
KatanaInterface::KatanaInterface() : Interface()
{
  data_size = sizeof(KatanaInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (KatanaInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_BYTE, "sensor_value", 16, &data->sensor_value);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
  add_fieldinfo(IFT_FLOAT, "phi", 1, &data->phi);
  add_fieldinfo(IFT_FLOAT, "theta", 1, &data->theta);
  add_fieldinfo(IFT_FLOAT, "psi", 1, &data->psi);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_UINT32, "error_code", 1, &data->error_code);
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
  add_fieldinfo(IFT_BOOL, "calibrated", 1, &data->calibrated);
  add_fieldinfo(IFT_BYTE, "max_velocity", 1, &data->max_velocity);
  add_fieldinfo(IFT_BYTE, "num_motors", 1, &data->num_motors);
  add_messageinfo("StopMessage");
  add_messageinfo("FlushMessage");
  add_messageinfo("ParkMessage");
  add_messageinfo("LinearGotoMessage");
  add_messageinfo("CalibrateMessage");
  add_messageinfo("OpenGripperMessage");
  add_messageinfo("CloseGripperMessage");
  add_messageinfo("SetEnabledMessage");
  add_messageinfo("SetMaxVelocityMessage");
  unsigned char tmp_hash[] = {0x36, 0x80, 0x78, 0x4, 0x9f, 0x14, 0x1b, 0x27, 0x38, 0x40, 0x77, 0xb6, 0xc0, 0x7, 0xe, 0x73};
  set_hash(tmp_hash);
}

/** Destructor */
KatanaInterface::~KatanaInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get sensor_value value.
 * Sensor
    values. Use SENSOR_* indexes for accessing the values.
 * @return sensor_value value
 */
uint8_t *
KatanaInterface::sensor_value() const
{
  return data->sensor_value;
}

/** Get sensor_value value at given index.
 * Sensor
    values. Use SENSOR_* indexes for accessing the values.
 * @param index index of value
 * @return sensor_value value
 * @exception Exception thrown if index is out of bounds
 */
uint8_t
KatanaInterface::sensor_value(unsigned int index) const
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  return data->sensor_value[index];
}

/** Get maximum length of sensor_value value.
 * @return length of sensor_value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_sensor_value() const
{
  return 16;
}

/** Set sensor_value value.
 * Sensor
    values. Use SENSOR_* indexes for accessing the values.
 * @param new_sensor_value new sensor_value value
 */
void
KatanaInterface::set_sensor_value(const uint8_t * new_sensor_value)
{
  memcpy(data->sensor_value, new_sensor_value, sizeof(uint8_t) * 16);
  data_changed = true;
}

/** Set sensor_value value at given index.
 * Sensor
    values. Use SENSOR_* indexes for accessing the values.
 * @param new_sensor_value new sensor_value value
 * @param index index for of the value
 */
void
KatanaInterface::set_sensor_value(unsigned int index, const uint8_t new_sensor_value)
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  data->sensor_value[index] = new_sensor_value;
}
/** Get x value.
 * X-Coordinate for tool position
    compared to base coordinate system.
 * @return x value
 */
float
KatanaInterface::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X-Coordinate for tool position
    compared to base coordinate system.
 * @param new_x new x value
 */
void
KatanaInterface::set_x(const float new_x)
{
  data->x = new_x;
  data_changed = true;
}

/** Get y value.
 * Y-Coordinate for tool position
    compared to base coordinate system.
 * @return y value
 */
float
KatanaInterface::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y-Coordinate for tool position
    compared to base coordinate system.
 * @param new_y new y value
 */
void
KatanaInterface::set_y(const float new_y)
{
  data->y = new_y;
  data_changed = true;
}

/** Get z value.
 * Z-Coordinate for tool position
    compared to base coordinate system.
 * @return z value
 */
float
KatanaInterface::z() const
{
  return data->z;
}

/** Get maximum length of z value.
 * @return length of z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_z() const
{
  return 1;
}

/** Set z value.
 * Z-Coordinate for tool position
    compared to base coordinate system.
 * @param new_z new z value
 */
void
KatanaInterface::set_z(const float new_z)
{
  data->z = new_z;
  data_changed = true;
}

/** Get phi value.
 * Euler angle Phi of tool orientation.
 * @return phi value
 */
float
KatanaInterface::phi() const
{
  return data->phi;
}

/** Get maximum length of phi value.
 * @return length of phi value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_phi() const
{
  return 1;
}

/** Set phi value.
 * Euler angle Phi of tool orientation.
 * @param new_phi new phi value
 */
void
KatanaInterface::set_phi(const float new_phi)
{
  data->phi = new_phi;
  data_changed = true;
}

/** Get theta value.
 * Euler angle Theta of tool orientation.
 * @return theta value
 */
float
KatanaInterface::theta() const
{
  return data->theta;
}

/** Get maximum length of theta value.
 * @return length of theta value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_theta() const
{
  return 1;
}

/** Set theta value.
 * Euler angle Theta of tool orientation.
 * @param new_theta new theta value
 */
void
KatanaInterface::set_theta(const float new_theta)
{
  data->theta = new_theta;
  data_changed = true;
}

/** Get psi value.
 * Euler angle Psi of tool orientation..
 * @return psi value
 */
float
KatanaInterface::psi() const
{
  return data->psi;
}

/** Get maximum length of psi value.
 * @return length of psi value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_psi() const
{
  return 1;
}

/** Set psi value.
 * Euler angle Psi of tool orientation..
 * @param new_psi new psi value
 */
void
KatanaInterface::set_psi(const float new_psi)
{
  data->psi = new_psi;
  data_changed = true;
}

/** Get msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @return msgid value
 */
uint32_t
KatanaInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @param new_msgid new msgid value
 */
void
KatanaInterface::set_msgid(const uint32_t new_msgid)
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
KatanaInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @param new_final new final value
 */
void
KatanaInterface::set_final(const bool new_final)
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
KatanaInterface::error_code() const
{
  return data->error_code;
}

/** Get maximum length of error_code value.
 * @return length of error_code value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_error_code() const
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
KatanaInterface::set_error_code(const uint32_t new_error_code)
{
  data->error_code = new_error_code;
  data_changed = true;
}

/** Get enabled value.
 * Are motors enabled?
 * @return enabled value
 */
bool
KatanaInterface::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * Are motors enabled?
 * @param new_enabled new enabled value
 */
void
KatanaInterface::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
  data_changed = true;
}

/** Get calibrated value.
 * Has arm been calibrated?
 * @return calibrated value
 */
bool
KatanaInterface::is_calibrated() const
{
  return data->calibrated;
}

/** Get maximum length of calibrated value.
 * @return length of calibrated value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_calibrated() const
{
  return 1;
}

/** Set calibrated value.
 * Has arm been calibrated?
 * @param new_calibrated new calibrated value
 */
void
KatanaInterface::set_calibrated(const bool new_calibrated)
{
  data->calibrated = new_calibrated;
  data_changed = true;
}

/** Get max_velocity value.
 * Maximum velocity
 * @return max_velocity value
 */
uint8_t
KatanaInterface::max_velocity() const
{
  return data->max_velocity;
}

/** Get maximum length of max_velocity value.
 * @return length of max_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_max_velocity() const
{
  return 1;
}

/** Set max_velocity value.
 * Maximum velocity
 * @param new_max_velocity new max_velocity value
 */
void
KatanaInterface::set_max_velocity(const uint8_t new_max_velocity)
{
  data->max_velocity = new_max_velocity;
  data_changed = true;
}

/** Get num_motors value.
 * Number of motors
 * @return num_motors value
 */
uint8_t
KatanaInterface::num_motors() const
{
  return data->num_motors;
}

/** Get maximum length of num_motors value.
 * @return length of num_motors value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::maxlenof_num_motors() const
{
  return 1;
}

/** Set num_motors value.
 * Number of motors
 * @param new_num_motors new num_motors value
 */
void
KatanaInterface::set_num_motors(const uint8_t new_num_motors)
{
  data->num_motors = new_num_motors;
  data_changed = true;
}

/* =========== message create =========== */
Message *
KatanaInterface::create_message(const char *type) const
{
  if ( strncmp("StopMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopMessage();
  } else if ( strncmp("FlushMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new FlushMessage();
  } else if ( strncmp("ParkMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ParkMessage();
  } else if ( strncmp("LinearGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new LinearGotoMessage();
  } else if ( strncmp("CalibrateMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CalibrateMessage();
  } else if ( strncmp("OpenGripperMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new OpenGripperMessage();
  } else if ( strncmp("CloseGripperMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CloseGripperMessage();
  } else if ( strncmp("SetEnabledMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetEnabledMessage();
  } else if ( strncmp("SetMaxVelocityMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMaxVelocityMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
KatanaInterface::copy_values(const Interface *other)
{
  const KatanaInterface *oi = dynamic_cast<const KatanaInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(KatanaInterface_data_t));
}

const char *
KatanaInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class KatanaInterface::StopMessage <interfaces/KatanaInterface.h>
 * StopMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
KatanaInterface::StopMessage::StopMessage() : Message("StopMessage")
{
  data_size = sizeof(StopMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
KatanaInterface::StopMessage::~StopMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
KatanaInterface::StopMessage::StopMessage(const StopMessage *m) : Message("StopMessage")
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
KatanaInterface::StopMessage::clone() const
{
  return new KatanaInterface::StopMessage(this);
}
/** @class KatanaInterface::FlushMessage <interfaces/KatanaInterface.h>
 * FlushMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
KatanaInterface::FlushMessage::FlushMessage() : Message("FlushMessage")
{
  data_size = sizeof(FlushMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (FlushMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
KatanaInterface::FlushMessage::~FlushMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
KatanaInterface::FlushMessage::FlushMessage(const FlushMessage *m) : Message("FlushMessage")
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
KatanaInterface::FlushMessage::clone() const
{
  return new KatanaInterface::FlushMessage(this);
}
/** @class KatanaInterface::ParkMessage <interfaces/KatanaInterface.h>
 * ParkMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
KatanaInterface::ParkMessage::ParkMessage() : Message("ParkMessage")
{
  data_size = sizeof(ParkMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ParkMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
KatanaInterface::ParkMessage::~ParkMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
KatanaInterface::ParkMessage::ParkMessage(const ParkMessage *m) : Message("ParkMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ParkMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
KatanaInterface::ParkMessage::clone() const
{
  return new KatanaInterface::ParkMessage(this);
}
/** @class KatanaInterface::LinearGotoMessage <interfaces/KatanaInterface.h>
 * LinearGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_z initial value for z
 * @param ini_phi initial value for phi
 * @param ini_theta initial value for theta
 * @param ini_psi initial value for psi
 */
KatanaInterface::LinearGotoMessage::LinearGotoMessage(const float ini_x, const float ini_y, const float ini_z, const float ini_phi, const float ini_theta, const float ini_psi) : Message("LinearGotoMessage")
{
  data_size = sizeof(LinearGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (LinearGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->z = ini_z;
  data->phi = ini_phi;
  data->theta = ini_theta;
  data->psi = ini_psi;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
  add_fieldinfo(IFT_FLOAT, "phi", 1, &data->phi);
  add_fieldinfo(IFT_FLOAT, "theta", 1, &data->theta);
  add_fieldinfo(IFT_FLOAT, "psi", 1, &data->psi);
}
/** Constructor */
KatanaInterface::LinearGotoMessage::LinearGotoMessage() : Message("LinearGotoMessage")
{
  data_size = sizeof(LinearGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (LinearGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
  add_fieldinfo(IFT_FLOAT, "phi", 1, &data->phi);
  add_fieldinfo(IFT_FLOAT, "theta", 1, &data->theta);
  add_fieldinfo(IFT_FLOAT, "psi", 1, &data->psi);
}

/** Destructor */
KatanaInterface::LinearGotoMessage::~LinearGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
KatanaInterface::LinearGotoMessage::LinearGotoMessage(const LinearGotoMessage *m) : Message("LinearGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (LinearGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X-Coordinate for tool position
    compared to base coordinate system.
 * @return x value
 */
float
KatanaInterface::LinearGotoMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::LinearGotoMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X-Coordinate for tool position
    compared to base coordinate system.
 * @param new_x new x value
 */
void
KatanaInterface::LinearGotoMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y-Coordinate for tool position
    compared to base coordinate system.
 * @return y value
 */
float
KatanaInterface::LinearGotoMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::LinearGotoMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y-Coordinate for tool position
    compared to base coordinate system.
 * @param new_y new y value
 */
void
KatanaInterface::LinearGotoMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get z value.
 * Z-Coordinate for tool position
    compared to base coordinate system.
 * @return z value
 */
float
KatanaInterface::LinearGotoMessage::z() const
{
  return data->z;
}

/** Get maximum length of z value.
 * @return length of z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::LinearGotoMessage::maxlenof_z() const
{
  return 1;
}

/** Set z value.
 * Z-Coordinate for tool position
    compared to base coordinate system.
 * @param new_z new z value
 */
void
KatanaInterface::LinearGotoMessage::set_z(const float new_z)
{
  data->z = new_z;
}

/** Get phi value.
 * Euler angle Phi of tool orientation.
 * @return phi value
 */
float
KatanaInterface::LinearGotoMessage::phi() const
{
  return data->phi;
}

/** Get maximum length of phi value.
 * @return length of phi value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::LinearGotoMessage::maxlenof_phi() const
{
  return 1;
}

/** Set phi value.
 * Euler angle Phi of tool orientation.
 * @param new_phi new phi value
 */
void
KatanaInterface::LinearGotoMessage::set_phi(const float new_phi)
{
  data->phi = new_phi;
}

/** Get theta value.
 * Euler angle Theta of tool orientation.
 * @return theta value
 */
float
KatanaInterface::LinearGotoMessage::theta() const
{
  return data->theta;
}

/** Get maximum length of theta value.
 * @return length of theta value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::LinearGotoMessage::maxlenof_theta() const
{
  return 1;
}

/** Set theta value.
 * Euler angle Theta of tool orientation.
 * @param new_theta new theta value
 */
void
KatanaInterface::LinearGotoMessage::set_theta(const float new_theta)
{
  data->theta = new_theta;
}

/** Get psi value.
 * Euler angle Psi of tool orientation..
 * @return psi value
 */
float
KatanaInterface::LinearGotoMessage::psi() const
{
  return data->psi;
}

/** Get maximum length of psi value.
 * @return length of psi value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::LinearGotoMessage::maxlenof_psi() const
{
  return 1;
}

/** Set psi value.
 * Euler angle Psi of tool orientation..
 * @param new_psi new psi value
 */
void
KatanaInterface::LinearGotoMessage::set_psi(const float new_psi)
{
  data->psi = new_psi;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
KatanaInterface::LinearGotoMessage::clone() const
{
  return new KatanaInterface::LinearGotoMessage(this);
}
/** @class KatanaInterface::CalibrateMessage <interfaces/KatanaInterface.h>
 * CalibrateMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
KatanaInterface::CalibrateMessage::CalibrateMessage() : Message("CalibrateMessage")
{
  data_size = sizeof(CalibrateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CalibrateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
KatanaInterface::CalibrateMessage::~CalibrateMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
KatanaInterface::CalibrateMessage::CalibrateMessage(const CalibrateMessage *m) : Message("CalibrateMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CalibrateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
KatanaInterface::CalibrateMessage::clone() const
{
  return new KatanaInterface::CalibrateMessage(this);
}
/** @class KatanaInterface::OpenGripperMessage <interfaces/KatanaInterface.h>
 * OpenGripperMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
KatanaInterface::OpenGripperMessage::OpenGripperMessage() : Message("OpenGripperMessage")
{
  data_size = sizeof(OpenGripperMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (OpenGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
KatanaInterface::OpenGripperMessage::~OpenGripperMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
KatanaInterface::OpenGripperMessage::OpenGripperMessage(const OpenGripperMessage *m) : Message("OpenGripperMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (OpenGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
KatanaInterface::OpenGripperMessage::clone() const
{
  return new KatanaInterface::OpenGripperMessage(this);
}
/** @class KatanaInterface::CloseGripperMessage <interfaces/KatanaInterface.h>
 * CloseGripperMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
KatanaInterface::CloseGripperMessage::CloseGripperMessage() : Message("CloseGripperMessage")
{
  data_size = sizeof(CloseGripperMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CloseGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
KatanaInterface::CloseGripperMessage::~CloseGripperMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
KatanaInterface::CloseGripperMessage::CloseGripperMessage(const CloseGripperMessage *m) : Message("CloseGripperMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CloseGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
KatanaInterface::CloseGripperMessage::clone() const
{
  return new KatanaInterface::CloseGripperMessage(this);
}
/** @class KatanaInterface::SetEnabledMessage <interfaces/KatanaInterface.h>
 * SetEnabledMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_enabled initial value for enabled
 */
KatanaInterface::SetEnabledMessage::SetEnabledMessage(const bool ini_enabled) : Message("SetEnabledMessage")
{
  data_size = sizeof(SetEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->enabled = ini_enabled;
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
}
/** Constructor */
KatanaInterface::SetEnabledMessage::SetEnabledMessage() : Message("SetEnabledMessage")
{
  data_size = sizeof(SetEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
}

/** Destructor */
KatanaInterface::SetEnabledMessage::~SetEnabledMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
KatanaInterface::SetEnabledMessage::SetEnabledMessage(const SetEnabledMessage *m) : Message("SetEnabledMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get enabled value.
 * Are motors enabled?
 * @return enabled value
 */
bool
KatanaInterface::SetEnabledMessage::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::SetEnabledMessage::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * Are motors enabled?
 * @param new_enabled new enabled value
 */
void
KatanaInterface::SetEnabledMessage::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
KatanaInterface::SetEnabledMessage::clone() const
{
  return new KatanaInterface::SetEnabledMessage(this);
}
/** @class KatanaInterface::SetMaxVelocityMessage <interfaces/KatanaInterface.h>
 * SetMaxVelocityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_max_velocity initial value for max_velocity
 */
KatanaInterface::SetMaxVelocityMessage::SetMaxVelocityMessage(const uint8_t ini_max_velocity) : Message("SetMaxVelocityMessage")
{
  data_size = sizeof(SetMaxVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMaxVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->max_velocity = ini_max_velocity;
  add_fieldinfo(IFT_BYTE, "max_velocity", 1, &data->max_velocity);
}
/** Constructor */
KatanaInterface::SetMaxVelocityMessage::SetMaxVelocityMessage() : Message("SetMaxVelocityMessage")
{
  data_size = sizeof(SetMaxVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMaxVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_BYTE, "max_velocity", 1, &data->max_velocity);
}

/** Destructor */
KatanaInterface::SetMaxVelocityMessage::~SetMaxVelocityMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
KatanaInterface::SetMaxVelocityMessage::SetMaxVelocityMessage(const SetMaxVelocityMessage *m) : Message("SetMaxVelocityMessage")
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
uint8_t
KatanaInterface::SetMaxVelocityMessage::max_velocity() const
{
  return data->max_velocity;
}

/** Get maximum length of max_velocity value.
 * @return length of max_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
KatanaInterface::SetMaxVelocityMessage::maxlenof_max_velocity() const
{
  return 1;
}

/** Set max_velocity value.
 * Maximum velocity
 * @param new_max_velocity new max_velocity value
 */
void
KatanaInterface::SetMaxVelocityMessage::set_max_velocity(const uint8_t new_max_velocity)
{
  data->max_velocity = new_max_velocity;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
KatanaInterface::SetMaxVelocityMessage::clone() const
{
  return new KatanaInterface::SetMaxVelocityMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
KatanaInterface::message_valid(const Message *message) const
{
  const StopMessage *m0 = dynamic_cast<const StopMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const FlushMessage *m1 = dynamic_cast<const FlushMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const ParkMessage *m2 = dynamic_cast<const ParkMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const LinearGotoMessage *m3 = dynamic_cast<const LinearGotoMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const CalibrateMessage *m4 = dynamic_cast<const CalibrateMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const OpenGripperMessage *m5 = dynamic_cast<const OpenGripperMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const CloseGripperMessage *m6 = dynamic_cast<const CloseGripperMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const SetEnabledMessage *m7 = dynamic_cast<const SetEnabledMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const SetMaxVelocityMessage *m8 = dynamic_cast<const SetMaxVelocityMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(KatanaInterface)
/// @endcond


} // end namespace fawkes
