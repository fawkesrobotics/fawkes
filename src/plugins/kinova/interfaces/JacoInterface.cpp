
/***************************************************************************
 *  JacoInterface.cpp - Fawkes BlackBoard Interface - JacoInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
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

#include <interfaces/JacoInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class JacoInterface <interfaces/JacoInterface.h>
 * JacoInterface Fawkes BlackBoard Interface.
 * 
      Interface providing access to a Kinova Jaco arm.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
JacoInterface::JacoInterface() : Interface()
{
  data_size = sizeof(JacoInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (JacoInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_BOOL, "connected", 1, &data->connected);
  add_fieldinfo(IFT_BOOL, "initialized", 1, &data->initialized);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
  add_fieldinfo(IFT_FLOAT, "euler1", 1, &data->euler1);
  add_fieldinfo(IFT_FLOAT, "euler2", 1, &data->euler2);
  add_fieldinfo(IFT_FLOAT, "euler3", 1, &data->euler3);
  add_fieldinfo(IFT_FLOAT, "joints", 6, &data->joints);
  add_fieldinfo(IFT_FLOAT, "finger1", 1, &data->finger1);
  add_fieldinfo(IFT_FLOAT, "finger2", 1, &data->finger2);
  add_fieldinfo(IFT_FLOAT, "finger3", 1, &data->finger3);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_messageinfo("CalibrateMessage");
  add_messageinfo("RetractMessage");
  add_messageinfo("StopMessage");
  add_messageinfo("CartesianGotoMessage");
  add_messageinfo("AngularGotoMessage");
  add_messageinfo("OpenGripperMessage");
  add_messageinfo("CloseGripperMessage");
  add_messageinfo("JoystickPushMessage");
  add_messageinfo("JoystickReleaseMessage");
  unsigned char tmp_hash[] = {0xfe, 0xf4, 0x9c, 0xf2, 0x79, 0x25, 0x11, 0x30, 0x27, 0x2f, 0x15, 0x24, 0x4f, 0x64, 0xee, 0x76};
  set_hash(tmp_hash);
}

/** Destructor */
JacoInterface::~JacoInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get connected value.
 * Is JacoArm connected/ready?
 * @return connected value
 */
bool
JacoInterface::is_connected() const
{
  return data->connected;
}

/** Get maximum length of connected value.
 * @return length of connected value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_connected() const
{
  return 1;
}

/** Set connected value.
 * Is JacoArm connected/ready?
 * @param new_connected new connected value
 */
void
JacoInterface::set_connected(const bool new_connected)
{
  data->connected = new_connected;
  data_changed = true;
}

/** Get initialized value.
 * Checks if Jaco arm has been initialized once after switched on.
 * @return initialized value
 */
bool
JacoInterface::is_initialized() const
{
  return data->initialized;
}

/** Get maximum length of initialized value.
 * @return length of initialized value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_initialized() const
{
  return 1;
}

/** Set initialized value.
 * Checks if Jaco arm has been initialized once after switched on.
 * @param new_initialized new initialized value
 */
void
JacoInterface::set_initialized(const bool new_initialized)
{
  data->initialized = new_initialized;
  data_changed = true;
}

/** Get x value.
 * X-Coordinate of tool translation.
 * @return x value
 */
float
JacoInterface::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X-Coordinate of tool translation.
 * @param new_x new x value
 */
void
JacoInterface::set_x(const float new_x)
{
  data->x = new_x;
  data_changed = true;
}

/** Get y value.
 * Y-Coordinate op tool translation.
 * @return y value
 */
float
JacoInterface::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y-Coordinate op tool translation.
 * @param new_y new y value
 */
void
JacoInterface::set_y(const float new_y)
{
  data->y = new_y;
  data_changed = true;
}

/** Get z value.
 * Z-Coordinate of tool translation.
 * @return z value
 */
float
JacoInterface::z() const
{
  return data->z;
}

/** Get maximum length of z value.
 * @return length of z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_z() const
{
  return 1;
}

/** Set z value.
 * Z-Coordinate of tool translation.
 * @param new_z new z value
 */
void
JacoInterface::set_z(const float new_z)
{
  data->z = new_z;
  data_changed = true;
}

/** Get euler1 value.
 * 1st Euler angle of tool rotation.
 * @return euler1 value
 */
float
JacoInterface::euler1() const
{
  return data->euler1;
}

/** Get maximum length of euler1 value.
 * @return length of euler1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_euler1() const
{
  return 1;
}

/** Set euler1 value.
 * 1st Euler angle of tool rotation.
 * @param new_euler1 new euler1 value
 */
void
JacoInterface::set_euler1(const float new_euler1)
{
  data->euler1 = new_euler1;
  data_changed = true;
}

/** Get euler2 value.
 * 2nd Euler angle of tool rotation.
 * @return euler2 value
 */
float
JacoInterface::euler2() const
{
  return data->euler2;
}

/** Get maximum length of euler2 value.
 * @return length of euler2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_euler2() const
{
  return 1;
}

/** Set euler2 value.
 * 2nd Euler angle of tool rotation.
 * @param new_euler2 new euler2 value
 */
void
JacoInterface::set_euler2(const float new_euler2)
{
  data->euler2 = new_euler2;
  data_changed = true;
}

/** Get euler3 value.
 * 3rd Euler angle of tool rotation.
 * @return euler3 value
 */
float
JacoInterface::euler3() const
{
  return data->euler3;
}

/** Get maximum length of euler3 value.
 * @return length of euler3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_euler3() const
{
  return 1;
}

/** Set euler3 value.
 * 3rd Euler angle of tool rotation.
 * @param new_euler3 new euler3 value
 */
void
JacoInterface::set_euler3(const float new_euler3)
{
  data->euler3 = new_euler3;
  data_changed = true;
}

/** Get joints value.
 * Angle values of joints
 * @return joints value
 */
float *
JacoInterface::joints() const
{
  return data->joints;
}

/** Get joints value at given index.
 * Angle values of joints
 * @param index index of value
 * @return joints value
 * @exception Exception thrown if index is out of bounds
 */
float
JacoInterface::joints(unsigned int index) const
{
  if (index > 6) {
    throw Exception("Index value %u out of bounds (0..6)", index);
  }
  return data->joints[index];
}

/** Get maximum length of joints value.
 * @return length of joints value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_joints() const
{
  return 6;
}

/** Set joints value.
 * Angle values of joints
 * @param new_joints new joints value
 */
void
JacoInterface::set_joints(const float * new_joints)
{
  memcpy(data->joints, new_joints, sizeof(float) * 6);
  data_changed = true;
}

/** Set joints value at given index.
 * Angle values of joints
 * @param new_joints new joints value
 * @param index index for of the value
 */
void
JacoInterface::set_joints(unsigned int index, const float new_joints)
{
  if (index > 6) {
    throw Exception("Index value %u out of bounds (0..6)", index);
  }
  data->joints[index] = new_joints;
  data_changed = true;
}
/** Get finger1 value.
 * Angular value of finger 1.
 * @return finger1 value
 */
float
JacoInterface::finger1() const
{
  return data->finger1;
}

/** Get maximum length of finger1 value.
 * @return length of finger1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_finger1() const
{
  return 1;
}

/** Set finger1 value.
 * Angular value of finger 1.
 * @param new_finger1 new finger1 value
 */
void
JacoInterface::set_finger1(const float new_finger1)
{
  data->finger1 = new_finger1;
  data_changed = true;
}

/** Get finger2 value.
 * Angular value of finger 2.
 * @return finger2 value
 */
float
JacoInterface::finger2() const
{
  return data->finger2;
}

/** Get maximum length of finger2 value.
 * @return length of finger2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_finger2() const
{
  return 1;
}

/** Set finger2 value.
 * Angular value of finger 2.
 * @param new_finger2 new finger2 value
 */
void
JacoInterface::set_finger2(const float new_finger2)
{
  data->finger2 = new_finger2;
  data_changed = true;
}

/** Get finger3 value.
 * Angular value of finger 3.
 * @return finger3 value
 */
float
JacoInterface::finger3() const
{
  return data->finger3;
}

/** Get maximum length of finger3 value.
 * @return length of finger3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_finger3() const
{
  return 1;
}

/** Set finger3 value.
 * Angular value of finger 3.
 * @param new_finger3 new finger3 value
 */
void
JacoInterface::set_finger3(const float new_finger3)
{
  data->finger3 = new_finger3;
  data_changed = true;
}

/** Get msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @return msgid value
 */
uint32_t
JacoInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @param new_msgid new msgid value
 */
void
JacoInterface::set_msgid(const uint32_t new_msgid)
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
JacoInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @param new_final new final value
 */
void
JacoInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/* =========== message create =========== */
Message *
JacoInterface::create_message(const char *type) const
{
  if ( strncmp("CalibrateMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CalibrateMessage();
  } else if ( strncmp("RetractMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RetractMessage();
  } else if ( strncmp("StopMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopMessage();
  } else if ( strncmp("CartesianGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CartesianGotoMessage();
  } else if ( strncmp("AngularGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AngularGotoMessage();
  } else if ( strncmp("OpenGripperMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new OpenGripperMessage();
  } else if ( strncmp("CloseGripperMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CloseGripperMessage();
  } else if ( strncmp("JoystickPushMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new JoystickPushMessage();
  } else if ( strncmp("JoystickReleaseMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new JoystickReleaseMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
JacoInterface::copy_values(const Interface *other)
{
  const JacoInterface *oi = dynamic_cast<const JacoInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(JacoInterface_data_t));
}

const char *
JacoInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class JacoInterface::CalibrateMessage <interfaces/JacoInterface.h>
 * CalibrateMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
JacoInterface::CalibrateMessage::CalibrateMessage() : Message("CalibrateMessage")
{
  data_size = sizeof(CalibrateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CalibrateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
JacoInterface::CalibrateMessage::~CalibrateMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoInterface::CalibrateMessage::CalibrateMessage(const CalibrateMessage *m) : Message("CalibrateMessage")
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
JacoInterface::CalibrateMessage::clone() const
{
  return new JacoInterface::CalibrateMessage(this);
}
/** @class JacoInterface::RetractMessage <interfaces/JacoInterface.h>
 * RetractMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
JacoInterface::RetractMessage::RetractMessage() : Message("RetractMessage")
{
  data_size = sizeof(RetractMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RetractMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
JacoInterface::RetractMessage::~RetractMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoInterface::RetractMessage::RetractMessage(const RetractMessage *m) : Message("RetractMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (RetractMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JacoInterface::RetractMessage::clone() const
{
  return new JacoInterface::RetractMessage(this);
}
/** @class JacoInterface::StopMessage <interfaces/JacoInterface.h>
 * StopMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
JacoInterface::StopMessage::StopMessage() : Message("StopMessage")
{
  data_size = sizeof(StopMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
JacoInterface::StopMessage::~StopMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoInterface::StopMessage::StopMessage(const StopMessage *m) : Message("StopMessage")
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
JacoInterface::StopMessage::clone() const
{
  return new JacoInterface::StopMessage(this);
}
/** @class JacoInterface::CartesianGotoMessage <interfaces/JacoInterface.h>
 * CartesianGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_z initial value for z
 * @param ini_e1 initial value for e1
 * @param ini_e2 initial value for e2
 * @param ini_e3 initial value for e3
 */
JacoInterface::CartesianGotoMessage::CartesianGotoMessage(const float ini_x, const float ini_y, const float ini_z, const float ini_e1, const float ini_e2, const float ini_e3) : Message("CartesianGotoMessage")
{
  data_size = sizeof(CartesianGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->z = ini_z;
  data->e1 = ini_e1;
  data->e2 = ini_e2;
  data->e3 = ini_e3;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
  add_fieldinfo(IFT_FLOAT, "e1", 1, &data->e1);
  add_fieldinfo(IFT_FLOAT, "e2", 1, &data->e2);
  add_fieldinfo(IFT_FLOAT, "e3", 1, &data->e3);
}
/** Constructor */
JacoInterface::CartesianGotoMessage::CartesianGotoMessage() : Message("CartesianGotoMessage")
{
  data_size = sizeof(CartesianGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "z", 1, &data->z);
  add_fieldinfo(IFT_FLOAT, "e1", 1, &data->e1);
  add_fieldinfo(IFT_FLOAT, "e2", 1, &data->e2);
  add_fieldinfo(IFT_FLOAT, "e3", 1, &data->e3);
}

/** Destructor */
JacoInterface::CartesianGotoMessage::~CartesianGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoInterface::CartesianGotoMessage::CartesianGotoMessage(const CartesianGotoMessage *m) : Message("CartesianGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X-coordinate of target
 * @return x value
 */
float
JacoInterface::CartesianGotoMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::CartesianGotoMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X-coordinate of target
 * @param new_x new x value
 */
void
JacoInterface::CartesianGotoMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y-coordinate of target
 * @return y value
 */
float
JacoInterface::CartesianGotoMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::CartesianGotoMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y-coordinate of target
 * @param new_y new y value
 */
void
JacoInterface::CartesianGotoMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get z value.
 * Z-coordinate of target
 * @return z value
 */
float
JacoInterface::CartesianGotoMessage::z() const
{
  return data->z;
}

/** Get maximum length of z value.
 * @return length of z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::CartesianGotoMessage::maxlenof_z() const
{
  return 1;
}

/** Set z value.
 * Z-coordinate of target
 * @param new_z new z value
 */
void
JacoInterface::CartesianGotoMessage::set_z(const float new_z)
{
  data->z = new_z;
}

/** Get e1 value.
 * 1st Euler angle of target rotation
 * @return e1 value
 */
float
JacoInterface::CartesianGotoMessage::e1() const
{
  return data->e1;
}

/** Get maximum length of e1 value.
 * @return length of e1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::CartesianGotoMessage::maxlenof_e1() const
{
  return 1;
}

/** Set e1 value.
 * 1st Euler angle of target rotation
 * @param new_e1 new e1 value
 */
void
JacoInterface::CartesianGotoMessage::set_e1(const float new_e1)
{
  data->e1 = new_e1;
}

/** Get e2 value.
 * 2nd Euler angle of target rotation
 * @return e2 value
 */
float
JacoInterface::CartesianGotoMessage::e2() const
{
  return data->e2;
}

/** Get maximum length of e2 value.
 * @return length of e2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::CartesianGotoMessage::maxlenof_e2() const
{
  return 1;
}

/** Set e2 value.
 * 2nd Euler angle of target rotation
 * @param new_e2 new e2 value
 */
void
JacoInterface::CartesianGotoMessage::set_e2(const float new_e2)
{
  data->e2 = new_e2;
}

/** Get e3 value.
 * 3rd Euler angle of target rotation
 * @return e3 value
 */
float
JacoInterface::CartesianGotoMessage::e3() const
{
  return data->e3;
}

/** Get maximum length of e3 value.
 * @return length of e3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::CartesianGotoMessage::maxlenof_e3() const
{
  return 1;
}

/** Set e3 value.
 * 3rd Euler angle of target rotation
 * @param new_e3 new e3 value
 */
void
JacoInterface::CartesianGotoMessage::set_e3(const float new_e3)
{
  data->e3 = new_e3;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JacoInterface::CartesianGotoMessage::clone() const
{
  return new JacoInterface::CartesianGotoMessage(this);
}
/** @class JacoInterface::AngularGotoMessage <interfaces/JacoInterface.h>
 * AngularGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_j1 initial value for j1
 * @param ini_j2 initial value for j2
 * @param ini_j3 initial value for j3
 * @param ini_j4 initial value for j4
 * @param ini_j5 initial value for j5
 * @param ini_j6 initial value for j6
 */
JacoInterface::AngularGotoMessage::AngularGotoMessage(const float ini_j1, const float ini_j2, const float ini_j3, const float ini_j4, const float ini_j5, const float ini_j6) : Message("AngularGotoMessage")
{
  data_size = sizeof(AngularGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AngularGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->j1 = ini_j1;
  data->j2 = ini_j2;
  data->j3 = ini_j3;
  data->j4 = ini_j4;
  data->j5 = ini_j5;
  data->j6 = ini_j6;
  add_fieldinfo(IFT_FLOAT, "j1", 1, &data->j1);
  add_fieldinfo(IFT_FLOAT, "j2", 1, &data->j2);
  add_fieldinfo(IFT_FLOAT, "j3", 1, &data->j3);
  add_fieldinfo(IFT_FLOAT, "j4", 1, &data->j4);
  add_fieldinfo(IFT_FLOAT, "j5", 1, &data->j5);
  add_fieldinfo(IFT_FLOAT, "j6", 1, &data->j6);
}
/** Constructor */
JacoInterface::AngularGotoMessage::AngularGotoMessage() : Message("AngularGotoMessage")
{
  data_size = sizeof(AngularGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AngularGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "j1", 1, &data->j1);
  add_fieldinfo(IFT_FLOAT, "j2", 1, &data->j2);
  add_fieldinfo(IFT_FLOAT, "j3", 1, &data->j3);
  add_fieldinfo(IFT_FLOAT, "j4", 1, &data->j4);
  add_fieldinfo(IFT_FLOAT, "j5", 1, &data->j5);
  add_fieldinfo(IFT_FLOAT, "j6", 1, &data->j6);
}

/** Destructor */
JacoInterface::AngularGotoMessage::~AngularGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoInterface::AngularGotoMessage::AngularGotoMessage(const AngularGotoMessage *m) : Message("AngularGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AngularGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get j1 value.
 * Angular value of 1st joint
 * @return j1 value
 */
float
JacoInterface::AngularGotoMessage::j1() const
{
  return data->j1;
}

/** Get maximum length of j1 value.
 * @return length of j1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::AngularGotoMessage::maxlenof_j1() const
{
  return 1;
}

/** Set j1 value.
 * Angular value of 1st joint
 * @param new_j1 new j1 value
 */
void
JacoInterface::AngularGotoMessage::set_j1(const float new_j1)
{
  data->j1 = new_j1;
}

/** Get j2 value.
 * Angular value of 2nd joint
 * @return j2 value
 */
float
JacoInterface::AngularGotoMessage::j2() const
{
  return data->j2;
}

/** Get maximum length of j2 value.
 * @return length of j2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::AngularGotoMessage::maxlenof_j2() const
{
  return 1;
}

/** Set j2 value.
 * Angular value of 2nd joint
 * @param new_j2 new j2 value
 */
void
JacoInterface::AngularGotoMessage::set_j2(const float new_j2)
{
  data->j2 = new_j2;
}

/** Get j3 value.
 * Angular value of 3rd joint
 * @return j3 value
 */
float
JacoInterface::AngularGotoMessage::j3() const
{
  return data->j3;
}

/** Get maximum length of j3 value.
 * @return length of j3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::AngularGotoMessage::maxlenof_j3() const
{
  return 1;
}

/** Set j3 value.
 * Angular value of 3rd joint
 * @param new_j3 new j3 value
 */
void
JacoInterface::AngularGotoMessage::set_j3(const float new_j3)
{
  data->j3 = new_j3;
}

/** Get j4 value.
 * Angular value of 4th joint
 * @return j4 value
 */
float
JacoInterface::AngularGotoMessage::j4() const
{
  return data->j4;
}

/** Get maximum length of j4 value.
 * @return length of j4 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::AngularGotoMessage::maxlenof_j4() const
{
  return 1;
}

/** Set j4 value.
 * Angular value of 4th joint
 * @param new_j4 new j4 value
 */
void
JacoInterface::AngularGotoMessage::set_j4(const float new_j4)
{
  data->j4 = new_j4;
}

/** Get j5 value.
 * Angular value of 5th joint
 * @return j5 value
 */
float
JacoInterface::AngularGotoMessage::j5() const
{
  return data->j5;
}

/** Get maximum length of j5 value.
 * @return length of j5 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::AngularGotoMessage::maxlenof_j5() const
{
  return 1;
}

/** Set j5 value.
 * Angular value of 5th joint
 * @param new_j5 new j5 value
 */
void
JacoInterface::AngularGotoMessage::set_j5(const float new_j5)
{
  data->j5 = new_j5;
}

/** Get j6 value.
 * Angular value of 6th joint
 * @return j6 value
 */
float
JacoInterface::AngularGotoMessage::j6() const
{
  return data->j6;
}

/** Get maximum length of j6 value.
 * @return length of j6 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::AngularGotoMessage::maxlenof_j6() const
{
  return 1;
}

/** Set j6 value.
 * Angular value of 6th joint
 * @param new_j6 new j6 value
 */
void
JacoInterface::AngularGotoMessage::set_j6(const float new_j6)
{
  data->j6 = new_j6;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JacoInterface::AngularGotoMessage::clone() const
{
  return new JacoInterface::AngularGotoMessage(this);
}
/** @class JacoInterface::OpenGripperMessage <interfaces/JacoInterface.h>
 * OpenGripperMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
JacoInterface::OpenGripperMessage::OpenGripperMessage() : Message("OpenGripperMessage")
{
  data_size = sizeof(OpenGripperMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (OpenGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
JacoInterface::OpenGripperMessage::~OpenGripperMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoInterface::OpenGripperMessage::OpenGripperMessage(const OpenGripperMessage *m) : Message("OpenGripperMessage")
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
JacoInterface::OpenGripperMessage::clone() const
{
  return new JacoInterface::OpenGripperMessage(this);
}
/** @class JacoInterface::CloseGripperMessage <interfaces/JacoInterface.h>
 * CloseGripperMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
JacoInterface::CloseGripperMessage::CloseGripperMessage() : Message("CloseGripperMessage")
{
  data_size = sizeof(CloseGripperMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CloseGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
JacoInterface::CloseGripperMessage::~CloseGripperMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoInterface::CloseGripperMessage::CloseGripperMessage(const CloseGripperMessage *m) : Message("CloseGripperMessage")
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
JacoInterface::CloseGripperMessage::clone() const
{
  return new JacoInterface::CloseGripperMessage(this);
}
/** @class JacoInterface::JoystickPushMessage <interfaces/JacoInterface.h>
 * JoystickPushMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_button initial value for button
 */
JacoInterface::JoystickPushMessage::JoystickPushMessage(const uint32_t ini_button) : Message("JoystickPushMessage")
{
  data_size = sizeof(JoystickPushMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (JoystickPushMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->button = ini_button;
  add_fieldinfo(IFT_UINT32, "button", 1, &data->button);
}
/** Constructor */
JacoInterface::JoystickPushMessage::JoystickPushMessage() : Message("JoystickPushMessage")
{
  data_size = sizeof(JoystickPushMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (JoystickPushMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "button", 1, &data->button);
}

/** Destructor */
JacoInterface::JoystickPushMessage::~JoystickPushMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoInterface::JoystickPushMessage::JoystickPushMessage(const JoystickPushMessage *m) : Message("JoystickPushMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (JoystickPushMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get button value.
 * Button ID to push.
 * @return button value
 */
uint32_t
JacoInterface::JoystickPushMessage::button() const
{
  return data->button;
}

/** Get maximum length of button value.
 * @return length of button value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoInterface::JoystickPushMessage::maxlenof_button() const
{
  return 1;
}

/** Set button value.
 * Button ID to push.
 * @param new_button new button value
 */
void
JacoInterface::JoystickPushMessage::set_button(const uint32_t new_button)
{
  data->button = new_button;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JacoInterface::JoystickPushMessage::clone() const
{
  return new JacoInterface::JoystickPushMessage(this);
}
/** @class JacoInterface::JoystickReleaseMessage <interfaces/JacoInterface.h>
 * JoystickReleaseMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
JacoInterface::JoystickReleaseMessage::JoystickReleaseMessage() : Message("JoystickReleaseMessage")
{
  data_size = sizeof(JoystickReleaseMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (JoystickReleaseMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
JacoInterface::JoystickReleaseMessage::~JoystickReleaseMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoInterface::JoystickReleaseMessage::JoystickReleaseMessage(const JoystickReleaseMessage *m) : Message("JoystickReleaseMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (JoystickReleaseMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JacoInterface::JoystickReleaseMessage::clone() const
{
  return new JacoInterface::JoystickReleaseMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
JacoInterface::message_valid(const Message *message) const
{
  const CalibrateMessage *m0 = dynamic_cast<const CalibrateMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const RetractMessage *m1 = dynamic_cast<const RetractMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const StopMessage *m2 = dynamic_cast<const StopMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const CartesianGotoMessage *m3 = dynamic_cast<const CartesianGotoMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const AngularGotoMessage *m4 = dynamic_cast<const AngularGotoMessage *>(message);
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
  const JoystickPushMessage *m7 = dynamic_cast<const JoystickPushMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const JoystickReleaseMessage *m8 = dynamic_cast<const JoystickReleaseMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(JacoInterface)
/// @endcond


} // end namespace fawkes
