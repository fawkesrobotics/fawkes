
/***************************************************************************
 *  JacoBimanualInterface.cpp - Fawkes BlackBoard Interface - JacoBimanualInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2014  Bahram Maleki-Fard
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

#include <interfaces/JacoBimanualInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class JacoBimanualInterface <interfaces/JacoBimanualInterface.h>
 * JacoBimanualInterface Fawkes BlackBoard Interface.
 * 
      Interface for coordinate bimanual manipulation with a Kinova Jaco arm.
    
 * @ingroup FawkesInterfaces
 */


/** ERROR_NONE constant */
const uint32_t JacoBimanualInterface::ERROR_NONE = 0u;
/** ERROR_UNSPECIFIC constant */
const uint32_t JacoBimanualInterface::ERROR_UNSPECIFIC = 1u;
/** ERROR_NO_IK constant */
const uint32_t JacoBimanualInterface::ERROR_NO_IK = 2u;
/** ERROR_PLANNING constant */
const uint32_t JacoBimanualInterface::ERROR_PLANNING = 4u;

/** Constructor */
JacoBimanualInterface::JacoBimanualInterface() : Interface()
{
  data_size = sizeof(JacoBimanualInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (JacoBimanualInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_UINT32, "error_code", 1, &data->error_code);
  add_fieldinfo(IFT_BOOL, "constrained", 1, &data->constrained);
  add_messageinfo("CartesianGotoMessage");
  add_messageinfo("MoveGripperMessage");
  add_messageinfo("SetPlannerParamsMessage");
  add_messageinfo("SetConstrainedMessage");
  unsigned char tmp_hash[] = {0x7c, 0x62, 0x7a, 0x5a, 0xc6, 0xc1, 0xb4, 0x12, 0x6f, 0xa4, 0x89, 0x89, 0xb8, 0xe5, 0x1, 0x66};
  set_hash(tmp_hash);
}

/** Destructor */
JacoBimanualInterface::~JacoBimanualInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @return msgid value
 */
uint32_t
JacoBimanualInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @param new_msgid new msgid value
 */
void
JacoBimanualInterface::set_msgid(const uint32_t new_msgid)
{
  data->msgid = new_msgid;
  data_changed = true;
}

/** Get final value.
 * True, if the last command has been finished,
      false if it is still running
 * @return final value
 */
bool
JacoBimanualInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last command has been finished,
      false if it is still running
 * @param new_final new final value
 */
void
JacoBimanualInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/** Get error_code value.
 * Error code, set if
      final is true. 0 if no error occured, an error code from ERROR_*
      constants otherwise.
 * @return error_code value
 */
uint32_t
JacoBimanualInterface::error_code() const
{
  return data->error_code;
}

/** Get maximum length of error_code value.
 * @return length of error_code value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::maxlenof_error_code() const
{
  return 1;
}

/** Set error_code value.
 * Error code, set if
      final is true. 0 if no error occured, an error code from ERROR_*
      constants otherwise.
 * @param new_error_code new error_code value
 */
void
JacoBimanualInterface::set_error_code(const uint32_t new_error_code)
{
  data->error_code = new_error_code;
  data_changed = true;
}

/** Get constrained value.
 * Wheter planning is using constraint-function.
      This is an OpenRAVE internal constraint function.
 * @return constrained value
 */
bool
JacoBimanualInterface::is_constrained() const
{
  return data->constrained;
}

/** Get maximum length of constrained value.
 * @return length of constrained value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::maxlenof_constrained() const
{
  return 1;
}

/** Set constrained value.
 * Wheter planning is using constraint-function.
      This is an OpenRAVE internal constraint function.
 * @param new_constrained new constrained value
 */
void
JacoBimanualInterface::set_constrained(const bool new_constrained)
{
  data->constrained = new_constrained;
  data_changed = true;
}

/* =========== message create =========== */
Message *
JacoBimanualInterface::create_message(const char *type) const
{
  if ( strncmp("CartesianGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CartesianGotoMessage();
  } else if ( strncmp("MoveGripperMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new MoveGripperMessage();
  } else if ( strncmp("SetPlannerParamsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetPlannerParamsMessage();
  } else if ( strncmp("SetConstrainedMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetConstrainedMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
JacoBimanualInterface::copy_values(const Interface *other)
{
  const JacoBimanualInterface *oi = dynamic_cast<const JacoBimanualInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(JacoBimanualInterface_data_t));
}

const char *
JacoBimanualInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class JacoBimanualInterface::CartesianGotoMessage <interfaces/JacoBimanualInterface.h>
 * CartesianGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_l_x initial value for l_x
 * @param ini_l_y initial value for l_y
 * @param ini_l_z initial value for l_z
 * @param ini_l_e1 initial value for l_e1
 * @param ini_l_e2 initial value for l_e2
 * @param ini_l_e3 initial value for l_e3
 * @param ini_r_x initial value for r_x
 * @param ini_r_y initial value for r_y
 * @param ini_r_z initial value for r_z
 * @param ini_r_e1 initial value for r_e1
 * @param ini_r_e2 initial value for r_e2
 * @param ini_r_e3 initial value for r_e3
 */
JacoBimanualInterface::CartesianGotoMessage::CartesianGotoMessage(const float ini_l_x, const float ini_l_y, const float ini_l_z, const float ini_l_e1, const float ini_l_e2, const float ini_l_e3, const float ini_r_x, const float ini_r_y, const float ini_r_z, const float ini_r_e1, const float ini_r_e2, const float ini_r_e3) : Message("CartesianGotoMessage")
{
  data_size = sizeof(CartesianGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->l_x = ini_l_x;
  data->l_y = ini_l_y;
  data->l_z = ini_l_z;
  data->l_e1 = ini_l_e1;
  data->l_e2 = ini_l_e2;
  data->l_e3 = ini_l_e3;
  data->r_x = ini_r_x;
  data->r_y = ini_r_y;
  data->r_z = ini_r_z;
  data->r_e1 = ini_r_e1;
  data->r_e2 = ini_r_e2;
  data->r_e3 = ini_r_e3;
  add_fieldinfo(IFT_FLOAT, "l_x", 1, &data->l_x);
  add_fieldinfo(IFT_FLOAT, "l_y", 1, &data->l_y);
  add_fieldinfo(IFT_FLOAT, "l_z", 1, &data->l_z);
  add_fieldinfo(IFT_FLOAT, "l_e1", 1, &data->l_e1);
  add_fieldinfo(IFT_FLOAT, "l_e2", 1, &data->l_e2);
  add_fieldinfo(IFT_FLOAT, "l_e3", 1, &data->l_e3);
  add_fieldinfo(IFT_FLOAT, "r_x", 1, &data->r_x);
  add_fieldinfo(IFT_FLOAT, "r_y", 1, &data->r_y);
  add_fieldinfo(IFT_FLOAT, "r_z", 1, &data->r_z);
  add_fieldinfo(IFT_FLOAT, "r_e1", 1, &data->r_e1);
  add_fieldinfo(IFT_FLOAT, "r_e2", 1, &data->r_e2);
  add_fieldinfo(IFT_FLOAT, "r_e3", 1, &data->r_e3);
}
/** Constructor */
JacoBimanualInterface::CartesianGotoMessage::CartesianGotoMessage() : Message("CartesianGotoMessage")
{
  data_size = sizeof(CartesianGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "l_x", 1, &data->l_x);
  add_fieldinfo(IFT_FLOAT, "l_y", 1, &data->l_y);
  add_fieldinfo(IFT_FLOAT, "l_z", 1, &data->l_z);
  add_fieldinfo(IFT_FLOAT, "l_e1", 1, &data->l_e1);
  add_fieldinfo(IFT_FLOAT, "l_e2", 1, &data->l_e2);
  add_fieldinfo(IFT_FLOAT, "l_e3", 1, &data->l_e3);
  add_fieldinfo(IFT_FLOAT, "r_x", 1, &data->r_x);
  add_fieldinfo(IFT_FLOAT, "r_y", 1, &data->r_y);
  add_fieldinfo(IFT_FLOAT, "r_z", 1, &data->r_z);
  add_fieldinfo(IFT_FLOAT, "r_e1", 1, &data->r_e1);
  add_fieldinfo(IFT_FLOAT, "r_e2", 1, &data->r_e2);
  add_fieldinfo(IFT_FLOAT, "r_e3", 1, &data->r_e3);
}

/** Destructor */
JacoBimanualInterface::CartesianGotoMessage::~CartesianGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoBimanualInterface::CartesianGotoMessage::CartesianGotoMessage(const CartesianGotoMessage *m) : Message("CartesianGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get l_x value.
 * X-coordinate of left manipulator
 * @return l_x value
 */
float
JacoBimanualInterface::CartesianGotoMessage::l_x() const
{
  return data->l_x;
}

/** Get maximum length of l_x value.
 * @return length of l_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_l_x() const
{
  return 1;
}

/** Set l_x value.
 * X-coordinate of left manipulator
 * @param new_l_x new l_x value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_l_x(const float new_l_x)
{
  data->l_x = new_l_x;
}

/** Get l_y value.
 * Y-coordinate of left manipulator
 * @return l_y value
 */
float
JacoBimanualInterface::CartesianGotoMessage::l_y() const
{
  return data->l_y;
}

/** Get maximum length of l_y value.
 * @return length of l_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_l_y() const
{
  return 1;
}

/** Set l_y value.
 * Y-coordinate of left manipulator
 * @param new_l_y new l_y value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_l_y(const float new_l_y)
{
  data->l_y = new_l_y;
}

/** Get l_z value.
 * Z-coordinate of left manipulator
 * @return l_z value
 */
float
JacoBimanualInterface::CartesianGotoMessage::l_z() const
{
  return data->l_z;
}

/** Get maximum length of l_z value.
 * @return length of l_z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_l_z() const
{
  return 1;
}

/** Set l_z value.
 * Z-coordinate of left manipulator
 * @param new_l_z new l_z value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_l_z(const float new_l_z)
{
  data->l_z = new_l_z;
}

/** Get l_e1 value.
 * 1st Euler angle of left manipulator rotation
 * @return l_e1 value
 */
float
JacoBimanualInterface::CartesianGotoMessage::l_e1() const
{
  return data->l_e1;
}

/** Get maximum length of l_e1 value.
 * @return length of l_e1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_l_e1() const
{
  return 1;
}

/** Set l_e1 value.
 * 1st Euler angle of left manipulator rotation
 * @param new_l_e1 new l_e1 value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_l_e1(const float new_l_e1)
{
  data->l_e1 = new_l_e1;
}

/** Get l_e2 value.
 * 2nd Euler angle of left manipulator rotation
 * @return l_e2 value
 */
float
JacoBimanualInterface::CartesianGotoMessage::l_e2() const
{
  return data->l_e2;
}

/** Get maximum length of l_e2 value.
 * @return length of l_e2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_l_e2() const
{
  return 1;
}

/** Set l_e2 value.
 * 2nd Euler angle of left manipulator rotation
 * @param new_l_e2 new l_e2 value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_l_e2(const float new_l_e2)
{
  data->l_e2 = new_l_e2;
}

/** Get l_e3 value.
 * 3rd Euler angle of left manipulator rotation
 * @return l_e3 value
 */
float
JacoBimanualInterface::CartesianGotoMessage::l_e3() const
{
  return data->l_e3;
}

/** Get maximum length of l_e3 value.
 * @return length of l_e3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_l_e3() const
{
  return 1;
}

/** Set l_e3 value.
 * 3rd Euler angle of left manipulator rotation
 * @param new_l_e3 new l_e3 value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_l_e3(const float new_l_e3)
{
  data->l_e3 = new_l_e3;
}

/** Get r_x value.
 * X-coordinate of right manipulator
 * @return r_x value
 */
float
JacoBimanualInterface::CartesianGotoMessage::r_x() const
{
  return data->r_x;
}

/** Get maximum length of r_x value.
 * @return length of r_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_r_x() const
{
  return 1;
}

/** Set r_x value.
 * X-coordinate of right manipulator
 * @param new_r_x new r_x value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_r_x(const float new_r_x)
{
  data->r_x = new_r_x;
}

/** Get r_y value.
 * Y-coordinate of right manipulator
 * @return r_y value
 */
float
JacoBimanualInterface::CartesianGotoMessage::r_y() const
{
  return data->r_y;
}

/** Get maximum length of r_y value.
 * @return length of r_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_r_y() const
{
  return 1;
}

/** Set r_y value.
 * Y-coordinate of right manipulator
 * @param new_r_y new r_y value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_r_y(const float new_r_y)
{
  data->r_y = new_r_y;
}

/** Get r_z value.
 * Z-coordinate of right manipulator
 * @return r_z value
 */
float
JacoBimanualInterface::CartesianGotoMessage::r_z() const
{
  return data->r_z;
}

/** Get maximum length of r_z value.
 * @return length of r_z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_r_z() const
{
  return 1;
}

/** Set r_z value.
 * Z-coordinate of right manipulator
 * @param new_r_z new r_z value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_r_z(const float new_r_z)
{
  data->r_z = new_r_z;
}

/** Get r_e1 value.
 * 1st Euler angle of right manipulator rotation
 * @return r_e1 value
 */
float
JacoBimanualInterface::CartesianGotoMessage::r_e1() const
{
  return data->r_e1;
}

/** Get maximum length of r_e1 value.
 * @return length of r_e1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_r_e1() const
{
  return 1;
}

/** Set r_e1 value.
 * 1st Euler angle of right manipulator rotation
 * @param new_r_e1 new r_e1 value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_r_e1(const float new_r_e1)
{
  data->r_e1 = new_r_e1;
}

/** Get r_e2 value.
 * 2nd Euler angle of right manipulator rotation
 * @return r_e2 value
 */
float
JacoBimanualInterface::CartesianGotoMessage::r_e2() const
{
  return data->r_e2;
}

/** Get maximum length of r_e2 value.
 * @return length of r_e2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_r_e2() const
{
  return 1;
}

/** Set r_e2 value.
 * 2nd Euler angle of right manipulator rotation
 * @param new_r_e2 new r_e2 value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_r_e2(const float new_r_e2)
{
  data->r_e2 = new_r_e2;
}

/** Get r_e3 value.
 * 3rd Euler angle of right manipulator rotation
 * @return r_e3 value
 */
float
JacoBimanualInterface::CartesianGotoMessage::r_e3() const
{
  return data->r_e3;
}

/** Get maximum length of r_e3 value.
 * @return length of r_e3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::CartesianGotoMessage::maxlenof_r_e3() const
{
  return 1;
}

/** Set r_e3 value.
 * 3rd Euler angle of right manipulator rotation
 * @param new_r_e3 new r_e3 value
 */
void
JacoBimanualInterface::CartesianGotoMessage::set_r_e3(const float new_r_e3)
{
  data->r_e3 = new_r_e3;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JacoBimanualInterface::CartesianGotoMessage::clone() const
{
  return new JacoBimanualInterface::CartesianGotoMessage(this);
}
/** @class JacoBimanualInterface::MoveGripperMessage <interfaces/JacoBimanualInterface.h>
 * MoveGripperMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_l_finger1 initial value for l_finger1
 * @param ini_l_finger2 initial value for l_finger2
 * @param ini_l_finger3 initial value for l_finger3
 * @param ini_r_finger1 initial value for r_finger1
 * @param ini_r_finger2 initial value for r_finger2
 * @param ini_r_finger3 initial value for r_finger3
 */
JacoBimanualInterface::MoveGripperMessage::MoveGripperMessage(const float ini_l_finger1, const float ini_l_finger2, const float ini_l_finger3, const float ini_r_finger1, const float ini_r_finger2, const float ini_r_finger3) : Message("MoveGripperMessage")
{
  data_size = sizeof(MoveGripperMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->l_finger1 = ini_l_finger1;
  data->l_finger2 = ini_l_finger2;
  data->l_finger3 = ini_l_finger3;
  data->r_finger1 = ini_r_finger1;
  data->r_finger2 = ini_r_finger2;
  data->r_finger3 = ini_r_finger3;
  add_fieldinfo(IFT_FLOAT, "l_finger1", 1, &data->l_finger1);
  add_fieldinfo(IFT_FLOAT, "l_finger2", 1, &data->l_finger2);
  add_fieldinfo(IFT_FLOAT, "l_finger3", 1, &data->l_finger3);
  add_fieldinfo(IFT_FLOAT, "r_finger1", 1, &data->r_finger1);
  add_fieldinfo(IFT_FLOAT, "r_finger2", 1, &data->r_finger2);
  add_fieldinfo(IFT_FLOAT, "r_finger3", 1, &data->r_finger3);
}
/** Constructor */
JacoBimanualInterface::MoveGripperMessage::MoveGripperMessage() : Message("MoveGripperMessage")
{
  data_size = sizeof(MoveGripperMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "l_finger1", 1, &data->l_finger1);
  add_fieldinfo(IFT_FLOAT, "l_finger2", 1, &data->l_finger2);
  add_fieldinfo(IFT_FLOAT, "l_finger3", 1, &data->l_finger3);
  add_fieldinfo(IFT_FLOAT, "r_finger1", 1, &data->r_finger1);
  add_fieldinfo(IFT_FLOAT, "r_finger2", 1, &data->r_finger2);
  add_fieldinfo(IFT_FLOAT, "r_finger3", 1, &data->r_finger3);
}

/** Destructor */
JacoBimanualInterface::MoveGripperMessage::~MoveGripperMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoBimanualInterface::MoveGripperMessage::MoveGripperMessage(const MoveGripperMessage *m) : Message("MoveGripperMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (MoveGripperMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get l_finger1 value.
 * Value of finger 1 on left gripper. Range [0,60]
 * @return l_finger1 value
 */
float
JacoBimanualInterface::MoveGripperMessage::l_finger1() const
{
  return data->l_finger1;
}

/** Get maximum length of l_finger1 value.
 * @return length of l_finger1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::MoveGripperMessage::maxlenof_l_finger1() const
{
  return 1;
}

/** Set l_finger1 value.
 * Value of finger 1 on left gripper. Range [0,60]
 * @param new_l_finger1 new l_finger1 value
 */
void
JacoBimanualInterface::MoveGripperMessage::set_l_finger1(const float new_l_finger1)
{
  data->l_finger1 = new_l_finger1;
}

/** Get l_finger2 value.
 * Value of finger 2 on left gripper. Range [0,60]
 * @return l_finger2 value
 */
float
JacoBimanualInterface::MoveGripperMessage::l_finger2() const
{
  return data->l_finger2;
}

/** Get maximum length of l_finger2 value.
 * @return length of l_finger2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::MoveGripperMessage::maxlenof_l_finger2() const
{
  return 1;
}

/** Set l_finger2 value.
 * Value of finger 2 on left gripper. Range [0,60]
 * @param new_l_finger2 new l_finger2 value
 */
void
JacoBimanualInterface::MoveGripperMessage::set_l_finger2(const float new_l_finger2)
{
  data->l_finger2 = new_l_finger2;
}

/** Get l_finger3 value.
 * Value of finger 3 on left gripper. Range [0,60]
 * @return l_finger3 value
 */
float
JacoBimanualInterface::MoveGripperMessage::l_finger3() const
{
  return data->l_finger3;
}

/** Get maximum length of l_finger3 value.
 * @return length of l_finger3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::MoveGripperMessage::maxlenof_l_finger3() const
{
  return 1;
}

/** Set l_finger3 value.
 * Value of finger 3 on left gripper. Range [0,60]
 * @param new_l_finger3 new l_finger3 value
 */
void
JacoBimanualInterface::MoveGripperMessage::set_l_finger3(const float new_l_finger3)
{
  data->l_finger3 = new_l_finger3;
}

/** Get r_finger1 value.
 * Value of finger 1 on right gripper. Range [0,60]
 * @return r_finger1 value
 */
float
JacoBimanualInterface::MoveGripperMessage::r_finger1() const
{
  return data->r_finger1;
}

/** Get maximum length of r_finger1 value.
 * @return length of r_finger1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::MoveGripperMessage::maxlenof_r_finger1() const
{
  return 1;
}

/** Set r_finger1 value.
 * Value of finger 1 on right gripper. Range [0,60]
 * @param new_r_finger1 new r_finger1 value
 */
void
JacoBimanualInterface::MoveGripperMessage::set_r_finger1(const float new_r_finger1)
{
  data->r_finger1 = new_r_finger1;
}

/** Get r_finger2 value.
 * Value of finger 2 on right gripper. Range [0,60]
 * @return r_finger2 value
 */
float
JacoBimanualInterface::MoveGripperMessage::r_finger2() const
{
  return data->r_finger2;
}

/** Get maximum length of r_finger2 value.
 * @return length of r_finger2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::MoveGripperMessage::maxlenof_r_finger2() const
{
  return 1;
}

/** Set r_finger2 value.
 * Value of finger 2 on right gripper. Range [0,60]
 * @param new_r_finger2 new r_finger2 value
 */
void
JacoBimanualInterface::MoveGripperMessage::set_r_finger2(const float new_r_finger2)
{
  data->r_finger2 = new_r_finger2;
}

/** Get r_finger3 value.
 * Value of finger 3 on right gripper. Range [0,60]
 * @return r_finger3 value
 */
float
JacoBimanualInterface::MoveGripperMessage::r_finger3() const
{
  return data->r_finger3;
}

/** Get maximum length of r_finger3 value.
 * @return length of r_finger3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::MoveGripperMessage::maxlenof_r_finger3() const
{
  return 1;
}

/** Set r_finger3 value.
 * Value of finger 3 on right gripper. Range [0,60]
 * @param new_r_finger3 new r_finger3 value
 */
void
JacoBimanualInterface::MoveGripperMessage::set_r_finger3(const float new_r_finger3)
{
  data->r_finger3 = new_r_finger3;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JacoBimanualInterface::MoveGripperMessage::clone() const
{
  return new JacoBimanualInterface::MoveGripperMessage(this);
}
/** @class JacoBimanualInterface::SetPlannerParamsMessage <interfaces/JacoBimanualInterface.h>
 * SetPlannerParamsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_params initial value for params
 */
JacoBimanualInterface::SetPlannerParamsMessage::SetPlannerParamsMessage(const char * ini_params) : Message("SetPlannerParamsMessage")
{
  data_size = sizeof(SetPlannerParamsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPlannerParamsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->params, ini_params, 1024-1);
  data->params[1024-1] = 0;
  add_fieldinfo(IFT_STRING, "params", 1024, data->params);
}
/** Constructor */
JacoBimanualInterface::SetPlannerParamsMessage::SetPlannerParamsMessage() : Message("SetPlannerParamsMessage")
{
  data_size = sizeof(SetPlannerParamsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPlannerParamsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "params", 1024, data->params);
}

/** Destructor */
JacoBimanualInterface::SetPlannerParamsMessage::~SetPlannerParamsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoBimanualInterface::SetPlannerParamsMessage::SetPlannerParamsMessage(const SetPlannerParamsMessage *m) : Message("SetPlannerParamsMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetPlannerParamsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get params value.
 * Planner parameters
 * @return params value
 */
char *
JacoBimanualInterface::SetPlannerParamsMessage::params() const
{
  return data->params;
}

/** Get maximum length of params value.
 * @return length of params value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::SetPlannerParamsMessage::maxlenof_params() const
{
  return 1024;
}

/** Set params value.
 * Planner parameters
 * @param new_params new params value
 */
void
JacoBimanualInterface::SetPlannerParamsMessage::set_params(const char * new_params)
{
  strncpy(data->params, new_params, sizeof(data->params)-1);
  data->params[sizeof(data->params)-1] = 0;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JacoBimanualInterface::SetPlannerParamsMessage::clone() const
{
  return new JacoBimanualInterface::SetPlannerParamsMessage(this);
}
/** @class JacoBimanualInterface::SetConstrainedMessage <interfaces/JacoBimanualInterface.h>
 * SetConstrainedMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_constrained initial value for constrained
 */
JacoBimanualInterface::SetConstrainedMessage::SetConstrainedMessage(const bool ini_constrained) : Message("SetConstrainedMessage")
{
  data_size = sizeof(SetConstrainedMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetConstrainedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->constrained = ini_constrained;
  add_fieldinfo(IFT_BOOL, "constrained", 1, &data->constrained);
}
/** Constructor */
JacoBimanualInterface::SetConstrainedMessage::SetConstrainedMessage() : Message("SetConstrainedMessage")
{
  data_size = sizeof(SetConstrainedMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetConstrainedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_BOOL, "constrained", 1, &data->constrained);
}

/** Destructor */
JacoBimanualInterface::SetConstrainedMessage::~SetConstrainedMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
JacoBimanualInterface::SetConstrainedMessage::SetConstrainedMessage(const SetConstrainedMessage *m) : Message("SetConstrainedMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetConstrainedMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get constrained value.
 * Wheter planning is using constraint-function.
      This is an OpenRAVE internal constraint function.
 * @return constrained value
 */
bool
JacoBimanualInterface::SetConstrainedMessage::is_constrained() const
{
  return data->constrained;
}

/** Get maximum length of constrained value.
 * @return length of constrained value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
JacoBimanualInterface::SetConstrainedMessage::maxlenof_constrained() const
{
  return 1;
}

/** Set constrained value.
 * Wheter planning is using constraint-function.
      This is an OpenRAVE internal constraint function.
 * @param new_constrained new constrained value
 */
void
JacoBimanualInterface::SetConstrainedMessage::set_constrained(const bool new_constrained)
{
  data->constrained = new_constrained;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
JacoBimanualInterface::SetConstrainedMessage::clone() const
{
  return new JacoBimanualInterface::SetConstrainedMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
JacoBimanualInterface::message_valid(const Message *message) const
{
  const CartesianGotoMessage *m0 = dynamic_cast<const CartesianGotoMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const MoveGripperMessage *m1 = dynamic_cast<const MoveGripperMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const SetPlannerParamsMessage *m2 = dynamic_cast<const SetPlannerParamsMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const SetConstrainedMessage *m3 = dynamic_cast<const SetConstrainedMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(JacoBimanualInterface)
/// @endcond


} // end namespace fawkes
