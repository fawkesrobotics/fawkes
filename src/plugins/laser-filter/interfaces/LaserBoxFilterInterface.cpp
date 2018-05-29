
/***************************************************************************
 *  LaserBoxFilterInterface.cpp - Fawkes BlackBoard Interface - LaserBoxFilterInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2018  Nicolas Limpert
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

#include <interfaces/LaserBoxFilterInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class LaserBoxFilterInterface <interfaces/LaserBoxFilterInterface.h>
 * LaserBoxFilterInterface Fawkes BlackBoard Interface.
 * 
      Interface to create new laser filters at runtime.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
LaserBoxFilterInterface::LaserBoxFilterInterface() : Interface()
{
  data_size = sizeof(LaserBoxFilterInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (LaserBoxFilterInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "num_boxes", 1, &data->num_boxes);
  add_messageinfo("CreateNewBoxFilterMessage");
  unsigned char tmp_hash[] = {0xd5, 0xd3, 0x35, 0xa5, 0xf5, 0xeb, 0xfe, 0xe0, 0x2e, 0x9e, 0xda, 0xa8, 0x77, 0x6f, 0x3, 0x74};
  set_hash(tmp_hash);
}

/** Destructor */
LaserBoxFilterInterface::~LaserBoxFilterInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get num_boxes value.
 * The number of currently applied boxes
 * @return num_boxes value
 */
uint32_t
LaserBoxFilterInterface::num_boxes() const
{
  return data->num_boxes;
}

/** Get maximum length of num_boxes value.
 * @return length of num_boxes value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserBoxFilterInterface::maxlenof_num_boxes() const
{
  return 1;
}

/** Set num_boxes value.
 * The number of currently applied boxes
 * @param new_num_boxes new num_boxes value
 */
void
LaserBoxFilterInterface::set_num_boxes(const uint32_t new_num_boxes)
{
  data->num_boxes = new_num_boxes;
  data_changed = true;
}

/* =========== message create =========== */
Message *
LaserBoxFilterInterface::create_message(const char *type) const
{
  if ( strncmp("CreateNewBoxFilterMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CreateNewBoxFilterMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
LaserBoxFilterInterface::copy_values(const Interface *other)
{
  const LaserBoxFilterInterface *oi = dynamic_cast<const LaserBoxFilterInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(LaserBoxFilterInterface_data_t));
}

const char *
LaserBoxFilterInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class LaserBoxFilterInterface::CreateNewBoxFilterMessage <interfaces/LaserBoxFilterInterface.h>
 * CreateNewBoxFilterMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_p1 initial value for p1
 * @param ini_p2 initial value for p2
 * @param ini_p3 initial value for p3
 * @param ini_p4 initial value for p4
 */
LaserBoxFilterInterface::CreateNewBoxFilterMessage::CreateNewBoxFilterMessage(const double * ini_p1, const double * ini_p2, const double * ini_p3, const double * ini_p4) : Message("CreateNewBoxFilterMessage")
{
  data_size = sizeof(CreateNewBoxFilterMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CreateNewBoxFilterMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  memcpy(data->p1, ini_p1, sizeof(double) * 2);
  memcpy(data->p2, ini_p2, sizeof(double) * 2);
  memcpy(data->p3, ini_p3, sizeof(double) * 2);
  memcpy(data->p4, ini_p4, sizeof(double) * 2);
  add_fieldinfo(IFT_DOUBLE, "p1", 2, &data->p1);
  add_fieldinfo(IFT_DOUBLE, "p2", 2, &data->p2);
  add_fieldinfo(IFT_DOUBLE, "p3", 2, &data->p3);
  add_fieldinfo(IFT_DOUBLE, "p4", 2, &data->p4);
}
/** Constructor */
LaserBoxFilterInterface::CreateNewBoxFilterMessage::CreateNewBoxFilterMessage() : Message("CreateNewBoxFilterMessage")
{
  data_size = sizeof(CreateNewBoxFilterMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CreateNewBoxFilterMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_DOUBLE, "p1", 2, &data->p1);
  add_fieldinfo(IFT_DOUBLE, "p2", 2, &data->p2);
  add_fieldinfo(IFT_DOUBLE, "p3", 2, &data->p3);
  add_fieldinfo(IFT_DOUBLE, "p4", 2, &data->p4);
}

/** Destructor */
LaserBoxFilterInterface::CreateNewBoxFilterMessage::~CreateNewBoxFilterMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
LaserBoxFilterInterface::CreateNewBoxFilterMessage::CreateNewBoxFilterMessage(const CreateNewBoxFilterMessage *m) : Message("CreateNewBoxFilterMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CreateNewBoxFilterMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get p1 value.
 * x,y coordinates of first vector
 * @return p1 value
 */
double *
LaserBoxFilterInterface::CreateNewBoxFilterMessage::p1() const
{
  return data->p1;
}

/** Get p1 value at given index.
 * x,y coordinates of first vector
 * @param index index of value
 * @return p1 value
 * @exception Exception thrown if index is out of bounds
 */
double
LaserBoxFilterInterface::CreateNewBoxFilterMessage::p1(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->p1[index];
}

/** Get maximum length of p1 value.
 * @return length of p1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserBoxFilterInterface::CreateNewBoxFilterMessage::maxlenof_p1() const
{
  return 2;
}

/** Set p1 value.
 * x,y coordinates of first vector
 * @param new_p1 new p1 value
 */
void
LaserBoxFilterInterface::CreateNewBoxFilterMessage::set_p1(const double * new_p1)
{
  memcpy(data->p1, new_p1, sizeof(double) * 2);
}

/** Set p1 value at given index.
 * x,y coordinates of first vector
 * @param new_p1 new p1 value
 * @param index index for of the value
 */
void
LaserBoxFilterInterface::CreateNewBoxFilterMessage::set_p1(unsigned int index, const double new_p1)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->p1[index] = new_p1;
}
/** Get p2 value.
 * x,y coordinates of second vector
 * @return p2 value
 */
double *
LaserBoxFilterInterface::CreateNewBoxFilterMessage::p2() const
{
  return data->p2;
}

/** Get p2 value at given index.
 * x,y coordinates of second vector
 * @param index index of value
 * @return p2 value
 * @exception Exception thrown if index is out of bounds
 */
double
LaserBoxFilterInterface::CreateNewBoxFilterMessage::p2(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->p2[index];
}

/** Get maximum length of p2 value.
 * @return length of p2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserBoxFilterInterface::CreateNewBoxFilterMessage::maxlenof_p2() const
{
  return 2;
}

/** Set p2 value.
 * x,y coordinates of second vector
 * @param new_p2 new p2 value
 */
void
LaserBoxFilterInterface::CreateNewBoxFilterMessage::set_p2(const double * new_p2)
{
  memcpy(data->p2, new_p2, sizeof(double) * 2);
}

/** Set p2 value at given index.
 * x,y coordinates of second vector
 * @param new_p2 new p2 value
 * @param index index for of the value
 */
void
LaserBoxFilterInterface::CreateNewBoxFilterMessage::set_p2(unsigned int index, const double new_p2)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->p2[index] = new_p2;
}
/** Get p3 value.
 * x,y coordinates of third vector
 * @return p3 value
 */
double *
LaserBoxFilterInterface::CreateNewBoxFilterMessage::p3() const
{
  return data->p3;
}

/** Get p3 value at given index.
 * x,y coordinates of third vector
 * @param index index of value
 * @return p3 value
 * @exception Exception thrown if index is out of bounds
 */
double
LaserBoxFilterInterface::CreateNewBoxFilterMessage::p3(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->p3[index];
}

/** Get maximum length of p3 value.
 * @return length of p3 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserBoxFilterInterface::CreateNewBoxFilterMessage::maxlenof_p3() const
{
  return 2;
}

/** Set p3 value.
 * x,y coordinates of third vector
 * @param new_p3 new p3 value
 */
void
LaserBoxFilterInterface::CreateNewBoxFilterMessage::set_p3(const double * new_p3)
{
  memcpy(data->p3, new_p3, sizeof(double) * 2);
}

/** Set p3 value at given index.
 * x,y coordinates of third vector
 * @param new_p3 new p3 value
 * @param index index for of the value
 */
void
LaserBoxFilterInterface::CreateNewBoxFilterMessage::set_p3(unsigned int index, const double new_p3)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->p3[index] = new_p3;
}
/** Get p4 value.
 * x,y coordinates of fourth vector
 * @return p4 value
 */
double *
LaserBoxFilterInterface::CreateNewBoxFilterMessage::p4() const
{
  return data->p4;
}

/** Get p4 value at given index.
 * x,y coordinates of fourth vector
 * @param index index of value
 * @return p4 value
 * @exception Exception thrown if index is out of bounds
 */
double
LaserBoxFilterInterface::CreateNewBoxFilterMessage::p4(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->p4[index];
}

/** Get maximum length of p4 value.
 * @return length of p4 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserBoxFilterInterface::CreateNewBoxFilterMessage::maxlenof_p4() const
{
  return 2;
}

/** Set p4 value.
 * x,y coordinates of fourth vector
 * @param new_p4 new p4 value
 */
void
LaserBoxFilterInterface::CreateNewBoxFilterMessage::set_p4(const double * new_p4)
{
  memcpy(data->p4, new_p4, sizeof(double) * 2);
}

/** Set p4 value at given index.
 * x,y coordinates of fourth vector
 * @param new_p4 new p4 value
 * @param index index for of the value
 */
void
LaserBoxFilterInterface::CreateNewBoxFilterMessage::set_p4(unsigned int index, const double new_p4)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->p4[index] = new_p4;
}
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
LaserBoxFilterInterface::CreateNewBoxFilterMessage::clone() const
{
  return new LaserBoxFilterInterface::CreateNewBoxFilterMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
LaserBoxFilterInterface::message_valid(const Message *message) const
{
  const CreateNewBoxFilterMessage *m0 = dynamic_cast<const CreateNewBoxFilterMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(LaserBoxFilterInterface)
/// @endcond


} // end namespace fawkes
