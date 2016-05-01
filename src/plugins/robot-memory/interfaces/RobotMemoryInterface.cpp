
/***************************************************************************
 *  RobotMemoryInterface.cpp - Fawkes BlackBoard Interface - RobotMemoryInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  Frederik Zwilling
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

#include <interfaces/RobotMemoryInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class RobotMemoryInterface <interfaces/RobotMemoryInterface.h>
 * RobotMemoryInterface Fawkes BlackBoard Interface.
 * 
      Interface to access robot Memory
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
RobotMemoryInterface::RobotMemoryInterface() : Interface()
{
  data_size = sizeof(RobotMemoryInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (RobotMemoryInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "error", 1024, data->error);
  add_fieldinfo(IFT_STRING, "result", 1024, data->result);
  add_messageinfo("QueryMessage");
  unsigned char tmp_hash[] = {0xb5, 0xf6, 0xa0, 0x13, 0xd1, 0x30, 0x84, 0x19, 0xb6, 0x98, 0x2c, 0x3d, 0x92, 0xa4, 0x7a, 0xf};
  set_hash(tmp_hash);
}

/** Destructor */
RobotMemoryInterface::~RobotMemoryInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get error value.
 * Error of last query
 * @return error value
 */
char *
RobotMemoryInterface::error() const
{
  return data->error;
}

/** Get maximum length of error value.
 * @return length of error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotMemoryInterface::maxlenof_error() const
{
  return 1024;
}

/** Set error value.
 * Error of last query
 * @param new_error new error value
 */
void
RobotMemoryInterface::set_error(const char * new_error)
{
  strncpy(data->error, new_error, sizeof(data->error));
  data_changed = true;
}

/** Get result value.
 * Result of last query
 * @return result value
 */
char *
RobotMemoryInterface::result() const
{
  return data->result;
}

/** Get maximum length of result value.
 * @return length of result value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotMemoryInterface::maxlenof_result() const
{
  return 1024;
}

/** Set result value.
 * Result of last query
 * @param new_result new result value
 */
void
RobotMemoryInterface::set_result(const char * new_result)
{
  strncpy(data->result, new_result, sizeof(data->result));
  data_changed = true;
}

/* =========== message create =========== */
Message *
RobotMemoryInterface::create_message(const char *type) const
{
  if ( strncmp("QueryMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new QueryMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
RobotMemoryInterface::copy_values(const Interface *other)
{
  const RobotMemoryInterface *oi = dynamic_cast<const RobotMemoryInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(RobotMemoryInterface_data_t));
}

const char *
RobotMemoryInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class RobotMemoryInterface::QueryMessage <interfaces/RobotMemoryInterface.h>
 * QueryMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_query initial value for query
 */
RobotMemoryInterface::QueryMessage::QueryMessage(const char * ini_query) : Message("QueryMessage")
{
  data_size = sizeof(QueryMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (QueryMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->query, ini_query, 1024);
  add_fieldinfo(IFT_STRING, "query", 1024, data->query);
}
/** Constructor */
RobotMemoryInterface::QueryMessage::QueryMessage() : Message("QueryMessage")
{
  data_size = sizeof(QueryMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (QueryMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "query", 1024, data->query);
}

/** Destructor */
RobotMemoryInterface::QueryMessage::~QueryMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
RobotMemoryInterface::QueryMessage::QueryMessage(const QueryMessage *m) : Message("QueryMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (QueryMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get query value.
 * Query as JSON string
 * @return query value
 */
char *
RobotMemoryInterface::QueryMessage::query() const
{
  return data->query;
}

/** Get maximum length of query value.
 * @return length of query value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotMemoryInterface::QueryMessage::maxlenof_query() const
{
  return 1024;
}

/** Set query value.
 * Query as JSON string
 * @param new_query new query value
 */
void
RobotMemoryInterface::QueryMessage::set_query(const char * new_query)
{
  strncpy(data->query, new_query, sizeof(data->query));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
RobotMemoryInterface::QueryMessage::clone() const
{
  return new RobotMemoryInterface::QueryMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
RobotMemoryInterface::message_valid(const Message *message) const
{
  const QueryMessage *m0 = dynamic_cast<const QueryMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(RobotMemoryInterface)
/// @endcond


} // end namespace fawkes
