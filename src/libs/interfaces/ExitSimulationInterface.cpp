
/***************************************************************************
 *  ExitSimulationInterface.cpp - Fawkes BlackBoard Interface - ExitSimulationInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  Gesche Gierse
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

#include <interfaces/ExitSimulationInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class ExitSimulationInterface <interfaces/ExitSimulationInterface.h>
 * ExitSimulationInterface Fawkes BlackBoard Interface.
 * Exit simulation interface. Use this to exit fawkes and the simulation.
 * @ingroup FawkesInterfaces
 */



/** Constructor */
ExitSimulationInterface::ExitSimulationInterface() : Interface()
{
  data_size = sizeof(ExitSimulationInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (ExitSimulationInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_BOOL, "shutdown_initiated", 1, &data->shutdown_initiated);
  add_messageinfo("ExitSimulationMessage");
  unsigned char tmp_hash[] = {0xbf, 0xa, 0x70, 0x60, 0x7f, 0xe8, 0xb2, 0xaf, 0x54, 0xce, 0x2d, 0xf7, 0xff, 0x79, 0x84, 0x40};
  set_hash(tmp_hash);
}

/** Destructor */
ExitSimulationInterface::~ExitSimulationInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get shutdown_initiated value.
 * Whether a shutdown was initiated
 * @return shutdown_initiated value
 */
bool
ExitSimulationInterface::is_shutdown_initiated() const
{
  return data->shutdown_initiated;
}

/** Get maximum length of shutdown_initiated value.
 * @return length of shutdown_initiated value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ExitSimulationInterface::maxlenof_shutdown_initiated() const
{
  return 1;
}

/** Set shutdown_initiated value.
 * Whether a shutdown was initiated
 * @param new_shutdown_initiated new shutdown_initiated value
 */
void
ExitSimulationInterface::set_shutdown_initiated(const bool new_shutdown_initiated)
{
  data->shutdown_initiated = new_shutdown_initiated;
  data_changed = true;
}

/* =========== message create =========== */
Message *
ExitSimulationInterface::create_message(const char *type) const
{
  if ( strncmp("ExitSimulationMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ExitSimulationMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
ExitSimulationInterface::copy_values(const Interface *other)
{
  const ExitSimulationInterface *oi = dynamic_cast<const ExitSimulationInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(ExitSimulationInterface_data_t));
}

const char *
ExitSimulationInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class ExitSimulationInterface::ExitSimulationMessage <interfaces/ExitSimulationInterface.h>
 * ExitSimulationMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
ExitSimulationInterface::ExitSimulationMessage::ExitSimulationMessage() : Message("ExitSimulationMessage")
{
  data_size = sizeof(ExitSimulationMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ExitSimulationMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
ExitSimulationInterface::ExitSimulationMessage::~ExitSimulationMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
ExitSimulationInterface::ExitSimulationMessage::ExitSimulationMessage(const ExitSimulationMessage *m) : Message("ExitSimulationMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ExitSimulationMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
ExitSimulationInterface::ExitSimulationMessage::clone() const
{
  return new ExitSimulationInterface::ExitSimulationMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
ExitSimulationInterface::message_valid(const Message *message) const
{
  const ExitSimulationMessage *m0 = dynamic_cast<const ExitSimulationMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(ExitSimulationInterface)
/// @endcond


} // end namespace fawkes
