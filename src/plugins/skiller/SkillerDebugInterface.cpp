
/***************************************************************************
 *  SkillerDebugInterface.cpp - Fawkes BlackBoard Interface - SkillerDebugInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
 *
 *  $Id$
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

#include <interfaces/SkillerDebugInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class SkillerDebugInterface <interfaces/SkillerDebugInterface.h>
 * SkillerDebugInterface Fawkes BlackBoard Interface.
 * 
      This interface provides internal skiller data that should allow for
      easier debugging of skills and the skiller in general. The most notable
      feature is a graph representation in the dot language of the available
      skills (and highlighting for the currently active skill).
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
SkillerDebugInterface::SkillerDebugInterface() : Interface()
{
  data_size = sizeof(SkillerDebugInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (SkillerDebugInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(Interface::IFT_STRING, "graph_fsm", 32, data->graph_fsm);
  add_fieldinfo(Interface::IFT_STRING, "graph", 8192, data->graph);
  unsigned char tmp_hash[] = {0x61, 0x5d, 0x9, 0x2d, 0x6f, 0x1, 0x20, 0xbc, 0xb1, 0xba, 0x95, 0xfa, 0xdc, 0x75, 0x73, 0xa4};
  set_hash(tmp_hash);
}

/** Destructor */
SkillerDebugInterface::~SkillerDebugInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get graph_fsm value.
 * 
      The finite state machine (FSM) the current graph has been updated for.
    
 * @return graph_fsm value
 */
char *
SkillerDebugInterface::graph_fsm() const
{
  return data->graph_fsm;
}

/** Get maximum length of graph_fsm value.
 * @return length of graph_fsm value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerDebugInterface::maxlenof_graph_fsm() const
{
  return 32;
}

/** Set graph_fsm value.
 * 
      The finite state machine (FSM) the current graph has been updated for.
    
 * @param new_graph_fsm new graph_fsm value
 */
void
SkillerDebugInterface::set_graph_fsm(const char * new_graph_fsm)
{
  strncpy(data->graph_fsm, new_graph_fsm, sizeof(data->graph_fsm));
}

/** Get graph value.
 * 
      The selected graph in a dot string representation.
    
 * @return graph value
 */
char *
SkillerDebugInterface::graph() const
{
  return data->graph;
}

/** Get maximum length of graph value.
 * @return length of graph value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerDebugInterface::maxlenof_graph() const
{
  return 8192;
}

/** Set graph value.
 * 
      The selected graph in a dot string representation.
    
 * @param new_graph new graph value
 */
void
SkillerDebugInterface::set_graph(const char * new_graph)
{
  strncpy(data->graph, new_graph, sizeof(data->graph));
}

/* =========== message create =========== */
Message *
SkillerDebugInterface::create_message(const char *type) const
{
  if ( strncmp("SetGraphMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetGraphMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
SkillerDebugInterface::copy_values(const Interface *other)
{
  const SkillerDebugInterface *oi = dynamic_cast<const SkillerDebugInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(SkillerDebugInterface_data_t));
}

/* =========== messages =========== */
/** @class SkillerDebugInterface::SetGraphMessage <interfaces/SkillerDebugInterface.h>
 * SetGraphMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_graph_fsm initial value for graph_fsm
 */
SkillerDebugInterface::SetGraphMessage::SetGraphMessage(const char * ini_graph_fsm) : Message("SetGraphMessage")
{
  data_size = sizeof(SetGraphMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGraphMessage_data_t *)data_ptr;
  strncpy(data->graph_fsm, ini_graph_fsm, 32);
}
/** Constructor */
SkillerDebugInterface::SetGraphMessage::SetGraphMessage() : Message("SetGraphMessage")
{
  data_size = sizeof(SetGraphMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGraphMessage_data_t *)data_ptr;
}

/** Destructor */
SkillerDebugInterface::SetGraphMessage::~SetGraphMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerDebugInterface::SetGraphMessage::SetGraphMessage(const SetGraphMessage *m) : Message("SetGraphMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetGraphMessage_data_t *)data_ptr;
}

/* Methods */
/** Get graph_fsm value.
 * 
      The finite state machine (FSM) the current graph has been updated for.
    
 * @return graph_fsm value
 */
char *
SkillerDebugInterface::SetGraphMessage::graph_fsm() const
{
  return data->graph_fsm;
}

/** Get maximum length of graph_fsm value.
 * @return length of graph_fsm value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerDebugInterface::SetGraphMessage::maxlenof_graph_fsm() const
{
  return 32;
}

/** Set graph_fsm value.
 * 
      The finite state machine (FSM) the current graph has been updated for.
    
 * @param new_graph_fsm new graph_fsm value
 */
void
SkillerDebugInterface::SetGraphMessage::set_graph_fsm(const char * new_graph_fsm)
{
  strncpy(data->graph_fsm, new_graph_fsm, sizeof(data->graph_fsm));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SkillerDebugInterface::SetGraphMessage::clone() const
{
  return new SkillerDebugInterface::SetGraphMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
SkillerDebugInterface::message_valid(const Message *message) const
{
  const SetGraphMessage *m0 = dynamic_cast<const SetGraphMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(SkillerDebugInterface)
/// @endcond


} // end namespace fawkes
