
/***************************************************************************
 *  SkillerDebugInterface.cpp - Fawkes BlackBoard Interface - SkillerDebugInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
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
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "graph_fsm", 32, data->graph_fsm);
  add_fieldinfo(IFT_STRING, "graph", 8192, data->graph);
  add_fieldinfo(IFT_ENUM, "graph_dir", 1, &data->graph_dir, "GraphDirectionEnum");
  add_fieldinfo(IFT_BOOL, "graph_colored", 1, &data->graph_colored);
  add_messageinfo("SetGraphMessage");
  add_messageinfo("SetGraphDirectionMessage");
  add_messageinfo("SetGraphColoredMessage");
  unsigned char tmp_hash[] = {0xcf, 0x3d, 0x2f, 0xf8, 0x80, 0x6e, 0x8f, 0xf4, 0x81, 0xa6, 0x7f, 0xd9, 0xb0, 0x29, 0xfc, 0x62};
  set_hash(tmp_hash);
}

/** Destructor */
SkillerDebugInterface::~SkillerDebugInterface()
{
  free(data_ptr);
}
/** Convert GraphDirectionEnum constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
SkillerDebugInterface::tostring_GraphDirectionEnum(GraphDirectionEnum value) const
{
  switch (value) {
  case GD_TOP_BOTTOM: return "GD_TOP_BOTTOM";
  case GD_BOTTOM_TOP: return "GD_BOTTOM_TOP";
  case GD_LEFT_RIGHT: return "GD_LEFT_RIGHT";
  case GD_RIGHT_LEFT: return "GD_RIGHT_LEFT";
  default: return "UNKNOWN";
  }
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
  data_changed = true;
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
  data_changed = true;
}

/** Get graph_dir value.
 * 
      Primary direction of current graph.
    
 * @return graph_dir value
 */
SkillerDebugInterface::GraphDirectionEnum
SkillerDebugInterface::graph_dir() const
{
  return data->graph_dir;
}

/** Get maximum length of graph_dir value.
 * @return length of graph_dir value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerDebugInterface::maxlenof_graph_dir() const
{
  return 1;
}

/** Set graph_dir value.
 * 
      Primary direction of current graph.
    
 * @param new_graph_dir new graph_dir value
 */
void
SkillerDebugInterface::set_graph_dir(const GraphDirectionEnum new_graph_dir)
{
  data->graph_dir = new_graph_dir;
  data_changed = true;
}

/** Get graph_colored value.
 * 
      True if the graph is colored, false otherwise.
    
 * @return graph_colored value
 */
bool
SkillerDebugInterface::is_graph_colored() const
{
  return data->graph_colored;
}

/** Get maximum length of graph_colored value.
 * @return length of graph_colored value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerDebugInterface::maxlenof_graph_colored() const
{
  return 1;
}

/** Set graph_colored value.
 * 
      True if the graph is colored, false otherwise.
    
 * @param new_graph_colored new graph_colored value
 */
void
SkillerDebugInterface::set_graph_colored(const bool new_graph_colored)
{
  data->graph_colored = new_graph_colored;
  data_changed = true;
}

/* =========== message create =========== */
Message *
SkillerDebugInterface::create_message(const char *type) const
{
  if ( strncmp("SetGraphMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetGraphMessage();
  } else if ( strncmp("SetGraphDirectionMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetGraphDirectionMessage();
  } else if ( strncmp("SetGraphColoredMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetGraphColoredMessage();
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

const char *
SkillerDebugInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "GraphDirectionEnum") == 0) {
    return tostring_GraphDirectionEnum((GraphDirectionEnum)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
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
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->graph_fsm, ini_graph_fsm, 32);
  add_fieldinfo(IFT_STRING, "graph_fsm", 32, data->graph_fsm);
}
/** Constructor */
SkillerDebugInterface::SetGraphMessage::SetGraphMessage() : Message("SetGraphMessage")
{
  data_size = sizeof(SetGraphMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGraphMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "graph_fsm", 32, data->graph_fsm);
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
  data_ts   = (message_data_ts_t *)data_ptr;
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
/** @class SkillerDebugInterface::SetGraphDirectionMessage <interfaces/SkillerDebugInterface.h>
 * SetGraphDirectionMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_graph_dir initial value for graph_dir
 */
SkillerDebugInterface::SetGraphDirectionMessage::SetGraphDirectionMessage(const GraphDirectionEnum ini_graph_dir) : Message("SetGraphDirectionMessage")
{
  data_size = sizeof(SetGraphDirectionMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGraphDirectionMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->graph_dir = ini_graph_dir;
  add_fieldinfo(IFT_ENUM, "graph_dir", 1, &data->graph_dir, "GraphDirectionEnum");
}
/** Constructor */
SkillerDebugInterface::SetGraphDirectionMessage::SetGraphDirectionMessage() : Message("SetGraphDirectionMessage")
{
  data_size = sizeof(SetGraphDirectionMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGraphDirectionMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_ENUM, "graph_dir", 1, &data->graph_dir, "GraphDirectionEnum");
}

/** Destructor */
SkillerDebugInterface::SetGraphDirectionMessage::~SetGraphDirectionMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerDebugInterface::SetGraphDirectionMessage::SetGraphDirectionMessage(const SetGraphDirectionMessage *m) : Message("SetGraphDirectionMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetGraphDirectionMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get graph_dir value.
 * 
      Primary direction of current graph.
    
 * @return graph_dir value
 */
SkillerDebugInterface::GraphDirectionEnum
SkillerDebugInterface::SetGraphDirectionMessage::graph_dir() const
{
  return data->graph_dir;
}

/** Get maximum length of graph_dir value.
 * @return length of graph_dir value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerDebugInterface::SetGraphDirectionMessage::maxlenof_graph_dir() const
{
  return 1;
}

/** Set graph_dir value.
 * 
      Primary direction of current graph.
    
 * @param new_graph_dir new graph_dir value
 */
void
SkillerDebugInterface::SetGraphDirectionMessage::set_graph_dir(const GraphDirectionEnum new_graph_dir)
{
  data->graph_dir = new_graph_dir;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SkillerDebugInterface::SetGraphDirectionMessage::clone() const
{
  return new SkillerDebugInterface::SetGraphDirectionMessage(this);
}
/** @class SkillerDebugInterface::SetGraphColoredMessage <interfaces/SkillerDebugInterface.h>
 * SetGraphColoredMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_graph_colored initial value for graph_colored
 */
SkillerDebugInterface::SetGraphColoredMessage::SetGraphColoredMessage(const bool ini_graph_colored) : Message("SetGraphColoredMessage")
{
  data_size = sizeof(SetGraphColoredMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGraphColoredMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->graph_colored = ini_graph_colored;
  add_fieldinfo(IFT_BOOL, "graph_colored", 1, &data->graph_colored);
}
/** Constructor */
SkillerDebugInterface::SetGraphColoredMessage::SetGraphColoredMessage() : Message("SetGraphColoredMessage")
{
  data_size = sizeof(SetGraphColoredMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGraphColoredMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_BOOL, "graph_colored", 1, &data->graph_colored);
}

/** Destructor */
SkillerDebugInterface::SetGraphColoredMessage::~SetGraphColoredMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerDebugInterface::SetGraphColoredMessage::SetGraphColoredMessage(const SetGraphColoredMessage *m) : Message("SetGraphColoredMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetGraphColoredMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get graph_colored value.
 * 
      True if the graph is colored, false otherwise.
    
 * @return graph_colored value
 */
bool
SkillerDebugInterface::SetGraphColoredMessage::is_graph_colored() const
{
  return data->graph_colored;
}

/** Get maximum length of graph_colored value.
 * @return length of graph_colored value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerDebugInterface::SetGraphColoredMessage::maxlenof_graph_colored() const
{
  return 1;
}

/** Set graph_colored value.
 * 
      True if the graph is colored, false otherwise.
    
 * @param new_graph_colored new graph_colored value
 */
void
SkillerDebugInterface::SetGraphColoredMessage::set_graph_colored(const bool new_graph_colored)
{
  data->graph_colored = new_graph_colored;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SkillerDebugInterface::SetGraphColoredMessage::clone() const
{
  return new SkillerDebugInterface::SetGraphColoredMessage(this);
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
  const SetGraphDirectionMessage *m1 = dynamic_cast<const SetGraphDirectionMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const SetGraphColoredMessage *m2 = dynamic_cast<const SetGraphColoredMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(SkillerDebugInterface)
/// @endcond


} // end namespace fawkes
