
/***************************************************************************
 *  NavGraphGeneratorInterface.cpp - Fawkes BlackBoard Interface - NavGraphGeneratorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2015  Tim Niemueller
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

#include <interfaces/NavGraphGeneratorInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class NavGraphGeneratorInterface <interfaces/NavGraphGeneratorInterface.h>
 * NavGraphGeneratorInterface Fawkes BlackBoard Interface.
 * 
      This interfaces is used to instruct the navgraph-generator.
      It allows to add obstacles and points of interest and perform
      a computation run of the graph generation.

      Operations to modify the parameters (clearing, adding/removing
      obstacles or points of interest etc.) take effect only once a
      ComputeMessage is sent. This can be used, for example, to first
      clear the graph, update it with the latest information, and
      finally generate the graph.

      As soon as any instruction has been received, the generato shall
      only listen to messages from the same sender. Only after a
      computation has been performed can another node send messages
      again.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
NavGraphGeneratorInterface::NavGraphGeneratorInterface() : Interface()
{
  data_size = sizeof(NavGraphGeneratorInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (NavGraphGeneratorInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "flags", 1, &data->flags);
  add_messageinfo("ClearMessage");
  add_messageinfo("SetBoundingBoxMessage");
  add_messageinfo("AddObstacleMessage");
  add_messageinfo("RemoveObstacleMessage");
  add_messageinfo("AddPointOfInterestMessage");
  add_messageinfo("SetPointOfInterestPropertyMessage");
  add_messageinfo("RemovePointOfInterestMessage");
  add_messageinfo("ComputeMessage");
  unsigned char tmp_hash[] = {0xe9, 0x12, 0x77, 0x11, 0x85, 0xa6, 0xca, 0xc8, 0xa6, 0xfa, 0xd2, 0x48, 0xab, 0x3a, 0xd0, 0xad};
  set_hash(tmp_hash);
}

/** Destructor */
NavGraphGeneratorInterface::~NavGraphGeneratorInterface()
{
  free(data_ptr);
}
/** Convert ConnectionMode constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
NavGraphGeneratorInterface::tostring_ConnectionMode(ConnectionMode value) const
{
  switch (value) {
  case CLOSEST_NODE: return "CLOSEST_NODE";
  case CLOSEST_EDGE: return "CLOSEST_EDGE";
  case CLOSEST_EDGE_OR_NODE: return "CLOSEST_EDGE_OR_NODE";
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
NavGraphGeneratorInterface::flags() const
{
  return data->flags;
}

/** Get maximum length of flags value.
 * @return length of flags value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::maxlenof_flags() const
{
  return 1;
}

/** Set flags value.
 * Bit-wise combination of
    FLAG_* constants denoting navigator component features.
 * @param new_flags new flags value
 */
void
NavGraphGeneratorInterface::set_flags(const uint32_t new_flags)
{
  data->flags = new_flags;
  data_changed = true;
}

/* =========== message create =========== */
Message *
NavGraphGeneratorInterface::create_message(const char *type) const
{
  if ( strncmp("ClearMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ClearMessage();
  } else if ( strncmp("SetBoundingBoxMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetBoundingBoxMessage();
  } else if ( strncmp("AddObstacleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddObstacleMessage();
  } else if ( strncmp("RemoveObstacleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RemoveObstacleMessage();
  } else if ( strncmp("AddPointOfInterestMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddPointOfInterestMessage();
  } else if ( strncmp("SetPointOfInterestPropertyMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetPointOfInterestPropertyMessage();
  } else if ( strncmp("RemovePointOfInterestMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RemovePointOfInterestMessage();
  } else if ( strncmp("ComputeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ComputeMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
NavGraphGeneratorInterface::copy_values(const Interface *other)
{
  const NavGraphGeneratorInterface *oi = dynamic_cast<const NavGraphGeneratorInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(NavGraphGeneratorInterface_data_t));
}

const char *
NavGraphGeneratorInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "ConnectionMode") == 0) {
    return tostring_ConnectionMode((ConnectionMode)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class NavGraphGeneratorInterface::ClearMessage <interfaces/NavGraphGeneratorInterface.h>
 * ClearMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
NavGraphGeneratorInterface::ClearMessage::ClearMessage() : Message("ClearMessage")
{
  data_size = sizeof(ClearMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ClearMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
NavGraphGeneratorInterface::ClearMessage::~ClearMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::ClearMessage::ClearMessage(const ClearMessage *m) : Message("ClearMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ClearMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::ClearMessage::clone() const
{
  return new NavGraphGeneratorInterface::ClearMessage(this);
}
/** @class NavGraphGeneratorInterface::SetBoundingBoxMessage <interfaces/NavGraphGeneratorInterface.h>
 * SetBoundingBoxMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_p1_x initial value for p1_x
 * @param ini_p1_y initial value for p1_y
 * @param ini_p2_x initial value for p2_x
 * @param ini_p2_y initial value for p2_y
 */
NavGraphGeneratorInterface::SetBoundingBoxMessage::SetBoundingBoxMessage(const float ini_p1_x, const float ini_p1_y, const float ini_p2_x, const float ini_p2_y) : Message("SetBoundingBoxMessage")
{
  data_size = sizeof(SetBoundingBoxMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetBoundingBoxMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->p1_x = ini_p1_x;
  data->p1_y = ini_p1_y;
  data->p2_x = ini_p2_x;
  data->p2_y = ini_p2_y;
  add_fieldinfo(IFT_FLOAT, "p1_x", 1, &data->p1_x);
  add_fieldinfo(IFT_FLOAT, "p1_y", 1, &data->p1_y);
  add_fieldinfo(IFT_FLOAT, "p2_x", 1, &data->p2_x);
  add_fieldinfo(IFT_FLOAT, "p2_y", 1, &data->p2_y);
}
/** Constructor */
NavGraphGeneratorInterface::SetBoundingBoxMessage::SetBoundingBoxMessage() : Message("SetBoundingBoxMessage")
{
  data_size = sizeof(SetBoundingBoxMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetBoundingBoxMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "p1_x", 1, &data->p1_x);
  add_fieldinfo(IFT_FLOAT, "p1_y", 1, &data->p1_y);
  add_fieldinfo(IFT_FLOAT, "p2_x", 1, &data->p2_x);
  add_fieldinfo(IFT_FLOAT, "p2_y", 1, &data->p2_y);
}

/** Destructor */
NavGraphGeneratorInterface::SetBoundingBoxMessage::~SetBoundingBoxMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::SetBoundingBoxMessage::SetBoundingBoxMessage(const SetBoundingBoxMessage *m) : Message("SetBoundingBoxMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetBoundingBoxMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get p1_x value.
 * X coordinate of bbox start point in global frame.
 * @return p1_x value
 */
float
NavGraphGeneratorInterface::SetBoundingBoxMessage::p1_x() const
{
  return data->p1_x;
}

/** Get maximum length of p1_x value.
 * @return length of p1_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetBoundingBoxMessage::maxlenof_p1_x() const
{
  return 1;
}

/** Set p1_x value.
 * X coordinate of bbox start point in global frame.
 * @param new_p1_x new p1_x value
 */
void
NavGraphGeneratorInterface::SetBoundingBoxMessage::set_p1_x(const float new_p1_x)
{
  data->p1_x = new_p1_x;
}

/** Get p1_y value.
 * Y coordinate of bbox start point in global frame.
 * @return p1_y value
 */
float
NavGraphGeneratorInterface::SetBoundingBoxMessage::p1_y() const
{
  return data->p1_y;
}

/** Get maximum length of p1_y value.
 * @return length of p1_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetBoundingBoxMessage::maxlenof_p1_y() const
{
  return 1;
}

/** Set p1_y value.
 * Y coordinate of bbox start point in global frame.
 * @param new_p1_y new p1_y value
 */
void
NavGraphGeneratorInterface::SetBoundingBoxMessage::set_p1_y(const float new_p1_y)
{
  data->p1_y = new_p1_y;
}

/** Get p2_x value.
 * X coordinate of bbox end point in global frame.
 * @return p2_x value
 */
float
NavGraphGeneratorInterface::SetBoundingBoxMessage::p2_x() const
{
  return data->p2_x;
}

/** Get maximum length of p2_x value.
 * @return length of p2_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetBoundingBoxMessage::maxlenof_p2_x() const
{
  return 1;
}

/** Set p2_x value.
 * X coordinate of bbox end point in global frame.
 * @param new_p2_x new p2_x value
 */
void
NavGraphGeneratorInterface::SetBoundingBoxMessage::set_p2_x(const float new_p2_x)
{
  data->p2_x = new_p2_x;
}

/** Get p2_y value.
 * Y coordinate of bbox end point in global frame.
 * @return p2_y value
 */
float
NavGraphGeneratorInterface::SetBoundingBoxMessage::p2_y() const
{
  return data->p2_y;
}

/** Get maximum length of p2_y value.
 * @return length of p2_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetBoundingBoxMessage::maxlenof_p2_y() const
{
  return 1;
}

/** Set p2_y value.
 * Y coordinate of bbox end point in global frame.
 * @param new_p2_y new p2_y value
 */
void
NavGraphGeneratorInterface::SetBoundingBoxMessage::set_p2_y(const float new_p2_y)
{
  data->p2_y = new_p2_y;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::SetBoundingBoxMessage::clone() const
{
  return new NavGraphGeneratorInterface::SetBoundingBoxMessage(this);
}
/** @class NavGraphGeneratorInterface::AddObstacleMessage <interfaces/NavGraphGeneratorInterface.h>
 * AddObstacleMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_id initial value for id
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 */
NavGraphGeneratorInterface::AddObstacleMessage::AddObstacleMessage(const char * ini_id, const float ini_x, const float ini_y) : Message("AddObstacleMessage")
{
  data_size = sizeof(AddObstacleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddObstacleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->id, ini_id, 64);
  data->x = ini_x;
  data->y = ini_y;
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
}
/** Constructor */
NavGraphGeneratorInterface::AddObstacleMessage::AddObstacleMessage() : Message("AddObstacleMessage")
{
  data_size = sizeof(AddObstacleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddObstacleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
}

/** Destructor */
NavGraphGeneratorInterface::AddObstacleMessage::~AddObstacleMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::AddObstacleMessage::AddObstacleMessage(const AddObstacleMessage *m) : Message("AddObstacleMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AddObstacleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get id value.
 * 
      ID of the obstacle. Can later be used to remove it again.
    
 * @return id value
 */
char *
NavGraphGeneratorInterface::AddObstacleMessage::id() const
{
  return data->id;
}

/** Get maximum length of id value.
 * @return length of id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddObstacleMessage::maxlenof_id() const
{
  return 64;
}

/** Set id value.
 * 
      ID of the obstacle. Can later be used to remove it again.
    
 * @param new_id new id value
 */
void
NavGraphGeneratorInterface::AddObstacleMessage::set_id(const char * new_id)
{
  strncpy(data->id, new_id, sizeof(data->id));
}

/** Get x value.
 * X coordinate of obstacle in global frame.
 * @return x value
 */
float
NavGraphGeneratorInterface::AddObstacleMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddObstacleMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X coordinate of obstacle in global frame.
 * @param new_x new x value
 */
void
NavGraphGeneratorInterface::AddObstacleMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y coordinate of obstacle in global frame.
 * @return y value
 */
float
NavGraphGeneratorInterface::AddObstacleMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddObstacleMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y coordinate of obstacle in global frame.
 * @param new_y new y value
 */
void
NavGraphGeneratorInterface::AddObstacleMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::AddObstacleMessage::clone() const
{
  return new NavGraphGeneratorInterface::AddObstacleMessage(this);
}
/** @class NavGraphGeneratorInterface::RemoveObstacleMessage <interfaces/NavGraphGeneratorInterface.h>
 * RemoveObstacleMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_id initial value for id
 */
NavGraphGeneratorInterface::RemoveObstacleMessage::RemoveObstacleMessage(const char * ini_id) : Message("RemoveObstacleMessage")
{
  data_size = sizeof(RemoveObstacleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RemoveObstacleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->id, ini_id, 64);
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
}
/** Constructor */
NavGraphGeneratorInterface::RemoveObstacleMessage::RemoveObstacleMessage() : Message("RemoveObstacleMessage")
{
  data_size = sizeof(RemoveObstacleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RemoveObstacleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
}

/** Destructor */
NavGraphGeneratorInterface::RemoveObstacleMessage::~RemoveObstacleMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::RemoveObstacleMessage::RemoveObstacleMessage(const RemoveObstacleMessage *m) : Message("RemoveObstacleMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (RemoveObstacleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get id value.
 * 
      ID of the obstacle to remove.
    
 * @return id value
 */
char *
NavGraphGeneratorInterface::RemoveObstacleMessage::id() const
{
  return data->id;
}

/** Get maximum length of id value.
 * @return length of id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::RemoveObstacleMessage::maxlenof_id() const
{
  return 64;
}

/** Set id value.
 * 
      ID of the obstacle to remove.
    
 * @param new_id new id value
 */
void
NavGraphGeneratorInterface::RemoveObstacleMessage::set_id(const char * new_id)
{
  strncpy(data->id, new_id, sizeof(data->id));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::RemoveObstacleMessage::clone() const
{
  return new NavGraphGeneratorInterface::RemoveObstacleMessage(this);
}
/** @class NavGraphGeneratorInterface::AddPointOfInterestMessage <interfaces/NavGraphGeneratorInterface.h>
 * AddPointOfInterestMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_id initial value for id
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_mode initial value for mode
 */
NavGraphGeneratorInterface::AddPointOfInterestMessage::AddPointOfInterestMessage(const char * ini_id, const float ini_x, const float ini_y, const ConnectionMode ini_mode) : Message("AddPointOfInterestMessage")
{
  data_size = sizeof(AddPointOfInterestMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddPointOfInterestMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->id, ini_id, 64);
  data->x = ini_x;
  data->y = ini_y;
  data->mode = ini_mode;
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_ENUM, "mode", 1, &data->mode, "ConnectionMode");
}
/** Constructor */
NavGraphGeneratorInterface::AddPointOfInterestMessage::AddPointOfInterestMessage() : Message("AddPointOfInterestMessage")
{
  data_size = sizeof(AddPointOfInterestMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddPointOfInterestMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_ENUM, "mode", 1, &data->mode, "ConnectionMode");
}

/** Destructor */
NavGraphGeneratorInterface::AddPointOfInterestMessage::~AddPointOfInterestMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::AddPointOfInterestMessage::AddPointOfInterestMessage(const AddPointOfInterestMessage *m) : Message("AddPointOfInterestMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AddPointOfInterestMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get id value.
 * 
      ID of the obstacle. Can later be used to remove it again.
    
 * @return id value
 */
char *
NavGraphGeneratorInterface::AddPointOfInterestMessage::id() const
{
  return data->id;
}

/** Get maximum length of id value.
 * @return length of id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddPointOfInterestMessage::maxlenof_id() const
{
  return 64;
}

/** Set id value.
 * 
      ID of the obstacle. Can later be used to remove it again.
    
 * @param new_id new id value
 */
void
NavGraphGeneratorInterface::AddPointOfInterestMessage::set_id(const char * new_id)
{
  strncpy(data->id, new_id, sizeof(data->id));
}

/** Get x value.
 * X coordinate of obstacle in global frame.
 * @return x value
 */
float
NavGraphGeneratorInterface::AddPointOfInterestMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddPointOfInterestMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X coordinate of obstacle in global frame.
 * @param new_x new x value
 */
void
NavGraphGeneratorInterface::AddPointOfInterestMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y coordinate of obstacle in global frame.
 * @return y value
 */
float
NavGraphGeneratorInterface::AddPointOfInterestMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddPointOfInterestMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y coordinate of obstacle in global frame.
 * @param new_y new y value
 */
void
NavGraphGeneratorInterface::AddPointOfInterestMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get mode value.
 * 
      The connection mode to use to connect the POI with the graph.
    
 * @return mode value
 */
NavGraphGeneratorInterface::ConnectionMode
NavGraphGeneratorInterface::AddPointOfInterestMessage::mode() const
{
  return (NavGraphGeneratorInterface::ConnectionMode)data->mode;
}

/** Get maximum length of mode value.
 * @return length of mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddPointOfInterestMessage::maxlenof_mode() const
{
  return 1;
}

/** Set mode value.
 * 
      The connection mode to use to connect the POI with the graph.
    
 * @param new_mode new mode value
 */
void
NavGraphGeneratorInterface::AddPointOfInterestMessage::set_mode(const ConnectionMode new_mode)
{
  data->mode = new_mode;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::AddPointOfInterestMessage::clone() const
{
  return new NavGraphGeneratorInterface::AddPointOfInterestMessage(this);
}
/** @class NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage <interfaces/NavGraphGeneratorInterface.h>
 * SetPointOfInterestPropertyMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_id initial value for id
 * @param ini_property_name initial value for property_name
 * @param ini_property_value initial value for property_value
 */
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::SetPointOfInterestPropertyMessage(const char * ini_id, const char * ini_property_name, const char * ini_property_value) : Message("SetPointOfInterestPropertyMessage")
{
  data_size = sizeof(SetPointOfInterestPropertyMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPointOfInterestPropertyMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->id, ini_id, 64);
  strncpy(data->property_name, ini_property_name, 64);
  strncpy(data->property_value, ini_property_value, 1024);
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_STRING, "property_name", 64, data->property_name);
  add_fieldinfo(IFT_STRING, "property_value", 1024, data->property_value);
}
/** Constructor */
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::SetPointOfInterestPropertyMessage() : Message("SetPointOfInterestPropertyMessage")
{
  data_size = sizeof(SetPointOfInterestPropertyMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPointOfInterestPropertyMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_STRING, "property_name", 64, data->property_name);
  add_fieldinfo(IFT_STRING, "property_value", 1024, data->property_value);
}

/** Destructor */
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::~SetPointOfInterestPropertyMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::SetPointOfInterestPropertyMessage(const SetPointOfInterestPropertyMessage *m) : Message("SetPointOfInterestPropertyMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetPointOfInterestPropertyMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get id value.
 * 
      ID of the point of interest.
    
 * @return id value
 */
char *
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::id() const
{
  return data->id;
}

/** Get maximum length of id value.
 * @return length of id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::maxlenof_id() const
{
  return 64;
}

/** Set id value.
 * 
      ID of the point of interest.
    
 * @param new_id new id value
 */
void
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::set_id(const char * new_id)
{
  strncpy(data->id, new_id, sizeof(data->id));
}

/** Get property_name value.
 * Name of the property to set.
 * @return property_name value
 */
char *
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::property_name() const
{
  return data->property_name;
}

/** Get maximum length of property_name value.
 * @return length of property_name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::maxlenof_property_name() const
{
  return 64;
}

/** Set property_name value.
 * Name of the property to set.
 * @param new_property_name new property_name value
 */
void
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::set_property_name(const char * new_property_name)
{
  strncpy(data->property_name, new_property_name, sizeof(data->property_name));
}

/** Get property_value value.
 * Value of the property
    to set.
 * @return property_value value
 */
char *
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::property_value() const
{
  return data->property_value;
}

/** Get maximum length of property_value value.
 * @return length of property_value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::maxlenof_property_value() const
{
  return 1024;
}

/** Set property_value value.
 * Value of the property
    to set.
 * @param new_property_value new property_value value
 */
void
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::set_property_value(const char * new_property_value)
{
  strncpy(data->property_value, new_property_value, sizeof(data->property_value));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage::clone() const
{
  return new NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage(this);
}
/** @class NavGraphGeneratorInterface::RemovePointOfInterestMessage <interfaces/NavGraphGeneratorInterface.h>
 * RemovePointOfInterestMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_id initial value for id
 */
NavGraphGeneratorInterface::RemovePointOfInterestMessage::RemovePointOfInterestMessage(const char * ini_id) : Message("RemovePointOfInterestMessage")
{
  data_size = sizeof(RemovePointOfInterestMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RemovePointOfInterestMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->id, ini_id, 64);
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
}
/** Constructor */
NavGraphGeneratorInterface::RemovePointOfInterestMessage::RemovePointOfInterestMessage() : Message("RemovePointOfInterestMessage")
{
  data_size = sizeof(RemovePointOfInterestMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RemovePointOfInterestMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
}

/** Destructor */
NavGraphGeneratorInterface::RemovePointOfInterestMessage::~RemovePointOfInterestMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::RemovePointOfInterestMessage::RemovePointOfInterestMessage(const RemovePointOfInterestMessage *m) : Message("RemovePointOfInterestMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (RemovePointOfInterestMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get id value.
 * 
      ID of the obstacle to remove.
    
 * @return id value
 */
char *
NavGraphGeneratorInterface::RemovePointOfInterestMessage::id() const
{
  return data->id;
}

/** Get maximum length of id value.
 * @return length of id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::RemovePointOfInterestMessage::maxlenof_id() const
{
  return 64;
}

/** Set id value.
 * 
      ID of the obstacle to remove.
    
 * @param new_id new id value
 */
void
NavGraphGeneratorInterface::RemovePointOfInterestMessage::set_id(const char * new_id)
{
  strncpy(data->id, new_id, sizeof(data->id));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::RemovePointOfInterestMessage::clone() const
{
  return new NavGraphGeneratorInterface::RemovePointOfInterestMessage(this);
}
/** @class NavGraphGeneratorInterface::ComputeMessage <interfaces/NavGraphGeneratorInterface.h>
 * ComputeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
NavGraphGeneratorInterface::ComputeMessage::ComputeMessage() : Message("ComputeMessage")
{
  data_size = sizeof(ComputeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ComputeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
NavGraphGeneratorInterface::ComputeMessage::~ComputeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::ComputeMessage::ComputeMessage(const ComputeMessage *m) : Message("ComputeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ComputeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::ComputeMessage::clone() const
{
  return new NavGraphGeneratorInterface::ComputeMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
NavGraphGeneratorInterface::message_valid(const Message *message) const
{
  const ClearMessage *m0 = dynamic_cast<const ClearMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetBoundingBoxMessage *m1 = dynamic_cast<const SetBoundingBoxMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const AddObstacleMessage *m2 = dynamic_cast<const AddObstacleMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const RemoveObstacleMessage *m3 = dynamic_cast<const RemoveObstacleMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const AddPointOfInterestMessage *m4 = dynamic_cast<const AddPointOfInterestMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const SetPointOfInterestPropertyMessage *m5 = dynamic_cast<const SetPointOfInterestPropertyMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const RemovePointOfInterestMessage *m6 = dynamic_cast<const RemovePointOfInterestMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const ComputeMessage *m7 = dynamic_cast<const ComputeMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NavGraphGeneratorInterface)
/// @endcond


} // end namespace fawkes
