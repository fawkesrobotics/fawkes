
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

#include <map>
#include <string>
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_UINT32, "flags", 1, &data->flags);
  add_messageinfo("ClearMessage");
  add_messageinfo("SetBoundingBoxMessage");
  add_messageinfo("SetFilterMessage");
  add_messageinfo("SetFilterParamFloatMessage");
  add_messageinfo("AddMapObstaclesMessage");
  add_messageinfo("AddObstacleMessage");
  add_messageinfo("RemoveObstacleMessage");
  add_messageinfo("AddPointOfInterestMessage");
  add_messageinfo("AddPointOfInterestWithOriMessage");
  add_messageinfo("SetPointOfInterestPropertyMessage");
  add_messageinfo("AddEdgeMessage");
  add_messageinfo("SetGraphDefaultPropertyMessage");
  add_messageinfo("SetCopyGraphDefaultPropertiesMessage");
  add_messageinfo("RemovePointOfInterestMessage");
  add_messageinfo("ComputeMessage");
  unsigned char tmp_hash[] = {0xd9, 0xef, 0x3c, 0x1e, 0x3d, 0x26, 0xa2, 0x70, 0x86, 0x9, 0xa2, 0x51, 0xe7, 0x86, 0x39, 0x9d};
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
  case NOT_CONNECTED: return "NOT_CONNECTED";
  case UNCONNECTED: return "UNCONNECTED";
  case CLOSEST_NODE: return "CLOSEST_NODE";
  case CLOSEST_EDGE: return "CLOSEST_EDGE";
  case CLOSEST_EDGE_OR_NODE: return "CLOSEST_EDGE_OR_NODE";
  default: return "UNKNOWN";
  }
}
/** Convert FilterType constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
NavGraphGeneratorInterface::tostring_FilterType(FilterType value) const
{
  switch (value) {
  case FILTER_EDGES_BY_MAP: return "FILTER_EDGES_BY_MAP";
  case FILTER_ORPHAN_NODES: return "FILTER_ORPHAN_NODES";
  case FILTER_MULTI_GRAPH: return "FILTER_MULTI_GRAPH";
  default: return "UNKNOWN";
  }
}
/** Convert EdgeMode constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
NavGraphGeneratorInterface::tostring_EdgeMode(EdgeMode value) const
{
  switch (value) {
  case NO_INTERSECTION: return "NO_INTERSECTION";
  case SPLIT_INTERSECTION: return "SPLIT_INTERSECTION";
  case FORCE: return "FORCE";
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
  } else if ( strncmp("SetFilterMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetFilterMessage();
  } else if ( strncmp("SetFilterParamFloatMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetFilterParamFloatMessage();
  } else if ( strncmp("AddMapObstaclesMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddMapObstaclesMessage();
  } else if ( strncmp("AddObstacleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddObstacleMessage();
  } else if ( strncmp("RemoveObstacleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RemoveObstacleMessage();
  } else if ( strncmp("AddPointOfInterestMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddPointOfInterestMessage();
  } else if ( strncmp("AddPointOfInterestWithOriMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddPointOfInterestWithOriMessage();
  } else if ( strncmp("SetPointOfInterestPropertyMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetPointOfInterestPropertyMessage();
  } else if ( strncmp("AddEdgeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddEdgeMessage();
  } else if ( strncmp("SetGraphDefaultPropertyMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetGraphDefaultPropertyMessage();
  } else if ( strncmp("SetCopyGraphDefaultPropertiesMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetCopyGraphDefaultPropertiesMessage();
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
  if (strcmp(enumtype, "FilterType") == 0) {
    return tostring_FilterType((FilterType)val);
  }
  if (strcmp(enumtype, "EdgeMode") == 0) {
    return tostring_EdgeMode((EdgeMode)val);
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
/** @class NavGraphGeneratorInterface::SetFilterMessage <interfaces/NavGraphGeneratorInterface.h>
 * SetFilterMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_filter initial value for filter
 * @param ini_enable initial value for enable
 */
NavGraphGeneratorInterface::SetFilterMessage::SetFilterMessage(const FilterType ini_filter, const bool ini_enable) : Message("SetFilterMessage")
{
  data_size = sizeof(SetFilterMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetFilterMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->filter = ini_filter;
  data->enable = ini_enable;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_ENUM, "filter", 1, &data->filter, "FilterType", &enum_map_FilterType);
  add_fieldinfo(IFT_BOOL, "enable", 1, &data->enable);
}
/** Constructor */
NavGraphGeneratorInterface::SetFilterMessage::SetFilterMessage() : Message("SetFilterMessage")
{
  data_size = sizeof(SetFilterMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetFilterMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_ENUM, "filter", 1, &data->filter, "FilterType", &enum_map_FilterType);
  add_fieldinfo(IFT_BOOL, "enable", 1, &data->enable);
}

/** Destructor */
NavGraphGeneratorInterface::SetFilterMessage::~SetFilterMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::SetFilterMessage::SetFilterMessage(const SetFilterMessage *m) : Message("SetFilterMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetFilterMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get filter value.
 * Which filter to
    enable/disable.
 * @return filter value
 */
NavGraphGeneratorInterface::FilterType
NavGraphGeneratorInterface::SetFilterMessage::filter() const
{
  return (NavGraphGeneratorInterface::FilterType)data->filter;
}

/** Get maximum length of filter value.
 * @return length of filter value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetFilterMessage::maxlenof_filter() const
{
  return 1;
}

/** Set filter value.
 * Which filter to
    enable/disable.
 * @param new_filter new filter value
 */
void
NavGraphGeneratorInterface::SetFilterMessage::set_filter(const FilterType new_filter)
{
  data->filter = new_filter;
}

/** Get enable value.
 * True to enable, false to
    disable
 * @return enable value
 */
bool
NavGraphGeneratorInterface::SetFilterMessage::is_enable() const
{
  return data->enable;
}

/** Get maximum length of enable value.
 * @return length of enable value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetFilterMessage::maxlenof_enable() const
{
  return 1;
}

/** Set enable value.
 * True to enable, false to
    disable
 * @param new_enable new enable value
 */
void
NavGraphGeneratorInterface::SetFilterMessage::set_enable(const bool new_enable)
{
  data->enable = new_enable;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::SetFilterMessage::clone() const
{
  return new NavGraphGeneratorInterface::SetFilterMessage(this);
}
/** @class NavGraphGeneratorInterface::SetFilterParamFloatMessage <interfaces/NavGraphGeneratorInterface.h>
 * SetFilterParamFloatMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_filter initial value for filter
 * @param ini_param initial value for param
 * @param ini_value initial value for value
 */
NavGraphGeneratorInterface::SetFilterParamFloatMessage::SetFilterParamFloatMessage(const FilterType ini_filter, const char * ini_param, const float ini_value) : Message("SetFilterParamFloatMessage")
{
  data_size = sizeof(SetFilterParamFloatMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetFilterParamFloatMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->filter = ini_filter;
  strncpy(data->param, ini_param, 32);
  data->value = ini_value;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_ENUM, "filter", 1, &data->filter, "FilterType", &enum_map_FilterType);
  add_fieldinfo(IFT_STRING, "param", 32, data->param);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
}
/** Constructor */
NavGraphGeneratorInterface::SetFilterParamFloatMessage::SetFilterParamFloatMessage() : Message("SetFilterParamFloatMessage")
{
  data_size = sizeof(SetFilterParamFloatMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetFilterParamFloatMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_ENUM, "filter", 1, &data->filter, "FilterType", &enum_map_FilterType);
  add_fieldinfo(IFT_STRING, "param", 32, data->param);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
}

/** Destructor */
NavGraphGeneratorInterface::SetFilterParamFloatMessage::~SetFilterParamFloatMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::SetFilterParamFloatMessage::SetFilterParamFloatMessage(const SetFilterParamFloatMessage *m) : Message("SetFilterParamFloatMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetFilterParamFloatMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get filter value.
 * Which filter to
    enable/disable.
 * @return filter value
 */
NavGraphGeneratorInterface::FilterType
NavGraphGeneratorInterface::SetFilterParamFloatMessage::filter() const
{
  return (NavGraphGeneratorInterface::FilterType)data->filter;
}

/** Get maximum length of filter value.
 * @return length of filter value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetFilterParamFloatMessage::maxlenof_filter() const
{
  return 1;
}

/** Set filter value.
 * Which filter to
    enable/disable.
 * @param new_filter new filter value
 */
void
NavGraphGeneratorInterface::SetFilterParamFloatMessage::set_filter(const FilterType new_filter)
{
  data->filter = new_filter;
}

/** Get param value.
 * Parameter name, see FilterType
    description for possible values.
 * @return param value
 */
char *
NavGraphGeneratorInterface::SetFilterParamFloatMessage::param() const
{
  return data->param;
}

/** Get maximum length of param value.
 * @return length of param value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetFilterParamFloatMessage::maxlenof_param() const
{
  return 32;
}

/** Set param value.
 * Parameter name, see FilterType
    description for possible values.
 * @param new_param new param value
 */
void
NavGraphGeneratorInterface::SetFilterParamFloatMessage::set_param(const char * new_param)
{
  strncpy(data->param, new_param, sizeof(data->param));
}

/** Get value value.
 * True to enable, false to
    disable
 * @return value value
 */
float
NavGraphGeneratorInterface::SetFilterParamFloatMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetFilterParamFloatMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * True to enable, false to
    disable
 * @param new_value new value value
 */
void
NavGraphGeneratorInterface::SetFilterParamFloatMessage::set_value(const float new_value)
{
  data->value = new_value;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::SetFilterParamFloatMessage::clone() const
{
  return new NavGraphGeneratorInterface::SetFilterParamFloatMessage(this);
}
/** @class NavGraphGeneratorInterface::AddMapObstaclesMessage <interfaces/NavGraphGeneratorInterface.h>
 * AddMapObstaclesMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_max_line_point_distance initial value for max_line_point_distance
 */
NavGraphGeneratorInterface::AddMapObstaclesMessage::AddMapObstaclesMessage(const float ini_max_line_point_distance) : Message("AddMapObstaclesMessage")
{
  data_size = sizeof(AddMapObstaclesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddMapObstaclesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->max_line_point_distance = ini_max_line_point_distance;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_FLOAT, "max_line_point_distance", 1, &data->max_line_point_distance);
}
/** Constructor */
NavGraphGeneratorInterface::AddMapObstaclesMessage::AddMapObstaclesMessage() : Message("AddMapObstaclesMessage")
{
  data_size = sizeof(AddMapObstaclesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddMapObstaclesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_FLOAT, "max_line_point_distance", 1, &data->max_line_point_distance);
}

/** Destructor */
NavGraphGeneratorInterface::AddMapObstaclesMessage::~AddMapObstaclesMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::AddMapObstaclesMessage::AddMapObstaclesMessage(const AddMapObstaclesMessage *m) : Message("AddMapObstaclesMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AddMapObstaclesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get max_line_point_distance value.
 * 
      For points generated on lines found in the map, do not exceed
      this threshold in terms of maximum distance of points on line.
    
 * @return max_line_point_distance value
 */
float
NavGraphGeneratorInterface::AddMapObstaclesMessage::max_line_point_distance() const
{
  return data->max_line_point_distance;
}

/** Get maximum length of max_line_point_distance value.
 * @return length of max_line_point_distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddMapObstaclesMessage::maxlenof_max_line_point_distance() const
{
  return 1;
}

/** Set max_line_point_distance value.
 * 
      For points generated on lines found in the map, do not exceed
      this threshold in terms of maximum distance of points on line.
    
 * @param new_max_line_point_distance new max_line_point_distance value
 */
void
NavGraphGeneratorInterface::AddMapObstaclesMessage::set_max_line_point_distance(const float new_max_line_point_distance)
{
  data->max_line_point_distance = new_max_line_point_distance;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::AddMapObstaclesMessage::clone() const
{
  return new NavGraphGeneratorInterface::AddMapObstaclesMessage(this);
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_ENUM, "mode", 1, &data->mode, "ConnectionMode", &enum_map_ConnectionMode);
}
/** Constructor */
NavGraphGeneratorInterface::AddPointOfInterestMessage::AddPointOfInterestMessage() : Message("AddPointOfInterestMessage")
{
  data_size = sizeof(AddPointOfInterestMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddPointOfInterestMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_ENUM, "mode", 1, &data->mode, "ConnectionMode", &enum_map_ConnectionMode);
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
/** @class NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage <interfaces/NavGraphGeneratorInterface.h>
 * AddPointOfInterestWithOriMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_id initial value for id
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_ori initial value for ori
 * @param ini_mode initial value for mode
 */
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::AddPointOfInterestWithOriMessage(const char * ini_id, const float ini_x, const float ini_y, const float ini_ori, const ConnectionMode ini_mode) : Message("AddPointOfInterestWithOriMessage")
{
  data_size = sizeof(AddPointOfInterestWithOriMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddPointOfInterestWithOriMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->id, ini_id, 64);
  data->x = ini_x;
  data->y = ini_y;
  data->ori = ini_ori;
  data->mode = ini_mode;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "ori", 1, &data->ori);
  add_fieldinfo(IFT_ENUM, "mode", 1, &data->mode, "ConnectionMode", &enum_map_ConnectionMode);
}
/** Constructor */
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::AddPointOfInterestWithOriMessage() : Message("AddPointOfInterestWithOriMessage")
{
  data_size = sizeof(AddPointOfInterestWithOriMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddPointOfInterestWithOriMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "ori", 1, &data->ori);
  add_fieldinfo(IFT_ENUM, "mode", 1, &data->mode, "ConnectionMode", &enum_map_ConnectionMode);
}

/** Destructor */
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::~AddPointOfInterestWithOriMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::AddPointOfInterestWithOriMessage(const AddPointOfInterestWithOriMessage *m) : Message("AddPointOfInterestWithOriMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AddPointOfInterestWithOriMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get id value.
 * 
      ID of the obstacle. Can later be used to remove it again.
    
 * @return id value
 */
char *
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::id() const
{
  return data->id;
}

/** Get maximum length of id value.
 * @return length of id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::maxlenof_id() const
{
  return 64;
}

/** Set id value.
 * 
      ID of the obstacle. Can later be used to remove it again.
    
 * @param new_id new id value
 */
void
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::set_id(const char * new_id)
{
  strncpy(data->id, new_id, sizeof(data->id));
}

/** Get x value.
 * X coordinate of obstacle in global frame.
 * @return x value
 */
float
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X coordinate of obstacle in global frame.
 * @param new_x new x value
 */
void
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y coordinate of obstacle in global frame.
 * @return y value
 */
float
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y coordinate of obstacle in global frame.
 * @param new_y new y value
 */
void
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get ori value.
 * Orientation for target point (rad).
 * @return ori value
 */
float
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::ori() const
{
  return data->ori;
}

/** Get maximum length of ori value.
 * @return length of ori value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::maxlenof_ori() const
{
  return 1;
}

/** Set ori value.
 * Orientation for target point (rad).
 * @param new_ori new ori value
 */
void
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::set_ori(const float new_ori)
{
  data->ori = new_ori;
}

/** Get mode value.
 * 
      The connection mode to use to connect the POI with the graph.
    
 * @return mode value
 */
NavGraphGeneratorInterface::ConnectionMode
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::mode() const
{
  return (NavGraphGeneratorInterface::ConnectionMode)data->mode;
}

/** Get maximum length of mode value.
 * @return length of mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::maxlenof_mode() const
{
  return 1;
}

/** Set mode value.
 * 
      The connection mode to use to connect the POI with the graph.
    
 * @param new_mode new mode value
 */
void
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::set_mode(const ConnectionMode new_mode)
{
  data->mode = new_mode;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage::clone() const
{
  return new NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage(this);
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
/** @class NavGraphGeneratorInterface::AddEdgeMessage <interfaces/NavGraphGeneratorInterface.h>
 * AddEdgeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_p1 initial value for p1
 * @param ini_p2 initial value for p2
 * @param ini_directed initial value for directed
 * @param ini_mode initial value for mode
 */
NavGraphGeneratorInterface::AddEdgeMessage::AddEdgeMessage(const char * ini_p1, const char * ini_p2, const bool ini_directed, const EdgeMode ini_mode) : Message("AddEdgeMessage")
{
  data_size = sizeof(AddEdgeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddEdgeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->p1, ini_p1, 64);
  strncpy(data->p2, ini_p2, 64);
  data->directed = ini_directed;
  data->mode = ini_mode;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_STRING, "p1", 64, data->p1);
  add_fieldinfo(IFT_STRING, "p2", 64, data->p2);
  add_fieldinfo(IFT_BOOL, "directed", 1, &data->directed);
  add_fieldinfo(IFT_ENUM, "mode", 1, &data->mode, "EdgeMode", &enum_map_EdgeMode);
}
/** Constructor */
NavGraphGeneratorInterface::AddEdgeMessage::AddEdgeMessage() : Message("AddEdgeMessage")
{
  data_size = sizeof(AddEdgeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddEdgeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_STRING, "p1", 64, data->p1);
  add_fieldinfo(IFT_STRING, "p2", 64, data->p2);
  add_fieldinfo(IFT_BOOL, "directed", 1, &data->directed);
  add_fieldinfo(IFT_ENUM, "mode", 1, &data->mode, "EdgeMode", &enum_map_EdgeMode);
}

/** Destructor */
NavGraphGeneratorInterface::AddEdgeMessage::~AddEdgeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::AddEdgeMessage::AddEdgeMessage(const AddEdgeMessage *m) : Message("AddEdgeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AddEdgeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get p1 value.
 * ID of first node.
 * @return p1 value
 */
char *
NavGraphGeneratorInterface::AddEdgeMessage::p1() const
{
  return data->p1;
}

/** Get maximum length of p1 value.
 * @return length of p1 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddEdgeMessage::maxlenof_p1() const
{
  return 64;
}

/** Set p1 value.
 * ID of first node.
 * @param new_p1 new p1 value
 */
void
NavGraphGeneratorInterface::AddEdgeMessage::set_p1(const char * new_p1)
{
  strncpy(data->p1, new_p1, sizeof(data->p1));
}

/** Get p2 value.
 * ID of second node.
 * @return p2 value
 */
char *
NavGraphGeneratorInterface::AddEdgeMessage::p2() const
{
  return data->p2;
}

/** Get maximum length of p2 value.
 * @return length of p2 value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddEdgeMessage::maxlenof_p2() const
{
  return 64;
}

/** Set p2 value.
 * ID of second node.
 * @param new_p2 new p2 value
 */
void
NavGraphGeneratorInterface::AddEdgeMessage::set_p2(const char * new_p2)
{
  strncpy(data->p2, new_p2, sizeof(data->p2));
}

/** Get directed value.
 * 
      True to create a directed edge from p1 to p2, otherwise the edge
      is assumed to be undirected.
    
 * @return directed value
 */
bool
NavGraphGeneratorInterface::AddEdgeMessage::is_directed() const
{
  return data->directed;
}

/** Get maximum length of directed value.
 * @return length of directed value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddEdgeMessage::maxlenof_directed() const
{
  return 1;
}

/** Set directed value.
 * 
      True to create a directed edge from p1 to p2, otherwise the edge
      is assumed to be undirected.
    
 * @param new_directed new directed value
 */
void
NavGraphGeneratorInterface::AddEdgeMessage::set_directed(const bool new_directed)
{
  data->directed = new_directed;
}

/** Get mode value.
 * The edge insertion mode.
 * @return mode value
 */
NavGraphGeneratorInterface::EdgeMode
NavGraphGeneratorInterface::AddEdgeMessage::mode() const
{
  return (NavGraphGeneratorInterface::EdgeMode)data->mode;
}

/** Get maximum length of mode value.
 * @return length of mode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::AddEdgeMessage::maxlenof_mode() const
{
  return 1;
}

/** Set mode value.
 * The edge insertion mode.
 * @param new_mode new mode value
 */
void
NavGraphGeneratorInterface::AddEdgeMessage::set_mode(const EdgeMode new_mode)
{
  data->mode = new_mode;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::AddEdgeMessage::clone() const
{
  return new NavGraphGeneratorInterface::AddEdgeMessage(this);
}
/** @class NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage <interfaces/NavGraphGeneratorInterface.h>
 * SetGraphDefaultPropertyMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_property_name initial value for property_name
 * @param ini_property_value initial value for property_value
 */
NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage::SetGraphDefaultPropertyMessage(const char * ini_property_name, const char * ini_property_value) : Message("SetGraphDefaultPropertyMessage")
{
  data_size = sizeof(SetGraphDefaultPropertyMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGraphDefaultPropertyMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->property_name, ini_property_name, 64);
  strncpy(data->property_value, ini_property_value, 1024);
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_STRING, "property_name", 64, data->property_name);
  add_fieldinfo(IFT_STRING, "property_value", 1024, data->property_value);
}
/** Constructor */
NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage::SetGraphDefaultPropertyMessage() : Message("SetGraphDefaultPropertyMessage")
{
  data_size = sizeof(SetGraphDefaultPropertyMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetGraphDefaultPropertyMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_STRING, "property_name", 64, data->property_name);
  add_fieldinfo(IFT_STRING, "property_value", 1024, data->property_value);
}

/** Destructor */
NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage::~SetGraphDefaultPropertyMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage::SetGraphDefaultPropertyMessage(const SetGraphDefaultPropertyMessage *m) : Message("SetGraphDefaultPropertyMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetGraphDefaultPropertyMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get property_name value.
 * Name of the property to set.
 * @return property_name value
 */
char *
NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage::property_name() const
{
  return data->property_name;
}

/** Get maximum length of property_name value.
 * @return length of property_name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage::maxlenof_property_name() const
{
  return 64;
}

/** Set property_name value.
 * Name of the property to set.
 * @param new_property_name new property_name value
 */
void
NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage::set_property_name(const char * new_property_name)
{
  strncpy(data->property_name, new_property_name, sizeof(data->property_name));
}

/** Get property_value value.
 * Value of the property
    to set.
 * @return property_value value
 */
char *
NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage::property_value() const
{
  return data->property_value;
}

/** Get maximum length of property_value value.
 * @return length of property_value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage::maxlenof_property_value() const
{
  return 1024;
}

/** Set property_value value.
 * Value of the property
    to set.
 * @param new_property_value new property_value value
 */
void
NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage::set_property_value(const char * new_property_value)
{
  strncpy(data->property_value, new_property_value, sizeof(data->property_value));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage::clone() const
{
  return new NavGraphGeneratorInterface::SetGraphDefaultPropertyMessage(this);
}
/** @class NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage <interfaces/NavGraphGeneratorInterface.h>
 * SetCopyGraphDefaultPropertiesMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_enable_copy initial value for enable_copy
 */
NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage::SetCopyGraphDefaultPropertiesMessage(const bool ini_enable_copy) : Message("SetCopyGraphDefaultPropertiesMessage")
{
  data_size = sizeof(SetCopyGraphDefaultPropertiesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetCopyGraphDefaultPropertiesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->enable_copy = ini_enable_copy;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_BOOL, "enable_copy", 1, &data->enable_copy);
}
/** Constructor */
NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage::SetCopyGraphDefaultPropertiesMessage() : Message("SetCopyGraphDefaultPropertiesMessage")
{
  data_size = sizeof(SetCopyGraphDefaultPropertiesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetCopyGraphDefaultPropertiesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
  add_fieldinfo(IFT_BOOL, "enable_copy", 1, &data->enable_copy);
}

/** Destructor */
NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage::~SetCopyGraphDefaultPropertiesMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage::SetCopyGraphDefaultPropertiesMessage(const SetCopyGraphDefaultPropertiesMessage *m) : Message("SetCopyGraphDefaultPropertiesMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetCopyGraphDefaultPropertiesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get enable_copy value.
 * True to enable copying
    (default) false to disable).
 * @return enable_copy value
 */
bool
NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage::is_enable_copy() const
{
  return data->enable_copy;
}

/** Get maximum length of enable_copy value.
 * @return length of enable_copy value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage::maxlenof_enable_copy() const
{
  return 1;
}

/** Set enable_copy value.
 * True to enable copying
    (default) false to disable).
 * @param new_enable_copy new enable_copy value
 */
void
NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage::set_enable_copy(const bool new_enable_copy)
{
  data->enable_copy = new_enable_copy;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage::clone() const
{
  return new NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage(this);
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
  enum_map_ConnectionMode[(int)NOT_CONNECTED] = "NOT_CONNECTED";
  enum_map_ConnectionMode[(int)UNCONNECTED] = "UNCONNECTED";
  enum_map_ConnectionMode[(int)CLOSEST_NODE] = "CLOSEST_NODE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE] = "CLOSEST_EDGE";
  enum_map_ConnectionMode[(int)CLOSEST_EDGE_OR_NODE] = "CLOSEST_EDGE_OR_NODE";
  enum_map_FilterType[(int)FILTER_EDGES_BY_MAP] = "FILTER_EDGES_BY_MAP";
  enum_map_FilterType[(int)FILTER_ORPHAN_NODES] = "FILTER_ORPHAN_NODES";
  enum_map_FilterType[(int)FILTER_MULTI_GRAPH] = "FILTER_MULTI_GRAPH";
  enum_map_EdgeMode[(int)NO_INTERSECTION] = "NO_INTERSECTION";
  enum_map_EdgeMode[(int)SPLIT_INTERSECTION] = "SPLIT_INTERSECTION";
  enum_map_EdgeMode[(int)FORCE] = "FORCE";
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
  const SetFilterMessage *m2 = dynamic_cast<const SetFilterMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const SetFilterParamFloatMessage *m3 = dynamic_cast<const SetFilterParamFloatMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const AddMapObstaclesMessage *m4 = dynamic_cast<const AddMapObstaclesMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const AddObstacleMessage *m5 = dynamic_cast<const AddObstacleMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const RemoveObstacleMessage *m6 = dynamic_cast<const RemoveObstacleMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const AddPointOfInterestMessage *m7 = dynamic_cast<const AddPointOfInterestMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const AddPointOfInterestWithOriMessage *m8 = dynamic_cast<const AddPointOfInterestWithOriMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  const SetPointOfInterestPropertyMessage *m9 = dynamic_cast<const SetPointOfInterestPropertyMessage *>(message);
  if ( m9 != NULL ) {
    return true;
  }
  const AddEdgeMessage *m10 = dynamic_cast<const AddEdgeMessage *>(message);
  if ( m10 != NULL ) {
    return true;
  }
  const SetGraphDefaultPropertyMessage *m11 = dynamic_cast<const SetGraphDefaultPropertyMessage *>(message);
  if ( m11 != NULL ) {
    return true;
  }
  const SetCopyGraphDefaultPropertiesMessage *m12 = dynamic_cast<const SetCopyGraphDefaultPropertiesMessage *>(message);
  if ( m12 != NULL ) {
    return true;
  }
  const RemovePointOfInterestMessage *m13 = dynamic_cast<const RemovePointOfInterestMessage *>(message);
  if ( m13 != NULL ) {
    return true;
  }
  const ComputeMessage *m14 = dynamic_cast<const ComputeMessage *>(message);
  if ( m14 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NavGraphGeneratorInterface)
/// @endcond


} // end namespace fawkes
