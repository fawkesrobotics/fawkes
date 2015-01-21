
/***************************************************************************
 *  LaserLineInterface.cpp - Fawkes BlackBoard Interface - LaserLineInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Tim Niemueller
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

#include <interfaces/LaserLineInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class LaserLineInterface <interfaces/LaserLineInterface.h>
 * LaserLineInterface Fawkes BlackBoard Interface.
 * Line parameterization.
 * @ingroup FawkesInterfaces
 */



/** Constructor */
LaserLineInterface::LaserLineInterface() : Interface()
{
  data_size = sizeof(LaserLineInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (LaserLineInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "frame_id", 32, data->frame_id);
  add_fieldinfo(IFT_INT32, "visibility_history", 1, &data->visibility_history);
  add_fieldinfo(IFT_FLOAT, "point_on_line", 3, &data->point_on_line);
  add_fieldinfo(IFT_FLOAT, "line_direction", 3, &data->line_direction);
  add_fieldinfo(IFT_FLOAT, "bearing", 1, &data->bearing);
  unsigned char tmp_hash[] = {0xb6, 0xdd, 0xc9, 0x29, 0x18, 0x34, 0xf3, 0x78, 0x4b, 0x77, 0xd, 0xe0, 0x65, 0x81, 0xcc, 0x1a};
  set_hash(tmp_hash);
}

/** Destructor */
LaserLineInterface::~LaserLineInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get frame_id value.
 * 
      Coordinate frame ID of data.
    
 * @return frame_id value
 */
char *
LaserLineInterface::frame_id() const
{
  return data->frame_id;
}

/** Get maximum length of frame_id value.
 * @return length of frame_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserLineInterface::maxlenof_frame_id() const
{
  return 32;
}

/** Set frame_id value.
 * 
      Coordinate frame ID of data.
    
 * @param new_frame_id new frame_id value
 */
void
LaserLineInterface::set_frame_id(const char * new_frame_id)
{
  strncpy(data->frame_id, new_frame_id, sizeof(data->frame_id));
  data_changed = true;
}

/** Get visibility_history value.
 * 
      The visibilitiy history indicates the number of consecutive positive or negative
      sightings. If the history is negative, there have been as many negative sightings
      (object not visible) as the absolute value of the history. A positive value denotes
      as many positive sightings. 0 shall only be used during the initialization of the
      interface or if the visibility history is not updated.
    
 * @return visibility_history value
 */
int32_t
LaserLineInterface::visibility_history() const
{
  return data->visibility_history;
}

/** Get maximum length of visibility_history value.
 * @return length of visibility_history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserLineInterface::maxlenof_visibility_history() const
{
  return 1;
}

/** Set visibility_history value.
 * 
      The visibilitiy history indicates the number of consecutive positive or negative
      sightings. If the history is negative, there have been as many negative sightings
      (object not visible) as the absolute value of the history. A positive value denotes
      as many positive sightings. 0 shall only be used during the initialization of the
      interface or if the visibility history is not updated.
    
 * @param new_visibility_history new visibility_history value
 */
void
LaserLineInterface::set_visibility_history(const int32_t new_visibility_history)
{
  data->visibility_history = new_visibility_history;
  data_changed = true;
}

/** Get point_on_line value.
 * 
      Vector to some point on the line
    
 * @return point_on_line value
 */
float *
LaserLineInterface::point_on_line() const
{
  return data->point_on_line;
}

/** Get point_on_line value at given index.
 * 
      Vector to some point on the line
    
 * @param index index of value
 * @return point_on_line value
 * @exception Exception thrown if index is out of bounds
 */
float
LaserLineInterface::point_on_line(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->point_on_line[index];
}

/** Get maximum length of point_on_line value.
 * @return length of point_on_line value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserLineInterface::maxlenof_point_on_line() const
{
  return 3;
}

/** Set point_on_line value.
 * 
      Vector to some point on the line
    
 * @param new_point_on_line new point_on_line value
 */
void
LaserLineInterface::set_point_on_line(const float * new_point_on_line)
{
  memcpy(data->point_on_line, new_point_on_line, sizeof(float) * 3);
  data_changed = true;
}

/** Set point_on_line value at given index.
 * 
      Vector to some point on the line
    
 * @param new_point_on_line new point_on_line value
 * @param index index for of the value
 */
void
LaserLineInterface::set_point_on_line(unsigned int index, const float new_point_on_line)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->point_on_line[index] = new_point_on_line;
  data_changed = true;
}
/** Get line_direction value.
 * 
      Vector in the direction of the line.
    
 * @return line_direction value
 */
float *
LaserLineInterface::line_direction() const
{
  return data->line_direction;
}

/** Get line_direction value at given index.
 * 
      Vector in the direction of the line.
    
 * @param index index of value
 * @return line_direction value
 * @exception Exception thrown if index is out of bounds
 */
float
LaserLineInterface::line_direction(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->line_direction[index];
}

/** Get maximum length of line_direction value.
 * @return length of line_direction value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserLineInterface::maxlenof_line_direction() const
{
  return 3;
}

/** Set line_direction value.
 * 
      Vector in the direction of the line.
    
 * @param new_line_direction new line_direction value
 */
void
LaserLineInterface::set_line_direction(const float * new_line_direction)
{
  memcpy(data->line_direction, new_line_direction, sizeof(float) * 3);
  data_changed = true;
}

/** Set line_direction value at given index.
 * 
      Vector in the direction of the line.
    
 * @param new_line_direction new line_direction value
 * @param index index for of the value
 */
void
LaserLineInterface::set_line_direction(unsigned int index, const float new_line_direction)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->line_direction[index] = new_line_direction;
  data_changed = true;
}
/** Get bearing value.
 * 
      Direction towards the line, i.e. if the robot turns by this
      angle the robot will stand parallel to the line.
    
 * @return bearing value
 */
float
LaserLineInterface::bearing() const
{
  return data->bearing;
}

/** Get maximum length of bearing value.
 * @return length of bearing value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LaserLineInterface::maxlenof_bearing() const
{
  return 1;
}

/** Set bearing value.
 * 
      Direction towards the line, i.e. if the robot turns by this
      angle the robot will stand parallel to the line.
    
 * @param new_bearing new bearing value
 */
void
LaserLineInterface::set_bearing(const float new_bearing)
{
  data->bearing = new_bearing;
  data_changed = true;
}

/* =========== message create =========== */
Message *
LaserLineInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
LaserLineInterface::copy_values(const Interface *other)
{
  const LaserLineInterface *oi = dynamic_cast<const LaserLineInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(LaserLineInterface_data_t));
}

const char *
LaserLineInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
LaserLineInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(LaserLineInterface)
/// @endcond


} // end namespace fawkes
