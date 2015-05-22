
/***************************************************************************
 *  Laser1080Interface.cpp - Fawkes BlackBoard Interface - Laser1080Interface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008-2015  Tim Niemueller
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

#include <interfaces/Laser1080Interface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class Laser1080Interface <interfaces/Laser1080Interface.h>
 * Laser1080Interface Fawkes BlackBoard Interface.
 * 
      This interface provides access to data of a laser scanner that produces
      up to 1080 beams per scan (i.e. 1/3 degree resolution).
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
Laser1080Interface::Laser1080Interface() : Interface()
{
  data_size = sizeof(Laser1080Interface_data_t);
  data_ptr  = malloc(data_size);
  data      = (Laser1080Interface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_FLOAT, "distances", 1080, &data->distances);
  add_fieldinfo(IFT_BOOL, "clockwise_angle", 1, &data->clockwise_angle);
  unsigned char tmp_hash[] = {0xa7, 0xab, 0x1f, 0x20, 0xdb, 0x24, 0xf9, 0x1b, 0x4e, 0xd6, 0x8b, 0xfa, 0x65, 0x25, 0xe5, 0x22};
  set_hash(tmp_hash);
}

/** Destructor */
Laser1080Interface::~Laser1080Interface()
{
  free(data_ptr);
}
/* Methods */
/** Get frame value.
 * 
      Coordinate frame in which the data is presented.
    
 * @return frame value
 */
char *
Laser1080Interface::frame() const
{
  return data->frame;
}

/** Get maximum length of frame value.
 * @return length of frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Laser1080Interface::maxlenof_frame() const
{
  return 32;
}

/** Set frame value.
 * 
      Coordinate frame in which the data is presented.
    
 * @param new_frame new frame value
 */
void
Laser1080Interface::set_frame(const char * new_frame)
{
  strncpy(data->frame, new_frame, sizeof(data->frame));
  data_changed = true;
}

/** Get distances value.
 * 
      The distances in meter of the beams.
    
 * @return distances value
 */
float *
Laser1080Interface::distances() const
{
  return data->distances;
}

/** Get distances value at given index.
 * 
      The distances in meter of the beams.
    
 * @param index index of value
 * @return distances value
 * @exception Exception thrown if index is out of bounds
 */
float
Laser1080Interface::distances(unsigned int index) const
{
  if (index > 1080) {
    throw Exception("Index value %u out of bounds (0..1080)", index);
  }
  return data->distances[index];
}

/** Get maximum length of distances value.
 * @return length of distances value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Laser1080Interface::maxlenof_distances() const
{
  return 1080;
}

/** Set distances value.
 * 
      The distances in meter of the beams.
    
 * @param new_distances new distances value
 */
void
Laser1080Interface::set_distances(const float * new_distances)
{
  memcpy(data->distances, new_distances, sizeof(float) * 1080);
  data_changed = true;
}

/** Set distances value at given index.
 * 
      The distances in meter of the beams.
    
 * @param new_distances new distances value
 * @param index index for of the value
 */
void
Laser1080Interface::set_distances(unsigned int index, const float new_distances)
{
  if (index > 1080) {
    throw Exception("Index value %u out of bounds (0..1080)", index);
  }
  data->distances[index] = new_distances;
  data_changed = true;
}
/** Get clockwise_angle value.
 * 
      True if the angle grows clockwise.
    
 * @return clockwise_angle value
 */
bool
Laser1080Interface::is_clockwise_angle() const
{
  return data->clockwise_angle;
}

/** Get maximum length of clockwise_angle value.
 * @return length of clockwise_angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Laser1080Interface::maxlenof_clockwise_angle() const
{
  return 1;
}

/** Set clockwise_angle value.
 * 
      True if the angle grows clockwise.
    
 * @param new_clockwise_angle new clockwise_angle value
 */
void
Laser1080Interface::set_clockwise_angle(const bool new_clockwise_angle)
{
  data->clockwise_angle = new_clockwise_angle;
  data_changed = true;
}

/* =========== message create =========== */
Message *
Laser1080Interface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
Laser1080Interface::copy_values(const Interface *other)
{
  const Laser1080Interface *oi = dynamic_cast<const Laser1080Interface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(Laser1080Interface_data_t));
}

const char *
Laser1080Interface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
Laser1080Interface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(Laser1080Interface)
/// @endcond


} // end namespace fawkes
