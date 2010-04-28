
/***************************************************************************
 *  Laser720Interface.cpp - Fawkes BlackBoard Interface - Laser720Interface
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

#include <interfaces/Laser720Interface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class Laser720Interface <interfaces/Laser720Interface.h>
 * Laser720Interface Fawkes BlackBoard Interface.
 * 
      This interface provides access to data of a laser scanner that produces
      720 beams per scan.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
Laser720Interface::Laser720Interface() : Interface()
{
  data_size = sizeof(Laser720Interface_data_t);
  data_ptr  = malloc(data_size);
  data      = (Laser720Interface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_FLOAT, "distances", 720, &data->distances);
  add_fieldinfo(IFT_BOOL, "clockwise_angle", 1, &data->clockwise_angle);
  unsigned char tmp_hash[] = {0x8a, 0x9, 0x94, 0x1a, 0xe4, 0x3c, 0xa5, 0xde, 0x5, 0xe7, 0x8c, 0x6e, 0x3b, 0x7f, 0x34, 0x5};
  set_hash(tmp_hash);
}

/** Destructor */
Laser720Interface::~Laser720Interface()
{
  free(data_ptr);
}
/* Methods */
/** Get distances value.
 * 
      The distances in meter of the beams.
    
 * @return distances value
 */
float *
Laser720Interface::distances() const
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
Laser720Interface::distances(unsigned int index) const
{
  if (index > 720) {
    throw Exception("Index value %u out of bounds (0..720)", index);
  }
  return data->distances[index];
}

/** Get maximum length of distances value.
 * @return length of distances value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Laser720Interface::maxlenof_distances() const
{
  return 720;
}

/** Set distances value.
 * 
      The distances in meter of the beams.
    
 * @param new_distances new distances value
 */
void
Laser720Interface::set_distances(const float * new_distances)
{
  memcpy(data->distances, new_distances, sizeof(float) * 720);
  data_changed = true;
}

/** Set distances value at given index.
 * 
      The distances in meter of the beams.
    
 * @param new_distances new distances value
 * @param index index for of the value
 */
void
Laser720Interface::set_distances(unsigned int index, const float new_distances)
{
  if (index > 720) {
    throw Exception("Index value %u out of bounds (0..720)", index);
  }
  data->distances[index] = new_distances;
}
/** Get clockwise_angle value.
 * 
      True if the angle grows clockwise.
    
 * @return clockwise_angle value
 */
bool
Laser720Interface::is_clockwise_angle() const
{
  return data->clockwise_angle;
}

/** Get maximum length of clockwise_angle value.
 * @return length of clockwise_angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Laser720Interface::maxlenof_clockwise_angle() const
{
  return 1;
}

/** Set clockwise_angle value.
 * 
      True if the angle grows clockwise.
    
 * @param new_clockwise_angle new clockwise_angle value
 */
void
Laser720Interface::set_clockwise_angle(const bool new_clockwise_angle)
{
  data->clockwise_angle = new_clockwise_angle;
  data_changed = true;
}

/* =========== message create =========== */
Message *
Laser720Interface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
Laser720Interface::copy_values(const Interface *other)
{
  const Laser720Interface *oi = dynamic_cast<const Laser720Interface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(Laser720Interface_data_t));
}

const char *
Laser720Interface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
Laser720Interface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(Laser720Interface)
/// @endcond


} // end namespace fawkes
