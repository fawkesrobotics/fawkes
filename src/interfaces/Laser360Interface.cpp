
/***************************************************************************
 *  Laser360Interface.cpp - Fawkes BlackBoard Interface - Laser360Interface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008-2009  Tim Niemueller
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

#include <interfaces/Laser360Interface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class Laser360Interface <interfaces/Laser360Interface.h>
 * Laser360Interface Fawkes BlackBoard Interface.
 * 
      This interface provides access to data of a laser scanner that produces
      360 beams per scan. The inter-beam distance is 1 deg, 0 deg is
      "forward", i.e. in the Fawkes coordinate system pointing towards
      the cartesian point (1,0). The direction in which the angle
      grows is indicated by the clockwise_angle field.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
Laser360Interface::Laser360Interface() : Interface()
{
  data_size = sizeof(Laser360Interface_data_t);
  data_ptr  = malloc(data_size);
  data      = (Laser360Interface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_FLOAT, "distances", 360, &data->distances);
  add_fieldinfo(IFT_BOOL, "clockwise_angle", 1, &data->clockwise_angle);
  unsigned char tmp_hash[] = {0xf6, 0x3a, 0x26, 0x7b, 0x46, 0x96, 0x74, 0xad, 0x48, 0x1c, 0x32, 0x66, 0x2b, 0xfe, 0x41, 0x43};
  set_hash(tmp_hash);
}

/** Destructor */
Laser360Interface::~Laser360Interface()
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
Laser360Interface::distances() const
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
Laser360Interface::distances(unsigned int index) const
{
  if (index > 360) {
    throw Exception("Index value %u out of bounds (0..360)", index);
  }
  return data->distances[index];
}

/** Get maximum length of distances value.
 * @return length of distances value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Laser360Interface::maxlenof_distances() const
{
  return 360;
}

/** Set distances value.
 * 
      The distances in meter of the beams.
    
 * @param new_distances new distances value
 */
void
Laser360Interface::set_distances(const float * new_distances)
{
  memcpy(data->distances, new_distances, sizeof(float) * 360);
  data_changed = true;
}

/** Set distances value at given index.
 * 
      The distances in meter of the beams.
    
 * @param new_distances new distances value
 * @param index index for of the value
 */
void
Laser360Interface::set_distances(unsigned int index, const float new_distances)
{
  if (index > 360) {
    throw Exception("Index value %u out of bounds (0..360)", index);
  }
  data->distances[index] = new_distances;
}
/** Get clockwise_angle value.
 * 
      True if the angle grows clockwise.
    
 * @return clockwise_angle value
 */
bool
Laser360Interface::is_clockwise_angle() const
{
  return data->clockwise_angle;
}

/** Get maximum length of clockwise_angle value.
 * @return length of clockwise_angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Laser360Interface::maxlenof_clockwise_angle() const
{
  return 1;
}

/** Set clockwise_angle value.
 * 
      True if the angle grows clockwise.
    
 * @param new_clockwise_angle new clockwise_angle value
 */
void
Laser360Interface::set_clockwise_angle(const bool new_clockwise_angle)
{
  data->clockwise_angle = new_clockwise_angle;
  data_changed = true;
}

/* =========== message create =========== */
Message *
Laser360Interface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
Laser360Interface::copy_values(const Interface *other)
{
  const Laser360Interface *oi = dynamic_cast<const Laser360Interface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(Laser360Interface_data_t));
}

const char *
Laser360Interface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
Laser360Interface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(Laser360Interface)
/// @endcond


} // end namespace fawkes
