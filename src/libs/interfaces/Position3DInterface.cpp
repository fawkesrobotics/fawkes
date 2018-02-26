
/***************************************************************************
 *  Position3DInterface.cpp - Fawkes BlackBoard Interface - Position3DInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2011  Tim Niemueller
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

#include <interfaces/Position3DInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class Position3DInterface <interfaces/Position3DInterface.h>
 * Position3DInterface Fawkes BlackBoard Interface.
 * 
      Storage for a 3D pose in Euclidean space.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
Position3DInterface::Position3DInterface() : Interface()
{
  data_size = sizeof(Position3DInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (Position3DInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_INT32, "visibility_history", 1, &data->visibility_history);
  add_fieldinfo(IFT_DOUBLE, "rotation", 4, &data->rotation);
  add_fieldinfo(IFT_DOUBLE, "translation", 3, &data->translation);
  add_fieldinfo(IFT_DOUBLE, "covariance", 36, &data->covariance);
  unsigned char tmp_hash[] = {0xd6, 0x19, 0x3f, 0x58, 0x62, 0xbc, 0x72, 0xd6, 0x22, 0x36, 0xd3, 0x7, 0x55, 0xb5, 0x3a, 0x48};
  set_hash(tmp_hash);
}

/** Destructor */
Position3DInterface::~Position3DInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get frame value.
 * 
      Reference coordinate frame for the data.
    
 * @return frame value
 */
char *
Position3DInterface::frame() const
{
  return data->frame;
}

/** Get maximum length of frame value.
 * @return length of frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Position3DInterface::maxlenof_frame() const
{
  return 32;
}

/** Set frame value.
 * 
      Reference coordinate frame for the data.
    
 * @param new_frame new frame value
 */
void
Position3DInterface::set_frame(const char * new_frame)
{
  strncpy(data->frame, new_frame, sizeof(data->frame)-1);
  data->frame[sizeof(data->frame)-1] = 0;
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
Position3DInterface::visibility_history() const
{
  return data->visibility_history;
}

/** Get maximum length of visibility_history value.
 * @return length of visibility_history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Position3DInterface::maxlenof_visibility_history() const
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
Position3DInterface::set_visibility_history(const int32_t new_visibility_history)
{
  data->visibility_history = new_visibility_history;
  data_changed = true;
}

/** Get rotation value.
 * 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
    
 * @return rotation value
 */
double *
Position3DInterface::rotation() const
{
  return data->rotation;
}

/** Get rotation value at given index.
 * 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
    
 * @param index index of value
 * @return rotation value
 * @exception Exception thrown if index is out of bounds
 */
double
Position3DInterface::rotation(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->rotation[index];
}

/** Get maximum length of rotation value.
 * @return length of rotation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Position3DInterface::maxlenof_rotation() const
{
  return 4;
}

/** Set rotation value.
 * 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
    
 * @param new_rotation new rotation value
 */
void
Position3DInterface::set_rotation(const double * new_rotation)
{
  memcpy(data->rotation, new_rotation, sizeof(double) * 4);
  data_changed = true;
}

/** Set rotation value at given index.
 * 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
    
 * @param new_rotation new rotation value
 * @param index index for of the value
 */
void
Position3DInterface::set_rotation(unsigned int index, const double new_rotation)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->rotation[index] = new_rotation;
  data_changed = true;
}
/** Get translation value.
 * 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
    
 * @return translation value
 */
double *
Position3DInterface::translation() const
{
  return data->translation;
}

/** Get translation value at given index.
 * 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
    
 * @param index index of value
 * @return translation value
 * @exception Exception thrown if index is out of bounds
 */
double
Position3DInterface::translation(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->translation[index];
}

/** Get maximum length of translation value.
 * @return length of translation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Position3DInterface::maxlenof_translation() const
{
  return 3;
}

/** Set translation value.
 * 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
    
 * @param new_translation new translation value
 */
void
Position3DInterface::set_translation(const double * new_translation)
{
  memcpy(data->translation, new_translation, sizeof(double) * 3);
  data_changed = true;
}

/** Set translation value at given index.
 * 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
    
 * @param new_translation new translation value
 * @param index index for of the value
 */
void
Position3DInterface::set_translation(unsigned int index, const double new_translation)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->translation[index] = new_translation;
  data_changed = true;
}
/** Get covariance value.
 * 
      Row-major representation of the 6x6 covariance matrix.
      The orientation parameters use a fixed-axis representation.
      In order, the parameters are:
      (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    
 * @return covariance value
 */
double *
Position3DInterface::covariance() const
{
  return data->covariance;
}

/** Get covariance value at given index.
 * 
      Row-major representation of the 6x6 covariance matrix.
      The orientation parameters use a fixed-axis representation.
      In order, the parameters are:
      (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    
 * @param index index of value
 * @return covariance value
 * @exception Exception thrown if index is out of bounds
 */
double
Position3DInterface::covariance(unsigned int index) const
{
  if (index > 36) {
    throw Exception("Index value %u out of bounds (0..36)", index);
  }
  return data->covariance[index];
}

/** Get maximum length of covariance value.
 * @return length of covariance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
Position3DInterface::maxlenof_covariance() const
{
  return 36;
}

/** Set covariance value.
 * 
      Row-major representation of the 6x6 covariance matrix.
      The orientation parameters use a fixed-axis representation.
      In order, the parameters are:
      (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    
 * @param new_covariance new covariance value
 */
void
Position3DInterface::set_covariance(const double * new_covariance)
{
  memcpy(data->covariance, new_covariance, sizeof(double) * 36);
  data_changed = true;
}

/** Set covariance value at given index.
 * 
      Row-major representation of the 6x6 covariance matrix.
      The orientation parameters use a fixed-axis representation.
      In order, the parameters are:
      (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    
 * @param new_covariance new covariance value
 * @param index index for of the value
 */
void
Position3DInterface::set_covariance(unsigned int index, const double new_covariance)
{
  if (index > 36) {
    throw Exception("Index value %u out of bounds (0..36)", index);
  }
  data->covariance[index] = new_covariance;
  data_changed = true;
}
/* =========== message create =========== */
Message *
Position3DInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
Position3DInterface::copy_values(const Interface *other)
{
  const Position3DInterface *oi = dynamic_cast<const Position3DInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(Position3DInterface_data_t));
}

const char *
Position3DInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
Position3DInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(Position3DInterface)
/// @endcond


} // end namespace fawkes
