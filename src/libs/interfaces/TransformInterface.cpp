
/***************************************************************************
 *  TransformInterface.cpp - Fawkes BlackBoard Interface - TransformInterface
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

#include <interfaces/TransformInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class TransformInterface <interfaces/TransformInterface.h>
 * TransformInterface Fawkes BlackBoard Interface.
 * 
      This interface is used to publish transforms. It aims to be as
      compatible as possible with ROS' tf library and is used
      extensively by the Fawkes tf library.

      For this to work properly it is crucial to have correct
      timestamp set (cf. Interface::set_timestamp()). Set this as
      close as possible to the time of when the data, from which the
      transform is computed, has been acquired.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
TransformInterface::TransformInterface() : Interface()
{
  data_size = sizeof(TransformInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (TransformInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "frame", 64, data->frame);
  add_fieldinfo(IFT_STRING, "child_frame", 64, data->child_frame);
  add_fieldinfo(IFT_BOOL, "static_transform", 1, &data->static_transform);
  add_fieldinfo(IFT_DOUBLE, "translation", 3, &data->translation);
  add_fieldinfo(IFT_DOUBLE, "rotation", 4, &data->rotation);
  unsigned char tmp_hash[] = {0xb6, 0xb0, 0xd3, 0x96, 0xda, 0x61, 0xdd, 0xd3, 0x6, 0x9e, 0x66, 0x4d, 0x14, 0x54, 0x5e, 0xfb};
  set_hash(tmp_hash);
}

/** Destructor */
TransformInterface::~TransformInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get frame value.
 * 
      Parent frame ID. The given transform is relative to the origin
      of this coordinate frame.
    
 * @return frame value
 */
char *
TransformInterface::frame() const
{
  return data->frame;
}

/** Get maximum length of frame value.
 * @return length of frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TransformInterface::maxlenof_frame() const
{
  return 64;
}

/** Set frame value.
 * 
      Parent frame ID. The given transform is relative to the origin
      of this coordinate frame.
    
 * @param new_frame new frame value
 */
void
TransformInterface::set_frame(const char * new_frame)
{
  strncpy(data->frame, new_frame, sizeof(data->frame)-1);
  data->frame[sizeof(data->frame)-1] = 0;
  data_changed = true;
}

/** Get child_frame value.
 * 
      The ID of the child frame. The child frame's origin is at the
      given point in the parent frame denoted by the transform.
    
 * @return child_frame value
 */
char *
TransformInterface::child_frame() const
{
  return data->child_frame;
}

/** Get maximum length of child_frame value.
 * @return length of child_frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TransformInterface::maxlenof_child_frame() const
{
  return 64;
}

/** Set child_frame value.
 * 
      The ID of the child frame. The child frame's origin is at the
      given point in the parent frame denoted by the transform.
    
 * @param new_child_frame new child_frame value
 */
void
TransformInterface::set_child_frame(const char * new_child_frame)
{
  strncpy(data->child_frame, new_child_frame, sizeof(data->child_frame)-1);
  data->child_frame[sizeof(data->child_frame)-1] = 0;
  data_changed = true;
}

/** Get static_transform value.
 * 
	    True if the transform is static, i.e. it will never change
	    during its lifetime, false otherwise.
    
 * @return static_transform value
 */
bool
TransformInterface::is_static_transform() const
{
  return data->static_transform;
}

/** Get maximum length of static_transform value.
 * @return length of static_transform value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TransformInterface::maxlenof_static_transform() const
{
  return 1;
}

/** Set static_transform value.
 * 
	    True if the transform is static, i.e. it will never change
	    during its lifetime, false otherwise.
    
 * @param new_static_transform new static_transform value
 */
void
TransformInterface::set_static_transform(const bool new_static_transform)
{
  data->static_transform = new_static_transform;
  data_changed = true;
}

/** Get translation value.
 * 
      This array denotes the translation vector of the transform. The
      element indexes are ordered x, y, z, i.e. translation[0] is the
      X value of the translation vector.
    
 * @return translation value
 */
double *
TransformInterface::translation() const
{
  return data->translation;
}

/** Get translation value at given index.
 * 
      This array denotes the translation vector of the transform. The
      element indexes are ordered x, y, z, i.e. translation[0] is the
      X value of the translation vector.
    
 * @param index index of value
 * @return translation value
 * @exception Exception thrown if index is out of bounds
 */
double
TransformInterface::translation(unsigned int index) const
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
TransformInterface::maxlenof_translation() const
{
  return 3;
}

/** Set translation value.
 * 
      This array denotes the translation vector of the transform. The
      element indexes are ordered x, y, z, i.e. translation[0] is the
      X value of the translation vector.
    
 * @param new_translation new translation value
 */
void
TransformInterface::set_translation(const double * new_translation)
{
  memcpy(data->translation, new_translation, sizeof(double) * 3);
  data_changed = true;
}

/** Set translation value at given index.
 * 
      This array denotes the translation vector of the transform. The
      element indexes are ordered x, y, z, i.e. translation[0] is the
      X value of the translation vector.
    
 * @param new_translation new translation value
 * @param index index for of the value
 */
void
TransformInterface::set_translation(unsigned int index, const double new_translation)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->translation[index] = new_translation;
  data_changed = true;
}
/** Get rotation value.
 * 
      This array denotes the rotation quaternion of the transform. The
      element indexes are ordered x, y, z, w, i.e. translation[0] is
      the X value of the rotation quaternion and translation[3] is the
      W value.
    
 * @return rotation value
 */
double *
TransformInterface::rotation() const
{
  return data->rotation;
}

/** Get rotation value at given index.
 * 
      This array denotes the rotation quaternion of the transform. The
      element indexes are ordered x, y, z, w, i.e. translation[0] is
      the X value of the rotation quaternion and translation[3] is the
      W value.
    
 * @param index index of value
 * @return rotation value
 * @exception Exception thrown if index is out of bounds
 */
double
TransformInterface::rotation(unsigned int index) const
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
TransformInterface::maxlenof_rotation() const
{
  return 4;
}

/** Set rotation value.
 * 
      This array denotes the rotation quaternion of the transform. The
      element indexes are ordered x, y, z, w, i.e. translation[0] is
      the X value of the rotation quaternion and translation[3] is the
      W value.
    
 * @param new_rotation new rotation value
 */
void
TransformInterface::set_rotation(const double * new_rotation)
{
  memcpy(data->rotation, new_rotation, sizeof(double) * 4);
  data_changed = true;
}

/** Set rotation value at given index.
 * 
      This array denotes the rotation quaternion of the transform. The
      element indexes are ordered x, y, z, w, i.e. translation[0] is
      the X value of the rotation quaternion and translation[3] is the
      W value.
    
 * @param new_rotation new rotation value
 * @param index index for of the value
 */
void
TransformInterface::set_rotation(unsigned int index, const double new_rotation)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->rotation[index] = new_rotation;
  data_changed = true;
}
/* =========== message create =========== */
Message *
TransformInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
TransformInterface::copy_values(const Interface *other)
{
  const TransformInterface *oi = dynamic_cast<const TransformInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(TransformInterface_data_t));
}

const char *
TransformInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
TransformInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(TransformInterface)
/// @endcond


} // end namespace fawkes
