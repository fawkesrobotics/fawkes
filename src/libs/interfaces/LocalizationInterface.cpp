
/***************************************************************************
 *  LocalizationInterface.cpp - Fawkes BlackBoard Interface - LocalizationInterface
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

#include <interfaces/LocalizationInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class LocalizationInterface <interfaces/LocalizationInterface.h>
 * LocalizationInterface Fawkes BlackBoard Interface.
 * 
      Information and commands relevant to a self-localization
      component. This does not contain the pose as it is provided in a
      Position3DInterface.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
LocalizationInterface::LocalizationInterface() : Interface()
{
  data_size = sizeof(LocalizationInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (LocalizationInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "map", 64, data->map);
  add_messageinfo("SetInitialPoseMessage");
  unsigned char tmp_hash[] = {0x7f, 0x9, 0xec, 0xd1, 00, 0x3f, 0x3, 0xb7, 0x95, 0xce, 0xe, 0x1d, 0x6f, 0x48, 0x6c, 0xad};
  set_hash(tmp_hash);
}

/** Destructor */
LocalizationInterface::~LocalizationInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get map value.
 * The currently used map.
 * @return map value
 */
char *
LocalizationInterface::map() const
{
  return data->map;
}

/** Get maximum length of map value.
 * @return length of map value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LocalizationInterface::maxlenof_map() const
{
  return 64;
}

/** Set map value.
 * The currently used map.
 * @param new_map new map value
 */
void
LocalizationInterface::set_map(const char * new_map)
{
  strncpy(data->map, new_map, sizeof(data->map)-1);
  data->map[sizeof(data->map)-1] = 0;
  data_changed = true;
}

/* =========== message create =========== */
Message *
LocalizationInterface::create_message(const char *type) const
{
  if ( strncmp("SetInitialPoseMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetInitialPoseMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
LocalizationInterface::copy_values(const Interface *other)
{
  const LocalizationInterface *oi = dynamic_cast<const LocalizationInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(LocalizationInterface_data_t));
}

const char *
LocalizationInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class LocalizationInterface::SetInitialPoseMessage <interfaces/LocalizationInterface.h>
 * SetInitialPoseMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_frame initial value for frame
 * @param ini_rotation initial value for rotation
 * @param ini_translation initial value for translation
 * @param ini_covariance initial value for covariance
 */
LocalizationInterface::SetInitialPoseMessage::SetInitialPoseMessage(const char * ini_frame, const double * ini_rotation, const double * ini_translation, const double * ini_covariance) : Message("SetInitialPoseMessage")
{
  data_size = sizeof(SetInitialPoseMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetInitialPoseMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->frame, ini_frame, 32-1);
  data->frame[32-1] = 0;
  memcpy(data->rotation, ini_rotation, sizeof(double) * 4);
  memcpy(data->translation, ini_translation, sizeof(double) * 3);
  memcpy(data->covariance, ini_covariance, sizeof(double) * 36);
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_DOUBLE, "rotation", 4, &data->rotation);
  add_fieldinfo(IFT_DOUBLE, "translation", 3, &data->translation);
  add_fieldinfo(IFT_DOUBLE, "covariance", 36, &data->covariance);
}
/** Constructor */
LocalizationInterface::SetInitialPoseMessage::SetInitialPoseMessage() : Message("SetInitialPoseMessage")
{
  data_size = sizeof(SetInitialPoseMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetInitialPoseMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_DOUBLE, "rotation", 4, &data->rotation);
  add_fieldinfo(IFT_DOUBLE, "translation", 3, &data->translation);
  add_fieldinfo(IFT_DOUBLE, "covariance", 36, &data->covariance);
}

/** Destructor */
LocalizationInterface::SetInitialPoseMessage::~SetInitialPoseMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
LocalizationInterface::SetInitialPoseMessage::SetInitialPoseMessage(const SetInitialPoseMessage *m) : Message("SetInitialPoseMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetInitialPoseMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get frame value.
 * 
      Reference coordinate frame for the data.
    
 * @return frame value
 */
char *
LocalizationInterface::SetInitialPoseMessage::frame() const
{
  return data->frame;
}

/** Get maximum length of frame value.
 * @return length of frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
LocalizationInterface::SetInitialPoseMessage::maxlenof_frame() const
{
  return 32;
}

/** Set frame value.
 * 
      Reference coordinate frame for the data.
    
 * @param new_frame new frame value
 */
void
LocalizationInterface::SetInitialPoseMessage::set_frame(const char * new_frame)
{
  strncpy(data->frame, new_frame, sizeof(data->frame)-1);
  data->frame[sizeof(data->frame)-1] = 0;
}

/** Get rotation value.
 * 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
    
 * @return rotation value
 */
double *
LocalizationInterface::SetInitialPoseMessage::rotation() const
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
LocalizationInterface::SetInitialPoseMessage::rotation(unsigned int index) const
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
LocalizationInterface::SetInitialPoseMessage::maxlenof_rotation() const
{
  return 4;
}

/** Set rotation value.
 * 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
    
 * @param new_rotation new rotation value
 */
void
LocalizationInterface::SetInitialPoseMessage::set_rotation(const double * new_rotation)
{
  memcpy(data->rotation, new_rotation, sizeof(double) * 4);
}

/** Set rotation value at given index.
 * 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
    
 * @param new_rotation new rotation value
 * @param index index for of the value
 */
void
LocalizationInterface::SetInitialPoseMessage::set_rotation(unsigned int index, const double new_rotation)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->rotation[index] = new_rotation;
}
/** Get translation value.
 * 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
    
 * @return translation value
 */
double *
LocalizationInterface::SetInitialPoseMessage::translation() const
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
LocalizationInterface::SetInitialPoseMessage::translation(unsigned int index) const
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
LocalizationInterface::SetInitialPoseMessage::maxlenof_translation() const
{
  return 3;
}

/** Set translation value.
 * 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
    
 * @param new_translation new translation value
 */
void
LocalizationInterface::SetInitialPoseMessage::set_translation(const double * new_translation)
{
  memcpy(data->translation, new_translation, sizeof(double) * 3);
}

/** Set translation value at given index.
 * 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
    
 * @param new_translation new translation value
 * @param index index for of the value
 */
void
LocalizationInterface::SetInitialPoseMessage::set_translation(unsigned int index, const double new_translation)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->translation[index] = new_translation;
}
/** Get covariance value.
 * 
      Row-major representation of the 6x6 covariance matrix. The
      orientation parameters use a fixed-axis representation. In
      order, the parameters are: (x, y, z, rotation about X axis,
      rotation about Y axis, rotation about Z axis).
    
 * @return covariance value
 */
double *
LocalizationInterface::SetInitialPoseMessage::covariance() const
{
  return data->covariance;
}

/** Get covariance value at given index.
 * 
      Row-major representation of the 6x6 covariance matrix. The
      orientation parameters use a fixed-axis representation. In
      order, the parameters are: (x, y, z, rotation about X axis,
      rotation about Y axis, rotation about Z axis).
    
 * @param index index of value
 * @return covariance value
 * @exception Exception thrown if index is out of bounds
 */
double
LocalizationInterface::SetInitialPoseMessage::covariance(unsigned int index) const
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
LocalizationInterface::SetInitialPoseMessage::maxlenof_covariance() const
{
  return 36;
}

/** Set covariance value.
 * 
      Row-major representation of the 6x6 covariance matrix. The
      orientation parameters use a fixed-axis representation. In
      order, the parameters are: (x, y, z, rotation about X axis,
      rotation about Y axis, rotation about Z axis).
    
 * @param new_covariance new covariance value
 */
void
LocalizationInterface::SetInitialPoseMessage::set_covariance(const double * new_covariance)
{
  memcpy(data->covariance, new_covariance, sizeof(double) * 36);
}

/** Set covariance value at given index.
 * 
      Row-major representation of the 6x6 covariance matrix. The
      orientation parameters use a fixed-axis representation. In
      order, the parameters are: (x, y, z, rotation about X axis,
      rotation about Y axis, rotation about Z axis).
    
 * @param new_covariance new covariance value
 * @param index index for of the value
 */
void
LocalizationInterface::SetInitialPoseMessage::set_covariance(unsigned int index, const double new_covariance)
{
  if (index > 36) {
    throw Exception("Index value %u out of bounds (0..36)", index);
  }
  data->covariance[index] = new_covariance;
}
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
LocalizationInterface::SetInitialPoseMessage::clone() const
{
  return new LocalizationInterface::SetInitialPoseMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
LocalizationInterface::message_valid(const Message *message) const
{
  const SetInitialPoseMessage *m0 = dynamic_cast<const SetInitialPoseMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(LocalizationInterface)
/// @endcond


} // end namespace fawkes
