
/***************************************************************************
 *  CameraControlInterface.cpp - Fawkes BlackBoard Interface - CameraControlInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2014  Tim Niemueller
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

#include <interfaces/CameraControlInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class CameraControlInterface <interfaces/CameraControlInterface.h>
 * CameraControlInterface Fawkes BlackBoard Interface.
 * 
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
CameraControlInterface::CameraControlInterface() : Interface()
{
  data_size = sizeof(CameraControlInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (CameraControlInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_Effect[(int)EFF_NONE] = "EFF_NONE";
  enum_map_Effect[(int)EFF_PASTEL] = "EFF_PASTEL";
  enum_map_Effect[(int)EFF_NEGATIVE] = "EFF_NEGATIVE";
  enum_map_Effect[(int)EFF_BW] = "EFF_BW";
  enum_map_Effect[(int)EFF_SOLARIZE] = "EFF_SOLARIZE";
  add_fieldinfo(IFT_ENUM, "effect", 1, &data->effect, "Effect", &enum_map_Effect);
  add_fieldinfo(IFT_BOOL, "effect_supported", 1, &data->effect_supported);
  add_fieldinfo(IFT_UINT32, "zoom", 1, &data->zoom);
  add_fieldinfo(IFT_BOOL, "zoom_supported", 1, &data->zoom_supported);
  add_fieldinfo(IFT_UINT32, "zoom_max", 1, &data->zoom_max);
  add_fieldinfo(IFT_UINT32, "zoom_min", 1, &data->zoom_min);
  add_fieldinfo(IFT_BOOL, "mirror", 1, &data->mirror);
  add_fieldinfo(IFT_BOOL, "mirror_supported", 1, &data->mirror_supported);
  add_messageinfo("SetEffectMessage");
  add_messageinfo("SetZoomMessage");
  add_messageinfo("SetMirrorMessage");
  unsigned char tmp_hash[] = {0xc, 0xc9, 0x4a, 0x24, 0x89, 0xb8, 0x9c, 0xd1, 0x7f, 0xf5, 0xc4, 0xa3, 0x41, 0xca, 0x9a, 0xc1};
  set_hash(tmp_hash);
}

/** Destructor */
CameraControlInterface::~CameraControlInterface()
{
  free(data_ptr);
}
/** Convert Effect constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
CameraControlInterface::tostring_Effect(Effect value) const
{
  switch (value) {
  case EFF_NONE: return "EFF_NONE";
  case EFF_PASTEL: return "EFF_PASTEL";
  case EFF_NEGATIVE: return "EFF_NEGATIVE";
  case EFF_BW: return "EFF_BW";
  case EFF_SOLARIZE: return "EFF_SOLARIZE";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get effect value.
 * Currently active effect.
 * @return effect value
 */
CameraControlInterface::Effect
CameraControlInterface::effect() const
{
  return (CameraControlInterface::Effect)data->effect;
}

/** Get maximum length of effect value.
 * @return length of effect value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
CameraControlInterface::maxlenof_effect() const
{
  return 1;
}

/** Set effect value.
 * Currently active effect.
 * @param new_effect new effect value
 */
void
CameraControlInterface::set_effect(const Effect new_effect)
{
  data->effect = new_effect;
  data_changed = true;
}

/** Get effect_supported value.
 * Are effects supported?
 * @return effect_supported value
 */
bool
CameraControlInterface::is_effect_supported() const
{
  return data->effect_supported;
}

/** Get maximum length of effect_supported value.
 * @return length of effect_supported value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
CameraControlInterface::maxlenof_effect_supported() const
{
  return 1;
}

/** Set effect_supported value.
 * Are effects supported?
 * @param new_effect_supported new effect_supported value
 */
void
CameraControlInterface::set_effect_supported(const bool new_effect_supported)
{
  data->effect_supported = new_effect_supported;
  data_changed = true;
}

/** Get zoom value.
 * Current zoom setting.
 * @return zoom value
 */
uint32_t
CameraControlInterface::zoom() const
{
  return data->zoom;
}

/** Get maximum length of zoom value.
 * @return length of zoom value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
CameraControlInterface::maxlenof_zoom() const
{
  return 1;
}

/** Set zoom value.
 * Current zoom setting.
 * @param new_zoom new zoom value
 */
void
CameraControlInterface::set_zoom(const uint32_t new_zoom)
{
  data->zoom = new_zoom;
  data_changed = true;
}

/** Get zoom_supported value.
 * Is zooming supported?
 * @return zoom_supported value
 */
bool
CameraControlInterface::is_zoom_supported() const
{
  return data->zoom_supported;
}

/** Get maximum length of zoom_supported value.
 * @return length of zoom_supported value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
CameraControlInterface::maxlenof_zoom_supported() const
{
  return 1;
}

/** Set zoom_supported value.
 * Is zooming supported?
 * @param new_zoom_supported new zoom_supported value
 */
void
CameraControlInterface::set_zoom_supported(const bool new_zoom_supported)
{
  data->zoom_supported = new_zoom_supported;
  data_changed = true;
}

/** Get zoom_max value.
 * Maximum zoom value
 * @return zoom_max value
 */
uint32_t
CameraControlInterface::zoom_max() const
{
  return data->zoom_max;
}

/** Get maximum length of zoom_max value.
 * @return length of zoom_max value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
CameraControlInterface::maxlenof_zoom_max() const
{
  return 1;
}

/** Set zoom_max value.
 * Maximum zoom value
 * @param new_zoom_max new zoom_max value
 */
void
CameraControlInterface::set_zoom_max(const uint32_t new_zoom_max)
{
  data->zoom_max = new_zoom_max;
  data_changed = true;
}

/** Get zoom_min value.
 * Minimum zoom
 * @return zoom_min value
 */
uint32_t
CameraControlInterface::zoom_min() const
{
  return data->zoom_min;
}

/** Get maximum length of zoom_min value.
 * @return length of zoom_min value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
CameraControlInterface::maxlenof_zoom_min() const
{
  return 1;
}

/** Set zoom_min value.
 * Minimum zoom
 * @param new_zoom_min new zoom_min value
 */
void
CameraControlInterface::set_zoom_min(const uint32_t new_zoom_min)
{
  data->zoom_min = new_zoom_min;
  data_changed = true;
}

/** Get mirror value.
 * Is the image mirrored?
 * @return mirror value
 */
bool
CameraControlInterface::is_mirror() const
{
  return data->mirror;
}

/** Get maximum length of mirror value.
 * @return length of mirror value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
CameraControlInterface::maxlenof_mirror() const
{
  return 1;
}

/** Set mirror value.
 * Is the image mirrored?
 * @param new_mirror new mirror value
 */
void
CameraControlInterface::set_mirror(const bool new_mirror)
{
  data->mirror = new_mirror;
  data_changed = true;
}

/** Get mirror_supported value.
 * Is mirroring supported?
 * @return mirror_supported value
 */
bool
CameraControlInterface::is_mirror_supported() const
{
  return data->mirror_supported;
}

/** Get maximum length of mirror_supported value.
 * @return length of mirror_supported value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
CameraControlInterface::maxlenof_mirror_supported() const
{
  return 1;
}

/** Set mirror_supported value.
 * Is mirroring supported?
 * @param new_mirror_supported new mirror_supported value
 */
void
CameraControlInterface::set_mirror_supported(const bool new_mirror_supported)
{
  data->mirror_supported = new_mirror_supported;
  data_changed = true;
}

/* =========== message create =========== */
Message *
CameraControlInterface::create_message(const char *type) const
{
  if ( strncmp("SetEffectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetEffectMessage();
  } else if ( strncmp("SetZoomMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetZoomMessage();
  } else if ( strncmp("SetMirrorMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMirrorMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
CameraControlInterface::copy_values(const Interface *other)
{
  const CameraControlInterface *oi = dynamic_cast<const CameraControlInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(CameraControlInterface_data_t));
}

const char *
CameraControlInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "Effect") == 0) {
    return tostring_Effect((Effect)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class CameraControlInterface::SetEffectMessage <interfaces/CameraControlInterface.h>
 * SetEffectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_effect initial value for effect
 */
CameraControlInterface::SetEffectMessage::SetEffectMessage(const Effect ini_effect) : Message("SetEffectMessage")
{
  data_size = sizeof(SetEffectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEffectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->effect = ini_effect;
  enum_map_Effect[(int)EFF_NONE] = "EFF_NONE";
  enum_map_Effect[(int)EFF_PASTEL] = "EFF_PASTEL";
  enum_map_Effect[(int)EFF_NEGATIVE] = "EFF_NEGATIVE";
  enum_map_Effect[(int)EFF_BW] = "EFF_BW";
  enum_map_Effect[(int)EFF_SOLARIZE] = "EFF_SOLARIZE";
  add_fieldinfo(IFT_ENUM, "effect", 1, &data->effect, "Effect", &enum_map_Effect);
}
/** Constructor */
CameraControlInterface::SetEffectMessage::SetEffectMessage() : Message("SetEffectMessage")
{
  data_size = sizeof(SetEffectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEffectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Effect[(int)EFF_NONE] = "EFF_NONE";
  enum_map_Effect[(int)EFF_PASTEL] = "EFF_PASTEL";
  enum_map_Effect[(int)EFF_NEGATIVE] = "EFF_NEGATIVE";
  enum_map_Effect[(int)EFF_BW] = "EFF_BW";
  enum_map_Effect[(int)EFF_SOLARIZE] = "EFF_SOLARIZE";
  add_fieldinfo(IFT_ENUM, "effect", 1, &data->effect, "Effect", &enum_map_Effect);
}

/** Destructor */
CameraControlInterface::SetEffectMessage::~SetEffectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
CameraControlInterface::SetEffectMessage::SetEffectMessage(const SetEffectMessage *m) : Message("SetEffectMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetEffectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get effect value.
 * Currently active effect.
 * @return effect value
 */
CameraControlInterface::Effect
CameraControlInterface::SetEffectMessage::effect() const
{
  return (CameraControlInterface::Effect)data->effect;
}

/** Get maximum length of effect value.
 * @return length of effect value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
CameraControlInterface::SetEffectMessage::maxlenof_effect() const
{
  return 1;
}

/** Set effect value.
 * Currently active effect.
 * @param new_effect new effect value
 */
void
CameraControlInterface::SetEffectMessage::set_effect(const Effect new_effect)
{
  data->effect = new_effect;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
CameraControlInterface::SetEffectMessage::clone() const
{
  return new CameraControlInterface::SetEffectMessage(this);
}
/** @class CameraControlInterface::SetZoomMessage <interfaces/CameraControlInterface.h>
 * SetZoomMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_zoom initial value for zoom
 */
CameraControlInterface::SetZoomMessage::SetZoomMessage(const uint32_t ini_zoom) : Message("SetZoomMessage")
{
  data_size = sizeof(SetZoomMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetZoomMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->zoom = ini_zoom;
  enum_map_Effect[(int)EFF_NONE] = "EFF_NONE";
  enum_map_Effect[(int)EFF_PASTEL] = "EFF_PASTEL";
  enum_map_Effect[(int)EFF_NEGATIVE] = "EFF_NEGATIVE";
  enum_map_Effect[(int)EFF_BW] = "EFF_BW";
  enum_map_Effect[(int)EFF_SOLARIZE] = "EFF_SOLARIZE";
  add_fieldinfo(IFT_UINT32, "zoom", 1, &data->zoom);
}
/** Constructor */
CameraControlInterface::SetZoomMessage::SetZoomMessage() : Message("SetZoomMessage")
{
  data_size = sizeof(SetZoomMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetZoomMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Effect[(int)EFF_NONE] = "EFF_NONE";
  enum_map_Effect[(int)EFF_PASTEL] = "EFF_PASTEL";
  enum_map_Effect[(int)EFF_NEGATIVE] = "EFF_NEGATIVE";
  enum_map_Effect[(int)EFF_BW] = "EFF_BW";
  enum_map_Effect[(int)EFF_SOLARIZE] = "EFF_SOLARIZE";
  add_fieldinfo(IFT_UINT32, "zoom", 1, &data->zoom);
}

/** Destructor */
CameraControlInterface::SetZoomMessage::~SetZoomMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
CameraControlInterface::SetZoomMessage::SetZoomMessage(const SetZoomMessage *m) : Message("SetZoomMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetZoomMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get zoom value.
 * Current zoom setting.
 * @return zoom value
 */
uint32_t
CameraControlInterface::SetZoomMessage::zoom() const
{
  return data->zoom;
}

/** Get maximum length of zoom value.
 * @return length of zoom value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
CameraControlInterface::SetZoomMessage::maxlenof_zoom() const
{
  return 1;
}

/** Set zoom value.
 * Current zoom setting.
 * @param new_zoom new zoom value
 */
void
CameraControlInterface::SetZoomMessage::set_zoom(const uint32_t new_zoom)
{
  data->zoom = new_zoom;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
CameraControlInterface::SetZoomMessage::clone() const
{
  return new CameraControlInterface::SetZoomMessage(this);
}
/** @class CameraControlInterface::SetMirrorMessage <interfaces/CameraControlInterface.h>
 * SetMirrorMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_mirror initial value for mirror
 */
CameraControlInterface::SetMirrorMessage::SetMirrorMessage(const bool ini_mirror) : Message("SetMirrorMessage")
{
  data_size = sizeof(SetMirrorMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMirrorMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->mirror = ini_mirror;
  enum_map_Effect[(int)EFF_NONE] = "EFF_NONE";
  enum_map_Effect[(int)EFF_PASTEL] = "EFF_PASTEL";
  enum_map_Effect[(int)EFF_NEGATIVE] = "EFF_NEGATIVE";
  enum_map_Effect[(int)EFF_BW] = "EFF_BW";
  enum_map_Effect[(int)EFF_SOLARIZE] = "EFF_SOLARIZE";
  add_fieldinfo(IFT_BOOL, "mirror", 1, &data->mirror);
}
/** Constructor */
CameraControlInterface::SetMirrorMessage::SetMirrorMessage() : Message("SetMirrorMessage")
{
  data_size = sizeof(SetMirrorMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMirrorMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Effect[(int)EFF_NONE] = "EFF_NONE";
  enum_map_Effect[(int)EFF_PASTEL] = "EFF_PASTEL";
  enum_map_Effect[(int)EFF_NEGATIVE] = "EFF_NEGATIVE";
  enum_map_Effect[(int)EFF_BW] = "EFF_BW";
  enum_map_Effect[(int)EFF_SOLARIZE] = "EFF_SOLARIZE";
  add_fieldinfo(IFT_BOOL, "mirror", 1, &data->mirror);
}

/** Destructor */
CameraControlInterface::SetMirrorMessage::~SetMirrorMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
CameraControlInterface::SetMirrorMessage::SetMirrorMessage(const SetMirrorMessage *m) : Message("SetMirrorMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetMirrorMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get mirror value.
 * Is the image mirrored?
 * @return mirror value
 */
bool
CameraControlInterface::SetMirrorMessage::is_mirror() const
{
  return data->mirror;
}

/** Get maximum length of mirror value.
 * @return length of mirror value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
CameraControlInterface::SetMirrorMessage::maxlenof_mirror() const
{
  return 1;
}

/** Set mirror value.
 * Is the image mirrored?
 * @param new_mirror new mirror value
 */
void
CameraControlInterface::SetMirrorMessage::set_mirror(const bool new_mirror)
{
  data->mirror = new_mirror;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
CameraControlInterface::SetMirrorMessage::clone() const
{
  return new CameraControlInterface::SetMirrorMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
CameraControlInterface::message_valid(const Message *message) const
{
  const SetEffectMessage *m0 = dynamic_cast<const SetEffectMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetZoomMessage *m1 = dynamic_cast<const SetZoomMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const SetMirrorMessage *m2 = dynamic_cast<const SetMirrorMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(CameraControlInterface)
/// @endcond


} // end namespace fawkes
