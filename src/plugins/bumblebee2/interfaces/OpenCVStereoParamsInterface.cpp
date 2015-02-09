
/***************************************************************************
 *  OpenCVStereoParamsInterface.cpp - Fawkes BlackBoard Interface - OpenCVStereoParamsInterface
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

#include <interfaces/OpenCVStereoParamsInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class OpenCVStereoParamsInterface <interfaces/OpenCVStereoParamsInterface.h>
 * OpenCVStereoParamsInterface Fawkes BlackBoard Interface.
 * 
      Read and set stereo processing parameters of the OpenCV stereo
      correspondence matching module.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
OpenCVStereoParamsInterface::OpenCVStereoParamsInterface() : Interface()
{
  data_size = sizeof(OpenCVStereoParamsInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (OpenCVStereoParamsInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_ENUM, "pre_filter_type", 1, &data->pre_filter_type, "PreFilterType", &enum_map_PreFilterType);
  add_fieldinfo(IFT_UINT32, "pre_filter_size", 1, &data->pre_filter_size);
  add_fieldinfo(IFT_UINT32, "pre_filter_cap", 1, &data->pre_filter_cap);
  add_fieldinfo(IFT_UINT32, "sad_window_size", 1, &data->sad_window_size);
  add_fieldinfo(IFT_INT32, "min_disparity", 1, &data->min_disparity);
  add_fieldinfo(IFT_UINT32, "num_disparities", 1, &data->num_disparities);
  add_fieldinfo(IFT_UINT32, "texture_threshold", 1, &data->texture_threshold);
  add_fieldinfo(IFT_UINT32, "uniqueness_ratio", 1, &data->uniqueness_ratio);
  add_fieldinfo(IFT_UINT32, "speckle_window_size", 1, &data->speckle_window_size);
  add_fieldinfo(IFT_UINT32, "speckle_range", 1, &data->speckle_range);
  add_fieldinfo(IFT_BOOL, "try_smaller_windows", 1, &data->try_smaller_windows);
  add_messageinfo("SetPreFilterTypeMessage");
  add_messageinfo("SetPreFilterSizeMessage");
  add_messageinfo("SetPreFilterCapMessage");
  add_messageinfo("SetSADWindowSizeMessage");
  add_messageinfo("SetMinDisparityMessage");
  add_messageinfo("SetNumDisparitiesMessage");
  add_messageinfo("SetTextureThresholdMessage");
  add_messageinfo("SetUniquenessRatioMessage");
  add_messageinfo("SetSpeckleWindowSizeMessage");
  add_messageinfo("SetSpeckleRangeMessage");
  add_messageinfo("SetTrySmallerWindowsMessage");
  unsigned char tmp_hash[] = {0x3a, 0x8e, 0x8c, 0x21, 0xea, 0x8c, 0x83, 0x29, 0x91, 0xdd, 0x5f, 0x5f, 0x16, 0xbf, 0x5f, 0xa6};
  set_hash(tmp_hash);
}

/** Destructor */
OpenCVStereoParamsInterface::~OpenCVStereoParamsInterface()
{
  free(data_ptr);
}
/** Convert PreFilterType constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
OpenCVStereoParamsInterface::tostring_PreFilterType(PreFilterType value) const
{
  switch (value) {
  case PFT_NORMALIZED_RESPONSE: return "PFT_NORMALIZED_RESPONSE";
  case PFT_XSOBEL: return "PFT_XSOBEL";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get pre_filter_type value.
 * Pre-filtering method.
 * @return pre_filter_type value
 */
OpenCVStereoParamsInterface::PreFilterType
OpenCVStereoParamsInterface::pre_filter_type() const
{
  return (OpenCVStereoParamsInterface::PreFilterType)data->pre_filter_type;
}

/** Get maximum length of pre_filter_type value.
 * @return length of pre_filter_type value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::maxlenof_pre_filter_type() const
{
  return 1;
}

/** Set pre_filter_type value.
 * Pre-filtering method.
 * @param new_pre_filter_type new pre_filter_type value
 */
void
OpenCVStereoParamsInterface::set_pre_filter_type(const PreFilterType new_pre_filter_type)
{
  data->pre_filter_type = new_pre_filter_type;
  data_changed = true;
}

/** Get pre_filter_size value.
 * 
      Averaging window size: ~5x5..21x21.
    
 * @return pre_filter_size value
 */
uint32_t
OpenCVStereoParamsInterface::pre_filter_size() const
{
  return data->pre_filter_size;
}

/** Get maximum length of pre_filter_size value.
 * @return length of pre_filter_size value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::maxlenof_pre_filter_size() const
{
  return 1;
}

/** Set pre_filter_size value.
 * 
      Averaging window size: ~5x5..21x21.
    
 * @param new_pre_filter_size new pre_filter_size value
 */
void
OpenCVStereoParamsInterface::set_pre_filter_size(const uint32_t new_pre_filter_size)
{
  data->pre_filter_size = new_pre_filter_size;
  data_changed = true;
}

/** Get pre_filter_cap value.
 * 
      The output of pre-filtering is clipped by [-pre_filter_cap,pre_filter_cap].
    
 * @return pre_filter_cap value
 */
uint32_t
OpenCVStereoParamsInterface::pre_filter_cap() const
{
  return data->pre_filter_cap;
}

/** Get maximum length of pre_filter_cap value.
 * @return length of pre_filter_cap value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::maxlenof_pre_filter_cap() const
{
  return 1;
}

/** Set pre_filter_cap value.
 * 
      The output of pre-filtering is clipped by [-pre_filter_cap,pre_filter_cap].
    
 * @param new_pre_filter_cap new pre_filter_cap value
 */
void
OpenCVStereoParamsInterface::set_pre_filter_cap(const uint32_t new_pre_filter_cap)
{
  data->pre_filter_cap = new_pre_filter_cap;
  data_changed = true;
}

/** Get sad_window_size value.
 * 
      Correspondence using Sum of Absolute Difference (SAD) window size (5x5..21x21).
    
 * @return sad_window_size value
 */
uint32_t
OpenCVStereoParamsInterface::sad_window_size() const
{
  return data->sad_window_size;
}

/** Get maximum length of sad_window_size value.
 * @return length of sad_window_size value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::maxlenof_sad_window_size() const
{
  return 1;
}

/** Set sad_window_size value.
 * 
      Correspondence using Sum of Absolute Difference (SAD) window size (5x5..21x21).
    
 * @param new_sad_window_size new sad_window_size value
 */
void
OpenCVStereoParamsInterface::set_sad_window_size(const uint32_t new_sad_window_size)
{
  data->sad_window_size = new_sad_window_size;
  data_changed = true;
}

/** Get min_disparity value.
 * Minimum disparity (can be negative).
 * @return min_disparity value
 */
int32_t
OpenCVStereoParamsInterface::min_disparity() const
{
  return data->min_disparity;
}

/** Get maximum length of min_disparity value.
 * @return length of min_disparity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::maxlenof_min_disparity() const
{
  return 1;
}

/** Set min_disparity value.
 * Minimum disparity (can be negative).
 * @param new_min_disparity new min_disparity value
 */
void
OpenCVStereoParamsInterface::set_min_disparity(const int32_t new_min_disparity)
{
  data->min_disparity = new_min_disparity;
  data_changed = true;
}

/** Get num_disparities value.
 * 
      Number of disparities (maximum disparity - minimum disparity (> 0)).
    
 * @return num_disparities value
 */
uint32_t
OpenCVStereoParamsInterface::num_disparities() const
{
  return data->num_disparities;
}

/** Get maximum length of num_disparities value.
 * @return length of num_disparities value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::maxlenof_num_disparities() const
{
  return 1;
}

/** Set num_disparities value.
 * 
      Number of disparities (maximum disparity - minimum disparity (> 0)).
    
 * @param new_num_disparities new num_disparities value
 */
void
OpenCVStereoParamsInterface::set_num_disparities(const uint32_t new_num_disparities)
{
  data->num_disparities = new_num_disparities;
  data_changed = true;
}

/** Get texture_threshold value.
 * 
      The disparity is only computed for pixels with textured enough
      neighborhood.
    
 * @return texture_threshold value
 */
uint32_t
OpenCVStereoParamsInterface::texture_threshold() const
{
  return data->texture_threshold;
}

/** Get maximum length of texture_threshold value.
 * @return length of texture_threshold value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::maxlenof_texture_threshold() const
{
  return 1;
}

/** Set texture_threshold value.
 * 
      The disparity is only computed for pixels with textured enough
      neighborhood.
    
 * @param new_texture_threshold new texture_threshold value
 */
void
OpenCVStereoParamsInterface::set_texture_threshold(const uint32_t new_texture_threshold)
{
  data->texture_threshold = new_texture_threshold;
  data_changed = true;
}

/** Get uniqueness_ratio value.
 * 
      Accept the computed disparity d* only if
      SAD(d) >= SAD(d*)*(1 + uniquenessRatio/100.)
      for any d != d*+/-1 within the search range.
    
 * @return uniqueness_ratio value
 */
uint32_t
OpenCVStereoParamsInterface::uniqueness_ratio() const
{
  return data->uniqueness_ratio;
}

/** Get maximum length of uniqueness_ratio value.
 * @return length of uniqueness_ratio value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::maxlenof_uniqueness_ratio() const
{
  return 1;
}

/** Set uniqueness_ratio value.
 * 
      Accept the computed disparity d* only if
      SAD(d) >= SAD(d*)*(1 + uniquenessRatio/100.)
      for any d != d*+/-1 within the search range.
    
 * @param new_uniqueness_ratio new uniqueness_ratio value
 */
void
OpenCVStereoParamsInterface::set_uniqueness_ratio(const uint32_t new_uniqueness_ratio)
{
  data->uniqueness_ratio = new_uniqueness_ratio;
  data_changed = true;
}

/** Get speckle_window_size value.
 * 
      Disparity variation window size.
    
 * @return speckle_window_size value
 */
uint32_t
OpenCVStereoParamsInterface::speckle_window_size() const
{
  return data->speckle_window_size;
}

/** Get maximum length of speckle_window_size value.
 * @return length of speckle_window_size value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::maxlenof_speckle_window_size() const
{
  return 1;
}

/** Set speckle_window_size value.
 * 
      Disparity variation window size.
    
 * @param new_speckle_window_size new speckle_window_size value
 */
void
OpenCVStereoParamsInterface::set_speckle_window_size(const uint32_t new_speckle_window_size)
{
  data->speckle_window_size = new_speckle_window_size;
  data_changed = true;
}

/** Get speckle_range value.
 * 
      Acceptable range of variation in window.
    
 * @return speckle_range value
 */
uint32_t
OpenCVStereoParamsInterface::speckle_range() const
{
  return data->speckle_range;
}

/** Get maximum length of speckle_range value.
 * @return length of speckle_range value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::maxlenof_speckle_range() const
{
  return 1;
}

/** Set speckle_range value.
 * 
      Acceptable range of variation in window.
    
 * @param new_speckle_range new speckle_range value
 */
void
OpenCVStereoParamsInterface::set_speckle_range(const uint32_t new_speckle_range)
{
  data->speckle_range = new_speckle_range;
  data_changed = true;
}

/** Get try_smaller_windows value.
 * 
      If enabled, the results may be more accurate, at the expense of
      slower processing.
    
 * @return try_smaller_windows value
 */
bool
OpenCVStereoParamsInterface::is_try_smaller_windows() const
{
  return data->try_smaller_windows;
}

/** Get maximum length of try_smaller_windows value.
 * @return length of try_smaller_windows value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::maxlenof_try_smaller_windows() const
{
  return 1;
}

/** Set try_smaller_windows value.
 * 
      If enabled, the results may be more accurate, at the expense of
      slower processing.
    
 * @param new_try_smaller_windows new try_smaller_windows value
 */
void
OpenCVStereoParamsInterface::set_try_smaller_windows(const bool new_try_smaller_windows)
{
  data->try_smaller_windows = new_try_smaller_windows;
  data_changed = true;
}

/* =========== message create =========== */
Message *
OpenCVStereoParamsInterface::create_message(const char *type) const
{
  if ( strncmp("SetPreFilterTypeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetPreFilterTypeMessage();
  } else if ( strncmp("SetPreFilterSizeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetPreFilterSizeMessage();
  } else if ( strncmp("SetPreFilterCapMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetPreFilterCapMessage();
  } else if ( strncmp("SetSADWindowSizeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetSADWindowSizeMessage();
  } else if ( strncmp("SetMinDisparityMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMinDisparityMessage();
  } else if ( strncmp("SetNumDisparitiesMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetNumDisparitiesMessage();
  } else if ( strncmp("SetTextureThresholdMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetTextureThresholdMessage();
  } else if ( strncmp("SetUniquenessRatioMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetUniquenessRatioMessage();
  } else if ( strncmp("SetSpeckleWindowSizeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetSpeckleWindowSizeMessage();
  } else if ( strncmp("SetSpeckleRangeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetSpeckleRangeMessage();
  } else if ( strncmp("SetTrySmallerWindowsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetTrySmallerWindowsMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
OpenCVStereoParamsInterface::copy_values(const Interface *other)
{
  const OpenCVStereoParamsInterface *oi = dynamic_cast<const OpenCVStereoParamsInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(OpenCVStereoParamsInterface_data_t));
}

const char *
OpenCVStereoParamsInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "PreFilterType") == 0) {
    return tostring_PreFilterType((PreFilterType)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class OpenCVStereoParamsInterface::SetPreFilterTypeMessage <interfaces/OpenCVStereoParamsInterface.h>
 * SetPreFilterTypeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_pre_filter_type initial value for pre_filter_type
 */
OpenCVStereoParamsInterface::SetPreFilterTypeMessage::SetPreFilterTypeMessage(const PreFilterType ini_pre_filter_type) : Message("SetPreFilterTypeMessage")
{
  data_size = sizeof(SetPreFilterTypeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPreFilterTypeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->pre_filter_type = ini_pre_filter_type;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_ENUM, "pre_filter_type", 1, &data->pre_filter_type, "PreFilterType", &enum_map_PreFilterType);
}
/** Constructor */
OpenCVStereoParamsInterface::SetPreFilterTypeMessage::SetPreFilterTypeMessage() : Message("SetPreFilterTypeMessage")
{
  data_size = sizeof(SetPreFilterTypeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPreFilterTypeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_ENUM, "pre_filter_type", 1, &data->pre_filter_type, "PreFilterType", &enum_map_PreFilterType);
}

/** Destructor */
OpenCVStereoParamsInterface::SetPreFilterTypeMessage::~SetPreFilterTypeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenCVStereoParamsInterface::SetPreFilterTypeMessage::SetPreFilterTypeMessage(const SetPreFilterTypeMessage *m) : Message("SetPreFilterTypeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetPreFilterTypeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get pre_filter_type value.
 * Pre-filtering method.
 * @return pre_filter_type value
 */
OpenCVStereoParamsInterface::PreFilterType
OpenCVStereoParamsInterface::SetPreFilterTypeMessage::pre_filter_type() const
{
  return (OpenCVStereoParamsInterface::PreFilterType)data->pre_filter_type;
}

/** Get maximum length of pre_filter_type value.
 * @return length of pre_filter_type value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::SetPreFilterTypeMessage::maxlenof_pre_filter_type() const
{
  return 1;
}

/** Set pre_filter_type value.
 * Pre-filtering method.
 * @param new_pre_filter_type new pre_filter_type value
 */
void
OpenCVStereoParamsInterface::SetPreFilterTypeMessage::set_pre_filter_type(const PreFilterType new_pre_filter_type)
{
  data->pre_filter_type = new_pre_filter_type;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenCVStereoParamsInterface::SetPreFilterTypeMessage::clone() const
{
  return new OpenCVStereoParamsInterface::SetPreFilterTypeMessage(this);
}
/** @class OpenCVStereoParamsInterface::SetPreFilterSizeMessage <interfaces/OpenCVStereoParamsInterface.h>
 * SetPreFilterSizeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_pre_filter_size initial value for pre_filter_size
 */
OpenCVStereoParamsInterface::SetPreFilterSizeMessage::SetPreFilterSizeMessage(const uint32_t ini_pre_filter_size) : Message("SetPreFilterSizeMessage")
{
  data_size = sizeof(SetPreFilterSizeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPreFilterSizeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->pre_filter_size = ini_pre_filter_size;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "pre_filter_size", 1, &data->pre_filter_size);
}
/** Constructor */
OpenCVStereoParamsInterface::SetPreFilterSizeMessage::SetPreFilterSizeMessage() : Message("SetPreFilterSizeMessage")
{
  data_size = sizeof(SetPreFilterSizeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPreFilterSizeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "pre_filter_size", 1, &data->pre_filter_size);
}

/** Destructor */
OpenCVStereoParamsInterface::SetPreFilterSizeMessage::~SetPreFilterSizeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenCVStereoParamsInterface::SetPreFilterSizeMessage::SetPreFilterSizeMessage(const SetPreFilterSizeMessage *m) : Message("SetPreFilterSizeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetPreFilterSizeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get pre_filter_size value.
 * 
      Averaging window size: ~5x5..21x21.
    
 * @return pre_filter_size value
 */
uint32_t
OpenCVStereoParamsInterface::SetPreFilterSizeMessage::pre_filter_size() const
{
  return data->pre_filter_size;
}

/** Get maximum length of pre_filter_size value.
 * @return length of pre_filter_size value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::SetPreFilterSizeMessage::maxlenof_pre_filter_size() const
{
  return 1;
}

/** Set pre_filter_size value.
 * 
      Averaging window size: ~5x5..21x21.
    
 * @param new_pre_filter_size new pre_filter_size value
 */
void
OpenCVStereoParamsInterface::SetPreFilterSizeMessage::set_pre_filter_size(const uint32_t new_pre_filter_size)
{
  data->pre_filter_size = new_pre_filter_size;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenCVStereoParamsInterface::SetPreFilterSizeMessage::clone() const
{
  return new OpenCVStereoParamsInterface::SetPreFilterSizeMessage(this);
}
/** @class OpenCVStereoParamsInterface::SetPreFilterCapMessage <interfaces/OpenCVStereoParamsInterface.h>
 * SetPreFilterCapMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_pre_filter_cap initial value for pre_filter_cap
 */
OpenCVStereoParamsInterface::SetPreFilterCapMessage::SetPreFilterCapMessage(const uint32_t ini_pre_filter_cap) : Message("SetPreFilterCapMessage")
{
  data_size = sizeof(SetPreFilterCapMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPreFilterCapMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->pre_filter_cap = ini_pre_filter_cap;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "pre_filter_cap", 1, &data->pre_filter_cap);
}
/** Constructor */
OpenCVStereoParamsInterface::SetPreFilterCapMessage::SetPreFilterCapMessage() : Message("SetPreFilterCapMessage")
{
  data_size = sizeof(SetPreFilterCapMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetPreFilterCapMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "pre_filter_cap", 1, &data->pre_filter_cap);
}

/** Destructor */
OpenCVStereoParamsInterface::SetPreFilterCapMessage::~SetPreFilterCapMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenCVStereoParamsInterface::SetPreFilterCapMessage::SetPreFilterCapMessage(const SetPreFilterCapMessage *m) : Message("SetPreFilterCapMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetPreFilterCapMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get pre_filter_cap value.
 * 
      The output of pre-filtering is clipped by [-pre_filter_cap,pre_filter_cap].
    
 * @return pre_filter_cap value
 */
uint32_t
OpenCVStereoParamsInterface::SetPreFilterCapMessage::pre_filter_cap() const
{
  return data->pre_filter_cap;
}

/** Get maximum length of pre_filter_cap value.
 * @return length of pre_filter_cap value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::SetPreFilterCapMessage::maxlenof_pre_filter_cap() const
{
  return 1;
}

/** Set pre_filter_cap value.
 * 
      The output of pre-filtering is clipped by [-pre_filter_cap,pre_filter_cap].
    
 * @param new_pre_filter_cap new pre_filter_cap value
 */
void
OpenCVStereoParamsInterface::SetPreFilterCapMessage::set_pre_filter_cap(const uint32_t new_pre_filter_cap)
{
  data->pre_filter_cap = new_pre_filter_cap;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenCVStereoParamsInterface::SetPreFilterCapMessage::clone() const
{
  return new OpenCVStereoParamsInterface::SetPreFilterCapMessage(this);
}
/** @class OpenCVStereoParamsInterface::SetSADWindowSizeMessage <interfaces/OpenCVStereoParamsInterface.h>
 * SetSADWindowSizeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_sad_window_size initial value for sad_window_size
 */
OpenCVStereoParamsInterface::SetSADWindowSizeMessage::SetSADWindowSizeMessage(const uint32_t ini_sad_window_size) : Message("SetSADWindowSizeMessage")
{
  data_size = sizeof(SetSADWindowSizeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetSADWindowSizeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->sad_window_size = ini_sad_window_size;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "sad_window_size", 1, &data->sad_window_size);
}
/** Constructor */
OpenCVStereoParamsInterface::SetSADWindowSizeMessage::SetSADWindowSizeMessage() : Message("SetSADWindowSizeMessage")
{
  data_size = sizeof(SetSADWindowSizeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetSADWindowSizeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "sad_window_size", 1, &data->sad_window_size);
}

/** Destructor */
OpenCVStereoParamsInterface::SetSADWindowSizeMessage::~SetSADWindowSizeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenCVStereoParamsInterface::SetSADWindowSizeMessage::SetSADWindowSizeMessage(const SetSADWindowSizeMessage *m) : Message("SetSADWindowSizeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetSADWindowSizeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get sad_window_size value.
 * 
      Correspondence using Sum of Absolute Difference (SAD) window size (5x5..21x21).
    
 * @return sad_window_size value
 */
uint32_t
OpenCVStereoParamsInterface::SetSADWindowSizeMessage::sad_window_size() const
{
  return data->sad_window_size;
}

/** Get maximum length of sad_window_size value.
 * @return length of sad_window_size value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::SetSADWindowSizeMessage::maxlenof_sad_window_size() const
{
  return 1;
}

/** Set sad_window_size value.
 * 
      Correspondence using Sum of Absolute Difference (SAD) window size (5x5..21x21).
    
 * @param new_sad_window_size new sad_window_size value
 */
void
OpenCVStereoParamsInterface::SetSADWindowSizeMessage::set_sad_window_size(const uint32_t new_sad_window_size)
{
  data->sad_window_size = new_sad_window_size;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenCVStereoParamsInterface::SetSADWindowSizeMessage::clone() const
{
  return new OpenCVStereoParamsInterface::SetSADWindowSizeMessage(this);
}
/** @class OpenCVStereoParamsInterface::SetMinDisparityMessage <interfaces/OpenCVStereoParamsInterface.h>
 * SetMinDisparityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_min_disparity initial value for min_disparity
 */
OpenCVStereoParamsInterface::SetMinDisparityMessage::SetMinDisparityMessage(const int32_t ini_min_disparity) : Message("SetMinDisparityMessage")
{
  data_size = sizeof(SetMinDisparityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMinDisparityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->min_disparity = ini_min_disparity;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_INT32, "min_disparity", 1, &data->min_disparity);
}
/** Constructor */
OpenCVStereoParamsInterface::SetMinDisparityMessage::SetMinDisparityMessage() : Message("SetMinDisparityMessage")
{
  data_size = sizeof(SetMinDisparityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMinDisparityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_INT32, "min_disparity", 1, &data->min_disparity);
}

/** Destructor */
OpenCVStereoParamsInterface::SetMinDisparityMessage::~SetMinDisparityMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenCVStereoParamsInterface::SetMinDisparityMessage::SetMinDisparityMessage(const SetMinDisparityMessage *m) : Message("SetMinDisparityMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetMinDisparityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get min_disparity value.
 * Minimum disparity (can be negative).
 * @return min_disparity value
 */
int32_t
OpenCVStereoParamsInterface::SetMinDisparityMessage::min_disparity() const
{
  return data->min_disparity;
}

/** Get maximum length of min_disparity value.
 * @return length of min_disparity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::SetMinDisparityMessage::maxlenof_min_disparity() const
{
  return 1;
}

/** Set min_disparity value.
 * Minimum disparity (can be negative).
 * @param new_min_disparity new min_disparity value
 */
void
OpenCVStereoParamsInterface::SetMinDisparityMessage::set_min_disparity(const int32_t new_min_disparity)
{
  data->min_disparity = new_min_disparity;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenCVStereoParamsInterface::SetMinDisparityMessage::clone() const
{
  return new OpenCVStereoParamsInterface::SetMinDisparityMessage(this);
}
/** @class OpenCVStereoParamsInterface::SetNumDisparitiesMessage <interfaces/OpenCVStereoParamsInterface.h>
 * SetNumDisparitiesMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_num_disparities initial value for num_disparities
 */
OpenCVStereoParamsInterface::SetNumDisparitiesMessage::SetNumDisparitiesMessage(const uint32_t ini_num_disparities) : Message("SetNumDisparitiesMessage")
{
  data_size = sizeof(SetNumDisparitiesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetNumDisparitiesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->num_disparities = ini_num_disparities;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "num_disparities", 1, &data->num_disparities);
}
/** Constructor */
OpenCVStereoParamsInterface::SetNumDisparitiesMessage::SetNumDisparitiesMessage() : Message("SetNumDisparitiesMessage")
{
  data_size = sizeof(SetNumDisparitiesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetNumDisparitiesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "num_disparities", 1, &data->num_disparities);
}

/** Destructor */
OpenCVStereoParamsInterface::SetNumDisparitiesMessage::~SetNumDisparitiesMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenCVStereoParamsInterface::SetNumDisparitiesMessage::SetNumDisparitiesMessage(const SetNumDisparitiesMessage *m) : Message("SetNumDisparitiesMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetNumDisparitiesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get num_disparities value.
 * 
      Number of disparities (maximum disparity - minimum disparity (> 0)).
    
 * @return num_disparities value
 */
uint32_t
OpenCVStereoParamsInterface::SetNumDisparitiesMessage::num_disparities() const
{
  return data->num_disparities;
}

/** Get maximum length of num_disparities value.
 * @return length of num_disparities value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::SetNumDisparitiesMessage::maxlenof_num_disparities() const
{
  return 1;
}

/** Set num_disparities value.
 * 
      Number of disparities (maximum disparity - minimum disparity (> 0)).
    
 * @param new_num_disparities new num_disparities value
 */
void
OpenCVStereoParamsInterface::SetNumDisparitiesMessage::set_num_disparities(const uint32_t new_num_disparities)
{
  data->num_disparities = new_num_disparities;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenCVStereoParamsInterface::SetNumDisparitiesMessage::clone() const
{
  return new OpenCVStereoParamsInterface::SetNumDisparitiesMessage(this);
}
/** @class OpenCVStereoParamsInterface::SetTextureThresholdMessage <interfaces/OpenCVStereoParamsInterface.h>
 * SetTextureThresholdMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_texture_threshold initial value for texture_threshold
 */
OpenCVStereoParamsInterface::SetTextureThresholdMessage::SetTextureThresholdMessage(const uint32_t ini_texture_threshold) : Message("SetTextureThresholdMessage")
{
  data_size = sizeof(SetTextureThresholdMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTextureThresholdMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->texture_threshold = ini_texture_threshold;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "texture_threshold", 1, &data->texture_threshold);
}
/** Constructor */
OpenCVStereoParamsInterface::SetTextureThresholdMessage::SetTextureThresholdMessage() : Message("SetTextureThresholdMessage")
{
  data_size = sizeof(SetTextureThresholdMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTextureThresholdMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "texture_threshold", 1, &data->texture_threshold);
}

/** Destructor */
OpenCVStereoParamsInterface::SetTextureThresholdMessage::~SetTextureThresholdMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenCVStereoParamsInterface::SetTextureThresholdMessage::SetTextureThresholdMessage(const SetTextureThresholdMessage *m) : Message("SetTextureThresholdMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetTextureThresholdMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get texture_threshold value.
 * 
      The disparity is only computed for pixels with textured enough
      neighborhood.
    
 * @return texture_threshold value
 */
uint32_t
OpenCVStereoParamsInterface::SetTextureThresholdMessage::texture_threshold() const
{
  return data->texture_threshold;
}

/** Get maximum length of texture_threshold value.
 * @return length of texture_threshold value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::SetTextureThresholdMessage::maxlenof_texture_threshold() const
{
  return 1;
}

/** Set texture_threshold value.
 * 
      The disparity is only computed for pixels with textured enough
      neighborhood.
    
 * @param new_texture_threshold new texture_threshold value
 */
void
OpenCVStereoParamsInterface::SetTextureThresholdMessage::set_texture_threshold(const uint32_t new_texture_threshold)
{
  data->texture_threshold = new_texture_threshold;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenCVStereoParamsInterface::SetTextureThresholdMessage::clone() const
{
  return new OpenCVStereoParamsInterface::SetTextureThresholdMessage(this);
}
/** @class OpenCVStereoParamsInterface::SetUniquenessRatioMessage <interfaces/OpenCVStereoParamsInterface.h>
 * SetUniquenessRatioMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_uniqueness_ratio initial value for uniqueness_ratio
 */
OpenCVStereoParamsInterface::SetUniquenessRatioMessage::SetUniquenessRatioMessage(const uint32_t ini_uniqueness_ratio) : Message("SetUniquenessRatioMessage")
{
  data_size = sizeof(SetUniquenessRatioMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetUniquenessRatioMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->uniqueness_ratio = ini_uniqueness_ratio;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "uniqueness_ratio", 1, &data->uniqueness_ratio);
}
/** Constructor */
OpenCVStereoParamsInterface::SetUniquenessRatioMessage::SetUniquenessRatioMessage() : Message("SetUniquenessRatioMessage")
{
  data_size = sizeof(SetUniquenessRatioMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetUniquenessRatioMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "uniqueness_ratio", 1, &data->uniqueness_ratio);
}

/** Destructor */
OpenCVStereoParamsInterface::SetUniquenessRatioMessage::~SetUniquenessRatioMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenCVStereoParamsInterface::SetUniquenessRatioMessage::SetUniquenessRatioMessage(const SetUniquenessRatioMessage *m) : Message("SetUniquenessRatioMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetUniquenessRatioMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get uniqueness_ratio value.
 * 
      Accept the computed disparity d* only if
      SAD(d) >= SAD(d*)*(1 + uniquenessRatio/100.)
      for any d != d*+/-1 within the search range.
    
 * @return uniqueness_ratio value
 */
uint32_t
OpenCVStereoParamsInterface::SetUniquenessRatioMessage::uniqueness_ratio() const
{
  return data->uniqueness_ratio;
}

/** Get maximum length of uniqueness_ratio value.
 * @return length of uniqueness_ratio value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::SetUniquenessRatioMessage::maxlenof_uniqueness_ratio() const
{
  return 1;
}

/** Set uniqueness_ratio value.
 * 
      Accept the computed disparity d* only if
      SAD(d) >= SAD(d*)*(1 + uniquenessRatio/100.)
      for any d != d*+/-1 within the search range.
    
 * @param new_uniqueness_ratio new uniqueness_ratio value
 */
void
OpenCVStereoParamsInterface::SetUniquenessRatioMessage::set_uniqueness_ratio(const uint32_t new_uniqueness_ratio)
{
  data->uniqueness_ratio = new_uniqueness_ratio;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenCVStereoParamsInterface::SetUniquenessRatioMessage::clone() const
{
  return new OpenCVStereoParamsInterface::SetUniquenessRatioMessage(this);
}
/** @class OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage <interfaces/OpenCVStereoParamsInterface.h>
 * SetSpeckleWindowSizeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_speckle_window_size initial value for speckle_window_size
 */
OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage::SetSpeckleWindowSizeMessage(const uint32_t ini_speckle_window_size) : Message("SetSpeckleWindowSizeMessage")
{
  data_size = sizeof(SetSpeckleWindowSizeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetSpeckleWindowSizeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->speckle_window_size = ini_speckle_window_size;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "speckle_window_size", 1, &data->speckle_window_size);
}
/** Constructor */
OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage::SetSpeckleWindowSizeMessage() : Message("SetSpeckleWindowSizeMessage")
{
  data_size = sizeof(SetSpeckleWindowSizeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetSpeckleWindowSizeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "speckle_window_size", 1, &data->speckle_window_size);
}

/** Destructor */
OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage::~SetSpeckleWindowSizeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage::SetSpeckleWindowSizeMessage(const SetSpeckleWindowSizeMessage *m) : Message("SetSpeckleWindowSizeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetSpeckleWindowSizeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get speckle_window_size value.
 * 
      Disparity variation window size.
    
 * @return speckle_window_size value
 */
uint32_t
OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage::speckle_window_size() const
{
  return data->speckle_window_size;
}

/** Get maximum length of speckle_window_size value.
 * @return length of speckle_window_size value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage::maxlenof_speckle_window_size() const
{
  return 1;
}

/** Set speckle_window_size value.
 * 
      Disparity variation window size.
    
 * @param new_speckle_window_size new speckle_window_size value
 */
void
OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage::set_speckle_window_size(const uint32_t new_speckle_window_size)
{
  data->speckle_window_size = new_speckle_window_size;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage::clone() const
{
  return new OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage(this);
}
/** @class OpenCVStereoParamsInterface::SetSpeckleRangeMessage <interfaces/OpenCVStereoParamsInterface.h>
 * SetSpeckleRangeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_speckle_range initial value for speckle_range
 */
OpenCVStereoParamsInterface::SetSpeckleRangeMessage::SetSpeckleRangeMessage(const uint32_t ini_speckle_range) : Message("SetSpeckleRangeMessage")
{
  data_size = sizeof(SetSpeckleRangeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetSpeckleRangeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->speckle_range = ini_speckle_range;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "speckle_range", 1, &data->speckle_range);
}
/** Constructor */
OpenCVStereoParamsInterface::SetSpeckleRangeMessage::SetSpeckleRangeMessage() : Message("SetSpeckleRangeMessage")
{
  data_size = sizeof(SetSpeckleRangeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetSpeckleRangeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_UINT32, "speckle_range", 1, &data->speckle_range);
}

/** Destructor */
OpenCVStereoParamsInterface::SetSpeckleRangeMessage::~SetSpeckleRangeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenCVStereoParamsInterface::SetSpeckleRangeMessage::SetSpeckleRangeMessage(const SetSpeckleRangeMessage *m) : Message("SetSpeckleRangeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetSpeckleRangeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get speckle_range value.
 * 
      Acceptable range of variation in window.
    
 * @return speckle_range value
 */
uint32_t
OpenCVStereoParamsInterface::SetSpeckleRangeMessage::speckle_range() const
{
  return data->speckle_range;
}

/** Get maximum length of speckle_range value.
 * @return length of speckle_range value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::SetSpeckleRangeMessage::maxlenof_speckle_range() const
{
  return 1;
}

/** Set speckle_range value.
 * 
      Acceptable range of variation in window.
    
 * @param new_speckle_range new speckle_range value
 */
void
OpenCVStereoParamsInterface::SetSpeckleRangeMessage::set_speckle_range(const uint32_t new_speckle_range)
{
  data->speckle_range = new_speckle_range;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenCVStereoParamsInterface::SetSpeckleRangeMessage::clone() const
{
  return new OpenCVStereoParamsInterface::SetSpeckleRangeMessage(this);
}
/** @class OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage <interfaces/OpenCVStereoParamsInterface.h>
 * SetTrySmallerWindowsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_try_smaller_windows initial value for try_smaller_windows
 */
OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage::SetTrySmallerWindowsMessage(const bool ini_try_smaller_windows) : Message("SetTrySmallerWindowsMessage")
{
  data_size = sizeof(SetTrySmallerWindowsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTrySmallerWindowsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->try_smaller_windows = ini_try_smaller_windows;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_BOOL, "try_smaller_windows", 1, &data->try_smaller_windows);
}
/** Constructor */
OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage::SetTrySmallerWindowsMessage() : Message("SetTrySmallerWindowsMessage")
{
  data_size = sizeof(SetTrySmallerWindowsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTrySmallerWindowsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_PreFilterType[(int)PFT_NORMALIZED_RESPONSE] = "PFT_NORMALIZED_RESPONSE";
  enum_map_PreFilterType[(int)PFT_XSOBEL] = "PFT_XSOBEL";
  add_fieldinfo(IFT_BOOL, "try_smaller_windows", 1, &data->try_smaller_windows);
}

/** Destructor */
OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage::~SetTrySmallerWindowsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage::SetTrySmallerWindowsMessage(const SetTrySmallerWindowsMessage *m) : Message("SetTrySmallerWindowsMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetTrySmallerWindowsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get try_smaller_windows value.
 * 
      If enabled, the results may be more accurate, at the expense of
      slower processing.
    
 * @return try_smaller_windows value
 */
bool
OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage::is_try_smaller_windows() const
{
  return data->try_smaller_windows;
}

/** Get maximum length of try_smaller_windows value.
 * @return length of try_smaller_windows value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage::maxlenof_try_smaller_windows() const
{
  return 1;
}

/** Set try_smaller_windows value.
 * 
      If enabled, the results may be more accurate, at the expense of
      slower processing.
    
 * @param new_try_smaller_windows new try_smaller_windows value
 */
void
OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage::set_try_smaller_windows(const bool new_try_smaller_windows)
{
  data->try_smaller_windows = new_try_smaller_windows;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage::clone() const
{
  return new OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
OpenCVStereoParamsInterface::message_valid(const Message *message) const
{
  const SetPreFilterTypeMessage *m0 = dynamic_cast<const SetPreFilterTypeMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetPreFilterSizeMessage *m1 = dynamic_cast<const SetPreFilterSizeMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const SetPreFilterCapMessage *m2 = dynamic_cast<const SetPreFilterCapMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const SetSADWindowSizeMessage *m3 = dynamic_cast<const SetSADWindowSizeMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const SetMinDisparityMessage *m4 = dynamic_cast<const SetMinDisparityMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const SetNumDisparitiesMessage *m5 = dynamic_cast<const SetNumDisparitiesMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const SetTextureThresholdMessage *m6 = dynamic_cast<const SetTextureThresholdMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const SetUniquenessRatioMessage *m7 = dynamic_cast<const SetUniquenessRatioMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const SetSpeckleWindowSizeMessage *m8 = dynamic_cast<const SetSpeckleWindowSizeMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  const SetSpeckleRangeMessage *m9 = dynamic_cast<const SetSpeckleRangeMessage *>(message);
  if ( m9 != NULL ) {
    return true;
  }
  const SetTrySmallerWindowsMessage *m10 = dynamic_cast<const SetTrySmallerWindowsMessage *>(message);
  if ( m10 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(OpenCVStereoParamsInterface)
/// @endcond


} // end namespace fawkes
