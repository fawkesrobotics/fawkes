
/***************************************************************************
 *  FacerInterface.cpp - Fawkes BlackBoard Interface - FacerInterface
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

#include <interfaces/FacerInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class FacerInterface <interfaces/FacerInterface.h>
 * FacerInterface Fawkes BlackBoard Interface.
 * 
      The interface provides access to the face recognition plugin
      (facer). It provides basic status information about facer and
      allows for setting a specific mode and access the resolut.
      calling skills via messages. It can also be used to manually
      restart the Lua interpreter if something is wedged.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
FacerInterface::FacerInterface() : Interface()
{
  data_size = sizeof(FacerInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (FacerInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_ENUM, "opmode", 1, &data->opmode, "if_facer_opmode_t");
  add_fieldinfo(IFT_UINT32, "num_identities", 1, &data->num_identities);
  add_fieldinfo(IFT_UINT32, "recognized_identity", 1, &data->recognized_identity);
  add_fieldinfo(IFT_STRING, "recognized_name", 64, data->recognized_name);
  add_fieldinfo(IFT_UINT32, "num_detections", 1, &data->num_detections);
  add_fieldinfo(IFT_UINT32, "num_recognitions", 1, &data->num_recognitions);
  add_fieldinfo(IFT_UINT32, "most_likely_identity", 1, &data->most_likely_identity);
  add_fieldinfo(IFT_FLOAT, "history_ratio", 1, &data->history_ratio);
  add_fieldinfo(IFT_FLOAT, "sec_since_detection", 1, &data->sec_since_detection);
  add_fieldinfo(IFT_INT32, "visibility_history", 1, &data->visibility_history);
  add_fieldinfo(IFT_BOOL, "learning_in_progress", 1, &data->learning_in_progress);
  add_fieldinfo(IFT_FLOAT, "recording_progress", 1, &data->recording_progress);
  add_fieldinfo(IFT_FLOAT, "bearing", 1, &data->bearing);
  add_fieldinfo(IFT_FLOAT, "slope", 1, &data->slope);
  add_fieldinfo(IFT_UINT32, "requested_index", 1, &data->requested_index);
  add_fieldinfo(IFT_STRING, "requested_name", 64, data->requested_name);
  add_messageinfo("LearnFaceMessage");
  add_messageinfo("SetOpmodeMessage");
  add_messageinfo("EnableIdentityMessage");
  add_messageinfo("SetNameMessage");
  add_messageinfo("GetNameMessage");
  unsigned char tmp_hash[] = {0xe1, 0x12, 0xd2, 0x51, 0x1d, 0x24, 0x1b, 0x27, 0x86, 0xce, 0x29, 0x32, 0xd6, 0x5a, 0x5e, 0xb3};
  set_hash(tmp_hash);
}

/** Destructor */
FacerInterface::~FacerInterface()
{
  free(data_ptr);
}
/** Convert if_facer_opmode_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
FacerInterface::tostring_if_facer_opmode_t(if_facer_opmode_t value) const
{
  switch (value) {
  case OPMODE_DISABLED: return "OPMODE_DISABLED";
  case OPMODE_DETECTION: return "OPMODE_DETECTION";
  case OPMODE_RECOGNITION: return "OPMODE_RECOGNITION";
  case OPMODE_LEARNING: return "OPMODE_LEARNING";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get opmode value.
 * 
      Current opmode.
    
 * @return opmode value
 */
FacerInterface::if_facer_opmode_t
FacerInterface::opmode() const
{
  return data->opmode;
}

/** Get maximum length of opmode value.
 * @return length of opmode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_opmode() const
{
  return 1;
}

/** Set opmode value.
 * 
      Current opmode.
    
 * @param new_opmode new opmode value
 */
void
FacerInterface::set_opmode(const if_facer_opmode_t new_opmode)
{
  data->opmode = new_opmode;
  data_changed = true;
}

/** Get num_identities value.
 * 
      The number of identities in the database.
    
 * @return num_identities value
 */
uint32_t
FacerInterface::num_identities() const
{
  return data->num_identities;
}

/** Get maximum length of num_identities value.
 * @return length of num_identities value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_num_identities() const
{
  return 1;
}

/** Set num_identities value.
 * 
      The number of identities in the database.
    
 * @param new_num_identities new num_identities value
 */
void
FacerInterface::set_num_identities(const uint32_t new_num_identities)
{
  data->num_identities = new_num_identities;
  data_changed = true;
}

/** Get recognized_identity value.
 * 
      The index of the recognized identity.
    
 * @return recognized_identity value
 */
uint32_t
FacerInterface::recognized_identity() const
{
  return data->recognized_identity;
}

/** Get maximum length of recognized_identity value.
 * @return length of recognized_identity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_recognized_identity() const
{
  return 1;
}

/** Set recognized_identity value.
 * 
      The index of the recognized identity.
    
 * @param new_recognized_identity new recognized_identity value
 */
void
FacerInterface::set_recognized_identity(const uint32_t new_recognized_identity)
{
  data->recognized_identity = new_recognized_identity;
  data_changed = true;
}

/** Get recognized_name value.
 * 
      The name of the recognized identity.
    
 * @return recognized_name value
 */
char *
FacerInterface::recognized_name() const
{
  return data->recognized_name;
}

/** Get maximum length of recognized_name value.
 * @return length of recognized_name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_recognized_name() const
{
  return 64;
}

/** Set recognized_name value.
 * 
      The name of the recognized identity.
    
 * @param new_recognized_name new recognized_name value
 */
void
FacerInterface::set_recognized_name(const char * new_recognized_name)
{
  strncpy(data->recognized_name, new_recognized_name, sizeof(data->recognized_name));
  data_changed = true;
}

/** Get num_detections value.
 * 
      Number of currently detected faces.
    
 * @return num_detections value
 */
uint32_t
FacerInterface::num_detections() const
{
  return data->num_detections;
}

/** Get maximum length of num_detections value.
 * @return length of num_detections value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_num_detections() const
{
  return 1;
}

/** Set num_detections value.
 * 
      Number of currently detected faces.
    
 * @param new_num_detections new num_detections value
 */
void
FacerInterface::set_num_detections(const uint32_t new_num_detections)
{
  data->num_detections = new_num_detections;
  data_changed = true;
}

/** Get num_recognitions value.
 * 
      Number of recognized faces.
    
 * @return num_recognitions value
 */
uint32_t
FacerInterface::num_recognitions() const
{
  return data->num_recognitions;
}

/** Get maximum length of num_recognitions value.
 * @return length of num_recognitions value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_num_recognitions() const
{
  return 1;
}

/** Set num_recognitions value.
 * 
      Number of recognized faces.
    
 * @param new_num_recognitions new num_recognitions value
 */
void
FacerInterface::set_num_recognitions(const uint32_t new_num_recognitions)
{
  data->num_recognitions = new_num_recognitions;
  data_changed = true;
}

/** Get most_likely_identity value.
 * 
      The identity that was recognized most prevalently.
    
 * @return most_likely_identity value
 */
uint32_t
FacerInterface::most_likely_identity() const
{
  return data->most_likely_identity;
}

/** Get maximum length of most_likely_identity value.
 * @return length of most_likely_identity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_most_likely_identity() const
{
  return 1;
}

/** Set most_likely_identity value.
 * 
      The identity that was recognized most prevalently.
    
 * @param new_most_likely_identity new most_likely_identity value
 */
void
FacerInterface::set_most_likely_identity(const uint32_t new_most_likely_identity)
{
  data->most_likely_identity = new_most_likely_identity;
  data_changed = true;
}

/** Get history_ratio value.
 * 
      The ratio of the most likely identity showing up in the history
      and the length of the history.
    
 * @return history_ratio value
 */
float
FacerInterface::history_ratio() const
{
  return data->history_ratio;
}

/** Get maximum length of history_ratio value.
 * @return length of history_ratio value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_history_ratio() const
{
  return 1;
}

/** Set history_ratio value.
 * 
      The ratio of the most likely identity showing up in the history
      and the length of the history.
    
 * @param new_history_ratio new history_ratio value
 */
void
FacerInterface::set_history_ratio(const float new_history_ratio)
{
  data->history_ratio = new_history_ratio;
  data_changed = true;
}

/** Get sec_since_detection value.
 * 
      Time in seconds since the last successful detection.
    
 * @return sec_since_detection value
 */
float
FacerInterface::sec_since_detection() const
{
  return data->sec_since_detection;
}

/** Get maximum length of sec_since_detection value.
 * @return length of sec_since_detection value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_sec_since_detection() const
{
  return 1;
}

/** Set sec_since_detection value.
 * 
      Time in seconds since the last successful detection.
    
 * @param new_sec_since_detection new sec_since_detection value
 */
void
FacerInterface::set_sec_since_detection(const float new_sec_since_detection)
{
  data->sec_since_detection = new_sec_since_detection;
  data_changed = true;
}

/** Get visibility_history value.
 * 
      The number of consecutive sighting ( <= 1 ) and non-sightings
      ( >= -1 ), respectively.
    
 * @return visibility_history value
 */
int32_t
FacerInterface::visibility_history() const
{
  return data->visibility_history;
}

/** Get maximum length of visibility_history value.
 * @return length of visibility_history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_visibility_history() const
{
  return 1;
}

/** Set visibility_history value.
 * 
      The number of consecutive sighting ( <= 1 ) and non-sightings
      ( >= -1 ), respectively.
    
 * @param new_visibility_history new visibility_history value
 */
void
FacerInterface::set_visibility_history(const int32_t new_visibility_history)
{
  data->visibility_history = new_visibility_history;
  data_changed = true;
}

/** Get learning_in_progress value.
 * 
      Indicates whether a new identity is currently learnt. If
      learning is in progress only "old" faces can be recognized.
    
 * @return learning_in_progress value
 */
bool
FacerInterface::is_learning_in_progress() const
{
  return data->learning_in_progress;
}

/** Get maximum length of learning_in_progress value.
 * @return length of learning_in_progress value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_learning_in_progress() const
{
  return 1;
}

/** Set learning_in_progress value.
 * 
      Indicates whether a new identity is currently learnt. If
      learning is in progress only "old" faces can be recognized.
    
 * @param new_learning_in_progress new learning_in_progress value
 */
void
FacerInterface::set_learning_in_progress(const bool new_learning_in_progress)
{
  data->learning_in_progress = new_learning_in_progress;
  data_changed = true;
}

/** Get recording_progress value.
 * 
      Indicates the progress of recording images of a new face.
    
 * @return recording_progress value
 */
float
FacerInterface::recording_progress() const
{
  return data->recording_progress;
}

/** Get maximum length of recording_progress value.
 * @return length of recording_progress value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_recording_progress() const
{
  return 1;
}

/** Set recording_progress value.
 * 
      Indicates the progress of recording images of a new face.
    
 * @param new_recording_progress new recording_progress value
 */
void
FacerInterface::set_recording_progress(const float new_recording_progress)
{
  data->recording_progress = new_recording_progress;
  data_changed = true;
}

/** Get bearing value.
 * 
      The relative bearing to the recognized face in radians.
    
 * @return bearing value
 */
float
FacerInterface::bearing() const
{
  return data->bearing;
}

/** Get maximum length of bearing value.
 * @return length of bearing value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_bearing() const
{
  return 1;
}

/** Set bearing value.
 * 
      The relative bearing to the recognized face in radians.
    
 * @param new_bearing new bearing value
 */
void
FacerInterface::set_bearing(const float new_bearing)
{
  data->bearing = new_bearing;
  data_changed = true;
}

/** Get slope value.
 * 
      The relative slope to the recognized face in radians.
    
 * @return slope value
 */
float
FacerInterface::slope() const
{
  return data->slope;
}

/** Get maximum length of slope value.
 * @return length of slope value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_slope() const
{
  return 1;
}

/** Set slope value.
 * 
      The relative slope to the recognized face in radians.
    
 * @param new_slope new slope value
 */
void
FacerInterface::set_slope(const float new_slope)
{
  data->slope = new_slope;
  data_changed = true;
}

/** Get requested_index value.
 * 
      Index of the identity for which the name was requested.
    
 * @return requested_index value
 */
uint32_t
FacerInterface::requested_index() const
{
  return data->requested_index;
}

/** Get maximum length of requested_index value.
 * @return length of requested_index value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_requested_index() const
{
  return 1;
}

/** Set requested_index value.
 * 
      Index of the identity for which the name was requested.
    
 * @param new_requested_index new requested_index value
 */
void
FacerInterface::set_requested_index(const uint32_t new_requested_index)
{
  data->requested_index = new_requested_index;
  data_changed = true;
}

/** Get requested_name value.
 * 
      Requested name.
    
 * @return requested_name value
 */
char *
FacerInterface::requested_name() const
{
  return data->requested_name;
}

/** Get maximum length of requested_name value.
 * @return length of requested_name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::maxlenof_requested_name() const
{
  return 64;
}

/** Set requested_name value.
 * 
      Requested name.
    
 * @param new_requested_name new requested_name value
 */
void
FacerInterface::set_requested_name(const char * new_requested_name)
{
  strncpy(data->requested_name, new_requested_name, sizeof(data->requested_name));
  data_changed = true;
}

/* =========== message create =========== */
Message *
FacerInterface::create_message(const char *type) const
{
  if ( strncmp("LearnFaceMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new LearnFaceMessage();
  } else if ( strncmp("SetOpmodeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetOpmodeMessage();
  } else if ( strncmp("EnableIdentityMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new EnableIdentityMessage();
  } else if ( strncmp("SetNameMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetNameMessage();
  } else if ( strncmp("GetNameMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GetNameMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
FacerInterface::copy_values(const Interface *other)
{
  const FacerInterface *oi = dynamic_cast<const FacerInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(FacerInterface_data_t));
}

const char *
FacerInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "if_facer_opmode_t") == 0) {
    return tostring_if_facer_opmode_t((if_facer_opmode_t)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class FacerInterface::LearnFaceMessage <interfaces/FacerInterface.h>
 * LearnFaceMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_name initial value for name
 */
FacerInterface::LearnFaceMessage::LearnFaceMessage(const char * ini_name) : Message("LearnFaceMessage")
{
  data_size = sizeof(LearnFaceMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (LearnFaceMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->name, ini_name, 64);
  add_fieldinfo(IFT_STRING, "name", 64, data->name);
}
/** Constructor */
FacerInterface::LearnFaceMessage::LearnFaceMessage() : Message("LearnFaceMessage")
{
  data_size = sizeof(LearnFaceMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (LearnFaceMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "name", 64, data->name);
}

/** Destructor */
FacerInterface::LearnFaceMessage::~LearnFaceMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
FacerInterface::LearnFaceMessage::LearnFaceMessage(const LearnFaceMessage *m) : Message("LearnFaceMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (LearnFaceMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get name value.
 * The name assigned to the new identity.
 * @return name value
 */
char *
FacerInterface::LearnFaceMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::LearnFaceMessage::maxlenof_name() const
{
  return 64;
}

/** Set name value.
 * The name assigned to the new identity.
 * @param new_name new name value
 */
void
FacerInterface::LearnFaceMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
FacerInterface::LearnFaceMessage::clone() const
{
  return new FacerInterface::LearnFaceMessage(this);
}
/** @class FacerInterface::SetOpmodeMessage <interfaces/FacerInterface.h>
 * SetOpmodeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_opmode initial value for opmode
 */
FacerInterface::SetOpmodeMessage::SetOpmodeMessage(const if_facer_opmode_t ini_opmode) : Message("SetOpmodeMessage")
{
  data_size = sizeof(SetOpmodeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetOpmodeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->opmode = ini_opmode;
  add_fieldinfo(IFT_ENUM, "opmode", 1, &data->opmode, "if_facer_opmode_t");
}
/** Constructor */
FacerInterface::SetOpmodeMessage::SetOpmodeMessage() : Message("SetOpmodeMessage")
{
  data_size = sizeof(SetOpmodeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetOpmodeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_ENUM, "opmode", 1, &data->opmode, "if_facer_opmode_t");
}

/** Destructor */
FacerInterface::SetOpmodeMessage::~SetOpmodeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
FacerInterface::SetOpmodeMessage::SetOpmodeMessage(const SetOpmodeMessage *m) : Message("SetOpmodeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetOpmodeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get opmode value.
 * 
      Current opmode.
    
 * @return opmode value
 */
FacerInterface::if_facer_opmode_t
FacerInterface::SetOpmodeMessage::opmode() const
{
  return data->opmode;
}

/** Get maximum length of opmode value.
 * @return length of opmode value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::SetOpmodeMessage::maxlenof_opmode() const
{
  return 1;
}

/** Set opmode value.
 * 
      Current opmode.
    
 * @param new_opmode new opmode value
 */
void
FacerInterface::SetOpmodeMessage::set_opmode(const if_facer_opmode_t new_opmode)
{
  data->opmode = new_opmode;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
FacerInterface::SetOpmodeMessage::clone() const
{
  return new FacerInterface::SetOpmodeMessage(this);
}
/** @class FacerInterface::EnableIdentityMessage <interfaces/FacerInterface.h>
 * EnableIdentityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_index initial value for index
 * @param ini_enable initial value for enable
 */
FacerInterface::EnableIdentityMessage::EnableIdentityMessage(const uint32_t ini_index, const bool ini_enable) : Message("EnableIdentityMessage")
{
  data_size = sizeof(EnableIdentityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (EnableIdentityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->index = ini_index;
  data->enable = ini_enable;
  add_fieldinfo(IFT_UINT32, "index", 1, &data->index);
  add_fieldinfo(IFT_BOOL, "enable", 1, &data->enable);
}
/** Constructor */
FacerInterface::EnableIdentityMessage::EnableIdentityMessage() : Message("EnableIdentityMessage")
{
  data_size = sizeof(EnableIdentityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (EnableIdentityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "index", 1, &data->index);
  add_fieldinfo(IFT_BOOL, "enable", 1, &data->enable);
}

/** Destructor */
FacerInterface::EnableIdentityMessage::~EnableIdentityMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
FacerInterface::EnableIdentityMessage::EnableIdentityMessage(const EnableIdentityMessage *m) : Message("EnableIdentityMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (EnableIdentityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get index value.
 * Index of the identity.
 * @return index value
 */
uint32_t
FacerInterface::EnableIdentityMessage::index() const
{
  return data->index;
}

/** Get maximum length of index value.
 * @return length of index value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::EnableIdentityMessage::maxlenof_index() const
{
  return 1;
}

/** Set index value.
 * Index of the identity.
 * @param new_index new index value
 */
void
FacerInterface::EnableIdentityMessage::set_index(const uint32_t new_index)
{
  data->index = new_index;
}

/** Get enable value.
 * En-/disable flag.
 * @return enable value
 */
bool
FacerInterface::EnableIdentityMessage::is_enable() const
{
  return data->enable;
}

/** Get maximum length of enable value.
 * @return length of enable value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::EnableIdentityMessage::maxlenof_enable() const
{
  return 1;
}

/** Set enable value.
 * En-/disable flag.
 * @param new_enable new enable value
 */
void
FacerInterface::EnableIdentityMessage::set_enable(const bool new_enable)
{
  data->enable = new_enable;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
FacerInterface::EnableIdentityMessage::clone() const
{
  return new FacerInterface::EnableIdentityMessage(this);
}
/** @class FacerInterface::SetNameMessage <interfaces/FacerInterface.h>
 * SetNameMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_index initial value for index
 * @param ini_name initial value for name
 */
FacerInterface::SetNameMessage::SetNameMessage(const uint32_t ini_index, const char * ini_name) : Message("SetNameMessage")
{
  data_size = sizeof(SetNameMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetNameMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->index = ini_index;
  strncpy(data->name, ini_name, 64);
  add_fieldinfo(IFT_UINT32, "index", 1, &data->index);
  add_fieldinfo(IFT_STRING, "name", 64, data->name);
}
/** Constructor */
FacerInterface::SetNameMessage::SetNameMessage() : Message("SetNameMessage")
{
  data_size = sizeof(SetNameMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetNameMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "index", 1, &data->index);
  add_fieldinfo(IFT_STRING, "name", 64, data->name);
}

/** Destructor */
FacerInterface::SetNameMessage::~SetNameMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
FacerInterface::SetNameMessage::SetNameMessage(const SetNameMessage *m) : Message("SetNameMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetNameMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get index value.
 * Index of the identity.
 * @return index value
 */
uint32_t
FacerInterface::SetNameMessage::index() const
{
  return data->index;
}

/** Get maximum length of index value.
 * @return length of index value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::SetNameMessage::maxlenof_index() const
{
  return 1;
}

/** Set index value.
 * Index of the identity.
 * @param new_index new index value
 */
void
FacerInterface::SetNameMessage::set_index(const uint32_t new_index)
{
  data->index = new_index;
}

/** Get name value.
 * Name of the identity.
 * @return name value
 */
char *
FacerInterface::SetNameMessage::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::SetNameMessage::maxlenof_name() const
{
  return 64;
}

/** Set name value.
 * Name of the identity.
 * @param new_name new name value
 */
void
FacerInterface::SetNameMessage::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
FacerInterface::SetNameMessage::clone() const
{
  return new FacerInterface::SetNameMessage(this);
}
/** @class FacerInterface::GetNameMessage <interfaces/FacerInterface.h>
 * GetNameMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_index initial value for index
 */
FacerInterface::GetNameMessage::GetNameMessage(const uint32_t ini_index) : Message("GetNameMessage")
{
  data_size = sizeof(GetNameMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GetNameMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->index = ini_index;
  add_fieldinfo(IFT_UINT32, "index", 1, &data->index);
}
/** Constructor */
FacerInterface::GetNameMessage::GetNameMessage() : Message("GetNameMessage")
{
  data_size = sizeof(GetNameMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GetNameMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "index", 1, &data->index);
}

/** Destructor */
FacerInterface::GetNameMessage::~GetNameMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
FacerInterface::GetNameMessage::GetNameMessage(const GetNameMessage *m) : Message("GetNameMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GetNameMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get index value.
 * Index of the identity.
 * @return index value
 */
uint32_t
FacerInterface::GetNameMessage::index() const
{
  return data->index;
}

/** Get maximum length of index value.
 * @return length of index value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
FacerInterface::GetNameMessage::maxlenof_index() const
{
  return 1;
}

/** Set index value.
 * Index of the identity.
 * @param new_index new index value
 */
void
FacerInterface::GetNameMessage::set_index(const uint32_t new_index)
{
  data->index = new_index;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
FacerInterface::GetNameMessage::clone() const
{
  return new FacerInterface::GetNameMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
FacerInterface::message_valid(const Message *message) const
{
  const LearnFaceMessage *m0 = dynamic_cast<const LearnFaceMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetOpmodeMessage *m1 = dynamic_cast<const SetOpmodeMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const EnableIdentityMessage *m2 = dynamic_cast<const EnableIdentityMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const SetNameMessage *m3 = dynamic_cast<const SetNameMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const GetNameMessage *m4 = dynamic_cast<const GetNameMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(FacerInterface)
/// @endcond


} // end namespace fawkes
