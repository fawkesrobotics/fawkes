
/***************************************************************************
 *  PanTiltInterface.cpp - Fawkes BlackBoard Interface - PanTiltInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2009  Tim Niemueller
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

#include <interfaces/PanTiltInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class PanTiltInterface <interfaces/PanTiltInterface.h>
 * PanTiltInterface Fawkes BlackBoard Interface.
 * 
      Interface to access pan/tilt units.
    
 * @ingroup FawkesInterfaces
 */


/** FLAG_SUPPORTS_PAN constant */
const uint32_t PanTiltInterface::FLAG_SUPPORTS_PAN = 1u;
/** FLAG_SUPPORTS_TILT constant */
const uint32_t PanTiltInterface::FLAG_SUPPORTS_TILT = 2u;
/** ERROR_NONE constant */
const uint32_t PanTiltInterface::ERROR_NONE = 0u;
/** ERROR_UNSPECIFIC constant */
const uint32_t PanTiltInterface::ERROR_UNSPECIFIC = 1u;
/** ERROR_COMMUNICATION constant */
const uint32_t PanTiltInterface::ERROR_COMMUNICATION = 2u;
/** ERROR_PAN_OUTOFRANGE constant */
const uint32_t PanTiltInterface::ERROR_PAN_OUTOFRANGE = 4u;
/** ERROR_TILT_OUTOFRANGE constant */
const uint32_t PanTiltInterface::ERROR_TILT_OUTOFRANGE = 8u;

/** Constructor */
PanTiltInterface::PanTiltInterface() : Interface()
{
  data_size = sizeof(PanTiltInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (PanTiltInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "flags", 1, &data->flags);
  add_fieldinfo(IFT_FLOAT, "pan", 1, &data->pan);
  add_fieldinfo(IFT_FLOAT, "tilt", 1, &data->tilt);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_UINT32, "error_code", 1, &data->error_code);
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
  add_fieldinfo(IFT_BOOL, "calibrated", 1, &data->calibrated);
  add_fieldinfo(IFT_FLOAT, "min_pan", 1, &data->min_pan);
  add_fieldinfo(IFT_FLOAT, "max_pan", 1, &data->max_pan);
  add_fieldinfo(IFT_FLOAT, "min_tilt", 1, &data->min_tilt);
  add_fieldinfo(IFT_FLOAT, "max_tilt", 1, &data->max_tilt);
  add_fieldinfo(IFT_FLOAT, "max_pan_velocity", 1, &data->max_pan_velocity);
  add_fieldinfo(IFT_FLOAT, "max_tilt_velocity", 1, &data->max_tilt_velocity);
  add_fieldinfo(IFT_FLOAT, "pan_velocity", 1, &data->pan_velocity);
  add_fieldinfo(IFT_FLOAT, "tilt_velocity", 1, &data->tilt_velocity);
  add_fieldinfo(IFT_FLOAT, "pan_margin", 1, &data->pan_margin);
  add_fieldinfo(IFT_FLOAT, "tilt_margin", 1, &data->tilt_margin);
  add_messageinfo("StopMessage");
  add_messageinfo("FlushMessage");
  add_messageinfo("CalibrateMessage");
  add_messageinfo("ParkMessage");
  add_messageinfo("GotoMessage");
  add_messageinfo("TimedGotoMessage");
  add_messageinfo("SetEnabledMessage");
  add_messageinfo("SetVelocityMessage");
  add_messageinfo("SetMarginMessage");
  unsigned char tmp_hash[] = {0x3, 0xd7, 0x3b, 0xa8, 0x9f, 0x6d, 00, 0xb9, 0xf5, 0xf2, 0x2f, 0x92, 0x25, 0x1b, 0x87, 0x8e};
  set_hash(tmp_hash);
}

/** Destructor */
PanTiltInterface::~PanTiltInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get flags value.
 * Flags.
 * @return flags value
 */
uint32_t
PanTiltInterface::flags() const
{
  return data->flags;
}

/** Get maximum length of flags value.
 * @return length of flags value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_flags() const
{
  return 1;
}

/** Set flags value.
 * Flags.
 * @param new_flags new flags value
 */
void
PanTiltInterface::set_flags(const uint32_t new_flags)
{
  data->flags = new_flags;
  data_changed = true;
}

/** Get pan value.
 * Current pan.
 * @return pan value
 */
float
PanTiltInterface::pan() const
{
  return data->pan;
}

/** Get maximum length of pan value.
 * @return length of pan value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_pan() const
{
  return 1;
}

/** Set pan value.
 * Current pan.
 * @param new_pan new pan value
 */
void
PanTiltInterface::set_pan(const float new_pan)
{
  data->pan = new_pan;
  data_changed = true;
}

/** Get tilt value.
 * Current tilt.
 * @return tilt value
 */
float
PanTiltInterface::tilt() const
{
  return data->tilt;
}

/** Get maximum length of tilt value.
 * @return length of tilt value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_tilt() const
{
  return 1;
}

/** Set tilt value.
 * Current tilt.
 * @param new_tilt new tilt value
 */
void
PanTiltInterface::set_tilt(const float new_tilt)
{
  data->tilt = new_tilt;
  data_changed = true;
}

/** Get msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @return msgid value
 */
uint32_t
PanTiltInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @param new_msgid new msgid value
 */
void
PanTiltInterface::set_msgid(const uint32_t new_msgid)
{
  data->msgid = new_msgid;
  data_changed = true;
}

/** Get final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @return final value
 */
bool
PanTiltInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @param new_final new final value
 */
void
PanTiltInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/** Get error_code value.
 * Failure code set if
    final is true. 0 if no error occured, an error code from ERROR_*
    constants otherwise (or a bit-wise combination).
 * @return error_code value
 */
uint32_t
PanTiltInterface::error_code() const
{
  return data->error_code;
}

/** Get maximum length of error_code value.
 * @return length of error_code value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_error_code() const
{
  return 1;
}

/** Set error_code value.
 * Failure code set if
    final is true. 0 if no error occured, an error code from ERROR_*
    constants otherwise (or a bit-wise combination).
 * @param new_error_code new error_code value
 */
void
PanTiltInterface::set_error_code(const uint32_t new_error_code)
{
  data->error_code = new_error_code;
  data_changed = true;
}

/** Get enabled value.
 * Is the pan/tilt unit enabled?
 * @return enabled value
 */
bool
PanTiltInterface::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * Is the pan/tilt unit enabled?
 * @param new_enabled new enabled value
 */
void
PanTiltInterface::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
  data_changed = true;
}

/** Get calibrated value.
 * Is the pan/tilt unit calibrated?
 * @return calibrated value
 */
bool
PanTiltInterface::is_calibrated() const
{
  return data->calibrated;
}

/** Get maximum length of calibrated value.
 * @return length of calibrated value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_calibrated() const
{
  return 1;
}

/** Set calibrated value.
 * Is the pan/tilt unit calibrated?
 * @param new_calibrated new calibrated value
 */
void
PanTiltInterface::set_calibrated(const bool new_calibrated)
{
  data->calibrated = new_calibrated;
  data_changed = true;
}

/** Get min_pan value.
 * Minimum pan possible.
 * @return min_pan value
 */
float
PanTiltInterface::min_pan() const
{
  return data->min_pan;
}

/** Get maximum length of min_pan value.
 * @return length of min_pan value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_min_pan() const
{
  return 1;
}

/** Set min_pan value.
 * Minimum pan possible.
 * @param new_min_pan new min_pan value
 */
void
PanTiltInterface::set_min_pan(const float new_min_pan)
{
  data->min_pan = new_min_pan;
  data_changed = true;
}

/** Get max_pan value.
 * Maximum pan possible.
 * @return max_pan value
 */
float
PanTiltInterface::max_pan() const
{
  return data->max_pan;
}

/** Get maximum length of max_pan value.
 * @return length of max_pan value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_max_pan() const
{
  return 1;
}

/** Set max_pan value.
 * Maximum pan possible.
 * @param new_max_pan new max_pan value
 */
void
PanTiltInterface::set_max_pan(const float new_max_pan)
{
  data->max_pan = new_max_pan;
  data_changed = true;
}

/** Get min_tilt value.
 * Minimum tilt possible.
 * @return min_tilt value
 */
float
PanTiltInterface::min_tilt() const
{
  return data->min_tilt;
}

/** Get maximum length of min_tilt value.
 * @return length of min_tilt value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_min_tilt() const
{
  return 1;
}

/** Set min_tilt value.
 * Minimum tilt possible.
 * @param new_min_tilt new min_tilt value
 */
void
PanTiltInterface::set_min_tilt(const float new_min_tilt)
{
  data->min_tilt = new_min_tilt;
  data_changed = true;
}

/** Get max_tilt value.
 * Maximum tilt possible.
 * @return max_tilt value
 */
float
PanTiltInterface::max_tilt() const
{
  return data->max_tilt;
}

/** Get maximum length of max_tilt value.
 * @return length of max_tilt value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_max_tilt() const
{
  return 1;
}

/** Set max_tilt value.
 * Maximum tilt possible.
 * @param new_max_tilt new max_tilt value
 */
void
PanTiltInterface::set_max_tilt(const float new_max_tilt)
{
  data->max_tilt = new_max_tilt;
  data_changed = true;
}

/** Get max_pan_velocity value.
 * Maximum supported pan velocity.
 * @return max_pan_velocity value
 */
float
PanTiltInterface::max_pan_velocity() const
{
  return data->max_pan_velocity;
}

/** Get maximum length of max_pan_velocity value.
 * @return length of max_pan_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_max_pan_velocity() const
{
  return 1;
}

/** Set max_pan_velocity value.
 * Maximum supported pan velocity.
 * @param new_max_pan_velocity new max_pan_velocity value
 */
void
PanTiltInterface::set_max_pan_velocity(const float new_max_pan_velocity)
{
  data->max_pan_velocity = new_max_pan_velocity;
  data_changed = true;
}

/** Get max_tilt_velocity value.
 * Maximum supported tilt velocity.
 * @return max_tilt_velocity value
 */
float
PanTiltInterface::max_tilt_velocity() const
{
  return data->max_tilt_velocity;
}

/** Get maximum length of max_tilt_velocity value.
 * @return length of max_tilt_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_max_tilt_velocity() const
{
  return 1;
}

/** Set max_tilt_velocity value.
 * Maximum supported tilt velocity.
 * @param new_max_tilt_velocity new max_tilt_velocity value
 */
void
PanTiltInterface::set_max_tilt_velocity(const float new_max_tilt_velocity)
{
  data->max_tilt_velocity = new_max_tilt_velocity;
  data_changed = true;
}

/** Get pan_velocity value.
 * Maximum pan velocity currently reached.
 * @return pan_velocity value
 */
float
PanTiltInterface::pan_velocity() const
{
  return data->pan_velocity;
}

/** Get maximum length of pan_velocity value.
 * @return length of pan_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_pan_velocity() const
{
  return 1;
}

/** Set pan_velocity value.
 * Maximum pan velocity currently reached.
 * @param new_pan_velocity new pan_velocity value
 */
void
PanTiltInterface::set_pan_velocity(const float new_pan_velocity)
{
  data->pan_velocity = new_pan_velocity;
  data_changed = true;
}

/** Get tilt_velocity value.
 * Maximum tilt velocity currently reached.
 * @return tilt_velocity value
 */
float
PanTiltInterface::tilt_velocity() const
{
  return data->tilt_velocity;
}

/** Get maximum length of tilt_velocity value.
 * @return length of tilt_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_tilt_velocity() const
{
  return 1;
}

/** Set tilt_velocity value.
 * Maximum tilt velocity currently reached.
 * @param new_tilt_velocity new tilt_velocity value
 */
void
PanTiltInterface::set_tilt_velocity(const float new_tilt_velocity)
{
  data->tilt_velocity = new_tilt_velocity;
  data_changed = true;
}

/** Get pan_margin value.
 * Margin in radians around a
    target pan value to consider the motion as final.
 * @return pan_margin value
 */
float
PanTiltInterface::pan_margin() const
{
  return data->pan_margin;
}

/** Get maximum length of pan_margin value.
 * @return length of pan_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_pan_margin() const
{
  return 1;
}

/** Set pan_margin value.
 * Margin in radians around a
    target pan value to consider the motion as final.
 * @param new_pan_margin new pan_margin value
 */
void
PanTiltInterface::set_pan_margin(const float new_pan_margin)
{
  data->pan_margin = new_pan_margin;
  data_changed = true;
}

/** Get tilt_margin value.
 * Margin in radians around a
    target tilt value to consider the motion as final.
 * @return tilt_margin value
 */
float
PanTiltInterface::tilt_margin() const
{
  return data->tilt_margin;
}

/** Get maximum length of tilt_margin value.
 * @return length of tilt_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::maxlenof_tilt_margin() const
{
  return 1;
}

/** Set tilt_margin value.
 * Margin in radians around a
    target tilt value to consider the motion as final.
 * @param new_tilt_margin new tilt_margin value
 */
void
PanTiltInterface::set_tilt_margin(const float new_tilt_margin)
{
  data->tilt_margin = new_tilt_margin;
  data_changed = true;
}

/* =========== message create =========== */
Message *
PanTiltInterface::create_message(const char *type) const
{
  if ( strncmp("StopMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopMessage();
  } else if ( strncmp("FlushMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new FlushMessage();
  } else if ( strncmp("CalibrateMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CalibrateMessage();
  } else if ( strncmp("ParkMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ParkMessage();
  } else if ( strncmp("GotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GotoMessage();
  } else if ( strncmp("TimedGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TimedGotoMessage();
  } else if ( strncmp("SetEnabledMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetEnabledMessage();
  } else if ( strncmp("SetVelocityMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetVelocityMessage();
  } else if ( strncmp("SetMarginMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMarginMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
PanTiltInterface::copy_values(const Interface *other)
{
  const PanTiltInterface *oi = dynamic_cast<const PanTiltInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(PanTiltInterface_data_t));
}

const char *
PanTiltInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class PanTiltInterface::StopMessage <interfaces/PanTiltInterface.h>
 * StopMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
PanTiltInterface::StopMessage::StopMessage() : Message("StopMessage")
{
  data_size = sizeof(StopMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
PanTiltInterface::StopMessage::~StopMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PanTiltInterface::StopMessage::StopMessage(const StopMessage *m) : Message("StopMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PanTiltInterface::StopMessage::clone() const
{
  return new PanTiltInterface::StopMessage(this);
}
/** @class PanTiltInterface::FlushMessage <interfaces/PanTiltInterface.h>
 * FlushMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
PanTiltInterface::FlushMessage::FlushMessage() : Message("FlushMessage")
{
  data_size = sizeof(FlushMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (FlushMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
PanTiltInterface::FlushMessage::~FlushMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PanTiltInterface::FlushMessage::FlushMessage(const FlushMessage *m) : Message("FlushMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (FlushMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PanTiltInterface::FlushMessage::clone() const
{
  return new PanTiltInterface::FlushMessage(this);
}
/** @class PanTiltInterface::CalibrateMessage <interfaces/PanTiltInterface.h>
 * CalibrateMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
PanTiltInterface::CalibrateMessage::CalibrateMessage() : Message("CalibrateMessage")
{
  data_size = sizeof(CalibrateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CalibrateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
PanTiltInterface::CalibrateMessage::~CalibrateMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PanTiltInterface::CalibrateMessage::CalibrateMessage(const CalibrateMessage *m) : Message("CalibrateMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CalibrateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PanTiltInterface::CalibrateMessage::clone() const
{
  return new PanTiltInterface::CalibrateMessage(this);
}
/** @class PanTiltInterface::ParkMessage <interfaces/PanTiltInterface.h>
 * ParkMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
PanTiltInterface::ParkMessage::ParkMessage() : Message("ParkMessage")
{
  data_size = sizeof(ParkMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ParkMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
PanTiltInterface::ParkMessage::~ParkMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PanTiltInterface::ParkMessage::ParkMessage(const ParkMessage *m) : Message("ParkMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ParkMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PanTiltInterface::ParkMessage::clone() const
{
  return new PanTiltInterface::ParkMessage(this);
}
/** @class PanTiltInterface::GotoMessage <interfaces/PanTiltInterface.h>
 * GotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_pan initial value for pan
 * @param ini_tilt initial value for tilt
 */
PanTiltInterface::GotoMessage::GotoMessage(const float ini_pan, const float ini_tilt) : Message("GotoMessage")
{
  data_size = sizeof(GotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->pan = ini_pan;
  data->tilt = ini_tilt;
  add_fieldinfo(IFT_FLOAT, "pan", 1, &data->pan);
  add_fieldinfo(IFT_FLOAT, "tilt", 1, &data->tilt);
}
/** Constructor */
PanTiltInterface::GotoMessage::GotoMessage() : Message("GotoMessage")
{
  data_size = sizeof(GotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "pan", 1, &data->pan);
  add_fieldinfo(IFT_FLOAT, "tilt", 1, &data->tilt);
}

/** Destructor */
PanTiltInterface::GotoMessage::~GotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PanTiltInterface::GotoMessage::GotoMessage(const GotoMessage *m) : Message("GotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get pan value.
 * Current pan.
 * @return pan value
 */
float
PanTiltInterface::GotoMessage::pan() const
{
  return data->pan;
}

/** Get maximum length of pan value.
 * @return length of pan value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::GotoMessage::maxlenof_pan() const
{
  return 1;
}

/** Set pan value.
 * Current pan.
 * @param new_pan new pan value
 */
void
PanTiltInterface::GotoMessage::set_pan(const float new_pan)
{
  data->pan = new_pan;
}

/** Get tilt value.
 * Current tilt.
 * @return tilt value
 */
float
PanTiltInterface::GotoMessage::tilt() const
{
  return data->tilt;
}

/** Get maximum length of tilt value.
 * @return length of tilt value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::GotoMessage::maxlenof_tilt() const
{
  return 1;
}

/** Set tilt value.
 * Current tilt.
 * @param new_tilt new tilt value
 */
void
PanTiltInterface::GotoMessage::set_tilt(const float new_tilt)
{
  data->tilt = new_tilt;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PanTiltInterface::GotoMessage::clone() const
{
  return new PanTiltInterface::GotoMessage(this);
}
/** @class PanTiltInterface::TimedGotoMessage <interfaces/PanTiltInterface.h>
 * TimedGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_time_sec initial value for time_sec
 * @param ini_pan initial value for pan
 * @param ini_tilt initial value for tilt
 */
PanTiltInterface::TimedGotoMessage::TimedGotoMessage(const float ini_time_sec, const float ini_pan, const float ini_tilt) : Message("TimedGotoMessage")
{
  data_size = sizeof(TimedGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TimedGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->time_sec = ini_time_sec;
  data->pan = ini_pan;
  data->tilt = ini_tilt;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "pan", 1, &data->pan);
  add_fieldinfo(IFT_FLOAT, "tilt", 1, &data->tilt);
}
/** Constructor */
PanTiltInterface::TimedGotoMessage::TimedGotoMessage() : Message("TimedGotoMessage")
{
  data_size = sizeof(TimedGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TimedGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "pan", 1, &data->pan);
  add_fieldinfo(IFT_FLOAT, "tilt", 1, &data->tilt);
}

/** Destructor */
PanTiltInterface::TimedGotoMessage::~TimedGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PanTiltInterface::TimedGotoMessage::TimedGotoMessage(const TimedGotoMessage *m) : Message("TimedGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TimedGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get time_sec value.
 * Time in seconds when to reach
    the final position.
 * @return time_sec value
 */
float
PanTiltInterface::TimedGotoMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::TimedGotoMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time in seconds when to reach
    the final position.
 * @param new_time_sec new time_sec value
 */
void
PanTiltInterface::TimedGotoMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Get pan value.
 * Current pan.
 * @return pan value
 */
float
PanTiltInterface::TimedGotoMessage::pan() const
{
  return data->pan;
}

/** Get maximum length of pan value.
 * @return length of pan value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::TimedGotoMessage::maxlenof_pan() const
{
  return 1;
}

/** Set pan value.
 * Current pan.
 * @param new_pan new pan value
 */
void
PanTiltInterface::TimedGotoMessage::set_pan(const float new_pan)
{
  data->pan = new_pan;
}

/** Get tilt value.
 * Current tilt.
 * @return tilt value
 */
float
PanTiltInterface::TimedGotoMessage::tilt() const
{
  return data->tilt;
}

/** Get maximum length of tilt value.
 * @return length of tilt value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::TimedGotoMessage::maxlenof_tilt() const
{
  return 1;
}

/** Set tilt value.
 * Current tilt.
 * @param new_tilt new tilt value
 */
void
PanTiltInterface::TimedGotoMessage::set_tilt(const float new_tilt)
{
  data->tilt = new_tilt;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PanTiltInterface::TimedGotoMessage::clone() const
{
  return new PanTiltInterface::TimedGotoMessage(this);
}
/** @class PanTiltInterface::SetEnabledMessage <interfaces/PanTiltInterface.h>
 * SetEnabledMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_enabled initial value for enabled
 */
PanTiltInterface::SetEnabledMessage::SetEnabledMessage(const bool ini_enabled) : Message("SetEnabledMessage")
{
  data_size = sizeof(SetEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->enabled = ini_enabled;
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
}
/** Constructor */
PanTiltInterface::SetEnabledMessage::SetEnabledMessage() : Message("SetEnabledMessage")
{
  data_size = sizeof(SetEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
}

/** Destructor */
PanTiltInterface::SetEnabledMessage::~SetEnabledMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PanTiltInterface::SetEnabledMessage::SetEnabledMessage(const SetEnabledMessage *m) : Message("SetEnabledMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get enabled value.
 * Is the pan/tilt unit enabled?
 * @return enabled value
 */
bool
PanTiltInterface::SetEnabledMessage::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::SetEnabledMessage::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * Is the pan/tilt unit enabled?
 * @param new_enabled new enabled value
 */
void
PanTiltInterface::SetEnabledMessage::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PanTiltInterface::SetEnabledMessage::clone() const
{
  return new PanTiltInterface::SetEnabledMessage(this);
}
/** @class PanTiltInterface::SetVelocityMessage <interfaces/PanTiltInterface.h>
 * SetVelocityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_pan_velocity initial value for pan_velocity
 * @param ini_tilt_velocity initial value for tilt_velocity
 */
PanTiltInterface::SetVelocityMessage::SetVelocityMessage(const float ini_pan_velocity, const float ini_tilt_velocity) : Message("SetVelocityMessage")
{
  data_size = sizeof(SetVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->pan_velocity = ini_pan_velocity;
  data->tilt_velocity = ini_tilt_velocity;
  add_fieldinfo(IFT_FLOAT, "pan_velocity", 1, &data->pan_velocity);
  add_fieldinfo(IFT_FLOAT, "tilt_velocity", 1, &data->tilt_velocity);
}
/** Constructor */
PanTiltInterface::SetVelocityMessage::SetVelocityMessage() : Message("SetVelocityMessage")
{
  data_size = sizeof(SetVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "pan_velocity", 1, &data->pan_velocity);
  add_fieldinfo(IFT_FLOAT, "tilt_velocity", 1, &data->tilt_velocity);
}

/** Destructor */
PanTiltInterface::SetVelocityMessage::~SetVelocityMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PanTiltInterface::SetVelocityMessage::SetVelocityMessage(const SetVelocityMessage *m) : Message("SetVelocityMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get pan_velocity value.
 * Maximum pan velocity currently reached.
 * @return pan_velocity value
 */
float
PanTiltInterface::SetVelocityMessage::pan_velocity() const
{
  return data->pan_velocity;
}

/** Get maximum length of pan_velocity value.
 * @return length of pan_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::SetVelocityMessage::maxlenof_pan_velocity() const
{
  return 1;
}

/** Set pan_velocity value.
 * Maximum pan velocity currently reached.
 * @param new_pan_velocity new pan_velocity value
 */
void
PanTiltInterface::SetVelocityMessage::set_pan_velocity(const float new_pan_velocity)
{
  data->pan_velocity = new_pan_velocity;
}

/** Get tilt_velocity value.
 * Maximum tilt velocity currently reached.
 * @return tilt_velocity value
 */
float
PanTiltInterface::SetVelocityMessage::tilt_velocity() const
{
  return data->tilt_velocity;
}

/** Get maximum length of tilt_velocity value.
 * @return length of tilt_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::SetVelocityMessage::maxlenof_tilt_velocity() const
{
  return 1;
}

/** Set tilt_velocity value.
 * Maximum tilt velocity currently reached.
 * @param new_tilt_velocity new tilt_velocity value
 */
void
PanTiltInterface::SetVelocityMessage::set_tilt_velocity(const float new_tilt_velocity)
{
  data->tilt_velocity = new_tilt_velocity;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PanTiltInterface::SetVelocityMessage::clone() const
{
  return new PanTiltInterface::SetVelocityMessage(this);
}
/** @class PanTiltInterface::SetMarginMessage <interfaces/PanTiltInterface.h>
 * SetMarginMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_pan_margin initial value for pan_margin
 * @param ini_tilt_margin initial value for tilt_margin
 */
PanTiltInterface::SetMarginMessage::SetMarginMessage(const float ini_pan_margin, const float ini_tilt_margin) : Message("SetMarginMessage")
{
  data_size = sizeof(SetMarginMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMarginMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->pan_margin = ini_pan_margin;
  data->tilt_margin = ini_tilt_margin;
  add_fieldinfo(IFT_FLOAT, "pan_margin", 1, &data->pan_margin);
  add_fieldinfo(IFT_FLOAT, "tilt_margin", 1, &data->tilt_margin);
}
/** Constructor */
PanTiltInterface::SetMarginMessage::SetMarginMessage() : Message("SetMarginMessage")
{
  data_size = sizeof(SetMarginMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMarginMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "pan_margin", 1, &data->pan_margin);
  add_fieldinfo(IFT_FLOAT, "tilt_margin", 1, &data->tilt_margin);
}

/** Destructor */
PanTiltInterface::SetMarginMessage::~SetMarginMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
PanTiltInterface::SetMarginMessage::SetMarginMessage(const SetMarginMessage *m) : Message("SetMarginMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetMarginMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get pan_margin value.
 * Margin in radians around a
    target pan value to consider the motion as final.
 * @return pan_margin value
 */
float
PanTiltInterface::SetMarginMessage::pan_margin() const
{
  return data->pan_margin;
}

/** Get maximum length of pan_margin value.
 * @return length of pan_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::SetMarginMessage::maxlenof_pan_margin() const
{
  return 1;
}

/** Set pan_margin value.
 * Margin in radians around a
    target pan value to consider the motion as final.
 * @param new_pan_margin new pan_margin value
 */
void
PanTiltInterface::SetMarginMessage::set_pan_margin(const float new_pan_margin)
{
  data->pan_margin = new_pan_margin;
}

/** Get tilt_margin value.
 * Margin in radians around a
    target tilt value to consider the motion as final.
 * @return tilt_margin value
 */
float
PanTiltInterface::SetMarginMessage::tilt_margin() const
{
  return data->tilt_margin;
}

/** Get maximum length of tilt_margin value.
 * @return length of tilt_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PanTiltInterface::SetMarginMessage::maxlenof_tilt_margin() const
{
  return 1;
}

/** Set tilt_margin value.
 * Margin in radians around a
    target tilt value to consider the motion as final.
 * @param new_tilt_margin new tilt_margin value
 */
void
PanTiltInterface::SetMarginMessage::set_tilt_margin(const float new_tilt_margin)
{
  data->tilt_margin = new_tilt_margin;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
PanTiltInterface::SetMarginMessage::clone() const
{
  return new PanTiltInterface::SetMarginMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
PanTiltInterface::message_valid(const Message *message) const
{
  const StopMessage *m0 = dynamic_cast<const StopMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const FlushMessage *m1 = dynamic_cast<const FlushMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const CalibrateMessage *m2 = dynamic_cast<const CalibrateMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const ParkMessage *m3 = dynamic_cast<const ParkMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const GotoMessage *m4 = dynamic_cast<const GotoMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const TimedGotoMessage *m5 = dynamic_cast<const TimedGotoMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const SetEnabledMessage *m6 = dynamic_cast<const SetEnabledMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const SetVelocityMessage *m7 = dynamic_cast<const SetVelocityMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const SetMarginMessage *m8 = dynamic_cast<const SetMarginMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(PanTiltInterface)
/// @endcond


} // end namespace fawkes
