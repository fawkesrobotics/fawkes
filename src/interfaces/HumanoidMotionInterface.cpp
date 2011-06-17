
/***************************************************************************
 *  HumanoidMotionInterface.cpp - Fawkes BlackBoard Interface - HumanoidMotionInterface
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

#include <interfaces/HumanoidMotionInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class HumanoidMotionInterface <interfaces/HumanoidMotionInterface.h>
 * HumanoidMotionInterface Fawkes BlackBoard Interface.
 * 
      This interface provides acces to basic humanoid motion patterns.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
HumanoidMotionInterface::HumanoidMotionInterface() : Interface()
{
  data_size = sizeof(HumanoidMotionInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (HumanoidMotionInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_BOOL, "walking", 1, &data->walking);
  add_fieldinfo(IFT_ENUM, "supporting_leg", 1, &data->supporting_leg, "LegEnum");
  add_fieldinfo(IFT_BOOL, "arms_enabled", 1, &data->arms_enabled);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_messageinfo("StopMessage");
  add_messageinfo("WalkStraightMessage");
  add_messageinfo("WalkSidewaysMessage");
  add_messageinfo("WalkArcMessage");
  add_messageinfo("WalkVelocityMessage");
  add_messageinfo("TurnMessage");
  add_messageinfo("KickMessage");
  add_messageinfo("ParkMessage");
  add_messageinfo("GetUpMessage");
  add_messageinfo("StandupMessage");
  add_messageinfo("YawPitchHeadMessage");
  unsigned char tmp_hash[] = {0xfb, 0xe9, 0x6b, 0xa3, 0xa7, 0x60, 0x4b, 0x1d, 0x63, 0x9f, 0x1, 0xe, 0x6d, 0xc2, 0x6b, 0x67};
  set_hash(tmp_hash);
}

/** Destructor */
HumanoidMotionInterface::~HumanoidMotionInterface()
{
  free(data_ptr);
}
/** Convert LegEnum constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
HumanoidMotionInterface::tostring_LegEnum(LegEnum value) const
{
  switch (value) {
  case LEG_LEFT: return "LEG_LEFT";
  case LEG_RIGHT: return "LEG_RIGHT";
  default: return "UNKNOWN";
  }
}
/** Convert StandupEnum constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
HumanoidMotionInterface::tostring_StandupEnum(StandupEnum value) const
{
  switch (value) {
  case STANDUP_DETECT: return "STANDUP_DETECT";
  case STANDUP_BACK: return "STANDUP_BACK";
  case STANDUP_FRONT: return "STANDUP_FRONT";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get walking value.
 * True if the robot is moving.
 * @return walking value
 */
bool
HumanoidMotionInterface::is_walking() const
{
  return data->walking;
}

/** Get maximum length of walking value.
 * @return length of walking value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_walking() const
{
  return 1;
}

/** Set walking value.
 * True if the robot is moving.
 * @param new_walking new walking value
 */
void
HumanoidMotionInterface::set_walking(const bool new_walking)
{
  data->walking = new_walking;
  data_changed = true;
}

/** Get supporting_leg value.
 * Marks the supporting leg
 * @return supporting_leg value
 */
HumanoidMotionInterface::LegEnum
HumanoidMotionInterface::supporting_leg() const
{
  return (HumanoidMotionInterface::LegEnum)data->supporting_leg;
}

/** Get maximum length of supporting_leg value.
 * @return length of supporting_leg value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_supporting_leg() const
{
  return 1;
}

/** Set supporting_leg value.
 * Marks the supporting leg
 * @param new_supporting_leg new supporting_leg value
 */
void
HumanoidMotionInterface::set_supporting_leg(const LegEnum new_supporting_leg)
{
  data->supporting_leg = new_supporting_leg;
  data_changed = true;
}

/** Get arms_enabled value.
 * 
      If true the arms are controlled during walking for balancing.
    
 * @return arms_enabled value
 */
bool
HumanoidMotionInterface::is_arms_enabled() const
{
  return data->arms_enabled;
}

/** Get maximum length of arms_enabled value.
 * @return length of arms_enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_arms_enabled() const
{
  return 1;
}

/** Set arms_enabled value.
 * 
      If true the arms are controlled during walking for balancing.
    
 * @param new_arms_enabled new arms_enabled value
 */
void
HumanoidMotionInterface::set_arms_enabled(const bool new_arms_enabled)
{
  data->arms_enabled = new_arms_enabled;
  data_changed = true;
}

/** Get msgid value.
 * 
      The ID of the message that is currently being
      processed, or 0 if no message is being processed.
    
 * @return msgid value
 */
uint32_t
HumanoidMotionInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * 
      The ID of the message that is currently being
      processed, or 0 if no message is being processed.
    
 * @param new_msgid new msgid value
 */
void
HumanoidMotionInterface::set_msgid(const uint32_t new_msgid)
{
  data->msgid = new_msgid;
  data_changed = true;
}

/* =========== message create =========== */
Message *
HumanoidMotionInterface::create_message(const char *type) const
{
  if ( strncmp("StopMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopMessage();
  } else if ( strncmp("WalkStraightMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new WalkStraightMessage();
  } else if ( strncmp("WalkSidewaysMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new WalkSidewaysMessage();
  } else if ( strncmp("WalkArcMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new WalkArcMessage();
  } else if ( strncmp("WalkVelocityMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new WalkVelocityMessage();
  } else if ( strncmp("TurnMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TurnMessage();
  } else if ( strncmp("KickMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new KickMessage();
  } else if ( strncmp("ParkMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ParkMessage();
  } else if ( strncmp("GetUpMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GetUpMessage();
  } else if ( strncmp("StandupMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StandupMessage();
  } else if ( strncmp("YawPitchHeadMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new YawPitchHeadMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
HumanoidMotionInterface::copy_values(const Interface *other)
{
  const HumanoidMotionInterface *oi = dynamic_cast<const HumanoidMotionInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(HumanoidMotionInterface_data_t));
}

const char *
HumanoidMotionInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "LegEnum") == 0) {
    return tostring_LegEnum((LegEnum)val);
  }
  if (strcmp(enumtype, "StandupEnum") == 0) {
    return tostring_StandupEnum((StandupEnum)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class HumanoidMotionInterface::StopMessage <interfaces/HumanoidMotionInterface.h>
 * StopMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
HumanoidMotionInterface::StopMessage::StopMessage() : Message("StopMessage")
{
  data_size = sizeof(StopMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::StopMessage::~StopMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::StopMessage::StopMessage(const StopMessage *m) : Message("StopMessage")
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
HumanoidMotionInterface::StopMessage::clone() const
{
  return new HumanoidMotionInterface::StopMessage(this);
}
/** @class HumanoidMotionInterface::WalkStraightMessage <interfaces/HumanoidMotionInterface.h>
 * WalkStraightMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_distance initial value for distance
 */
HumanoidMotionInterface::WalkStraightMessage::WalkStraightMessage(const float ini_distance) : Message("WalkStraightMessage")
{
  data_size = sizeof(WalkStraightMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkStraightMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->distance = ini_distance;
  add_fieldinfo(IFT_FLOAT, "distance", 1, &data->distance);
}
/** Constructor */
HumanoidMotionInterface::WalkStraightMessage::WalkStraightMessage() : Message("WalkStraightMessage")
{
  data_size = sizeof(WalkStraightMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkStraightMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "distance", 1, &data->distance);
}

/** Destructor */
HumanoidMotionInterface::WalkStraightMessage::~WalkStraightMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::WalkStraightMessage::WalkStraightMessage(const WalkStraightMessage *m) : Message("WalkStraightMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (WalkStraightMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get distance value.
 * Distance in m to walk.
 * @return distance value
 */
float
HumanoidMotionInterface::WalkStraightMessage::distance() const
{
  return data->distance;
}

/** Get maximum length of distance value.
 * @return length of distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkStraightMessage::maxlenof_distance() const
{
  return 1;
}

/** Set distance value.
 * Distance in m to walk.
 * @param new_distance new distance value
 */
void
HumanoidMotionInterface::WalkStraightMessage::set_distance(const float new_distance)
{
  data->distance = new_distance;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::WalkStraightMessage::clone() const
{
  return new HumanoidMotionInterface::WalkStraightMessage(this);
}
/** @class HumanoidMotionInterface::WalkSidewaysMessage <interfaces/HumanoidMotionInterface.h>
 * WalkSidewaysMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_distance initial value for distance
 */
HumanoidMotionInterface::WalkSidewaysMessage::WalkSidewaysMessage(const float ini_distance) : Message("WalkSidewaysMessage")
{
  data_size = sizeof(WalkSidewaysMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkSidewaysMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->distance = ini_distance;
  add_fieldinfo(IFT_FLOAT, "distance", 1, &data->distance);
}
/** Constructor */
HumanoidMotionInterface::WalkSidewaysMessage::WalkSidewaysMessage() : Message("WalkSidewaysMessage")
{
  data_size = sizeof(WalkSidewaysMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkSidewaysMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "distance", 1, &data->distance);
}

/** Destructor */
HumanoidMotionInterface::WalkSidewaysMessage::~WalkSidewaysMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::WalkSidewaysMessage::WalkSidewaysMessage(const WalkSidewaysMessage *m) : Message("WalkSidewaysMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (WalkSidewaysMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get distance value.
 * Distance in m to walk.
 * @return distance value
 */
float
HumanoidMotionInterface::WalkSidewaysMessage::distance() const
{
  return data->distance;
}

/** Get maximum length of distance value.
 * @return length of distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkSidewaysMessage::maxlenof_distance() const
{
  return 1;
}

/** Set distance value.
 * Distance in m to walk.
 * @param new_distance new distance value
 */
void
HumanoidMotionInterface::WalkSidewaysMessage::set_distance(const float new_distance)
{
  data->distance = new_distance;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::WalkSidewaysMessage::clone() const
{
  return new HumanoidMotionInterface::WalkSidewaysMessage(this);
}
/** @class HumanoidMotionInterface::WalkArcMessage <interfaces/HumanoidMotionInterface.h>
 * WalkArcMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_angle initial value for angle
 * @param ini_radius initial value for radius
 */
HumanoidMotionInterface::WalkArcMessage::WalkArcMessage(const float ini_angle, const float ini_radius) : Message("WalkArcMessage")
{
  data_size = sizeof(WalkArcMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkArcMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->angle = ini_angle;
  data->radius = ini_radius;
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
  add_fieldinfo(IFT_FLOAT, "radius", 1, &data->radius);
}
/** Constructor */
HumanoidMotionInterface::WalkArcMessage::WalkArcMessage() : Message("WalkArcMessage")
{
  data_size = sizeof(WalkArcMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkArcMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
  add_fieldinfo(IFT_FLOAT, "radius", 1, &data->radius);
}

/** Destructor */
HumanoidMotionInterface::WalkArcMessage::~WalkArcMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::WalkArcMessage::WalkArcMessage(const WalkArcMessage *m) : Message("WalkArcMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (WalkArcMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get angle value.
 * Angle in radians to turn over the way.
 * @return angle value
 */
float
HumanoidMotionInterface::WalkArcMessage::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkArcMessage::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Angle in radians to turn over the way.
 * @param new_angle new angle value
 */
void
HumanoidMotionInterface::WalkArcMessage::set_angle(const float new_angle)
{
  data->angle = new_angle;
}

/** Get radius value.
 * Radius in m of the circle in m.
 * @return radius value
 */
float
HumanoidMotionInterface::WalkArcMessage::radius() const
{
  return data->radius;
}

/** Get maximum length of radius value.
 * @return length of radius value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkArcMessage::maxlenof_radius() const
{
  return 1;
}

/** Set radius value.
 * Radius in m of the circle in m.
 * @param new_radius new radius value
 */
void
HumanoidMotionInterface::WalkArcMessage::set_radius(const float new_radius)
{
  data->radius = new_radius;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::WalkArcMessage::clone() const
{
  return new HumanoidMotionInterface::WalkArcMessage(this);
}
/** @class HumanoidMotionInterface::WalkVelocityMessage <interfaces/HumanoidMotionInterface.h>
 * WalkVelocityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_theta initial value for theta
 * @param ini_speed initial value for speed
 */
HumanoidMotionInterface::WalkVelocityMessage::WalkVelocityMessage(const float ini_x, const float ini_y, const float ini_theta, const float ini_speed) : Message("WalkVelocityMessage")
{
  data_size = sizeof(WalkVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->theta = ini_theta;
  data->speed = ini_speed;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "theta", 1, &data->theta);
  add_fieldinfo(IFT_FLOAT, "speed", 1, &data->speed);
}
/** Constructor */
HumanoidMotionInterface::WalkVelocityMessage::WalkVelocityMessage() : Message("WalkVelocityMessage")
{
  data_size = sizeof(WalkVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "theta", 1, &data->theta);
  add_fieldinfo(IFT_FLOAT, "speed", 1, &data->speed);
}

/** Destructor */
HumanoidMotionInterface::WalkVelocityMessage::~WalkVelocityMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::WalkVelocityMessage::WalkVelocityMessage(const WalkVelocityMessage *m) : Message("WalkVelocityMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (WalkVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * 
      Fraction of MaxStepX. Use negative for backwards. [-1.0 to 1.0].
    
 * @return x value
 */
float
HumanoidMotionInterface::WalkVelocityMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkVelocityMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * 
      Fraction of MaxStepX. Use negative for backwards. [-1.0 to 1.0].
    
 * @param new_x new x value
 */
void
HumanoidMotionInterface::WalkVelocityMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * 
      Fraction of MaxStepY. Use negative for right. [-1.0 to 1.0].
    
 * @return y value
 */
float
HumanoidMotionInterface::WalkVelocityMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkVelocityMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * 
      Fraction of MaxStepY. Use negative for right. [-1.0 to 1.0].
    
 * @param new_y new y value
 */
void
HumanoidMotionInterface::WalkVelocityMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get theta value.
 * 
      Fraction of MaxStepTheta. Use negative for clockwise [-1.0 to 1.0].
    
 * @return theta value
 */
float
HumanoidMotionInterface::WalkVelocityMessage::theta() const
{
  return data->theta;
}

/** Get maximum length of theta value.
 * @return length of theta value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkVelocityMessage::maxlenof_theta() const
{
  return 1;
}

/** Set theta value.
 * 
      Fraction of MaxStepTheta. Use negative for clockwise [-1.0 to 1.0].
    
 * @param new_theta new theta value
 */
void
HumanoidMotionInterface::WalkVelocityMessage::set_theta(const float new_theta)
{
  data->theta = new_theta;
}

/** Get speed value.
 * 
      Fraction of MaxStepFrequency [0.0 to 1.0].
    
 * @return speed value
 */
float
HumanoidMotionInterface::WalkVelocityMessage::speed() const
{
  return data->speed;
}

/** Get maximum length of speed value.
 * @return length of speed value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkVelocityMessage::maxlenof_speed() const
{
  return 1;
}

/** Set speed value.
 * 
      Fraction of MaxStepFrequency [0.0 to 1.0].
    
 * @param new_speed new speed value
 */
void
HumanoidMotionInterface::WalkVelocityMessage::set_speed(const float new_speed)
{
  data->speed = new_speed;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::WalkVelocityMessage::clone() const
{
  return new HumanoidMotionInterface::WalkVelocityMessage(this);
}
/** @class HumanoidMotionInterface::TurnMessage <interfaces/HumanoidMotionInterface.h>
 * TurnMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_angle initial value for angle
 */
HumanoidMotionInterface::TurnMessage::TurnMessage(const float ini_angle) : Message("TurnMessage")
{
  data_size = sizeof(TurnMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->angle = ini_angle;
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
}
/** Constructor */
HumanoidMotionInterface::TurnMessage::TurnMessage() : Message("TurnMessage")
{
  data_size = sizeof(TurnMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
}

/** Destructor */
HumanoidMotionInterface::TurnMessage::~TurnMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::TurnMessage::TurnMessage(const TurnMessage *m) : Message("TurnMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get angle value.
 * Angle in radians to turn.
 * @return angle value
 */
float
HumanoidMotionInterface::TurnMessage::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::TurnMessage::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Angle in radians to turn.
 * @param new_angle new angle value
 */
void
HumanoidMotionInterface::TurnMessage::set_angle(const float new_angle)
{
  data->angle = new_angle;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::TurnMessage::clone() const
{
  return new HumanoidMotionInterface::TurnMessage(this);
}
/** @class HumanoidMotionInterface::KickMessage <interfaces/HumanoidMotionInterface.h>
 * KickMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_leg initial value for leg
 * @param ini_strength initial value for strength
 */
HumanoidMotionInterface::KickMessage::KickMessage(const LegEnum ini_leg, const float ini_strength) : Message("KickMessage")
{
  data_size = sizeof(KickMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (KickMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->leg = ini_leg;
  data->strength = ini_strength;
  add_fieldinfo(IFT_ENUM, "leg", 1, &data->leg, "LegEnum");
  add_fieldinfo(IFT_FLOAT, "strength", 1, &data->strength);
}
/** Constructor */
HumanoidMotionInterface::KickMessage::KickMessage() : Message("KickMessage")
{
  data_size = sizeof(KickMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (KickMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_ENUM, "leg", 1, &data->leg, "LegEnum");
  add_fieldinfo(IFT_FLOAT, "strength", 1, &data->strength);
}

/** Destructor */
HumanoidMotionInterface::KickMessage::~KickMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::KickMessage::KickMessage(const KickMessage *m) : Message("KickMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (KickMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get leg value.
 * Leg to kick with
 * @return leg value
 */
HumanoidMotionInterface::LegEnum
HumanoidMotionInterface::KickMessage::leg() const
{
  return (HumanoidMotionInterface::LegEnum)data->leg;
}

/** Get maximum length of leg value.
 * @return length of leg value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::KickMessage::maxlenof_leg() const
{
  return 1;
}

/** Set leg value.
 * Leg to kick with
 * @param new_leg new leg value
 */
void
HumanoidMotionInterface::KickMessage::set_leg(const LegEnum new_leg)
{
  data->leg = new_leg;
}

/** Get strength value.
 * Kick strength
 * @return strength value
 */
float
HumanoidMotionInterface::KickMessage::strength() const
{
  return data->strength;
}

/** Get maximum length of strength value.
 * @return length of strength value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::KickMessage::maxlenof_strength() const
{
  return 1;
}

/** Set strength value.
 * Kick strength
 * @param new_strength new strength value
 */
void
HumanoidMotionInterface::KickMessage::set_strength(const float new_strength)
{
  data->strength = new_strength;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::KickMessage::clone() const
{
  return new HumanoidMotionInterface::KickMessage(this);
}
/** @class HumanoidMotionInterface::ParkMessage <interfaces/HumanoidMotionInterface.h>
 * ParkMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
HumanoidMotionInterface::ParkMessage::ParkMessage() : Message("ParkMessage")
{
  data_size = sizeof(ParkMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ParkMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::ParkMessage::~ParkMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::ParkMessage::ParkMessage(const ParkMessage *m) : Message("ParkMessage")
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
HumanoidMotionInterface::ParkMessage::clone() const
{
  return new HumanoidMotionInterface::ParkMessage(this);
}
/** @class HumanoidMotionInterface::GetUpMessage <interfaces/HumanoidMotionInterface.h>
 * GetUpMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
HumanoidMotionInterface::GetUpMessage::GetUpMessage() : Message("GetUpMessage")
{
  data_size = sizeof(GetUpMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GetUpMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::GetUpMessage::~GetUpMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::GetUpMessage::GetUpMessage(const GetUpMessage *m) : Message("GetUpMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GetUpMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::GetUpMessage::clone() const
{
  return new HumanoidMotionInterface::GetUpMessage(this);
}
/** @class HumanoidMotionInterface::StandupMessage <interfaces/HumanoidMotionInterface.h>
 * StandupMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_from_pos initial value for from_pos
 */
HumanoidMotionInterface::StandupMessage::StandupMessage(const StandupEnum ini_from_pos) : Message("StandupMessage")
{
  data_size = sizeof(StandupMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StandupMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->from_pos = ini_from_pos;
  add_fieldinfo(IFT_ENUM, "from_pos", 1, &data->from_pos, "StandupEnum");
}
/** Constructor */
HumanoidMotionInterface::StandupMessage::StandupMessage() : Message("StandupMessage")
{
  data_size = sizeof(StandupMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StandupMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_ENUM, "from_pos", 1, &data->from_pos, "StandupEnum");
}

/** Destructor */
HumanoidMotionInterface::StandupMessage::~StandupMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::StandupMessage::StandupMessage(const StandupMessage *m) : Message("StandupMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StandupMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get from_pos value.
 * Position from where to standup.
 * @return from_pos value
 */
HumanoidMotionInterface::StandupEnum
HumanoidMotionInterface::StandupMessage::from_pos() const
{
  return (HumanoidMotionInterface::StandupEnum)data->from_pos;
}

/** Get maximum length of from_pos value.
 * @return length of from_pos value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::StandupMessage::maxlenof_from_pos() const
{
  return 1;
}

/** Set from_pos value.
 * Position from where to standup.
 * @param new_from_pos new from_pos value
 */
void
HumanoidMotionInterface::StandupMessage::set_from_pos(const StandupEnum new_from_pos)
{
  data->from_pos = new_from_pos;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::StandupMessage::clone() const
{
  return new HumanoidMotionInterface::StandupMessage(this);
}
/** @class HumanoidMotionInterface::YawPitchHeadMessage <interfaces/HumanoidMotionInterface.h>
 * YawPitchHeadMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_yaw initial value for yaw
 * @param ini_pitch initial value for pitch
 * @param ini_time_sec initial value for time_sec
 */
HumanoidMotionInterface::YawPitchHeadMessage::YawPitchHeadMessage(const float ini_yaw, const float ini_pitch, const float ini_time_sec) : Message("YawPitchHeadMessage")
{
  data_size = sizeof(YawPitchHeadMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (YawPitchHeadMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->yaw = ini_yaw;
  data->pitch = ini_pitch;
  data->time_sec = ini_time_sec;
  add_fieldinfo(IFT_FLOAT, "yaw", 1, &data->yaw);
  add_fieldinfo(IFT_FLOAT, "pitch", 1, &data->pitch);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
}
/** Constructor */
HumanoidMotionInterface::YawPitchHeadMessage::YawPitchHeadMessage() : Message("YawPitchHeadMessage")
{
  data_size = sizeof(YawPitchHeadMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (YawPitchHeadMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "yaw", 1, &data->yaw);
  add_fieldinfo(IFT_FLOAT, "pitch", 1, &data->pitch);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
}

/** Destructor */
HumanoidMotionInterface::YawPitchHeadMessage::~YawPitchHeadMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::YawPitchHeadMessage::YawPitchHeadMessage(const YawPitchHeadMessage *m) : Message("YawPitchHeadMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (YawPitchHeadMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get yaw value.
 * Desired yaw (horizontal orientation).
 * @return yaw value
 */
float
HumanoidMotionInterface::YawPitchHeadMessage::yaw() const
{
  return data->yaw;
}

/** Get maximum length of yaw value.
 * @return length of yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::YawPitchHeadMessage::maxlenof_yaw() const
{
  return 1;
}

/** Set yaw value.
 * Desired yaw (horizontal orientation).
 * @param new_yaw new yaw value
 */
void
HumanoidMotionInterface::YawPitchHeadMessage::set_yaw(const float new_yaw)
{
  data->yaw = new_yaw;
}

/** Get pitch value.
 * Desired pitch (vertical orientation).
 * @return pitch value
 */
float
HumanoidMotionInterface::YawPitchHeadMessage::pitch() const
{
  return data->pitch;
}

/** Get maximum length of pitch value.
 * @return length of pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::YawPitchHeadMessage::maxlenof_pitch() const
{
  return 1;
}

/** Set pitch value.
 * Desired pitch (vertical orientation).
 * @param new_pitch new pitch value
 */
void
HumanoidMotionInterface::YawPitchHeadMessage::set_pitch(const float new_pitch)
{
  data->pitch = new_pitch;
}

/** Get time_sec value.
 * Time in seconds when to reach the target.
 * @return time_sec value
 */
float
HumanoidMotionInterface::YawPitchHeadMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::YawPitchHeadMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time in seconds when to reach the target.
 * @param new_time_sec new time_sec value
 */
void
HumanoidMotionInterface::YawPitchHeadMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::YawPitchHeadMessage::clone() const
{
  return new HumanoidMotionInterface::YawPitchHeadMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
HumanoidMotionInterface::message_valid(const Message *message) const
{
  const StopMessage *m0 = dynamic_cast<const StopMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const WalkStraightMessage *m1 = dynamic_cast<const WalkStraightMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const WalkSidewaysMessage *m2 = dynamic_cast<const WalkSidewaysMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const WalkArcMessage *m3 = dynamic_cast<const WalkArcMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const WalkVelocityMessage *m4 = dynamic_cast<const WalkVelocityMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const TurnMessage *m5 = dynamic_cast<const TurnMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const KickMessage *m6 = dynamic_cast<const KickMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const ParkMessage *m7 = dynamic_cast<const ParkMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const GetUpMessage *m8 = dynamic_cast<const GetUpMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  const StandupMessage *m9 = dynamic_cast<const StandupMessage *>(message);
  if ( m9 != NULL ) {
    return true;
  }
  const YawPitchHeadMessage *m10 = dynamic_cast<const YawPitchHeadMessage *>(message);
  if ( m10 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(HumanoidMotionInterface)
/// @endcond


} // end namespace fawkes
