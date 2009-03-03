
/***************************************************************************
 *  NavigatorInterface.cpp - Fawkes BlackBoard Interface - NavigatorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
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

#include <interfaces/NavigatorInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class NavigatorInterface <interfaces/NavigatorInterface.h>
 * NavigatorInterface Fawkes BlackBoard Interface.
 * 
      The navigator interface is used by the navigator to export information about
      the current status of the navigator and to define all messages by which the navigator
      can be instructed.

      There are three coordinate systems, the robot system which is a right-handed cartesian
      coordinate system with the robot in its origin, X axis pointing forward, Y pointing to
      the left and Z pointing upwards. The second coordinate system is the so-called
      navigator system. It is a coordinate system similar to the robot system, but the
      origin is defined on the initialization of the navigator. The last system is the
      odometry system. It is again a similar system, but the origin is reset from time
      to time and the robot's position in this system gives the odometry deltas.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
NavigatorInterface::NavigatorInterface() : Interface()
{
  data_size = sizeof(NavigatorInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (NavigatorInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(Interface::IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(Interface::IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(Interface::IFT_FLOAT, "dest_x", 1, &data->dest_x);
  add_fieldinfo(Interface::IFT_FLOAT, "dest_y", 1, &data->dest_y);
  add_fieldinfo(Interface::IFT_FLOAT, "dest_dist", 1, &data->dest_dist);
  add_fieldinfo(Interface::IFT_UINT, "msgid", 1, &data->msgid);
  add_fieldinfo(Interface::IFT_BOOL, "final", 1, &data->final);
  unsigned char tmp_hash[] = {0xdc, 0x5e, 0x7b, 0xa9, 0x38, 0x5c, 0x71, 0xe2, 0xb7, 0x5a, 0x82, 0x43, 0x5e, 0x50, 0x13, 0xe1};
  set_hash(tmp_hash);
}

/** Destructor */
NavigatorInterface::~NavigatorInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get x value.
 * Current X-coordinate in the navigator coordinate system.
 * @return x value
 */
float
NavigatorInterface::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * Current X-coordinate in the navigator coordinate system.
 * @param new_x new x value
 */
void
NavigatorInterface::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Current Y-coordinate in the navigator coordinate system.
 * @return y value
 */
float
NavigatorInterface::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Current Y-coordinate in the navigator coordinate system.
 * @param new_y new y value
 */
void
NavigatorInterface::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get dest_x value.
 * X-coordinate of the current destination, or 0.0 if no target has been set.
 * @return dest_x value
 */
float
NavigatorInterface::dest_x() const
{
  return data->dest_x;
}

/** Get maximum length of dest_x value.
 * @return length of dest_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_dest_x() const
{
  return 1;
}

/** Set dest_x value.
 * X-coordinate of the current destination, or 0.0 if no target has been set.
 * @param new_dest_x new dest_x value
 */
void
NavigatorInterface::set_dest_x(const float new_dest_x)
{
  data->dest_x = new_dest_x;
}

/** Get dest_y value.
 * Y-coordinate of the current destination, or 0.0 if no target has been set.
 * @return dest_y value
 */
float
NavigatorInterface::dest_y() const
{
  return data->dest_y;
}

/** Get maximum length of dest_y value.
 * @return length of dest_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_dest_y() const
{
  return 1;
}

/** Set dest_y value.
 * Y-coordinate of the current destination, or 0.0 if no target has been set.
 * @param new_dest_y new dest_y value
 */
void
NavigatorInterface::set_dest_y(const float new_dest_y)
{
  data->dest_y = new_dest_y;
}

/** Get dest_dist value.
 * Distance to destination in m.
 * @return dest_dist value
 */
float
NavigatorInterface::dest_dist() const
{
  return data->dest_dist;
}

/** Get maximum length of dest_dist value.
 * @return length of dest_dist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_dest_dist() const
{
  return 1;
}

/** Set dest_dist value.
 * Distance to destination in m.
 * @param new_dest_dist new dest_dist value
 */
void
NavigatorInterface::set_dest_dist(const float new_dest_dist)
{
  data->dest_dist = new_dest_dist;
}

/** Get msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @return msgid value
 */
unsigned int
NavigatorInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @param new_msgid new msgid value
 */
void
NavigatorInterface::set_msgid(const unsigned int new_msgid)
{
  data->msgid = new_msgid;
}

/** Get final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @return final value
 */
bool
NavigatorInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @param new_final new final value
 */
void
NavigatorInterface::set_final(const bool new_final)
{
  data->final = new_final;
}

/* =========== message create =========== */
Message *
NavigatorInterface::create_message(const char *type) const
{
  if ( strncmp("StopMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopMessage();
  } else if ( strncmp("TurnMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TurnMessage();
  } else if ( strncmp("CartesianGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CartesianGotoMessage();
  } else if ( strncmp("PolarGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new PolarGotoMessage();
  } else if ( strncmp("MaxVelocityMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new MaxVelocityMessage();
  } else if ( strncmp("ObstacleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ObstacleMessage();
  } else if ( strncmp("ResetOdometryMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ResetOdometryMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
NavigatorInterface::copy_values(const Interface *other)
{
  const NavigatorInterface *oi = dynamic_cast<const NavigatorInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(NavigatorInterface_data_t));
}

/* =========== messages =========== */
/** @class NavigatorInterface::StopMessage <interfaces/NavigatorInterface.h>
 * StopMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
NavigatorInterface::StopMessage::StopMessage() : Message("StopMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
NavigatorInterface::StopMessage::~StopMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::StopMessage::StopMessage(const StopMessage *m) : Message("StopMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::StopMessage::clone() const
{
  return new NavigatorInterface::StopMessage(this);
}
/** @class NavigatorInterface::TurnMessage <interfaces/NavigatorInterface.h>
 * TurnMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_angle initial value for angle
 * @param ini_velocity initial value for velocity
 */
NavigatorInterface::TurnMessage::TurnMessage(const float ini_angle, const float ini_velocity) : Message("TurnMessage")
{
  data_size = sizeof(TurnMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
  data->angle = ini_angle;
  data->velocity = ini_velocity;
}
/** Constructor */
NavigatorInterface::TurnMessage::TurnMessage() : Message("TurnMessage")
{
  data_size = sizeof(TurnMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
}

/** Destructor */
NavigatorInterface::TurnMessage::~TurnMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::TurnMessage::TurnMessage(const TurnMessage *m) : Message("TurnMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
}

/* Methods */
/** Get angle value.
 * Angle of the turn.
 * @return angle value
 */
float
NavigatorInterface::TurnMessage::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::TurnMessage::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Angle of the turn.
 * @param new_angle new angle value
 */
void
NavigatorInterface::TurnMessage::set_angle(const float new_angle)
{
  data->angle = new_angle;
}

/** Get velocity value.
 * The desired turning velocity in rad/s,
      set to zero to use default value.
 * @return velocity value
 */
float
NavigatorInterface::TurnMessage::velocity() const
{
  return data->velocity;
}

/** Get maximum length of velocity value.
 * @return length of velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::TurnMessage::maxlenof_velocity() const
{
  return 1;
}

/** Set velocity value.
 * The desired turning velocity in rad/s,
      set to zero to use default value.
 * @param new_velocity new velocity value
 */
void
NavigatorInterface::TurnMessage::set_velocity(const float new_velocity)
{
  data->velocity = new_velocity;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::TurnMessage::clone() const
{
  return new NavigatorInterface::TurnMessage(this);
}
/** @class NavigatorInterface::CartesianGotoMessage <interfaces/NavigatorInterface.h>
 * CartesianGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_orientation initial value for orientation
 */
NavigatorInterface::CartesianGotoMessage::CartesianGotoMessage(const float ini_x, const float ini_y, const float ini_orientation) : Message("CartesianGotoMessage")
{
  data_size = sizeof(CartesianGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->orientation = ini_orientation;
}
/** Constructor */
NavigatorInterface::CartesianGotoMessage::CartesianGotoMessage() : Message("CartesianGotoMessage")
{
  data_size = sizeof(CartesianGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
}

/** Destructor */
NavigatorInterface::CartesianGotoMessage::~CartesianGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::CartesianGotoMessage::CartesianGotoMessage(const CartesianGotoMessage *m) : Message("CartesianGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CartesianGotoMessage_data_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X-coordinate of the target, in the robot's coordinate system.
 * @return x value
 */
float
NavigatorInterface::CartesianGotoMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::CartesianGotoMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X-coordinate of the target, in the robot's coordinate system.
 * @param new_x new x value
 */
void
NavigatorInterface::CartesianGotoMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y-coordinate of the target, in the robot's coordinate system.
 * @return y value
 */
float
NavigatorInterface::CartesianGotoMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::CartesianGotoMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y-coordinate of the target, in the robot's coordinate system.
 * @param new_y new y value
 */
void
NavigatorInterface::CartesianGotoMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get orientation value.
 * The orientation of the robot at the target.
 * @return orientation value
 */
float
NavigatorInterface::CartesianGotoMessage::orientation() const
{
  return data->orientation;
}

/** Get maximum length of orientation value.
 * @return length of orientation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::CartesianGotoMessage::maxlenof_orientation() const
{
  return 1;
}

/** Set orientation value.
 * The orientation of the robot at the target.
 * @param new_orientation new orientation value
 */
void
NavigatorInterface::CartesianGotoMessage::set_orientation(const float new_orientation)
{
  data->orientation = new_orientation;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::CartesianGotoMessage::clone() const
{
  return new NavigatorInterface::CartesianGotoMessage(this);
}
/** @class NavigatorInterface::PolarGotoMessage <interfaces/NavigatorInterface.h>
 * PolarGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_phi initial value for phi
 * @param ini_dist initial value for dist
 * @param ini_orientation initial value for orientation
 */
NavigatorInterface::PolarGotoMessage::PolarGotoMessage(const float ini_phi, const float ini_dist, const float ini_orientation) : Message("PolarGotoMessage")
{
  data_size = sizeof(PolarGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PolarGotoMessage_data_t *)data_ptr;
  data->phi = ini_phi;
  data->dist = ini_dist;
  data->orientation = ini_orientation;
}
/** Constructor */
NavigatorInterface::PolarGotoMessage::PolarGotoMessage() : Message("PolarGotoMessage")
{
  data_size = sizeof(PolarGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PolarGotoMessage_data_t *)data_ptr;
}

/** Destructor */
NavigatorInterface::PolarGotoMessage::~PolarGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::PolarGotoMessage::PolarGotoMessage(const PolarGotoMessage *m) : Message("PolarGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (PolarGotoMessage_data_t *)data_ptr;
}

/* Methods */
/** Get phi value.
 * Angle between the robot's front and the target.
 * @return phi value
 */
float
NavigatorInterface::PolarGotoMessage::phi() const
{
  return data->phi;
}

/** Get maximum length of phi value.
 * @return length of phi value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::PolarGotoMessage::maxlenof_phi() const
{
  return 1;
}

/** Set phi value.
 * Angle between the robot's front and the target.
 * @param new_phi new phi value
 */
void
NavigatorInterface::PolarGotoMessage::set_phi(const float new_phi)
{
  data->phi = new_phi;
}

/** Get dist value.
 * Distance to the target.
 * @return dist value
 */
float
NavigatorInterface::PolarGotoMessage::dist() const
{
  return data->dist;
}

/** Get maximum length of dist value.
 * @return length of dist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::PolarGotoMessage::maxlenof_dist() const
{
  return 1;
}

/** Set dist value.
 * Distance to the target.
 * @param new_dist new dist value
 */
void
NavigatorInterface::PolarGotoMessage::set_dist(const float new_dist)
{
  data->dist = new_dist;
}

/** Get orientation value.
 * The orientation of the robot at the target.
 * @return orientation value
 */
float
NavigatorInterface::PolarGotoMessage::orientation() const
{
  return data->orientation;
}

/** Get maximum length of orientation value.
 * @return length of orientation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::PolarGotoMessage::maxlenof_orientation() const
{
  return 1;
}

/** Set orientation value.
 * The orientation of the robot at the target.
 * @param new_orientation new orientation value
 */
void
NavigatorInterface::PolarGotoMessage::set_orientation(const float new_orientation)
{
  data->orientation = new_orientation;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::PolarGotoMessage::clone() const
{
  return new NavigatorInterface::PolarGotoMessage(this);
}
/** @class NavigatorInterface::MaxVelocityMessage <interfaces/NavigatorInterface.h>
 * MaxVelocityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_velocity initial value for velocity
 */
NavigatorInterface::MaxVelocityMessage::MaxVelocityMessage(const float ini_velocity) : Message("MaxVelocityMessage")
{
  data_size = sizeof(MaxVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MaxVelocityMessage_data_t *)data_ptr;
  data->velocity = ini_velocity;
}
/** Constructor */
NavigatorInterface::MaxVelocityMessage::MaxVelocityMessage() : Message("MaxVelocityMessage")
{
  data_size = sizeof(MaxVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MaxVelocityMessage_data_t *)data_ptr;
}

/** Destructor */
NavigatorInterface::MaxVelocityMessage::~MaxVelocityMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::MaxVelocityMessage::MaxVelocityMessage(const MaxVelocityMessage *m) : Message("MaxVelocityMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (MaxVelocityMessage_data_t *)data_ptr;
}

/* Methods */
/** Get velocity value.
 * Maximum velocity of the robot.
 * @return velocity value
 */
float
NavigatorInterface::MaxVelocityMessage::velocity() const
{
  return data->velocity;
}

/** Get maximum length of velocity value.
 * @return length of velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::MaxVelocityMessage::maxlenof_velocity() const
{
  return 1;
}

/** Set velocity value.
 * Maximum velocity of the robot.
 * @param new_velocity new velocity value
 */
void
NavigatorInterface::MaxVelocityMessage::set_velocity(const float new_velocity)
{
  data->velocity = new_velocity;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::MaxVelocityMessage::clone() const
{
  return new NavigatorInterface::MaxVelocityMessage(this);
}
/** @class NavigatorInterface::ObstacleMessage <interfaces/NavigatorInterface.h>
 * ObstacleMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_width initial value for width
 */
NavigatorInterface::ObstacleMessage::ObstacleMessage(const float ini_x, const float ini_y, const float ini_width) : Message("ObstacleMessage")
{
  data_size = sizeof(ObstacleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ObstacleMessage_data_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->width = ini_width;
}
/** Constructor */
NavigatorInterface::ObstacleMessage::ObstacleMessage() : Message("ObstacleMessage")
{
  data_size = sizeof(ObstacleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ObstacleMessage_data_t *)data_ptr;
}

/** Destructor */
NavigatorInterface::ObstacleMessage::~ObstacleMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::ObstacleMessage::ObstacleMessage(const ObstacleMessage *m) : Message("ObstacleMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ObstacleMessage_data_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X-coordinate of the obstacle.
 * @return x value
 */
float
NavigatorInterface::ObstacleMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::ObstacleMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X-coordinate of the obstacle.
 * @param new_x new x value
 */
void
NavigatorInterface::ObstacleMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y-coordinate of the obstacle.
 * @return y value
 */
float
NavigatorInterface::ObstacleMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::ObstacleMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y-coordinate of the obstacle.
 * @param new_y new y value
 */
void
NavigatorInterface::ObstacleMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get width value.
 * Width of the obstacle.
 * @return width value
 */
float
NavigatorInterface::ObstacleMessage::width() const
{
  return data->width;
}

/** Get maximum length of width value.
 * @return length of width value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavigatorInterface::ObstacleMessage::maxlenof_width() const
{
  return 1;
}

/** Set width value.
 * Width of the obstacle.
 * @param new_width new width value
 */
void
NavigatorInterface::ObstacleMessage::set_width(const float new_width)
{
  data->width = new_width;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::ObstacleMessage::clone() const
{
  return new NavigatorInterface::ObstacleMessage(this);
}
/** @class NavigatorInterface::ResetOdometryMessage <interfaces/NavigatorInterface.h>
 * ResetOdometryMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
NavigatorInterface::ResetOdometryMessage::ResetOdometryMessage() : Message("ResetOdometryMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
NavigatorInterface::ResetOdometryMessage::~ResetOdometryMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
NavigatorInterface::ResetOdometryMessage::ResetOdometryMessage(const ResetOdometryMessage *m) : Message("ResetOdometryMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavigatorInterface::ResetOdometryMessage::clone() const
{
  return new NavigatorInterface::ResetOdometryMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
NavigatorInterface::message_valid(const Message *message) const
{
  const StopMessage *m0 = dynamic_cast<const StopMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const TurnMessage *m1 = dynamic_cast<const TurnMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const CartesianGotoMessage *m2 = dynamic_cast<const CartesianGotoMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const PolarGotoMessage *m3 = dynamic_cast<const PolarGotoMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const MaxVelocityMessage *m4 = dynamic_cast<const MaxVelocityMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const ObstacleMessage *m5 = dynamic_cast<const ObstacleMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const ResetOdometryMessage *m6 = dynamic_cast<const ResetOdometryMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NavigatorInterface)
/// @endcond


} // end namespace fawkes
