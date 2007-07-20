
/***************************************************************************
 *  motor.cpp - Fawkes BlackBoard Interface - MotorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

#include <interfaces/motor.h>

#include <string.h>
#include <stdlib.h>

/** @class MotorInterface interfaces/motor.h
 * MotorInterface Fawkes BlackBoard Interface.
 * This are the actual RPMs of the motors taken from the VMC.
 */



/** Constructor */
MotorInterface::MotorInterface() : Interface()
{
  data_size = sizeof(MotorInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (MotorInterface_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::~MotorInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get RPM1 value.
 * RPM of motor 1.
 * @return RPM1 value
 */
int
MotorInterface::getRPM1()
{
  return data->RPM1;
}

/** Set RPM1 value.
 * RPM of motor 1.
 * @param newRPM1 new RPM1 value
 */
void
MotorInterface::setRPM1(int newRPM1)
{
  data->RPM1 = newRPM1;
}

/** Get RPM2 value.
 * RPM of motor 2.
 * @return RPM2 value
 */
int
MotorInterface::getRPM2()
{
  return data->RPM2;
}

/** Set RPM2 value.
 * RPM of motor 2.
 * @param newRPM2 new RPM2 value
 */
void
MotorInterface::setRPM2(int newRPM2)
{
  data->RPM2 = newRPM2;
}

/** Get RPM3 value.
 * RPM of motor 3.
 * @return RPM3 value
 */
int
MotorInterface::getRPM3()
{
  return data->RPM3;
}

/** Set RPM3 value.
 * RPM of motor 3.
 * @param newRPM3 new RPM3 value
 */
void
MotorInterface::setRPM3(int newRPM3)
{
  data->RPM3 = newRPM3;
}

/** Get ControllerID value.
 * The ID of the Controller Thread, which controlls the motors.
 * @return ControllerID value
 */
unsigned long int
MotorInterface::getControllerID()
{
  return data->ControllerID;
}

/** Set ControllerID value.
 * The ID of the Controller Thread, which controlls the motors.
 * @param newControllerID new ControllerID value
 */
void
MotorInterface::setControllerID(unsigned long int newControllerID)
{
  data->ControllerID = newControllerID;
}

/* =========== messages =========== */
/** @class MotorInterface::JoystickMessage interfaces/motor.h
 * JoystickMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniCmdForward initial value for CmdForward
 * @param iniCmdSideward initial value for CmdSideward
 * @param iniCmdRotation initial value for CmdRotation
 * @param iniCmdSpeed initial value for CmdSpeed
 */
MotorInterface::JoystickMessage::JoystickMessage(float iniCmdForward, float iniCmdSideward, float iniCmdRotation, float iniCmdSpeed) : Message()
{
  data_size = sizeof(JoystickMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (JoystickMessage_data_t *)data_ptr;
  data->CmdForward = iniCmdForward;
  data->CmdSideward = iniCmdSideward;
  data->CmdRotation = iniCmdRotation;
  data->CmdSpeed = iniCmdSpeed;
}
/** Constructor */
MotorInterface::JoystickMessage::JoystickMessage() : Message()
{
  data_size = sizeof(JoystickMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (JoystickMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::JoystickMessage::~JoystickMessage()
{
}
/* Methods */
/** Get CmdForward value.
 * The forward command.
 * @return CmdForward value
 */
float
MotorInterface::JoystickMessage::getCmdForward()
{
  return data->CmdForward;
}

/** Set CmdForward value.
 * The forward command.
 * @param newCmdForward new CmdForward value
 */
void
MotorInterface::JoystickMessage::setCmdForward(float newCmdForward)
{
  data->CmdForward = newCmdForward;
}

/** Get CmdSideward value.
 * The sideward command.
 * @return CmdSideward value
 */
float
MotorInterface::JoystickMessage::getCmdSideward()
{
  return data->CmdSideward;
}

/** Set CmdSideward value.
 * The sideward command.
 * @param newCmdSideward new CmdSideward value
 */
void
MotorInterface::JoystickMessage::setCmdSideward(float newCmdSideward)
{
  data->CmdSideward = newCmdSideward;
}

/** Get CmdRotation value.
 * The rotation command.
 * @return CmdRotation value
 */
float
MotorInterface::JoystickMessage::getCmdRotation()
{
  return data->CmdRotation;
}

/** Set CmdRotation value.
 * The rotation command.
 * @param newCmdRotation new CmdRotation value
 */
void
MotorInterface::JoystickMessage::setCmdRotation(float newCmdRotation)
{
  data->CmdRotation = newCmdRotation;
}

/** Get CmdSpeed value.
 * The speed command.
 * @return CmdSpeed value
 */
float
MotorInterface::JoystickMessage::getCmdSpeed()
{
  return data->CmdSpeed;
}

/** Set CmdSpeed value.
 * The speed command.
 * @param newCmdSpeed new CmdSpeed value
 */
void
MotorInterface::JoystickMessage::setCmdSpeed(float newCmdSpeed)
{
  data->CmdSpeed = newCmdSpeed;
}

/** @class MotorInterface::NavigatorMessage interfaces/motor.h
 * NavigatorMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniCmdForward initial value for CmdForward
 * @param iniCmdSideward initial value for CmdSideward
 * @param iniCmdRotation initial value for CmdRotation
 * @param iniCmdVelocity initial value for CmdVelocity
 */
MotorInterface::NavigatorMessage::NavigatorMessage(float iniCmdForward, float iniCmdSideward, float iniCmdRotation, float iniCmdVelocity) : Message()
{
  data_size = sizeof(NavigatorMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (NavigatorMessage_data_t *)data_ptr;
  data->CmdForward = iniCmdForward;
  data->CmdSideward = iniCmdSideward;
  data->CmdRotation = iniCmdRotation;
  data->CmdVelocity = iniCmdVelocity;
}
/** Constructor */
MotorInterface::NavigatorMessage::NavigatorMessage() : Message()
{
  data_size = sizeof(NavigatorMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (NavigatorMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::NavigatorMessage::~NavigatorMessage()
{
}
/* Methods */
/** Get CmdForward value.
 * The forward command.
 * @return CmdForward value
 */
float
MotorInterface::NavigatorMessage::getCmdForward()
{
  return data->CmdForward;
}

/** Set CmdForward value.
 * The forward command.
 * @param newCmdForward new CmdForward value
 */
void
MotorInterface::NavigatorMessage::setCmdForward(float newCmdForward)
{
  data->CmdForward = newCmdForward;
}

/** Get CmdSideward value.
 * The sideward command.
 * @return CmdSideward value
 */
float
MotorInterface::NavigatorMessage::getCmdSideward()
{
  return data->CmdSideward;
}

/** Set CmdSideward value.
 * The sideward command.
 * @param newCmdSideward new CmdSideward value
 */
void
MotorInterface::NavigatorMessage::setCmdSideward(float newCmdSideward)
{
  data->CmdSideward = newCmdSideward;
}

/** Get CmdRotation value.
 * The rotation command.
 * @return CmdRotation value
 */
float
MotorInterface::NavigatorMessage::getCmdRotation()
{
  return data->CmdRotation;
}

/** Set CmdRotation value.
 * The rotation command.
 * @param newCmdRotation new CmdRotation value
 */
void
MotorInterface::NavigatorMessage::setCmdRotation(float newCmdRotation)
{
  data->CmdRotation = newCmdRotation;
}

/** Get CmdVelocity value.
 * The velocity command.
 * @return CmdVelocity value
 */
float
MotorInterface::NavigatorMessage::getCmdVelocity()
{
  return data->CmdVelocity;
}

/** Set CmdVelocity value.
 * The velocity command.
 * @param newCmdVelocity new CmdVelocity value
 */
void
MotorInterface::NavigatorMessage::setCmdVelocity(float newCmdVelocity)
{
  data->CmdVelocity = newCmdVelocity;
}

/** @class MotorInterface::SwitchMessage interfaces/motor.h
 * SwitchMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniThreadId initial value for ThreadId
 */
MotorInterface::SwitchMessage::SwitchMessage(unsigned int iniThreadId) : Message()
{
  data_size = sizeof(SwitchMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (SwitchMessage_data_t *)data_ptr;
  data->ThreadId = iniThreadId;
}
/** Constructor */
MotorInterface::SwitchMessage::SwitchMessage() : Message()
{
  data_size = sizeof(SwitchMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (SwitchMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::SwitchMessage::~SwitchMessage()
{
}
/* Methods */
/** Get ThreadId value.
 * The Thread Id of the Thread
    		which is allowed to control the motors.
 * @return ThreadId value
 */
unsigned int
MotorInterface::SwitchMessage::getThreadId()
{
  return data->ThreadId;
}

/** Set ThreadId value.
 * The Thread Id of the Thread
    		which is allowed to control the motors.
 * @param newThreadId new ThreadId value
 */
void
MotorInterface::SwitchMessage::setThreadId(unsigned int newThreadId)
{
  data->ThreadId = newThreadId;
}

/** @class MotorInterface::SubscribeMessage interfaces/motor.h
 * SubscribeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniSubscriber initial value for Subscriber
 */
MotorInterface::SubscribeMessage::SubscribeMessage(unsigned int iniSubscriber) : Message()
{
  data_size = sizeof(SubscribeMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (SubscribeMessage_data_t *)data_ptr;
  data->Subscriber = iniSubscriber;
}
/** Constructor */
MotorInterface::SubscribeMessage::SubscribeMessage() : Message()
{
  data_size = sizeof(SubscribeMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (SubscribeMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::SubscribeMessage::~SubscribeMessage()
{
}
/* Methods */
/** Get Subscriber value.
 * 0 if unsubscribe and 1 if subscribe.
 * @return Subscriber value
 */
unsigned int
MotorInterface::SubscribeMessage::getSubscriber()
{
  return data->Subscriber;
}

/** Set Subscriber value.
 * 0 if unsubscribe and 1 if subscribe.
 * @param newSubscriber new Subscriber value
 */
void
MotorInterface::SubscribeMessage::setSubscriber(unsigned int newSubscriber)
{
  data->Subscriber = newSubscriber;
}

/** Check if message is valid an can be queued.
 * @param message Message to check
 */
bool
MotorInterface::messageValid(const Message *message) const
{
  const JoystickMessage *m0 = dynamic_cast<const JoystickMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const NavigatorMessage *m1 = dynamic_cast<const NavigatorMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const SwitchMessage *m2 = dynamic_cast<const SwitchMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const SubscribeMessage *m3 = dynamic_cast<const SubscribeMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(MotorInterface)
/// @endcond

