
/***************************************************************************
 *  motor.cpp - Fawkes BlackBoard Interface - MotorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Martin Liebenberg, Tim Niemueller
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


/** MOTOR_ENABLED constant */
const unsigned int MotorInterface::MOTOR_ENABLED = 0;
/** MOTOR_DISABLED constant */
const unsigned int MotorInterface::MOTOR_DISABLED = 1;

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
/** Get MotorState value.
 * 
      The current state of the motor.
    
 * @return MotorState value
 */
unsigned int
MotorInterface::getMotorState()
{
  return data->MotorState;
}

/** Set MotorState value.
 * 
      The current state of the motor.
    
 * @param newMotorState new MotorState value
 */
void
MotorInterface::setMotorState(const unsigned int newMotorState)
{
  data->MotorState = newMotorState;
}

/** Get RightRPM value.
 * 
      RPM of the motor on the right front of the robot.
    
 * @return RightRPM value
 */
int
MotorInterface::getRightRPM()
{
  return data->RightRPM;
}

/** Set RightRPM value.
 * 
      RPM of the motor on the right front of the robot.
    
 * @param newRightRPM new RightRPM value
 */
void
MotorInterface::setRightRPM(const int newRightRPM)
{
  data->RightRPM = newRightRPM;
}

/** Get BackRPM value.
 * 
      RPM of motor on the back of the robot.
    
 * @return BackRPM value
 */
int
MotorInterface::getBackRPM()
{
  return data->BackRPM;
}

/** Set BackRPM value.
 * 
      RPM of motor on the back of the robot.
    
 * @param newBackRPM new BackRPM value
 */
void
MotorInterface::setBackRPM(const int newBackRPM)
{
  data->BackRPM = newBackRPM;
}

/** Get LeftRPM value.
 * 
      RPM of the motor on the left front of the robot.
    
 * @return LeftRPM value
 */
int
MotorInterface::getLeftRPM()
{
  return data->LeftRPM;
}

/** Set LeftRPM value.
 * 
      RPM of the motor on the left front of the robot.
    
 * @param newLeftRPM new LeftRPM value
 */
void
MotorInterface::setLeftRPM(const int newLeftRPM)
{
  data->LeftRPM = newLeftRPM;
}

/** Get ControllerThreadID value.
 * 
     The ID of the controlling thread.
     Only from this thread command messages are accepted.
    
 * @return ControllerThreadID value
 */
unsigned long int
MotorInterface::getControllerThreadID()
{
  return data->ControllerThreadID;
}

/** Set ControllerThreadID value.
 * 
     The ID of the controlling thread.
     Only from this thread command messages are accepted.
    
 * @param newControllerThreadID new ControllerThreadID value
 */
void
MotorInterface::setControllerThreadID(const unsigned long int newControllerThreadID)
{
  data->ControllerThreadID = newControllerThreadID;
}

/** Get ControllerThreadName value.
 * 
     The name of the controlling thread.
  
 * @return ControllerThreadName value
 */
char *
MotorInterface::getControllerThreadName()
{
  return data->ControllerThreadName;
}

/** Set ControllerThreadName value.
 * 
     The name of the controlling thread.
  
 * @param newControllerThreadName new ControllerThreadName value
 */
void
MotorInterface::setControllerThreadName(const char * newControllerThreadName)
{
  strncpy(data->ControllerThreadName, newControllerThreadName, sizeof(data->ControllerThreadName));
}

/* =========== messages =========== */
/** @class MotorInterface::SetMotorStateMessage interfaces/motor.h
 * SetMotorStateMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniMotorState initial value for MotorState
 */
MotorInterface::SetMotorStateMessage::SetMotorStateMessage(unsigned int iniMotorState) : Message()
{
  data_size = sizeof(SetMotorStateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMotorStateMessage_data_t *)data_ptr;
  data->MotorState = iniMotorState;
}
/** Constructor */
MotorInterface::SetMotorStateMessage::SetMotorStateMessage() : Message()
{
  data_size = sizeof(SetMotorStateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMotorStateMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::SetMotorStateMessage::~SetMotorStateMessage()
{
}
/* Methods */
/** Get MotorState value.
 * 
      The new motor state to set. Use the MOTOR_* constants.
    
 * @return MotorState value
 */
unsigned int
MotorInterface::SetMotorStateMessage::getMotorState()
{
  return data->MotorState;
}

/** Set MotorState value.
 * 
      The new motor state to set. Use the MOTOR_* constants.
    
 * @param newMotorState new MotorState value
 */
void
MotorInterface::SetMotorStateMessage::setMotorState(const unsigned int newMotorState)
{
  data->MotorState = newMotorState;
}

/** @class MotorInterface::AquireControlMessage interfaces/motor.h
 * AquireControlMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniThreadID initial value for ThreadID
 * @param iniThreadName initial value for ThreadName
 */
MotorInterface::AquireControlMessage::AquireControlMessage(unsigned long int iniThreadID, char * iniThreadName) : Message()
{
  data_size = sizeof(AquireControlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AquireControlMessage_data_t *)data_ptr;
  data->ThreadID = iniThreadID;
  strncpy(data->ThreadName, iniThreadName, 64);
}
/** Constructor */
MotorInterface::AquireControlMessage::AquireControlMessage() : Message()
{
  data_size = sizeof(AquireControlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AquireControlMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::AquireControlMessage::~AquireControlMessage()
{
}
/* Methods */
/** Get ThreadID value.
 * 
      The thread ID of the thread which is allowed to control the motors.
      Set to zero to use the data of the current thread (the message is zeroed at
      creation automatically, so if you do not set anything the sending thread
      aquires the control.
    
 * @return ThreadID value
 */
unsigned long int
MotorInterface::AquireControlMessage::getThreadID()
{
  return data->ThreadID;
}

/** Set ThreadID value.
 * 
      The thread ID of the thread which is allowed to control the motors.
      Set to zero to use the data of the current thread (the message is zeroed at
      creation automatically, so if you do not set anything the sending thread
      aquires the control.
    
 * @param newThreadID new ThreadID value
 */
void
MotorInterface::AquireControlMessage::setThreadID(const unsigned long int newThreadID)
{
  data->ThreadID = newThreadID;
}

/** Get ThreadName value.
 * 
      The thread name of the aquiring thread.
    
 * @return ThreadName value
 */
char *
MotorInterface::AquireControlMessage::getThreadName()
{
  return data->ThreadName;
}

/** Set ThreadName value.
 * 
      The thread name of the aquiring thread.
    
 * @param newThreadName new ThreadName value
 */
void
MotorInterface::AquireControlMessage::setThreadName(const char * newThreadName)
{
  strncpy(data->ThreadName, newThreadName, sizeof(data->ThreadName));
}

/** @class MotorInterface::TransRotRPMMessage interfaces/motor.h
 * TransRotRPMMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniForward initial value for Forward
 * @param iniSideward initial value for Sideward
 * @param iniRotation initial value for Rotation
 * @param iniSpeed initial value for Speed
 */
MotorInterface::TransRotRPMMessage::TransRotRPMMessage(float iniForward, float iniSideward, float iniRotation, float iniSpeed) : Message()
{
  data_size = sizeof(TransRotRPMMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransRotRPMMessage_data_t *)data_ptr;
  data->Forward = iniForward;
  data->Sideward = iniSideward;
  data->Rotation = iniRotation;
  data->Speed = iniSpeed;
}
/** Constructor */
MotorInterface::TransRotRPMMessage::TransRotRPMMessage() : Message()
{
  data_size = sizeof(TransRotRPMMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransRotRPMMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::TransRotRPMMessage::~TransRotRPMMessage()
{
}
/* Methods */
/** Get Forward value.
 * The forward command.
 * @return Forward value
 */
float
MotorInterface::TransRotRPMMessage::getForward()
{
  return data->Forward;
}

/** Set Forward value.
 * The forward command.
 * @param newForward new Forward value
 */
void
MotorInterface::TransRotRPMMessage::setForward(const float newForward)
{
  data->Forward = newForward;
}

/** Get Sideward value.
 * The sideward command.
 * @return Sideward value
 */
float
MotorInterface::TransRotRPMMessage::getSideward()
{
  return data->Sideward;
}

/** Set Sideward value.
 * The sideward command.
 * @param newSideward new Sideward value
 */
void
MotorInterface::TransRotRPMMessage::setSideward(const float newSideward)
{
  data->Sideward = newSideward;
}

/** Get Rotation value.
 * The rotation command.
 * @return Rotation value
 */
float
MotorInterface::TransRotRPMMessage::getRotation()
{
  return data->Rotation;
}

/** Set Rotation value.
 * The rotation command.
 * @param newRotation new Rotation value
 */
void
MotorInterface::TransRotRPMMessage::setRotation(const float newRotation)
{
  data->Rotation = newRotation;
}

/** Get Speed value.
 * The speed command.
 * @return Speed value
 */
float
MotorInterface::TransRotRPMMessage::getSpeed()
{
  return data->Speed;
}

/** Set Speed value.
 * The speed command.
 * @param newSpeed new Speed value
 */
void
MotorInterface::TransRotRPMMessage::setSpeed(const float newSpeed)
{
  data->Speed = newSpeed;
}

/** Check if message is valid an can be queued.
 * @param message Message to check
 */
bool
MotorInterface::messageValid(const Message *message) const
{
  const SetMotorStateMessage *m0 = dynamic_cast<const SetMotorStateMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const AquireControlMessage *m1 = dynamic_cast<const AquireControlMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const TransRotRPMMessage *m2 = dynamic_cast<const TransRotRPMMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(MotorInterface)
/// @endcond

