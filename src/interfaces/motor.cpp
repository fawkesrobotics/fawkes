
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

/** Get RearRPM value.
 * 
      RPM of motor on the rear of the robot.
    
 * @return RearRPM value
 */
int
MotorInterface::getRearRPM()
{
  return data->RearRPM;
}

/** Set RearRPM value.
 * 
      RPM of motor on the rear of the robot.
    
 * @param newRearRPM new RearRPM value
 */
void
MotorInterface::setRearRPM(const int newRearRPM)
{
  data->RearRPM = newRearRPM;
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

/** @class MotorInterface::DriveRPMMessage interfaces/motor.h
 * DriveRPMMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniFrontRight initial value for FrontRight
 * @param iniFrontLeft initial value for FrontLeft
 * @param iniRear initial value for Rear
 */
MotorInterface::DriveRPMMessage::DriveRPMMessage(float iniFrontRight, float iniFrontLeft, float iniRear) : Message()
{
  data_size = sizeof(DriveRPMMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DriveRPMMessage_data_t *)data_ptr;
  data->FrontRight = iniFrontRight;
  data->FrontLeft = iniFrontLeft;
  data->Rear = iniRear;
}
/** Constructor */
MotorInterface::DriveRPMMessage::DriveRPMMessage() : Message()
{
  data_size = sizeof(DriveRPMMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DriveRPMMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::DriveRPMMessage::~DriveRPMMessage()
{
}
/* Methods */
/** Get FrontRight value.
 * Rotation in RPM of the right front wheel.
 * @return FrontRight value
 */
float
MotorInterface::DriveRPMMessage::getFrontRight()
{
  return data->FrontRight;
}

/** Set FrontRight value.
 * Rotation in RPM of the right front wheel.
 * @param newFrontRight new FrontRight value
 */
void
MotorInterface::DriveRPMMessage::setFrontRight(const float newFrontRight)
{
  data->FrontRight = newFrontRight;
}

/** Get FrontLeft value.
 * Rotation in RPM of the left front wheel.
 * @return FrontLeft value
 */
float
MotorInterface::DriveRPMMessage::getFrontLeft()
{
  return data->FrontLeft;
}

/** Set FrontLeft value.
 * Rotation in RPM of the left front wheel.
 * @param newFrontLeft new FrontLeft value
 */
void
MotorInterface::DriveRPMMessage::setFrontLeft(const float newFrontLeft)
{
  data->FrontLeft = newFrontLeft;
}

/** Get Rear value.
 * Rotation in RPM of the rear wheel.
 * @return Rear value
 */
float
MotorInterface::DriveRPMMessage::getRear()
{
  return data->Rear;
}

/** Set Rear value.
 * Rotation in RPM of the rear wheel.
 * @param newRear new Rear value
 */
void
MotorInterface::DriveRPMMessage::setRear(const float newRear)
{
  data->Rear = newRear;
}

/** @class MotorInterface::TransMessage interfaces/motor.h
 * TransMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniVX initial value for VX
 * @param iniVY initial value for VY
 */
MotorInterface::TransMessage::TransMessage(float iniVX, float iniVY) : Message()
{
  data_size = sizeof(TransMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransMessage_data_t *)data_ptr;
  data->VX = iniVX;
  data->VY = iniVY;
}
/** Constructor */
MotorInterface::TransMessage::TransMessage() : Message()
{
  data_size = sizeof(TransMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::TransMessage::~TransMessage()
{
}
/* Methods */
/** Get VX value.
 * Speed in X direction in m/s.
 * @return VX value
 */
float
MotorInterface::TransMessage::getVX()
{
  return data->VX;
}

/** Set VX value.
 * Speed in X direction in m/s.
 * @param newVX new VX value
 */
void
MotorInterface::TransMessage::setVX(const float newVX)
{
  data->VX = newVX;
}

/** Get VY value.
 * Speed in Y direction in m/s.
 * @return VY value
 */
float
MotorInterface::TransMessage::getVY()
{
  return data->VY;
}

/** Set VY value.
 * Speed in Y direction in m/s.
 * @param newVY new VY value
 */
void
MotorInterface::TransMessage::setVY(const float newVY)
{
  data->VY = newVY;
}

/** @class MotorInterface::RotMessage interfaces/motor.h
 * RotMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniOmega initial value for Omega
 */
MotorInterface::RotMessage::RotMessage(float iniOmega) : Message()
{
  data_size = sizeof(RotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RotMessage_data_t *)data_ptr;
  data->Omega = iniOmega;
}
/** Constructor */
MotorInterface::RotMessage::RotMessage() : Message()
{
  data_size = sizeof(RotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RotMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::RotMessage::~RotMessage()
{
}
/* Methods */
/** Get Omega value.
 * Angle rotation in rad/s.
 * @return Omega value
 */
float
MotorInterface::RotMessage::getOmega()
{
  return data->Omega;
}

/** Set Omega value.
 * Angle rotation in rad/s.
 * @param newOmega new Omega value
 */
void
MotorInterface::RotMessage::setOmega(const float newOmega)
{
  data->Omega = newOmega;
}

/** @class MotorInterface::TransRotMessage interfaces/motor.h
 * TransRotMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniVX initial value for VX
 * @param iniVY initial value for VY
 * @param iniOmega initial value for Omega
 */
MotorInterface::TransRotMessage::TransRotMessage(float iniVX, float iniVY, float iniOmega) : Message()
{
  data_size = sizeof(TransRotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransRotMessage_data_t *)data_ptr;
  data->VX = iniVX;
  data->VY = iniVY;
  data->Omega = iniOmega;
}
/** Constructor */
MotorInterface::TransRotMessage::TransRotMessage() : Message()
{
  data_size = sizeof(TransRotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransRotMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::TransRotMessage::~TransRotMessage()
{
}
/* Methods */
/** Get VX value.
 * Speed in X direction in m/s.
 * @return VX value
 */
float
MotorInterface::TransRotMessage::getVX()
{
  return data->VX;
}

/** Set VX value.
 * Speed in X direction in m/s.
 * @param newVX new VX value
 */
void
MotorInterface::TransRotMessage::setVX(const float newVX)
{
  data->VX = newVX;
}

/** Get VY value.
 * Speed in Y direction in m/s.
 * @return VY value
 */
float
MotorInterface::TransRotMessage::getVY()
{
  return data->VY;
}

/** Set VY value.
 * Speed in Y direction in m/s.
 * @param newVY new VY value
 */
void
MotorInterface::TransRotMessage::setVY(const float newVY)
{
  data->VY = newVY;
}

/** Get Omega value.
 * Angle rotation in rad/s.
 * @return Omega value
 */
float
MotorInterface::TransRotMessage::getOmega()
{
  return data->Omega;
}

/** Set Omega value.
 * Angle rotation in rad/s.
 * @param newOmega new Omega value
 */
void
MotorInterface::TransRotMessage::setOmega(const float newOmega)
{
  data->Omega = newOmega;
}

/** @class MotorInterface::OrbitMessage interfaces/motor.h
 * OrbitMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniPX initial value for PX
 * @param iniPY initial value for PY
 * @param iniOmega initial value for Omega
 */
MotorInterface::OrbitMessage::OrbitMessage(float iniPX, float iniPY, float iniOmega) : Message()
{
  data_size = sizeof(OrbitMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (OrbitMessage_data_t *)data_ptr;
  data->PX = iniPX;
  data->PY = iniPY;
  data->Omega = iniOmega;
}
/** Constructor */
MotorInterface::OrbitMessage::OrbitMessage() : Message()
{
  data_size = sizeof(OrbitMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (OrbitMessage_data_t *)data_ptr;
}
/** Destructor */
MotorInterface::OrbitMessage::~OrbitMessage()
{
}
/* Methods */
/** Get PX value.
 * Point's X coordinate to orbit.
 * @return PX value
 */
float
MotorInterface::OrbitMessage::getPX()
{
  return data->PX;
}

/** Set PX value.
 * Point's X coordinate to orbit.
 * @param newPX new PX value
 */
void
MotorInterface::OrbitMessage::setPX(const float newPX)
{
  data->PX = newPX;
}

/** Get PY value.
 * Point's Y coordinate to orbit.
 * @return PY value
 */
float
MotorInterface::OrbitMessage::getPY()
{
  return data->PY;
}

/** Set PY value.
 * Point's Y coordinate to orbit.
 * @param newPY new PY value
 */
void
MotorInterface::OrbitMessage::setPY(const float newPY)
{
  data->PY = newPY;
}

/** Get Omega value.
 * Angular speed around point in rad/s.
 * @return Omega value
 */
float
MotorInterface::OrbitMessage::getOmega()
{
  return data->Omega;
}

/** Set Omega value.
 * Angular speed around point in rad/s.
 * @param newOmega new Omega value
 */
void
MotorInterface::OrbitMessage::setOmega(const float newOmega)
{
  data->Omega = newOmega;
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
  const DriveRPMMessage *m3 = dynamic_cast<const DriveRPMMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const TransMessage *m4 = dynamic_cast<const TransMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const RotMessage *m5 = dynamic_cast<const RotMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const TransRotMessage *m6 = dynamic_cast<const TransRotMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const OrbitMessage *m7 = dynamic_cast<const OrbitMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(MotorInterface)
/// @endcond

