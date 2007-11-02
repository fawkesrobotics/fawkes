
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
/** Get motor_state value.
 * 
      The current state of the motor.
    
 * @return motor_state value
 */
unsigned int
MotorInterface::motor_state()
{
  return data->motor_state;
}

/** Set motor_state value.
 * 
      The current state of the motor.
    
 * @param new_motor_state new motor_state value
 */
void
MotorInterface::set_motor_state(const unsigned int new_motor_state)
{
  data->motor_state = new_motor_state;
}

/** Get right_rpm value.
 * 
      RPM of the motor on the right front of the robot.
    
 * @return right_rpm value
 */
int
MotorInterface::right_rpm()
{
  return data->right_rpm;
}

/** Set right_rpm value.
 * 
      RPM of the motor on the right front of the robot.
    
 * @param new_right_rpm new right_rpm value
 */
void
MotorInterface::set_right_rpm(const int new_right_rpm)
{
  data->right_rpm = new_right_rpm;
}

/** Get rear_rpm value.
 * 
      RPM of motor on the rear of the robot.
    
 * @return rear_rpm value
 */
int
MotorInterface::rear_rpm()
{
  return data->rear_rpm;
}

/** Set rear_rpm value.
 * 
      RPM of motor on the rear of the robot.
    
 * @param new_rear_rpm new rear_rpm value
 */
void
MotorInterface::set_rear_rpm(const int new_rear_rpm)
{
  data->rear_rpm = new_rear_rpm;
}

/** Get left_rpm value.
 * 
      RPM of the motor on the left front of the robot.
    
 * @return left_rpm value
 */
int
MotorInterface::left_rpm()
{
  return data->left_rpm;
}

/** Set left_rpm value.
 * 
      RPM of the motor on the left front of the robot.
    
 * @param new_left_rpm new left_rpm value
 */
void
MotorInterface::set_left_rpm(const int new_left_rpm)
{
  data->left_rpm = new_left_rpm;
}

/** Get controller_thread_id value.
 * 
     The ID of the controlling thread.
     Only from this thread command messages are accepted.
    
 * @return controller_thread_id value
 */
unsigned long int
MotorInterface::controller_thread_id()
{
  return data->controller_thread_id;
}

/** Set controller_thread_id value.
 * 
     The ID of the controlling thread.
     Only from this thread command messages are accepted.
    
 * @param new_controller_thread_id new controller_thread_id value
 */
void
MotorInterface::set_controller_thread_id(const unsigned long int new_controller_thread_id)
{
  data->controller_thread_id = new_controller_thread_id;
}

/** Get controller_thread_name value.
 * 
     The name of the controlling thread.
  
 * @return controller_thread_name value
 */
char *
MotorInterface::controller_thread_name()
{
  return data->controller_thread_name;
}

/** Set controller_thread_name value.
 * 
     The name of the controlling thread.
  
 * @param new_controller_thread_name new controller_thread_name value
 */
void
MotorInterface::set_controller_thread_name(const char * new_controller_thread_name)
{
  strncpy(data->controller_thread_name, new_controller_thread_name, sizeof(data->controller_thread_name));
}

/* =========== messages =========== */
/** @class MotorInterface::SetMotorStateMessage interfaces/motor.h
 * SetMotorStateMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_motor_state initial value for motor_state
 */
MotorInterface::SetMotorStateMessage::SetMotorStateMessage(unsigned int ini_motor_state) : Message()
{
  data_size = sizeof(SetMotorStateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMotorStateMessage_data_t *)data_ptr;
  data->motor_state = ini_motor_state;
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
/** Get motor_state value.
 * 
      The new motor state to set. Use the MOTOR_* constants.
    
 * @return motor_state value
 */
unsigned int
MotorInterface::SetMotorStateMessage::motor_state()
{
  return data->motor_state;
}

/** Set motor_state value.
 * 
      The new motor state to set. Use the MOTOR_* constants.
    
 * @param new_motor_state new motor_state value
 */
void
MotorInterface::SetMotorStateMessage::set_motor_state(const unsigned int new_motor_state)
{
  data->motor_state = new_motor_state;
}

/** @class MotorInterface::AquireControlMessage interfaces/motor.h
 * AquireControlMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_thread_id initial value for thread_id
 * @param ini_thread_name initial value for thread_name
 */
MotorInterface::AquireControlMessage::AquireControlMessage(unsigned long int ini_thread_id, char * ini_thread_name) : Message()
{
  data_size = sizeof(AquireControlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AquireControlMessage_data_t *)data_ptr;
  data->thread_id = ini_thread_id;
  strncpy(data->thread_name, ini_thread_name, 64);
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
/** Get thread_id value.
 * 
      The thread ID of the thread which is allowed to control the motors.
      Set to zero to use the data of the current thread (the message is zeroed at
      creation automatically, so if you do not set anything the sending thread
      aquires the control.
    
 * @return thread_id value
 */
unsigned long int
MotorInterface::AquireControlMessage::thread_id()
{
  return data->thread_id;
}

/** Set thread_id value.
 * 
      The thread ID of the thread which is allowed to control the motors.
      Set to zero to use the data of the current thread (the message is zeroed at
      creation automatically, so if you do not set anything the sending thread
      aquires the control.
    
 * @param new_thread_id new thread_id value
 */
void
MotorInterface::AquireControlMessage::set_thread_id(const unsigned long int new_thread_id)
{
  data->thread_id = new_thread_id;
}

/** Get thread_name value.
 * 
      The thread name of the aquiring thread.
    
 * @return thread_name value
 */
char *
MotorInterface::AquireControlMessage::thread_name()
{
  return data->thread_name;
}

/** Set thread_name value.
 * 
      The thread name of the aquiring thread.
    
 * @param new_thread_name new thread_name value
 */
void
MotorInterface::AquireControlMessage::set_thread_name(const char * new_thread_name)
{
  strncpy(data->thread_name, new_thread_name, sizeof(data->thread_name));
}

/** @class MotorInterface::TransRotRPMMessage interfaces/motor.h
 * TransRotRPMMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_forward initial value for forward
 * @param ini_sideward initial value for sideward
 * @param ini_rotation initial value for rotation
 * @param ini_speed initial value for speed
 */
MotorInterface::TransRotRPMMessage::TransRotRPMMessage(float ini_forward, float ini_sideward, float ini_rotation, float ini_speed) : Message()
{
  data_size = sizeof(TransRotRPMMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransRotRPMMessage_data_t *)data_ptr;
  data->forward = ini_forward;
  data->sideward = ini_sideward;
  data->rotation = ini_rotation;
  data->speed = ini_speed;
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
/** Get forward value.
 * The forward command.
 * @return forward value
 */
float
MotorInterface::TransRotRPMMessage::forward()
{
  return data->forward;
}

/** Set forward value.
 * The forward command.
 * @param new_forward new forward value
 */
void
MotorInterface::TransRotRPMMessage::set_forward(const float new_forward)
{
  data->forward = new_forward;
}

/** Get sideward value.
 * The sideward command.
 * @return sideward value
 */
float
MotorInterface::TransRotRPMMessage::sideward()
{
  return data->sideward;
}

/** Set sideward value.
 * The sideward command.
 * @param new_sideward new sideward value
 */
void
MotorInterface::TransRotRPMMessage::set_sideward(const float new_sideward)
{
  data->sideward = new_sideward;
}

/** Get rotation value.
 * The rotation command.
 * @return rotation value
 */
float
MotorInterface::TransRotRPMMessage::rotation()
{
  return data->rotation;
}

/** Set rotation value.
 * The rotation command.
 * @param new_rotation new rotation value
 */
void
MotorInterface::TransRotRPMMessage::set_rotation(const float new_rotation)
{
  data->rotation = new_rotation;
}

/** Get speed value.
 * The speed command.
 * @return speed value
 */
float
MotorInterface::TransRotRPMMessage::speed()
{
  return data->speed;
}

/** Set speed value.
 * The speed command.
 * @param new_speed new speed value
 */
void
MotorInterface::TransRotRPMMessage::set_speed(const float new_speed)
{
  data->speed = new_speed;
}

/** @class MotorInterface::DriveRPMMessage interfaces/motor.h
 * DriveRPMMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_front_right initial value for front_right
 * @param ini_front_left initial value for front_left
 * @param ini_rear initial value for rear
 */
MotorInterface::DriveRPMMessage::DriveRPMMessage(float ini_front_right, float ini_front_left, float ini_rear) : Message()
{
  data_size = sizeof(DriveRPMMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DriveRPMMessage_data_t *)data_ptr;
  data->front_right = ini_front_right;
  data->front_left = ini_front_left;
  data->rear = ini_rear;
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
/** Get front_right value.
 * Rotation in RPM of the right front wheel.
 * @return front_right value
 */
float
MotorInterface::DriveRPMMessage::front_right()
{
  return data->front_right;
}

/** Set front_right value.
 * Rotation in RPM of the right front wheel.
 * @param new_front_right new front_right value
 */
void
MotorInterface::DriveRPMMessage::set_front_right(const float new_front_right)
{
  data->front_right = new_front_right;
}

/** Get front_left value.
 * Rotation in RPM of the left front wheel.
 * @return front_left value
 */
float
MotorInterface::DriveRPMMessage::front_left()
{
  return data->front_left;
}

/** Set front_left value.
 * Rotation in RPM of the left front wheel.
 * @param new_front_left new front_left value
 */
void
MotorInterface::DriveRPMMessage::set_front_left(const float new_front_left)
{
  data->front_left = new_front_left;
}

/** Get rear value.
 * Rotation in RPM of the rear wheel.
 * @return rear value
 */
float
MotorInterface::DriveRPMMessage::rear()
{
  return data->rear;
}

/** Set rear value.
 * Rotation in RPM of the rear wheel.
 * @param new_rear new rear value
 */
void
MotorInterface::DriveRPMMessage::set_rear(const float new_rear)
{
  data->rear = new_rear;
}

/** @class MotorInterface::TransMessage interfaces/motor.h
 * TransMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_vx initial value for vx
 * @param ini_vy initial value for vy
 */
MotorInterface::TransMessage::TransMessage(float ini_vx, float ini_vy) : Message()
{
  data_size = sizeof(TransMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransMessage_data_t *)data_ptr;
  data->vx = ini_vx;
  data->vy = ini_vy;
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
/** Get vx value.
 * Speed in X direction in m/s.
 * @return vx value
 */
float
MotorInterface::TransMessage::vx()
{
  return data->vx;
}

/** Set vx value.
 * Speed in X direction in m/s.
 * @param new_vx new vx value
 */
void
MotorInterface::TransMessage::set_vx(const float new_vx)
{
  data->vx = new_vx;
}

/** Get vy value.
 * Speed in Y direction in m/s.
 * @return vy value
 */
float
MotorInterface::TransMessage::vy()
{
  return data->vy;
}

/** Set vy value.
 * Speed in Y direction in m/s.
 * @param new_vy new vy value
 */
void
MotorInterface::TransMessage::set_vy(const float new_vy)
{
  data->vy = new_vy;
}

/** @class MotorInterface::RotMessage interfaces/motor.h
 * RotMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_omega initial value for omega
 */
MotorInterface::RotMessage::RotMessage(float ini_omega) : Message()
{
  data_size = sizeof(RotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RotMessage_data_t *)data_ptr;
  data->omega = ini_omega;
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
/** Get omega value.
 * Angle rotation in rad/s.
 * @return omega value
 */
float
MotorInterface::RotMessage::omega()
{
  return data->omega;
}

/** Set omega value.
 * Angle rotation in rad/s.
 * @param new_omega new omega value
 */
void
MotorInterface::RotMessage::set_omega(const float new_omega)
{
  data->omega = new_omega;
}

/** @class MotorInterface::TransRotMessage interfaces/motor.h
 * TransRotMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_vx initial value for vx
 * @param ini_vy initial value for vy
 * @param ini_omega initial value for omega
 */
MotorInterface::TransRotMessage::TransRotMessage(float ini_vx, float ini_vy, float ini_omega) : Message()
{
  data_size = sizeof(TransRotMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TransRotMessage_data_t *)data_ptr;
  data->vx = ini_vx;
  data->vy = ini_vy;
  data->omega = ini_omega;
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
/** Get vx value.
 * Speed in X direction in m/s.
 * @return vx value
 */
float
MotorInterface::TransRotMessage::vx()
{
  return data->vx;
}

/** Set vx value.
 * Speed in X direction in m/s.
 * @param new_vx new vx value
 */
void
MotorInterface::TransRotMessage::set_vx(const float new_vx)
{
  data->vx = new_vx;
}

/** Get vy value.
 * Speed in Y direction in m/s.
 * @return vy value
 */
float
MotorInterface::TransRotMessage::vy()
{
  return data->vy;
}

/** Set vy value.
 * Speed in Y direction in m/s.
 * @param new_vy new vy value
 */
void
MotorInterface::TransRotMessage::set_vy(const float new_vy)
{
  data->vy = new_vy;
}

/** Get omega value.
 * Angle rotation in rad/s.
 * @return omega value
 */
float
MotorInterface::TransRotMessage::omega()
{
  return data->omega;
}

/** Set omega value.
 * Angle rotation in rad/s.
 * @param new_omega new omega value
 */
void
MotorInterface::TransRotMessage::set_omega(const float new_omega)
{
  data->omega = new_omega;
}

/** @class MotorInterface::OrbitMessage interfaces/motor.h
 * OrbitMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_px initial value for px
 * @param ini_py initial value for py
 * @param ini_omega initial value for omega
 */
MotorInterface::OrbitMessage::OrbitMessage(float ini_px, float ini_py, float ini_omega) : Message()
{
  data_size = sizeof(OrbitMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (OrbitMessage_data_t *)data_ptr;
  data->px = ini_px;
  data->py = ini_py;
  data->omega = ini_omega;
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
/** Get px value.
 * Point's X coordinate to orbit.
 * @return px value
 */
float
MotorInterface::OrbitMessage::px()
{
  return data->px;
}

/** Set px value.
 * Point's X coordinate to orbit.
 * @param new_px new px value
 */
void
MotorInterface::OrbitMessage::set_px(const float new_px)
{
  data->px = new_px;
}

/** Get py value.
 * Point's Y coordinate to orbit.
 * @return py value
 */
float
MotorInterface::OrbitMessage::py()
{
  return data->py;
}

/** Set py value.
 * Point's Y coordinate to orbit.
 * @param new_py new py value
 */
void
MotorInterface::OrbitMessage::set_py(const float new_py)
{
  data->py = new_py;
}

/** Get omega value.
 * Angular speed around point in rad/s.
 * @return omega value
 */
float
MotorInterface::OrbitMessage::omega()
{
  return data->omega;
}

/** Set omega value.
 * Angular speed around point in rad/s.
 * @param new_omega new omega value
 */
void
MotorInterface::OrbitMessage::set_omega(const float new_omega)
{
  data->omega = new_omega;
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

