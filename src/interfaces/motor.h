
/***************************************************************************
 *  motor.h - Fawkes BlackBoard Interface - MotorInterface
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

#ifndef __INTERFACES_MOTOR_H_
#define __INTERFACES_MOTOR_H_

#include <interface/interface.h>
#include <interface/message.h>

class MotorInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(MotorInterface)
 /// @endcond
 public:
  /* constants */
  static const unsigned int MOTOR_ENABLED;
  static const unsigned int MOTOR_DISABLED;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int right_rpm; /**< 
      RPM of the motor on the right front of the robot.
     */
    int rear_rpm; /**< 
      RPM of motor on the rear of the robot.
     */
    int left_rpm; /**< 
      RPM of the motor on the left front of the robot.
     */
    unsigned long int controller_thread_id; /**< 
     The ID of the controlling thread.
     Only from this thread command messages are accepted.
     */
    unsigned int motor_state : 1; /**< 
      The current state of the motor.
     */
    char controller_thread_name[64]; /**< 
     The name of the controlling thread.
   */
  } MotorInterface_data_t;

  MotorInterface_data_t *data;

 public:
  /* messages */
  class SetMotorStateMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      unsigned int motor_state; /**< 
      The new motor state to set. Use the MOTOR_* constants.
     */
    } SetMotorStateMessage_data_t;

    SetMotorStateMessage_data_t *data;

   public:
    SetMotorStateMessage(unsigned int ini_motor_state);
    SetMotorStateMessage();
    ~SetMotorStateMessage();

    /* Methods */
    unsigned int motor_state();
    void set_motor_state(const unsigned int new_motor_state);
  };

  class AquireControlMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      unsigned long int thread_id; /**< 
      The thread ID of the thread which is allowed to control the motors.
      Set to zero to use the data of the current thread (the message is zeroed at
      creation automatically, so if you do not set anything the sending thread
      aquires the control.
     */
      char thread_name[64]; /**< 
      The thread name of the aquiring thread.
     */
    } AquireControlMessage_data_t;

    AquireControlMessage_data_t *data;

   public:
    AquireControlMessage(unsigned long int ini_thread_id, char * ini_thread_name);
    AquireControlMessage();
    ~AquireControlMessage();

    /* Methods */
    unsigned long int thread_id();
    void set_thread_id(const unsigned long int new_thread_id);
    char * thread_name();
    void set_thread_name(const char * new_thread_name);
  };

  class TransRotRPMMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float forward; /**< The forward command. */
      float sideward; /**< The sideward command. */
      float rotation; /**< The rotation command. */
      float speed; /**< The speed command. */
    } TransRotRPMMessage_data_t;

    TransRotRPMMessage_data_t *data;

   public:
    TransRotRPMMessage(float ini_forward, float ini_sideward, float ini_rotation, float ini_speed);
    TransRotRPMMessage();
    ~TransRotRPMMessage();

    /* Methods */
    float forward();
    void set_forward(const float new_forward);
    float sideward();
    void set_sideward(const float new_sideward);
    float rotation();
    void set_rotation(const float new_rotation);
    float speed();
    void set_speed(const float new_speed);
  };

  class DriveRPMMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float front_right; /**< Rotation in RPM of the right front wheel. */
      float front_left; /**< Rotation in RPM of the left front wheel. */
      float rear; /**< Rotation in RPM of the rear wheel. */
    } DriveRPMMessage_data_t;

    DriveRPMMessage_data_t *data;

   public:
    DriveRPMMessage(float ini_front_right, float ini_front_left, float ini_rear);
    DriveRPMMessage();
    ~DriveRPMMessage();

    /* Methods */
    float front_right();
    void set_front_right(const float new_front_right);
    float front_left();
    void set_front_left(const float new_front_left);
    float rear();
    void set_rear(const float new_rear);
  };

  class TransMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float vx; /**< Speed in X direction in m/s. */
      float vy; /**< Speed in Y direction in m/s. */
    } TransMessage_data_t;

    TransMessage_data_t *data;

   public:
    TransMessage(float ini_vx, float ini_vy);
    TransMessage();
    ~TransMessage();

    /* Methods */
    float vx();
    void set_vx(const float new_vx);
    float vy();
    void set_vy(const float new_vy);
  };

  class RotMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float omega; /**< Angle rotation in rad/s. */
    } RotMessage_data_t;

    RotMessage_data_t *data;

   public:
    RotMessage(float ini_omega);
    RotMessage();
    ~RotMessage();

    /* Methods */
    float omega();
    void set_omega(const float new_omega);
  };

  class TransRotMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float vx; /**< Speed in X direction in m/s. */
      float vy; /**< Speed in Y direction in m/s. */
      float omega; /**< Angle rotation in rad/s. */
    } TransRotMessage_data_t;

    TransRotMessage_data_t *data;

   public:
    TransRotMessage(float ini_vx, float ini_vy, float ini_omega);
    TransRotMessage();
    ~TransRotMessage();

    /* Methods */
    float vx();
    void set_vx(const float new_vx);
    float vy();
    void set_vy(const float new_vy);
    float omega();
    void set_omega(const float new_omega);
  };

  class OrbitMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float px; /**< Point's X coordinate to orbit. */
      float py; /**< Point's Y coordinate to orbit. */
      float omega; /**< Angular speed around point in rad/s. */
    } OrbitMessage_data_t;

    OrbitMessage_data_t *data;

   public:
    OrbitMessage(float ini_px, float ini_py, float ini_omega);
    OrbitMessage();
    ~OrbitMessage();

    /* Methods */
    float px();
    void set_px(const float new_px);
    float py();
    void set_py(const float new_py);
    float omega();
    void set_omega(const float new_omega);
  };

  virtual bool messageValid(const Message *message) const;
 private:
  MotorInterface();
  ~MotorInterface();

 public:
  /* Methods */
  unsigned int motor_state();
  void set_motor_state(const unsigned int new_motor_state);
  int right_rpm();
  void set_right_rpm(const int new_right_rpm);
  int rear_rpm();
  void set_rear_rpm(const int new_rear_rpm);
  int left_rpm();
  void set_left_rpm(const int new_left_rpm);
  unsigned long int controller_thread_id();
  void set_controller_thread_id(const unsigned long int new_controller_thread_id);
  char * controller_thread_name();
  void set_controller_thread_name(const char * new_controller_thread_name);

};

#endif
