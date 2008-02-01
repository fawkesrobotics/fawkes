
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
  static const unsigned int DRIVE_MODE_RPM;
  static const unsigned int DRIVE_MODE_TRANS;
  static const unsigned int DRIVE_MODE_ROT;
  static const unsigned int DRIVE_MODE_TRANS_ROT;
  static const unsigned int DRIVE_MODE_ORBIT;
  static const unsigned int DRIVE_MODE_LINE_TRANS_ROT;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int drive_mode; /**< 
      The current drive mode of the motor.
     */
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
    float odometry_path_length; /**< 
      The actual length of the robot's trajectory since the last ResetOdometry.
     */
    float odometry_position_x; /**< 
      The actual position of the robot relative to the position at the last ResetOdometry.
     */
    float odometry_position_y; /**< 
      The actual position of the robot relative to the position at the last ResetOdometry.
     */
    float odometry_orientation; /**< 
      The actual orientation of the robot relative to the orientation at the last ResetOdometry.
     */
    float vx; /**< 
      VX of the robot in m/s. Forward.
     */
    float vy; /**< 
      VY of the robot in m/s. Left.
     */
    float omega; /**< 
      Rotation speed of the robot in rad/s.
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

  class AcquireControlMessage : public Message
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
    } AcquireControlMessage_data_t;

    AcquireControlMessage_data_t *data;

   public:
    AcquireControlMessage(unsigned long int ini_thread_id, char * ini_thread_name);
    AcquireControlMessage();
    ~AcquireControlMessage();

    /* Methods */
    unsigned long int thread_id();
    void set_thread_id(const unsigned long int new_thread_id);
    char * thread_name();
    void set_thread_name(const char * new_thread_name);
  };

  class ResetOdometryMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
    } ResetOdometryMessage_data_t;

    ResetOdometryMessage_data_t *data;

   public:
    ResetOdometryMessage();
    ~ResetOdometryMessage();

    /* Methods */
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

  class LinTransRotMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float vx; /**< Speed for translation in X direction in m/s. */
      float vy; /**< Speed for translation in Y direction in m/s. */
      float omega; /**< Rotational speed in rad/s. */
    } LinTransRotMessage_data_t;

    LinTransRotMessage_data_t *data;

   public:
    LinTransRotMessage(float ini_vx, float ini_vy, float ini_omega);
    LinTransRotMessage();
    ~LinTransRotMessage();

    /* Methods */
    float vx();
    void set_vx(const float new_vx);
    float vy();
    void set_vy(const float new_vy);
    float omega();
    void set_omega(const float new_omega);
  };

  virtual bool message_valid(const Message *message) const;
 private:
  MotorInterface();
  ~MotorInterface();

 public:
  /* Methods */
  unsigned int motor_state();
  void set_motor_state(const unsigned int new_motor_state);
  unsigned int drive_mode();
  void set_drive_mode(const unsigned int new_drive_mode);
  int right_rpm();
  void set_right_rpm(const int new_right_rpm);
  int rear_rpm();
  void set_rear_rpm(const int new_rear_rpm);
  int left_rpm();
  void set_left_rpm(const int new_left_rpm);
  float odometry_path_length();
  void set_odometry_path_length(const float new_odometry_path_length);
  float odometry_position_x();
  void set_odometry_position_x(const float new_odometry_position_x);
  float odometry_position_y();
  void set_odometry_position_y(const float new_odometry_position_y);
  float odometry_orientation();
  void set_odometry_orientation(const float new_odometry_orientation);
  float vx();
  void set_vx(const float new_vx);
  float vy();
  void set_vy(const float new_vy);
  float omega();
  void set_omega(const float new_omega);
  unsigned long int controller_thread_id();
  void set_controller_thread_id(const unsigned long int new_controller_thread_id);
  char * controller_thread_name();
  void set_controller_thread_name(const char * new_controller_thread_name);

};

#endif
