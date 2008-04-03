
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
    unsigned int motor_state; /**< 
      The current state of the motor.
     */
    unsigned int drive_mode; /**< 
      The current drive mode of the motor.
     */
    unsigned int controller; /**< 
     The ID of the controller. The controller ID is the instance serial of the sending
     interface. Only from this interface instance command messages are accepted.
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
    char controller_thread_name[64]; /**< 
     The name of the controlling thread, for easier debugging. This is informative only
     and actually two threads may share an interface instance (although this should be
     avoided since the interface locking has to be reproduced for these threads then).
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
    SetMotorStateMessage(const unsigned int ini_motor_state);
    SetMotorStateMessage();
    ~SetMotorStateMessage();

    SetMotorStateMessage(const SetMotorStateMessage *m);
    /* Methods */
    unsigned int motor_state();
    void set_motor_state(const unsigned int new_motor_state);
    size_t maxlenof_motor_state() const;
    virtual Message * clone() const;
  };

  class AcquireControlMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      unsigned int controller; /**< 
     The ID of the controller. The controller ID is the instance serial of the sending
     interface. Only from this interface instance command messages are accepted.
     */
      char controller_thread_name[64]; /**< 
     The name of the controlling thread, for easier debugging. This is informative only
     and actually two threads may share an interface instance (although this should be
     avoided since the interface locking has to be reproduced for these threads then).
   */
    } AcquireControlMessage_data_t;

    AcquireControlMessage_data_t *data;

   public:
    AcquireControlMessage(const unsigned int ini_controller, const char * ini_controller_thread_name);
    AcquireControlMessage();
    ~AcquireControlMessage();

    AcquireControlMessage(const AcquireControlMessage *m);
    /* Methods */
    unsigned int controller();
    void set_controller(const unsigned int new_controller);
    size_t maxlenof_controller() const;
    char * controller_thread_name();
    void set_controller_thread_name(const char * new_controller_thread_name);
    size_t maxlenof_controller_thread_name() const;
    virtual Message * clone() const;
  };

  class ResetOdometryMessage : public Message
  {
   public:
    ResetOdometryMessage();
    ~ResetOdometryMessage();

    ResetOdometryMessage(const ResetOdometryMessage *m);
    /* Methods */
    virtual Message * clone() const;
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
    DriveRPMMessage(const float ini_front_right, const float ini_front_left, const float ini_rear);
    DriveRPMMessage();
    ~DriveRPMMessage();

    DriveRPMMessage(const DriveRPMMessage *m);
    /* Methods */
    float front_right();
    void set_front_right(const float new_front_right);
    size_t maxlenof_front_right() const;
    float front_left();
    void set_front_left(const float new_front_left);
    size_t maxlenof_front_left() const;
    float rear();
    void set_rear(const float new_rear);
    size_t maxlenof_rear() const;
    virtual Message * clone() const;
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
    TransMessage(const float ini_vx, const float ini_vy);
    TransMessage();
    ~TransMessage();

    TransMessage(const TransMessage *m);
    /* Methods */
    float vx();
    void set_vx(const float new_vx);
    size_t maxlenof_vx() const;
    float vy();
    void set_vy(const float new_vy);
    size_t maxlenof_vy() const;
    virtual Message * clone() const;
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
    RotMessage(const float ini_omega);
    RotMessage();
    ~RotMessage();

    RotMessage(const RotMessage *m);
    /* Methods */
    float omega();
    void set_omega(const float new_omega);
    size_t maxlenof_omega() const;
    virtual Message * clone() const;
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
    TransRotMessage(const float ini_vx, const float ini_vy, const float ini_omega);
    TransRotMessage();
    ~TransRotMessage();

    TransRotMessage(const TransRotMessage *m);
    /* Methods */
    float vx();
    void set_vx(const float new_vx);
    size_t maxlenof_vx() const;
    float vy();
    void set_vy(const float new_vy);
    size_t maxlenof_vy() const;
    float omega();
    void set_omega(const float new_omega);
    size_t maxlenof_omega() const;
    virtual Message * clone() const;
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
    OrbitMessage(const float ini_px, const float ini_py, const float ini_omega);
    OrbitMessage();
    ~OrbitMessage();

    OrbitMessage(const OrbitMessage *m);
    /* Methods */
    float px();
    void set_px(const float new_px);
    size_t maxlenof_px() const;
    float py();
    void set_py(const float new_py);
    size_t maxlenof_py() const;
    float omega();
    void set_omega(const float new_omega);
    size_t maxlenof_omega() const;
    virtual Message * clone() const;
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
    LinTransRotMessage(const float ini_vx, const float ini_vy, const float ini_omega);
    LinTransRotMessage();
    ~LinTransRotMessage();

    LinTransRotMessage(const LinTransRotMessage *m);
    /* Methods */
    float vx();
    void set_vx(const float new_vx);
    size_t maxlenof_vx() const;
    float vy();
    void set_vy(const float new_vy);
    size_t maxlenof_vy() const;
    float omega();
    void set_omega(const float new_omega);
    size_t maxlenof_omega() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  MotorInterface();
  ~MotorInterface();

 public:
  virtual Message * create_message(const char *type) const;

  /* Methods */
  unsigned int motor_state();
  void set_motor_state(const unsigned int new_motor_state);
  size_t maxlenof_motor_state() const;
  unsigned int drive_mode();
  void set_drive_mode(const unsigned int new_drive_mode);
  size_t maxlenof_drive_mode() const;
  int right_rpm();
  void set_right_rpm(const int new_right_rpm);
  size_t maxlenof_right_rpm() const;
  int rear_rpm();
  void set_rear_rpm(const int new_rear_rpm);
  size_t maxlenof_rear_rpm() const;
  int left_rpm();
  void set_left_rpm(const int new_left_rpm);
  size_t maxlenof_left_rpm() const;
  float odometry_path_length();
  void set_odometry_path_length(const float new_odometry_path_length);
  size_t maxlenof_odometry_path_length() const;
  float odometry_position_x();
  void set_odometry_position_x(const float new_odometry_position_x);
  size_t maxlenof_odometry_position_x() const;
  float odometry_position_y();
  void set_odometry_position_y(const float new_odometry_position_y);
  size_t maxlenof_odometry_position_y() const;
  float odometry_orientation();
  void set_odometry_orientation(const float new_odometry_orientation);
  size_t maxlenof_odometry_orientation() const;
  float vx();
  void set_vx(const float new_vx);
  size_t maxlenof_vx() const;
  float vy();
  void set_vy(const float new_vy);
  size_t maxlenof_vy() const;
  float omega();
  void set_omega(const float new_omega);
  size_t maxlenof_omega() const;
  unsigned int controller();
  void set_controller(const unsigned int new_controller);
  size_t maxlenof_controller() const;
  char * controller_thread_name();
  void set_controller_thread_name(const char * new_controller_thread_name);
  size_t maxlenof_controller_thread_name() const;

};

#endif
