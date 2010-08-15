
/***************************************************************************
 *  MotorInterface.h - Fawkes BlackBoard Interface - MotorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Martin Liebenberg, Tim Niemueller
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

#ifndef __INTERFACES_MOTORINTERFACE_H_
#define __INTERFACES_MOTORINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class MotorInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(MotorInterface)
 /// @endcond
 public:
  /* constants */
  static const uint32_t MOTOR_ENABLED;
  static const uint32_t MOTOR_DISABLED;
  static const uint32_t DRIVE_MODE_RPM;
  static const uint32_t DRIVE_MODE_TRANS;
  static const uint32_t DRIVE_MODE_ROT;
  static const uint32_t DRIVE_MODE_TRANS_ROT;
  static const uint32_t DRIVE_MODE_ORBIT;
  static const uint32_t DRIVE_MODE_LINE_TRANS_ROT;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint32_t motor_state; /**< 
      The current state of the motor.
     */
    uint32_t drive_mode; /**< 
      The current drive mode of the motor.
     */
    int32_t right_rpm; /**< 
      RPM of the motor on the right front of the robot.
     */
    int32_t rear_rpm; /**< 
      RPM of motor on the rear of the robot.
     */
    int32_t left_rpm; /**< 
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
    uint32_t controller; /**< 
     The ID of the controller. The controller ID is the instance serial of the sending
     interface. Only from this interface instance command messages are accepted.
     */
    char controller_thread_name[64]; /**< 
     The name of the controlling thread, for easier debugging. This is informative only
     and actually two threads may share an interface instance (although this should be
     avoided since the interface locking has to be reproduced for these threads then).
   */
  } MotorInterface_data_t;
#pragma pack(pop)

  MotorInterface_data_t *data;

 public:
  /* messages */
  class SetMotorStateMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint32_t motor_state; /**< 
      The new motor state to set. Use the MOTOR_* constants.
     */
    } SetMotorStateMessage_data_t;
#pragma pack(pop)

    SetMotorStateMessage_data_t *data;

   public:
    SetMotorStateMessage(const uint32_t ini_motor_state);
    SetMotorStateMessage();
    ~SetMotorStateMessage();

    SetMotorStateMessage(const SetMotorStateMessage *m);
    /* Methods */
    uint32_t motor_state() const;
    void set_motor_state(const uint32_t new_motor_state);
    size_t maxlenof_motor_state() const;
    virtual Message * clone() const;
  };

  class AcquireControlMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint32_t controller; /**< 
     The ID of the controller. The controller ID is the instance serial of the sending
     interface. Only from this interface instance command messages are accepted.
     */
      char controller_thread_name[64]; /**< 
     The name of the controlling thread, for easier debugging. This is informative only
     and actually two threads may share an interface instance (although this should be
     avoided since the interface locking has to be reproduced for these threads then).
   */
    } AcquireControlMessage_data_t;
#pragma pack(pop)

    AcquireControlMessage_data_t *data;

   public:
    AcquireControlMessage(const uint32_t ini_controller, const char * ini_controller_thread_name);
    AcquireControlMessage();
    ~AcquireControlMessage();

    AcquireControlMessage(const AcquireControlMessage *m);
    /* Methods */
    uint32_t controller() const;
    void set_controller(const uint32_t new_controller);
    size_t maxlenof_controller() const;
    char * controller_thread_name() const;
    void set_controller_thread_name(const char * new_controller_thread_name);
    size_t maxlenof_controller_thread_name() const;
    virtual Message * clone() const;
  };

  class ResetOdometryMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } ResetOdometryMessage_data_t;
#pragma pack(pop)

    ResetOdometryMessage_data_t *data;

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
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float front_right; /**< Rotation in RPM of the right front wheel. */
      float front_left; /**< Rotation in RPM of the left front wheel. */
      float rear; /**< Rotation in RPM of the rear wheel. */
    } DriveRPMMessage_data_t;
#pragma pack(pop)

    DriveRPMMessage_data_t *data;

   public:
    DriveRPMMessage(const float ini_front_right, const float ini_front_left, const float ini_rear);
    DriveRPMMessage();
    ~DriveRPMMessage();

    DriveRPMMessage(const DriveRPMMessage *m);
    /* Methods */
    float front_right() const;
    void set_front_right(const float new_front_right);
    size_t maxlenof_front_right() const;
    float front_left() const;
    void set_front_left(const float new_front_left);
    size_t maxlenof_front_left() const;
    float rear() const;
    void set_rear(const float new_rear);
    size_t maxlenof_rear() const;
    virtual Message * clone() const;
  };

  class GotoMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float x; /**< X distance in m. */
      float y; /**< Y distance in m. */
      float phi; /**< Angle relative to current angle in rad. */
      float time_sec; /**< When to reach the desired location. */
    } GotoMessage_data_t;
#pragma pack(pop)

    GotoMessage_data_t *data;

   public:
    GotoMessage(const float ini_x, const float ini_y, const float ini_phi, const float ini_time_sec);
    GotoMessage();
    ~GotoMessage();

    GotoMessage(const GotoMessage *m);
    /* Methods */
    float x() const;
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y() const;
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    float phi() const;
    void set_phi(const float new_phi);
    size_t maxlenof_phi() const;
    float time_sec() const;
    void set_time_sec(const float new_time_sec);
    size_t maxlenof_time_sec() const;
    virtual Message * clone() const;
  };

  class TransMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float vx; /**< Speed in X direction in m/s. */
      float vy; /**< Speed in Y direction in m/s. */
    } TransMessage_data_t;
#pragma pack(pop)

    TransMessage_data_t *data;

   public:
    TransMessage(const float ini_vx, const float ini_vy);
    TransMessage();
    ~TransMessage();

    TransMessage(const TransMessage *m);
    /* Methods */
    float vx() const;
    void set_vx(const float new_vx);
    size_t maxlenof_vx() const;
    float vy() const;
    void set_vy(const float new_vy);
    size_t maxlenof_vy() const;
    virtual Message * clone() const;
  };

  class RotMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float omega; /**< Angle rotation in rad/s. */
    } RotMessage_data_t;
#pragma pack(pop)

    RotMessage_data_t *data;

   public:
    RotMessage(const float ini_omega);
    RotMessage();
    ~RotMessage();

    RotMessage(const RotMessage *m);
    /* Methods */
    float omega() const;
    void set_omega(const float new_omega);
    size_t maxlenof_omega() const;
    virtual Message * clone() const;
  };

  class TransRotMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float vx; /**< Speed in X direction in m/s. */
      float vy; /**< Speed in Y direction in m/s. */
      float omega; /**< Angle rotation in rad/s. */
    } TransRotMessage_data_t;
#pragma pack(pop)

    TransRotMessage_data_t *data;

   public:
    TransRotMessage(const float ini_vx, const float ini_vy, const float ini_omega);
    TransRotMessage();
    ~TransRotMessage();

    TransRotMessage(const TransRotMessage *m);
    /* Methods */
    float vx() const;
    void set_vx(const float new_vx);
    size_t maxlenof_vx() const;
    float vy() const;
    void set_vy(const float new_vy);
    size_t maxlenof_vy() const;
    float omega() const;
    void set_omega(const float new_omega);
    size_t maxlenof_omega() const;
    virtual Message * clone() const;
  };

  class OrbitMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float px; /**< Point's X coordinate to orbit. */
      float py; /**< Point's Y coordinate to orbit. */
      float omega; /**< Angular speed around point in rad/s. */
    } OrbitMessage_data_t;
#pragma pack(pop)

    OrbitMessage_data_t *data;

   public:
    OrbitMessage(const float ini_px, const float ini_py, const float ini_omega);
    OrbitMessage();
    ~OrbitMessage();

    OrbitMessage(const OrbitMessage *m);
    /* Methods */
    float px() const;
    void set_px(const float new_px);
    size_t maxlenof_px() const;
    float py() const;
    void set_py(const float new_py);
    size_t maxlenof_py() const;
    float omega() const;
    void set_omega(const float new_omega);
    size_t maxlenof_omega() const;
    virtual Message * clone() const;
  };

  class LinTransRotMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float vx; /**< Speed for translation in X direction in m/s. */
      float vy; /**< Speed for translation in Y direction in m/s. */
      float omega; /**< Rotational speed in rad/s. */
    } LinTransRotMessage_data_t;
#pragma pack(pop)

    LinTransRotMessage_data_t *data;

   public:
    LinTransRotMessage(const float ini_vx, const float ini_vy, const float ini_omega);
    LinTransRotMessage();
    ~LinTransRotMessage();

    LinTransRotMessage(const LinTransRotMessage *m);
    /* Methods */
    float vx() const;
    void set_vx(const float new_vx);
    size_t maxlenof_vx() const;
    float vy() const;
    void set_vy(const float new_vy);
    size_t maxlenof_vy() const;
    float omega() const;
    void set_omega(const float new_omega);
    size_t maxlenof_omega() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  MotorInterface();
  ~MotorInterface();

 public:
  /* Methods */
  uint32_t motor_state() const;
  void set_motor_state(const uint32_t new_motor_state);
  size_t maxlenof_motor_state() const;
  uint32_t drive_mode() const;
  void set_drive_mode(const uint32_t new_drive_mode);
  size_t maxlenof_drive_mode() const;
  int32_t right_rpm() const;
  void set_right_rpm(const int32_t new_right_rpm);
  size_t maxlenof_right_rpm() const;
  int32_t rear_rpm() const;
  void set_rear_rpm(const int32_t new_rear_rpm);
  size_t maxlenof_rear_rpm() const;
  int32_t left_rpm() const;
  void set_left_rpm(const int32_t new_left_rpm);
  size_t maxlenof_left_rpm() const;
  float odometry_path_length() const;
  void set_odometry_path_length(const float new_odometry_path_length);
  size_t maxlenof_odometry_path_length() const;
  float odometry_position_x() const;
  void set_odometry_position_x(const float new_odometry_position_x);
  size_t maxlenof_odometry_position_x() const;
  float odometry_position_y() const;
  void set_odometry_position_y(const float new_odometry_position_y);
  size_t maxlenof_odometry_position_y() const;
  float odometry_orientation() const;
  void set_odometry_orientation(const float new_odometry_orientation);
  size_t maxlenof_odometry_orientation() const;
  float vx() const;
  void set_vx(const float new_vx);
  size_t maxlenof_vx() const;
  float vy() const;
  void set_vy(const float new_vy);
  size_t maxlenof_vy() const;
  float omega() const;
  void set_omega(const float new_omega);
  size_t maxlenof_omega() const;
  uint32_t controller() const;
  void set_controller(const uint32_t new_controller);
  size_t maxlenof_controller() const;
  char * controller_thread_name() const;
  void set_controller_thread_name(const char * new_controller_thread_name);
  size_t maxlenof_controller_thread_name() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
