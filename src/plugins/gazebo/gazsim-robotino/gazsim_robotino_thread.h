/***************************************************************************
 *  gazsim_robotino_thread.h - Thread simulate the Robotino in Gazebo
 *  by sending needed informations to the Robotino-plugin in Gazebo
 *  and recieving sensordata from Gazebo
 *
 *  Created: Fr 3. Mai 21:20:08 CEST 2013
 *  Copyright  2013  Frederik Zwilling
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_GAZSIM_ROBOTINO_THREAD_H_
#define __PLUGINS_GAZSIM_ROBOTINO_THREAD_H_

#include <list>

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <aspect/tf.h>
#include "../msgs/Float.pb.h"

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>


typedef const boost::shared_ptr<gazsim_msgs::Float const> ConstFloatPtr;

namespace fawkes {
  class BatteryInterface;
  class IMUInterface;
  class MotorInterface;
  class RobotinoSensorInterface;
  class SwitchInterface;
}

class RobotinoSimThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::TransformAspect,
  public fawkes::GazeboAspect
{
 public:
  RobotinoSimThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();
 private:
  //Publisher to send messages to gazebo
  gazebo::transport::PublisherPtr string_pub_;
  gazebo::transport::PublisherPtr motor_move_pub_;

  //Suscribers to recieve messages from gazebo
  gazebo::transport::SubscriberPtr gyro_sub_;
  gazebo::transport::SubscriberPtr infrared_puck_sensor_sub_;
  gazebo::transport::SubscriberPtr gripper_laser_left_sensor_sub_;
  gazebo::transport::SubscriberPtr gripper_laser_right_sensor_sub_;
  gazebo::transport::SubscriberPtr pos_sub_;

  //Handler functions for incoming messages
  void on_gyro_msg(ConstVector3dPtr &msg);
  void on_infrared_puck_sensor_msg(ConstLaserScanStampedPtr &msg);
  void on_gripper_laser_left_sensor_msg(ConstFloatPtr &msg);
  void on_gripper_laser_right_sensor_msg(ConstFloatPtr &msg);
  void on_pos_msg(ConstPosePtr &msg);

  //provided interfaces
  fawkes::RobotinoSensorInterface *sens_if_;
  fawkes::MotorInterface          *motor_if_;
  fawkes::SwitchInterface         *switch_if_;
  fawkes::IMUInterface            *imu_if_;

  //config values
  std::string cfg_frame_odom_;
  std::string cfg_frame_base_;
  std::string cfg_frame_imu_;
  double gripper_laser_threshold_;
  double gripper_laser_value_far_;
  double gripper_laser_value_near_;
  bool slippery_wheels_enabled_;
  double slippery_wheels_threshold_;
  double moving_speed_factor_;
  double rotation_speed_factor_;
  bool have_gripper_sensors_;
  int gripper_laser_left_pos_;
  int gripper_laser_right_pos_;
  int infrared_sensor_index_;

  //Helper variables for motor:

  //current motorMovements
  float vx_;
  float vy_;
  float vomega_;
  float des_vx_;
  float des_vy_;
  float des_vomega_;
  //last received odom position
  float x_;
  float y_;
  float ori_;
  float path_length_;
 
  //RobotinoSensorInterface values (stored here to write the interfaces only in the loop)
  bool gyro_available_;
  int gyro_buffer_size_;
  int gyro_buffer_index_new_;
  int gyro_buffer_index_delayed_;
  fawkes::Time *gyro_timestamp_buffer_;
  float *gyro_angle_buffer_;
  float gyro_delay_;
  float infrared_puck_sensor_dist_;
  float analog_in_left_;
  float analog_in_right_;

  //are there new values to write in the interfaces?
  bool new_data_;

  fawkes::Time last_pos_time_;
  fawkes::Time last_vel_set_time_;

  //Odometry offset
  float x_offset_;
  float y_offset_;
  float ori_offset_;

  //Helper functions:
  void process_motor_messages();
  void send_transroot(double vx, double vy, double omega);
  bool vel_changed(float before, float after, float relativeThreashold);  
};

#endif
