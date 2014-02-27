/***************************************************************************
 *  sim_motorinterface.h - Simulates the Motorinterface
 *
 *  Created: Tue Jun 18 11:35:52 2013
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

#ifndef __SIM_MOTOR_INTERFACE_H_
#define __SIM_MOTOR_INTERFACE_H_

#include "sim_interface.h"
#include <tf/transform_publisher.h>
#include <utils/time/clock.h>
#include <utils/time/time.h>
#include <interfaces/SwitchInterface.h>



namespace fawkes {
  class MotorInterface;
  class SwitchInterface;
}
/** @class SimMotorInterface
 * class to simulate the motor interface
 * @author Frederik Zwilling
 */
class SimMotorInterface: public SimInterface
{
 public:
  /** Constructor
   * @param controlPublisher Publisher for sending control msgs
   * @param logger Logger
   * @param blackboard Blackboard
   * @param gazebonode Gazebo node for communication
   * @param config Fawkes Config
   * @param clock Clock
   * @param tf_publisher TF_Publisher
   */
 SimMotorInterface(gazebo::transport::PublisherPtr controlPublisher, fawkes::Logger *logger, fawkes::BlackBoard *blackboard, gazebo::transport::NodePtr gazebonode, fawkes::Configuration *config, fawkes::Clock *clock, fawkes::tf::TransformPublisher *tf_publisher)
   : SimInterface(controlPublisher, logger, blackboard, gazebonode, "SimMotorInterface", config)
  {
    this->clock_ = clock;
    this->tf_publisher_ = tf_publisher;

    x_offset_ = 0.0;
    y_offset_ = 0.0;
    ori_offset_ = 0.0;
  };
  ///Destructor
  ~SimMotorInterface() {};

  virtual void init();
  virtual void loop();
  virtual void finalize();
 

 private:
  ///Publisher to send messages to gazebo
  gazebo::transport::PublisherPtr motor_move_pub_;

  ///Suscribers to recieve the robot's position from gazebo for odometry
  gazebo::transport::SubscriberPtr pos_sub_;

  ///provided interfaces
  fawkes::MotorInterface *motor_if_;
  fawkes::SwitchInterface *switch_if_;
  
  //Needed for publishing the /base_link /odom transform
  fawkes::Clock *clock_;
  fawkes::tf::TransformPublisher *tf_publisher_;

  //Helper variables:
  //motorMovements last sent to gazebo
  float vx_;
  float vy_;
  float  vomega_;
  float  x_;
  float  y_;
  float  ori_;
  float  path_length_;
  fawkes::Time last_pos_time_;
  fawkes::Time last_vel_set_time_;

  //Odometry offset
  float x_offset_;
  float  y_offset_;
  float  ori_offset_;

  //Helper functions:
  void process_messages();
  void send_transroot(double vx, double vy, double omega);

  //Handler functions for incoming messages
  void on_pos_msg(ConstPosePtr &msg);

  //config values
  bool slippery_wheels_enabled_;
  double slippery_wheels_threshold_;
  double  moving_speed_factor_;
  double  rotation_speed_factor_;
};

#endif
