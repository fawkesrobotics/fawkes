/***************************************************************************
 *  gazsim_light_front_plugin.cpp - Plugin provides 
 *     the detected light signals of the llsf-machines
 *
 *  Created: Tue Aug 20 22:33:18 2013
 *  Copyright  2013 Frederik Zwilling
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

#include "gazsim_light_front_thread.h"

#include <tf/types.h>
#include <stdio.h>
#include <math.h>
#include <utils/math/angle.h>

#include <interfaces/RobotinoLightInterface.h>
#include <interfaces/Position3DInterface.h>
#include <protobuf_msgs/Pose2D.pb.h>
#include <protobuf_msgs/MachineInfo.pb.h>
#include <protobuf_msgs/LightSignals.pb.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

using namespace fawkes;
using namespace gazebo;

/** @class LightFrontSimThread "gazsim_light_front_thread.h"
 * Thread simulates the Light-Front Plugin in Gazebo
 *
 * Gazebo sends the light signals of all machines and the plugin determines
 * On which machine the robotino as looking at with laser cluster results
 *
 * @author Frederik Zwilling
 */

/** Constructor. */
LightFrontSimThread::LightFrontSimThread()
  : Thread("LightFrontSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void LightFrontSimThread::init()
{
  logger->log_debug(name(), 
		    "Initializing Simulation of the Light Front Plugin");

  //read config values
  max_distance_ = config->get_float("/gazsim/light-front/max-distance");
  success_visibility_history_ = config->get_int("/gazsim/light-front/success-visibility-history");
  fail_visibility_history_ = config->get_int("/gazsim/light-front/fail-visibility-history");
  light_state_if_name_ = config->get_string("/plugins/light_front/light_state_if");
  light_pos_if_name_ = config->get_string("/plugins/light_front/light_position_if");

  //open interfaces
  light_if_ = blackboard->open_for_writing<RobotinoLightInterface>
    (light_state_if_name_.c_str());
  pose_if_ = blackboard->open_for_reading<fawkes::Position3DInterface>
    (light_pos_if_name_.c_str());

  //subscribing to gazebo publisher
  light_signals_sub_ = gazebonode->Subscribe
    (std::string("~/RobotinoSim/MachineVision/"), 
     &LightFrontSimThread::on_light_signals_msg, this);
  localization_sub_ = gazebonode->Subscribe(std::string("~/RobotinoSim/Gps/"), &LightFrontSimThread::on_localization_msg, this);
}

void LightFrontSimThread::finalize()
{
  blackboard->close(light_if_);
  blackboard->close(pose_if_);
}

void LightFrontSimThread::loop()
{
  //the acual work takes place in on_light_signals_msg

  //TODO switch interface
}

void LightFrontSimThread::on_light_signals_msg(ConstAllMachineSignalsPtr &msg)
{
  //logger->log_info(name(), "Got new Machine Light signals.\n");

  //read cluster position to determine 
  //on which machine the robotino is looking at
  //(the cluster returns the position relative to the robots pos and ori)
  pose_if_->read();
  double rel_x = pose_if_->translation(0);
  double rel_y = pose_if_->translation(1);
  double look_pos_x = robot_x_ 
    + cos(robot_ori_) * rel_x - sin(robot_ori_) * rel_y;
  double look_pos_y = robot_y_ 
    + sin(robot_ori_) * rel_x + cos(robot_ori_) * rel_y;
  //find machine nearest to the look_pos
  double min_distance = 1000.0;
  int index_min = 0;
  for(int i = 0; i < msg->machines_size(); i++)
  {
    llsf_msgs::MachineSignal machine = msg->machines(i);
    llsf_msgs::Pose2D pose = machine.pose();
    double x = pose.x();
    double y = pose.y();
    double distance = sqrt((look_pos_x - x) * (look_pos_x - x) 
			   + (look_pos_y - y) * (look_pos_y - y));
    if(distance < min_distance)
    {
      min_distance = distance;
      index_min = i;
    }
  }

  //if the distance is greater than a threashold, no light is determined
  if(min_distance > max_distance_)
  {
    //logger->log_info(name(), "Distance between light pos and machine (%f) is too big.\n", min_distance);
    //set ready and visibility history
    light_if_->set_ready(false);
    light_if_->set_visibility_history(fail_visibility_history_);
  }
  else{
    //read out lights
    llsf_msgs::MachineSignal machine = msg->machines(index_min);
    //printf("looking at machine :%s\n", machine.name().c_str());
    for(int i = 0; i < machine.lights_size(); i++)
    {
      llsf_msgs::LightSpec light_spec = machine.lights(i);
      RobotinoLightInterface::LightState state = RobotinoLightInterface::OFF;
      switch(light_spec.state())
      {
      case llsf_msgs::OFF: state = RobotinoLightInterface::OFF; break;
      case llsf_msgs::ON: state = RobotinoLightInterface::ON; break;
      case llsf_msgs::BLINK: state = RobotinoLightInterface::BLINKING; break;
      }
      switch(light_spec.color())
      {
      case llsf_msgs::RED: light_if_->set_red(state); break;
      case llsf_msgs::YELLOW: light_if_->set_yellow(state); break;
      case llsf_msgs::GREEN: light_if_->set_green(state); break;
      }
    }
    //set ready and visibility history
    light_if_->set_ready(true);
    light_if_->set_visibility_history(success_visibility_history_);
  }
  light_if_->write();
}

void LightFrontSimThread::on_localization_msg(ConstPosePtr &msg)
{
  //logger->log_info(name(), "Got new Localization data.\n");

  //read data from message
  robot_x_ = msg->position().x();
  robot_y_ = msg->position().y();
  robot_ori_ = msg->orientation().z();
}
