/***************************************************************************
 *  gazsim_puck_detection_plugin.cpp - Plugin provides 
 *     the positions of llsf-pucks
 *
 *  Created: Thu Aug 29 12:55:00 2013
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

#include "gazsim_puck_detection_thread.h"

#include <tf/types.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sstream>

#include <interfaces/SwitchInterface.h>
#include <interfaces/Position3DInterface.h>
#include <protobuf_msgs/Pose2D.pb.h>
#include <protobuf_msgs/PuckDetectionResult.pb.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

using namespace fawkes;
using namespace gazebo;

/** @class PuckDetectionSimThread "gazsim_puck_detection_thread.h"
 * Thread simulates the Omnivision-puck plugin
 *
 * @author Frederik Zwilling
 */

/** Constructor. */
PuckDetectionSimThread::PuckDetectionSimThread()
  : Thread("PuckDetectionSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void PuckDetectionSimThread::init()
{
  logger->log_debug(name(), 
		    "Initializing Simulation of the Puck Detection Plugin");

  //read config values
  max_distance_ = config->get_float("/gazsim/puck-detection/max-distance");
  success_visibility_history_ = config->get_int("/gazsim/puck-detection/success-visibility-history");
  fail_visibility_history_ = config->get_int("/gazsim/puck-detection/fail-visibility-history");
  number_pucks_ = config->get_float("/gazsim/puck-detection/number-pucks");
  
  //open interfaces (puck enumeration starts with 1)
  for(int i = 1; i <= number_pucks_; i++)
  {
    printf("Opening interface OmniPuck%d\n", i);
std::ostringstream ss;
    ss << "OmniPuck" << i;
    map_pos_if_[i] = blackboard->open_for_writing<fawkes::Position3DInterface>
      (ss.str().c_str());
  } 
  switch_if_ = blackboard->open_for_writing<fawkes::SwitchInterface>("omnivisionSwitch");

  //subscribing to gazebo publisher
  puck_positions_sub_ = gazebonode->Subscribe
    (std::string("~/RobotinoSim/PuckDetectionResult/"), 
     &PuckDetectionSimThread::on_puck_positions_msg, this);
}

void PuckDetectionSimThread::finalize()
{
  for(std::map<int, Position3DInterface*>::iterator it = map_pos_if_.begin(); it != map_pos_if_.end(); it++)
  {
    blackboard->close(it->second);
  }
  blackboard->close(switch_if_);
}

void PuckDetectionSimThread::loop()
{
  //the acual work takes place in on_light_signals_msg

  //check messages of the switch interface
  while (!switch_if_->msgq_empty()) {
    if (SwitchInterface::DisableSwitchMessage *msg =
	switch_if_->msgq_first_safe(msg)) {
      switch_if_->set_enabled(false);
    } else if (SwitchInterface::EnableSwitchMessage *msg =
	       switch_if_->msgq_first_safe(msg)) {
      switch_if_->set_enabled(true);
    }
    switch_if_->msgq_pop();
    switch_if_->write();
  }
}

void PuckDetectionSimThread::on_puck_positions_msg(ConstPuckDetectionResultPtr &msg)
{
  //logger->log_info(name(), "Got new Puck Positions.\n");

  //Only do anything if the switch is enabled
  if(!switch_if_->is_enabled())
  {
    std::list<fawkes::Position3DInterface*>::iterator puck;
    for(std::map<int, Position3DInterface*>::iterator it = map_pos_if_.begin(); it != map_pos_if_.end(); it++)
    {
      it->second->set_visibility_history(0);
      it->second->write();
    }
    return;
  }

  for(int i = 0; i < msg->positions_size(); i++)
  {
    int if_index = i + 1; //Pucks are enumerated starting with one
    llsf_msgs::Pose2D pose = msg->positions(i);
    //check if the puck is in detection range
    double distance = sqrt(pose.x() * pose.x() + pose.y() * pose.y());
    if(distance < max_distance_)
    {
      map_pos_if_[if_index]->set_translation(0, pose.x());
      map_pos_if_[if_index]->set_translation(1, pose.y());
      map_pos_if_[if_index]->set_translation(2, 0);
      map_pos_if_[if_index]->set_visibility_history(success_visibility_history_);

    }
    else
    {
      map_pos_if_[if_index]->set_visibility_history(fail_visibility_history_);
    }
    map_pos_if_[if_index]->set_frame("/base_link");
    map_pos_if_[if_index]->write();
  }
}
