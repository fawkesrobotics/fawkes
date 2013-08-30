/***************************************************************************
 *  gazsim_light_front_plugin.cpp - Plugin provides 
 *     the detected light signals of the llsf-machines
 *
 *  Created: Tue Aug 20 22:32:52 2013
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

#ifndef __PLUGINS_GAZSIM_LIGHT_FRONT_THREAD_H_
#define __PLUGINS_GAZSIM_LIGHT_FRONT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <interfaces/RobotinoLightInterface.h>
#include <interfaces/Position3DInterface.h>
#include <protobuf_msgs/LightSignals.pb.h>
#include <string.h>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>


typedef const boost::shared_ptr<llsf_msgs::AllMachineSignals const> ConstAllMachineSignalsPtr;

namespace fawkes {
  class RobotinoLightInterface;
}

class LightFrontSimThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GazeboAspect
{
 public:
  LightFrontSimThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  //Subscriber to receive light signal data from gazebo
  gazebo::transport::SubscriberPtr light_signals_sub_;
  //Subscriber to receive localization data from gazebo
  //(light front works relative, and the light signal msgs are absolute)
  gazebo::transport::SubscriberPtr localization_sub_;

  //interface needed to determine position of the light
  fawkes::Position3DInterface *pose_if_;

  //provided interface
  fawkes::RobotinoLightInterface *light_if_;

  //handler function for incoming messages about the machine light signals
  void on_light_signals_msg(ConstAllMachineSignalsPtr &msg);

  //handler function for incoming localization data messages
  void on_localization_msg(ConstPosePtr &msg);

  //config value for maximal distance before the plugin says 
  //it can not detect any light
  double max_distance_;

  int success_visibility_history_;
  int fail_visibility_history_;

  std::string light_pos_if_name_;
  std::string light_state_if_name_;

  //the robots position in the simulation
  double robot_x_, robot_y_, robot_ori_;
};

#endif
