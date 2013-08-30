/***************************************************************************
 *  gazsim_puck_detection_plugin.cpp - Plugin provides 
 *     the positions of llsf_pucks
 *
 *  Created: Thu Aug 29 11:43:06 2013
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

#ifndef __PLUGINS_GAZSIM_PUCK_DETECTION_THREAD_H_
#define __PLUGINS_GAZSIM_LIGHT_FRONT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <map>
#include <plugins/gazebo/aspect/gazebo.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>
#include <protobuf_msgs/PuckDetectionResult.pb.h>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>


typedef const boost::shared_ptr<llsf_msgs::PuckDetectionResult const> ConstPuckDetectionResultPtr;

namespace fawkes {
  class Position3DInterface;
}

class PuckDetectionSimThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GazeboAspect
{
 public:
  PuckDetectionSimThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  //Subscriber to receive puck positions from gazebo
  gazebo::transport::SubscriberPtr puck_positions_sub_;

  //interfaces to publish the positions
  std::map<int, fawkes::Position3DInterface*> map_pos_if_;
  //switch interface for enableing/disableing
  fawkes::SwitchInterface *switch_if_;

  //handler function for incoming messages about the machine light signals
  void on_puck_positions_msg(ConstPuckDetectionResultPtr &msg);

  //config values:
  //maximal distance before the plugin says it can not detect the puck
  double max_distance_;
  int success_visibility_history_;
  int fail_visibility_history_;
  int number_pucks_;
};

#endif
