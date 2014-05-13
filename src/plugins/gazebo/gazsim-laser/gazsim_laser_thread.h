/***************************************************************************
 *  gazsim_laser_thread.h - Thread simulate the Hokuyo in Gazebo
 *
 *  Created: Thu Aug 08 15:47:12 2013
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

#ifndef __PLUGINS_GAZSIM_LASER_THREAD_H_
#define __PLUGINS_GAZSIM_LASER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/gazebo/aspect/gazebo.h>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>


namespace fawkes {
  class Laser360Interface;
  class Time;
}

class LaserSimThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GazeboAspect
{
 public:
  LaserSimThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  ///Subscriber to receive laser data from gazebo
  gazebo::transport::SubscriberPtr laser_sub_;
  std::string laser_topic_;

  ///provided interface
  fawkes::Laser360Interface *laser_if_;

  ///storage for laser data
  float        *laser_data_;
  fawkes::Time *laser_time_;

  ///is there new information to write in the interface?
  bool new_data_;

  ///handler function for incoming laser data messages
  void on_laser_data_msg(ConstLaserScanStampedPtr &msg);

  ///maximal range of the laser sensor
  float max_range_;  

  std::string interface_id_;
  std::string frame_id_;
};

#endif
