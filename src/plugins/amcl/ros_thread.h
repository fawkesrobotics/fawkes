/***************************************************************************
 *  ros_thread.cpp - Thread to interact with ROS for amcl plugin
 *
 *  Created: Mon Jun 22 17:46:40 2015
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_AMCL_ROS_THREAD_H_
#define __PLUGINS_AMCL_ROS_THREAD_H_

#ifndef HAVE_ROS
#  error "ROS integration requires ROS support of system"
#endif

#include "amcl_thread.h"

#include "map/map.h"
#include "pf/pf.h"

#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/logging.h>

#include <interfaces/LocalizationInterface.h>

#include <plugins/ros/aspect/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace fawkes {
  class Mutex;
}

class AmclThread;

class AmclROSThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ROSAspect
{
public:
  AmclROSThread();
  virtual ~AmclROSThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  void publish_pose_array(const std::string &global_frame_id,
			  const pf_sample_set_t* set);
  void publish_pose(const std::string &global_frame_id,
		    const amcl_hyp_t &amcl_hyp,
		    const double last_covariance[36]);
  void publish_map(const std::string &global_frame_id,
		   const map_t *map);


  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run();}

 private:
  void initial_pose_received(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

private:
  std::string cfg_pose_ifname_;

  fawkes::LocalizationInterface   *loc_if_;

  ros::Publisher pose_pub_;
  ros::Publisher particlecloud_pub_;
  ros::Subscriber initial_pose_sub_;
  ros::Publisher map_pub_;
};

#endif
