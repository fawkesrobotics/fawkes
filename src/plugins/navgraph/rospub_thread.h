/***************************************************************************
 *  navgraph_rospub_thread.h - Publish navgraph to ROS
 *
 *  Created: Wed Jun 08 20:16:31 2016
 *  Copyright  2016  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAVGRAPH_NAVGRAPH_ROSPUB_THREAD_H_
#define __PLUGINS_NAVGRAPH_NAVGRAPH_ROSPUB_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <plugins/ros/aspect/ros.h>
#include <navgraph/aspect/navgraph.h>

#include <navgraph/navgraph.h>

#include <ros/publisher.h>
#include <ros/service_server.h>

#include <fawkes_msgs/NavGraphSearchPath.h>
#include <fawkes_msgs/NavGraphGetPairwiseCosts.h>

class NavGraphROSPubThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
	public fawkes::TransformAspect,
  public fawkes::ROSAspect,
	public fawkes::NavGraphAspect,
	public fawkes::NavGraph::ChangeListener
{
 public:
  NavGraphROSPubThread();
  virtual ~NavGraphROSPubThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  virtual void graph_changed() throw();
  
  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run();}

 private:
  void publish_graph();
  void convert_nodes(const std::vector<fawkes::NavGraphNode> &nodes,
                     std::vector<fawkes_msgs::NavGraphNode> &out);

  bool svs_search_path_cb(fawkes_msgs::NavGraphSearchPath::Request  &req,
                          fawkes_msgs::NavGraphSearchPath::Response &res);
  bool svs_get_pwcosts_cb(fawkes_msgs::NavGraphGetPairwiseCosts::Request  &req,
                          fawkes_msgs::NavGraphGetPairwiseCosts::Response &res);

 private:
  std::string  cfg_base_frame_;
  std::string  cfg_global_frame_;

  ros::Publisher pub_;
  ros::ServiceServer svs_search_path_;
  ros::ServiceServer svs_get_pwcosts_;
};

#endif
