
/***************************************************************************
 *  pcl_thread.cpp - Thread to exchange point clouds
 *
 *  Created: Mon Nov 07 02:26:35 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROS_PCL_THREAD_H_
#define __PLUGINS_ROS_PCL_THREAD_H_

#include "pcl_adapter.h"

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/pointcloud.h>
#include <plugins/ros/aspect/ros.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <interfaces/TransformInterface.h>
#include <core/threading/mutex.h>

#include <list>
#include <queue>

#include <sensor_msgs/PointCloud2.h>

class RosPointCloudThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::PointCloudAspect,
  public fawkes::ROSAspect
{
 public:
  RosPointCloudThread();
  virtual ~RosPointCloudThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  RosPointCloudAdapter *__adapter;

  typedef struct {
    ros::Publisher           pub;
    sensor_msgs::PointCloud2 msg;
  } PublisherInfo;
  std::map<std::string, PublisherInfo> __pubs;

};

#endif
