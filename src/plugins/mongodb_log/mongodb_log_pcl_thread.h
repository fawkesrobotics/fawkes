
/***************************************************************************
 *  mongodb_log_pcl_thread.h - Thread to log point clouds to MongoDB
 *
 *  Created: Mon Nov 07 02:26:35 2011
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
 *             2012       Bastian Klingen
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

#ifndef __PLUGINS_MONGODB_LOG_MONGODB_LOG_PCL_THREAD_H_
#define __PLUGINS_MONGODB_LOG_MONGODB_LOG_PCL_THREAD_H_

#include "pcl_adapter.h"

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/pointcloud.h>
#include <plugins/mongodb/aspect/mongodb.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <interfaces/TransformInterface.h>
#include <core/threading/mutex.h>

#include <list>
#include <queue>

#include <sensor_msgs/PointCloud2.h>

namespace mongo {
  class GridFS;
}

class MongoLogPointCloudThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::PointCloudAspect,
  public fawkes::MongoDBAspect
{
 public:
  MongoLogPointCloudThread();
  virtual ~MongoLogPointCloudThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  MongoLogPointCloudAdapter *__adapter;

  /// @cond INTERNALS
  typedef struct {
    std::string		     topic_name;
    sensor_msgs::PointCloud2 msg;
    fawkes::Time             last_sent;
  } PointCloudInfo;
  /// @endcond
  std::map<std::string, PointCloudInfo> __pcls;

  mongo::DBClientBase *__mongodb;
  mongo::GridFS       *__mongogrid;
  std::string          __collection;
  std::string          __database;
};

#endif
