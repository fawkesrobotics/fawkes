
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

#include <pcl_utils/pcl_adapter.h>

#include <core/threading/thread.h>
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

#if PCL_VERSION_COMPARE(>=,1,7,0)
#  include <pcl/PCLPointCloud2.h>
#else
#  include <sensor_msgs/PointCloud2.h>
#endif

namespace fawkes {
  class Mutex;
  class TimeWait;
}

namespace mongo {
  class GridFS;
}

class MongoLogPointCloudThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::PointCloudAspect,
  public fawkes::MongoDBAspect
{
 public:
  MongoLogPointCloudThread();
  virtual ~MongoLogPointCloudThread();

  virtual void init();
  virtual void loop();
  virtual bool prepare_finalize_user();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  PointCloudAdapter *adapter_;

  /// @cond INTERNALS
  typedef struct {
    std::string		     topic_name;
#if PCL_VERSION_COMPARE(>=,1,7,0)
    pcl::PCLPointCloud2      msg;
#else
    sensor_msgs::PointCloud2 msg;
#endif
    fawkes::Time             last_sent;
  } PointCloudInfo;
  /// @endcond
  std::map<std::string, PointCloudInfo> pcls_;

  mongo::DBClientBase *mongodb_;
  mongo::GridFS       *gridfs_;
  std::string          collection_;
  std::string          database_;

  fawkes::Mutex       *mutex_;
  fawkes::TimeWait    *wait_;

  bool                 cfg_flush_after_write_;
  unsigned int         cfg_chunk_size_;
  float                cfg_storage_interval_;
};

#endif
