
/***************************************************************************
 *  pointcloud_thread.h - OpenNI point cloud provider thread
 *
 *  Created: Fri Mar 25 23:48:05 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef __PLUGINS_OPENNI_PCL_FROMBUF_THREAD_H_
#define __PLUGINS_OPENNI_PCL_FROMBUF_THREAD_H_

// must be first for reliable ROS detection
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blocked_timing.h>
#include <aspect/pointcloud.h>
#include <fvutils/adapters/pcl.h>
#include <plugins/openni/aspect/openni.h>
#include <utils/time/time.h>

#include <map>

namespace firevision {
  class SharedMemoryImageBuffer;
}

class OpenNiPclOnlyThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::PointCloudAspect
{
 public:
  OpenNiPclOnlyThread();
  virtual ~OpenNiPclOnlyThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  firevision::SharedMemoryImageBuffer *__pcl_buf;
  fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZ> > __pcl;

  fawkes::Time __last_capture_time;
  unsigned int __width;
  unsigned int __height;
};

#endif
