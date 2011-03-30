
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

#ifndef __PLUGINS_OPENNI_POINTCLOUD_THREAD_H_
#define __PLUGINS_OPENNI_POINTCLOUD_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blocked_timing.h>
#include <plugins/openni/aspect/openni.h>

#include <XnCppWrapper.h>

#include <map>

namespace firevision {
  class SharedMemoryImageBuffer;
}

class OpenNiPointCloudThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::OpenNiAspect
{
 public:
  OpenNiPointCloudThread();
  virtual ~OpenNiPointCloudThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  xn::DepthGenerator                  *__depth_gen;
  xn::DepthMetaData                   *__depth_md;

  firevision::SharedMemoryImageBuffer *__pcl_buf;

  unsigned int __plane_size;
  float        __focal_length;
  float        __center_x;
  float        __center_y;
  unsigned int __width;
  unsigned int __height;

  XnUInt64     __no_sample_value;
  XnUInt64     __shadow_value;
};

#endif
