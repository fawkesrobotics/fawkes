
/***************************************************************************
 *  ir_pcl_thread.h - Robotino IR point cloud thread
 *
 *  Created: Mon Mar 26 14:05:08 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROBOTINO_IR_PCL_THREAD_H_
#define __PLUGINS_ROBOTINO_IR_PCL_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/pointcloud.h>
#include <pcl_utils/utils.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace fawkes {
  class RobotinoSensorInterface;
}

class RobotinoIrPclThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::PointCloudAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect
{
 public:
  RobotinoIrPclThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // members
  fawkes::RobotinoSensorInterface *sens_if_;

  fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZ> > pcl_xyz_;

  float *angle_sines_;
  float *angle_cosines_;
};


#endif
