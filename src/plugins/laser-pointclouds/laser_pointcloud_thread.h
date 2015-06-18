
/***************************************************************************
 *  laser_pointcloud_thread.h - Convert laser data to pointclouds
 *
 *  Created: Thu Nov 17 10:21:40 2011
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

#ifndef __PLUGINS_LASER_POINTCLOUDS_LASER_POINTCLOUD_THREAD_H_
#define __PLUGINS_LASER_POINTCLOUDS_LASER_POINTCLOUD_THREAD_H_

// must be first for reliable ROS detection
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/pointcloud.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/utils/lock_list.h>

namespace fawkes {
  class Interface;
  class Laser360Interface;
  class Laser720Interface;
  class Laser1080Interface;
}

class LaserPointCloudThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::PointCloudAspect,
  public fawkes::BlackBoardInterfaceObserver,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  LaserPointCloudThread();
  virtual ~LaserPointCloudThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // for BlackBoardInterfaceObserver
  virtual void bb_interface_created(const char *type, const char *id) throw();

  // for BlackBoardInterfaceListener
  virtual void bb_interface_writer_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();

 private:
  void conditional_close(fawkes::Interface *interface) throw();
  std::string interface_to_pcl_name(const char *interface_id);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  /// @cond INTERNALS
  typedef struct {
    std::string id;
    unsigned int size;
    union {
      fawkes::Laser360Interface *as360;
      fawkes::Laser720Interface *as720;
      fawkes::Laser1080Interface *as1080;
    } interface_typed;
    fawkes::Interface *interface;

    fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZ> > cloud;
  } InterfaceCloudMapping;
  /// @endcond

  fawkes::LockList<InterfaceCloudMapping> __mappings;

  float sin_angles360[360];
  float cos_angles360[360];
  float sin_angles720[720];
  float cos_angles720[720];
  float sin_angles1080[1080];
  float cos_angles1080[1080];
};

#endif
