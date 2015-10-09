
/***************************************************************************
 *  laserscan_thread.h - Thread to exchange laser scans
 *
 *  Created: Tue May 29 19:32:39 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROS_LASERSCAN_THREAD_H_
#define __PLUGINS_ROS_LASERSCAN_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <plugins/ros/aspect/ros.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>
#include <interfaces/Laser1080Interface.h>
#include <core/threading/mutex.h>
#include <utils/time/time.h>

#include <list>
#include <queue>

#include <ros/node_handle.h>
#include <sensor_msgs/LaserScan.h>

class RosLaserScanThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ROSAspect,
  public fawkes::BlackBoardInterfaceObserver,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  RosLaserScanThread();
  virtual ~RosLaserScanThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // for BlackBoardInterfaceObserver
  virtual void bb_interface_created(const char *type, const char *id) throw();

  // for BlackBoardInterfaceListener
  virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();
  virtual void bb_interface_writer_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();

 private:
  void laser_scan_message_cb(const ros::MessageEvent<sensor_msgs::LaserScan const> &msg_evt);
  void conditional_close(fawkes::Interface *interface) throw();
  std::string topic_name(const char *if_id, const char *suffix);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::list<fawkes::Laser360Interface *> __ls360_ifs;
  std::list<fawkes::Laser720Interface *> __ls720_ifs;
  std::list<fawkes::Laser1080Interface *> __ls1080_ifs;

  ros::Subscriber __sub_ls;

  /// @cond INTERNALS
  typedef struct {
    ros::Publisher           pub;
    sensor_msgs::LaserScan   msg;
  } PublisherInfo;
  /// @endcond
  std::map<std::string, PublisherInfo> __pubs;

  fawkes::Mutex *__ls_msg_queue_mutex;
  unsigned int __active_queue;
  std::queue<ros::MessageEvent<sensor_msgs::LaserScan const> >   __ls_msg_queues[2];

  std::map<std::string, fawkes::Laser360Interface *> __ls360_wifs;

  fawkes::Mutex *__seq_num_mutex;
  unsigned int   __seq_num;

};

#endif
