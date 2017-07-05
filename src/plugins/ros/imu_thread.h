/***************************************************************************
 *  imu_thread.h - Thread to publish IMU data to ROS
 *
 *  Created: Mon 03 Apr 2017 12:41:33 CEST 12:41
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#ifndef __PLUGINS_ROS_IMU_THREAD_H_
#define __PLUGINS_ROS_IMU_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <plugins/ros/aspect/ros.h>
#include <interfaces/IMUInterface.h>

#include <ros/node_handle.h>

class RosIMUThread
: public fawkes::Thread,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
  public fawkes::ROSAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  RosIMUThread();
  virtual ~RosIMUThread();

  virtual void init();
  virtual void finalize();

  virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

 private:
  ros::Publisher ros_pub_;
  fawkes::IMUInterface * iface_;
};

#endif /* __PLUGINS_ROS_IMU_THREAD_H_ */
