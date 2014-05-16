/***************************************************************************
 *  robot_description_thread.h - ROS Robot Description Plugin
 *
 *  Created: Fri May 16 15:29:17 2014
 *  Copyright  2014  Till Hofmann
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

#ifndef __PLUGINS_ROS_ROBOT_DESCRIPTION_THREAD_H_
#define __PLUGINS_ROS_ROBOT_DESCRIPTION_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <plugins/ros/aspect/ros.h>

#include <string>

class ROSRobotDescriptionThread
: public fawkes::Thread,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
  public fawkes::ROSAspect
{
 public:
  ROSRobotDescriptionThread();
  virtual ~ROSRobotDescriptionThread();

  virtual void init();
  virtual void finalize();
 private:
  std::string cfg_urdf_path_;
  std::string cfg_ros_param_;
};

#endif
