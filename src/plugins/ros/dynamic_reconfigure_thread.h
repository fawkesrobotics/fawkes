
/***************************************************************************
 *  dynamic_reconfigure_thread.h - Robotino ROS Dynamic Reconfigure Thread
 *
 *  Created: Fri May 05 20:07:27 2017
 *  Copyright  2017  Christoph Henke
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
 
#ifndef __ROS_DYNAMIC_RECONFIGURE_THREAD_H_
#define __ROS_DYNAMIC_RECONFIGURE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <plugins/ros/aspect/ros.h>

#include <interfaces/DynamicReconfigureInterface.h>

#include <ros/ros.h>

namespace fawkes {
  class DynamicReconfigureInterface;
}

class RosDynamicReconfigureThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ROSAspect
{
 public:
  RosDynamicReconfigureThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:

  // set methods for dynamic reconfiguration of different types
  bool set_dynreconf_value(const std::string& service, const std::string& parameter, const std::string value);
  bool set_dynreconf_value(const std::string& service, const std::string& parameter, const bool value);
  bool set_dynreconf_value(const std::string& service, const std::string& parameter, const int value);
  bool set_dynreconf_value(const std::string& service, const std::string& parameter, const double value);

  // method for resetting the dynamic reconfigure interface
  void reset_dynamic_reconfigure_interface();

 private:

  fawkes::DynamicReconfigureInterface *dynrec_if_;

};

#endif /* __ROS_DYNAMIC_RECONFIGURE_THREAD_H_ */
