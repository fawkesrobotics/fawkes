
/***************************************************************************
 *  webview_thread.h - Webview ROS integration thread
 *
 *  Created: Fri May 06 10:46:57 2011
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

#ifndef __PLUGINS_ROS_WEBVIEW_THREAD_H_
#define __PLUGINS_ROS_WEBVIEW_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/webview.h>
#include <plugins/ros/aspect/ros.h>

#include <ros/service_server.h>
#include <fawkes_msgs/WebviewUrlRegistration.h>
#include <fawkes_msgs/WebviewNavRegistration.h>

#include <map>
#include <string>

class ROSWebviewRequestProcessor;

class ROSWebviewThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::ROSAspect,
  public fawkes::WebviewAspect
{
 public:
  ROSWebviewThread();
  virtual ~ROSWebviewThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  bool srv_register_cb(fawkes_msgs::WebviewUrlRegistration::Request  &req,
                       fawkes_msgs::WebviewUrlRegistration::Response &resp);

  bool srv_unregister_cb(fawkes_msgs::WebviewUrlRegistration::Request  &req,
                         fawkes_msgs::WebviewUrlRegistration::Response &resp);

  bool srv_add_nav_cb(fawkes_msgs::WebviewNavRegistration::Request  &req,
                      fawkes_msgs::WebviewNavRegistration::Response &resp);

  bool srv_remove_nav_cb(fawkes_msgs::WebviewNavRegistration::Request  &req,
                         fawkes_msgs::WebviewNavRegistration::Response &resp);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  ros::ServiceServer __srv_register;
  ros::ServiceServer __srv_unregister;
  ros::ServiceServer __srv_add_nav;
  ros::ServiceServer __srv_remove_nav;

  std::map<std::string, ROSWebviewRequestProcessor *> __procs;
  std::map<std::string, std::string> __nav_entries;
};

#endif
