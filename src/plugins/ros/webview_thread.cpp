
/***************************************************************************
 *  webview_thread.cpp - Webview ROS integration thread
 *
 *  Created: Fri May 06 11:12:05 2011
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

#include "webview_thread.h"
#include "webview_reqproc.h"

#include <webview/url_manager.h>
#include <webview/nav_manager.h>

#include <ros/ros.h>

using namespace fawkes;

/** @class ROSWebviewThread "webview_thread.h"
 * Provide webview to ROS.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
ROSWebviewThread::ROSWebviewThread()
  : Thread("ROSWebviewThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
ROSWebviewThread::~ROSWebviewThread()
{
}


void
ROSWebviewThread::init()
{
	__srv_register   =
		rosnode->advertiseService("/webview/register",
		                          &ROSWebviewThread::srv_register_cb, this);
  __srv_unregister =
	  rosnode->advertiseService("/webview/unregister",
	                            &ROSWebviewThread::srv_unregister_cb, this);

  __srv_add_nav   =
	  rosnode->advertiseService("/webview/add_nav_entry",
	                            &ROSWebviewThread::srv_add_nav_cb, this);
  __srv_remove_nav =
    rosnode->advertiseService("/webview/remove_nav_entry",
                              &ROSWebviewThread::srv_remove_nav_cb, this);
}


void
ROSWebviewThread::finalize()
{
  __srv_register.shutdown();
  __srv_unregister.shutdown();

  __srv_add_nav.shutdown();
  __srv_remove_nav.shutdown();

  std::map<std::string, ROSWebviewRequestProcessor *>::iterator i;
  for (i = __procs.begin(); i != __procs.end(); ++i) {
    webview_url_manager->unregister_baseurl(i->first.c_str());
    delete i->second;
  }
  __procs.clear();

  std::map<std::string, std::string>::iterator ne;
  for (ne = __nav_entries.begin(); ne != __nav_entries.end(); ++ne) {
    webview_nav_manager->remove_nav_entry(ne->first);
  }
  __nav_entries.clear();
}


void
ROSWebviewThread::loop()
{
}


bool
ROSWebviewThread::srv_register_cb(fawkes_msgs::WebviewUrlRegistration::Request  &req,
                                  fawkes_msgs::WebviewUrlRegistration::Response &resp)
{
  if (__procs.find(req.url_prefix) != __procs.end()) {
    resp.success = false;
    resp.error = "A processor has already been registered for this prefix";
  } else {
    try {
      logger->log_info(name(), "Registering srv '%s' for URL '%s'",
		       req.service_name.c_str(), req.url_prefix.c_str());
      __procs[req.url_prefix] = new ROSWebviewRequestProcessor(rosnode, logger,
							       req.url_prefix,
							       req.service_name);
      try {
	webview_url_manager->register_baseurl(req.url_prefix.c_str(),
					      __procs[req.url_prefix]);
	resp.success = true;
      } catch (fawkes::Exception &e) {
	resp.success = false;
	resp.error = std::string("Failed to register processor: ") + e.what();
      }
    } catch (ros::Exception &e) {
      resp.success = false;
      resp.error = std::string("Registration failed: ") + e.what();
    }
  }
  return true;
}

bool
ROSWebviewThread::srv_unregister_cb(fawkes_msgs::WebviewUrlRegistration::Request  &req,
                                    fawkes_msgs::WebviewUrlRegistration::Response &resp)
{
  logger->log_debug(name(), "%s unregisters for %s", req.service_name.c_str(),
		    req.url_prefix.c_str());
  if (__procs.find(req.url_prefix) == __procs.end()) {
    resp.success = false;
    resp.error = "A processor has not been registered for this prefix";
  } else {
    if (__procs.find(req.url_prefix) != __procs.end()) {
      logger->log_info(name(), "De-registering URL '%s'",
		       req.url_prefix.c_str());
      webview_url_manager->unregister_baseurl(req.url_prefix.c_str());
      delete __procs[req.url_prefix];
      __procs.erase(req.url_prefix);
    } else {
      logger->log_warn(name(), "Unregister request for %s, but is not registered",
		       req.url_prefix.c_str());
    }
    resp.success = true;
  }
  return true;
}

bool
ROSWebviewThread::srv_add_nav_cb(fawkes_msgs::WebviewNavRegistration::Request  &req,
                                 fawkes_msgs::WebviewNavRegistration::Response &resp)
{
  try {
    webview_nav_manager->add_nav_entry(req.url, req.name);
    __nav_entries[req.url] = req.name;
    resp.success = true;
  } catch (Exception &e) {
    resp.success = false;
    resp.error = e.what();
  }

  return true;
}

bool
ROSWebviewThread::srv_remove_nav_cb(fawkes_msgs::WebviewNavRegistration::Request  &req,
                                    fawkes_msgs::WebviewNavRegistration::Response &resp)
{
  webview_nav_manager->remove_nav_entry(req.url);
  __nav_entries.erase(req.url);
  resp.success = true;
  return true;
}
