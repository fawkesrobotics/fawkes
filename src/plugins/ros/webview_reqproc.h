
/***************************************************************************
 *  webview_reqproc.h - Webview ROS request processor
 *
 *  Created: Fri May 06 17:23:14 2011
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

#ifndef __PLUGINS_ROS_WEBVIEW_REQPROC_H_
#define __PLUGINS_ROS_WEBVIEW_REQPROC_H_

#include <webview/request_processor.h>

#include <core/utils/lockptr.h>
#include <ros/ros.h>
#include <string>

namespace fawkes {
  class Logger;
}

class ROSWebviewRequestProcessor
: public fawkes::WebRequestProcessor
{
 public:
  ROSWebviewRequestProcessor(fawkes::LockPtr<ros::NodeHandle> nh,
			     fawkes::Logger *logger,
			     std::string &baseurl, std::string &srv_name);
  virtual ~ROSWebviewRequestProcessor();

  virtual fawkes::WebReply * process_request(const fawkes::WebRequest *request);

 private:
  std::string        __logcomp;

  std::string        __baseurl;
  std::string        __srv_name;
  ros::ServiceClient __srv_client;
};


#endif
