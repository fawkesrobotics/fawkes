
/***************************************************************************
 *  webview_reqproc.cpp - Webview ROS request processor
 *
 *  Created: Fri May 06 17:31:35 2011
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

#include "webview_reqproc.h"
#include <core/exception.h>
#include <logging/logger.h>
#include <webview/error_reply.h>
#include <fawkes_msgs/WebviewProcessRequest.h>

using namespace fawkes;

/** @class ROSWebviewRequestProcessor "webview_reqproc.h"
 * Convert webview requests to ROS service calls.
 * This request processor calls a ROS service to process the request and
 * produce the reply. This reply is then passed back to webview.
 *
 * This class requires the webview_msgs ROS package to be available.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param nh node handle to create service client handle
 * @param logger logger for log output
 * @param baseurl Base URL this processor is registered for
 * @param srv_name the ROS service name to query for requests
 */
ROSWebviewRequestProcessor::ROSWebviewRequestProcessor(LockPtr<ros::NodeHandle> nh,
                                                       Logger *logger,
                                                       std::string &baseurl,
                                                       std::string &srv_name)
{
  __baseurl  = baseurl;
  __srv_name = srv_name;
  __logcomp  = std::string("ROSWebviewRP[") + srv_name + "]";

  __srv_client = nh->serviceClient<fawkes_msgs::WebviewProcessRequest>(srv_name);
}

/** Destructor. */
ROSWebviewRequestProcessor::~ROSWebviewRequestProcessor()
{
  __srv_client.shutdown();
}

fawkes::WebReply *
ROSWebviewRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  //logger->log_debug(__logcomp.c_str(), "Processing %s", url);

  fawkes_msgs::WebviewProcessRequest srv;
  srv.request.url = request->url();
  //srv.request.method = method;
  //srv.request.version = version;
  //srv.request.upload_data =
  //  std::vector<uint8_t>((uint8_t *)upload_data,
  //			 (uint8_t *)&upload_data[*upload_data_size]);

  if (! __srv_client.exists()) {
    return new WebErrorPageReply(WebReply::HTTP_GONE,
				 "Service %s is no longer available",
				 __srv_name.c_str());

  } else if (__srv_client.call(srv)) {
    if (srv.response.code == WebReply::HTTP_OK) {
      WebReply *r = NULL;
      if (srv.response.wrap_in_page) {
	WebPageReply *pr =
	  new WebPageReply(srv.response.title, srv.response.body);
	pr->set_html_header(srv.response.html_header);
	r = pr;
      } else {
	r = new StaticWebReply(WebReply::HTTP_OK, srv.response.body);
      }

      std::vector<std::string>::iterator h;
      for (h = srv.response.headers.begin(); h != srv.response.headers.end(); ++h)
      {
	try {
	  r->add_header(*h);
	} catch (Exception &e) {
	  // ignore
	}
      }

      return r;
    } else {
      return new WebErrorPageReply((WebReply::Code)srv.response.code,
				   "Execution of service %s failed: %s",
				   __srv_name.c_str(),
				   srv.response.error.c_str());
    }
  } else {
    return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
				 "Execution of service %s failed",
				 __srv_name.c_str());
  }

  // should not happen...
  return NULL;
}
