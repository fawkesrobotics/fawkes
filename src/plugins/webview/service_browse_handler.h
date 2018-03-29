
/***************************************************************************
 *  service_browser.h - Webview service browser
 *
 *  Created: Thu Jul 02 17:54:57 2009 (RoboCup 2009, Graz)
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WEBVIEW_SERVICE_BROWSER_H_
#define __PLUGINS_WEBVIEW_SERVICE_BROWSER_H_

#include <netcomm/service_discovery/service.h>
#include <netcomm/service_discovery/browse_handler.h>

#include <map>
#include <string>

namespace fawkes {
  class Logger;
}

class WebviewServiceBrowseHandler : public fawkes::ServiceBrowseHandler
{
 public:
  WebviewServiceBrowseHandler(fawkes::Logger *logger, fawkes::NetworkService *webview_service);
  ~WebviewServiceBrowseHandler();

  virtual void all_for_now();
  virtual void cache_exhausted();
  virtual void browse_failed(const char *name,
			     const char *type,
			     const char *domain);
  virtual void service_added(const char *name,
			     const char *type,
			     const char *domain,
			     const char *host_name,
			     const char *interface,
			     const struct sockaddr *addr,
			     const socklen_t addr_size,
			     uint16_t port,
			     std::list<std::string> &txt,
			     int flags);
  virtual void service_removed(const char *name,
			       const char *type,
			       const char *domain);

  /** A map of services.
   * Maps service names to NetworkService instances describing the service
   * in more detail. */
  typedef std::map<std::string, fawkes::NetworkService *> ServiceList;

  ServiceList &  service_list();

 private:
  fawkes::Logger         *logger_;
  fawkes::NetworkService *webview_service_;
  ServiceList             service_list_;
};

#endif
