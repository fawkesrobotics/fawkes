
/***************************************************************************
 *  service_browse_handler.cpp - Webview service browser
 *
 *  Created: Thu Jul 02 18:00:20 2009 (RoboCup 2009, Graz)
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

#include "service_browse_handler.h"

#include <logging/logger.h>

using namespace fawkes;

/** @class WebviewServiceBrowseHandler "service_browse_handler.h"
 * Browse handler to detect other Webview instances on the network.
 * This browse handler is used to compile a list of other webview instances
 * on the local network. It is used to show a list of hosts in the footer of
 * webview pages.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger for informational logging
 * @param webview_service service of our own service as it was announced on the
 * network, used to filter it out from the list of services.
 */
WebviewServiceBrowseHandler::WebviewServiceBrowseHandler(fawkes::Logger *logger,
							 fawkes::NetworkService *webview_service)
{
  logger_          = logger;
  webview_service_ = webview_service;
}

WebviewServiceBrowseHandler::~WebviewServiceBrowseHandler()
{
  for (ServiceList::iterator s = service_list_.begin(); s != service_list_.end(); ++s) {
    delete s->second;
  }
  service_list_.clear();
}

void
WebviewServiceBrowseHandler::all_for_now()
{
  //logger_->log_debug("WebviewServiceBrowseHandler", "All for now");
}


void
WebviewServiceBrowseHandler::cache_exhausted()
{
  //logger_->log_debug("WebviewServiceBrowseHandler", "Cache exhausted");
}


void
WebviewServiceBrowseHandler::browse_failed(const char *name,
			     const char *type,
			     const char *domain)
{
  logger_->log_warn("WebviewServiceBrowseHandler", "Browsing for %s.%s in domain %s failed",
		     name, type, domain);
}


void
WebviewServiceBrowseHandler::service_added(const char *name,
					   const char *type,
					   const char *domain,
					   const char *host_name,
					   const char *interface,
					   const struct sockaddr *addr,
					   const socklen_t addr_size,
					   uint16_t port,
					   std::list<std::string> &txt,
					   int flags)
{
  if (service_list_.find(name) != service_list_.end()) {
    delete service_list_[name];
    service_list_.erase(name);
  }
  // Check for fawkesver txt record
  for (std::list<std::string>::iterator i = txt.begin(); i != txt.end(); ++i) {
    std::string::size_type eqind = i->find("=");
    if (eqind != std::string::npos) {
      std::string key = i->substr(0, eqind);
      std::string val = i->substr(eqind + 1);
      if (key == "fawkesver") {
	NetworkService *s = new NetworkService(name, type, domain, host_name, port,
					       addr, addr_size, txt);

	if (! (*s == *webview_service_)) {
	  logger_->log_debug("WebviewServiceBrowseHandler", "Service %s.%s on %s:%u added",
			      name, type, host_name, port);
	  service_list_[name] = s;
	} else {
	  delete s;
	}
	break;
      }
    }
  }
}


void
WebviewServiceBrowseHandler::service_removed(const char *name,
					     const char *type,
					     const char *domain)
{
  if (service_list_.find(name) != service_list_.end()) {
    delete service_list_[name];
    service_list_.erase(name);
  }
  logger_->log_debug("WebviewServiceBrowseHandler", "Service %s.%s has been removed",
		      name, type);
}


/** Get the service list.
 * @return a list of services found on the network.
 */
WebviewServiceBrowseHandler::ServiceList &
WebviewServiceBrowseHandler::service_list()
{
  return service_list_;
}
