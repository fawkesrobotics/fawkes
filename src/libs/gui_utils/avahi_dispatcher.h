
/***************************************************************************
 *  avahi_dispatcher.h - Avahi browser handler and dispatcher
 *
 *  Created: Wed Nov 05 11:30:13 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __LIBS_GUI_UTILS_AVAHI_DISPATCHER_H_
#define __LIBS_GUI_UTILS_AVAHI_DISPATCHER_H_

#include <cstddef>
#include <glibmm/dispatcher.h>
#include <netcomm/service_discovery/browse_handler.h>
#include <core/utils/lock_queue.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif
class AvahiThread;
class NetworkService;

class AvahiDispatcher
: public ServiceBrowseHandler
{
 public:
  AvahiDispatcher();

  sigc::signal<void>                    signal_all_for_now();
  sigc::signal<void>                    signal_cache_exhausted();
  sigc::signal<void>                    signal_browse_failed();
  sigc::signal<void, NetworkService *>  signal_service_added();
  sigc::signal<void, NetworkService *>  signal_service_removed();


  // from ServiceBrowseHandler
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


 private:
  virtual void on_all_for_now();
  virtual void on_cache_exhausted();
  virtual void on_browse_failed();
  virtual void on_service_added();
  virtual void on_service_removed();

 private:
  Glib::Dispatcher                     __dispatcher_all_for_now;
  Glib::Dispatcher                     __dispatcher_cache_exhausted;
  Glib::Dispatcher                     __dispatcher_browse_failed;
  Glib::Dispatcher                     __dispatcher_service_added;
  Glib::Dispatcher                     __dispatcher_service_removed;

  sigc::signal<void>                   __signal_all_for_now;
  sigc::signal<void>                   __signal_cache_exhausted;
  sigc::signal<void>                   __signal_browse_failed;
  sigc::signal<void, NetworkService *> __signal_service_added;
  sigc::signal<void, NetworkService *> __signal_service_removed;

  LockQueue<NetworkService *>          __queue_service_added;
  LockQueue<NetworkService *>          __queue_service_removed;
};

} // end namespace fawkes

#endif
