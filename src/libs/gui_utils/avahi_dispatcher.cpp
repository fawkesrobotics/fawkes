
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

#include <gui_utils/avahi_dispatcher.h>
#include <netcomm/service_discovery/service.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class AvahiDispatcher <gui_utils/avahi_dispatcher.h>
 * Avahi dispatcher.
 * This class facilitates a dispatcher that is used to get events generated
 * by an AvahiThread into the main loop of a Gtk application.
 * @author Tim Niemueller
 */

/** Constructor. */
AvahiDispatcher::AvahiDispatcher()
{
  __dispatcher_all_for_now.connect(sigc::mem_fun(*this, &AvahiDispatcher::on_all_for_now));
  __dispatcher_cache_exhausted.connect(sigc::mem_fun(*this, &AvahiDispatcher::on_cache_exhausted));
  __dispatcher_browse_failed.connect(sigc::mem_fun(*this, &AvahiDispatcher::on_browse_failed));
  __dispatcher_service_added.connect(sigc::mem_fun(*this, &AvahiDispatcher::on_service_added));
  __dispatcher_service_removed.connect(sigc::mem_fun(*this, &AvahiDispatcher::on_service_removed));
}


/** Get "all for now" signal.
 * @return "all for now" signal
 */
sigc::signal<void>
AvahiDispatcher::signal_all_for_now()
{
  return __signal_all_for_now;
}


/** Get "cache exhausted" signal.
 * @return "cache exhausted" signal
 */
sigc::signal<void>
AvahiDispatcher::signal_cache_exhausted()
{
  return __signal_cache_exhausted;
}


/** Get "browse failed" signal.
 * @return "browse failed" signal
 */
sigc::signal<void>
AvahiDispatcher::signal_browse_failed()
{
  return __signal_browse_failed;
}


/** Get "service added" signal.
 * @return "service added" signal
 */
sigc::signal<void, NetworkService *>
AvahiDispatcher::signal_service_added()
{
  return __signal_service_added;
}


/** Get "service remove" signal.
 * @return "service remove" signal
 */
sigc::signal<void, NetworkService *>
AvahiDispatcher::signal_service_removed()
{
  return __signal_service_removed;
}


void
AvahiDispatcher::all_for_now()
{
  __dispatcher_all_for_now();
}


void
AvahiDispatcher::cache_exhausted()
{
  __dispatcher_cache_exhausted();
}


void
AvahiDispatcher::browse_failed(const char *name,
			       const char *type,
			       const char *domain)
{
  __dispatcher_browse_failed();
}


void
AvahiDispatcher::service_added(const char *name,
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
  NetworkService *s = new NetworkService(name, type, domain, host_name, port,
					 addr, addr_size, txt);
  __queue_service_added.push_locked(s);
  __dispatcher_service_added();
}


void
AvahiDispatcher::service_removed(const char *name,
				 const char *type,
				 const char *domain)
{
  NetworkService *s = new NetworkService(name, type, domain);
  __queue_service_removed.push_locked(s);
  __dispatcher_service_removed();
}


void
AvahiDispatcher::on_all_for_now()
{
  __signal_all_for_now.emit();
}

void
AvahiDispatcher::on_cache_exhausted()
{
  __signal_cache_exhausted.emit();
}

void
AvahiDispatcher::on_browse_failed()
{
  __signal_browse_failed.emit();
}

void
AvahiDispatcher::on_service_added()
{
  __queue_service_added.lock();
  while (! __queue_service_added.empty()) {
    NetworkService *s = __queue_service_added.front();
    __signal_service_added.emit(s);
    delete s;
    __queue_service_added.pop();
  }
  __queue_service_added.unlock();
}

void
AvahiDispatcher::on_service_removed()
{
  __queue_service_removed.lock();
  while (! __queue_service_removed.empty()) {
    NetworkService *s = __queue_service_removed.front();
    __signal_service_removed.emit(s);
    delete s;
    __queue_service_removed.pop();
  }
  __queue_service_removed.unlock();
}

} // end namespace fawkes
