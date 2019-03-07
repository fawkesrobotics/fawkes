
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

/** @class AvahiDispatcher <gui_utils/avahi_dispatcher.h>
 * Avahi dispatcher.
 * This class facilitates a dispatcher that is used to get events generated
 * by an AvahiThread into the main loop of a Gtk application.
 * @author Tim Niemueller
 */

/** Constructor. */
AvahiDispatcher::AvahiDispatcher()
{
	dispatcher_all_for_now_.connect(sigc::mem_fun(*this, &AvahiDispatcher::on_all_for_now));
	dispatcher_cache_exhausted_.connect(sigc::mem_fun(*this, &AvahiDispatcher::on_cache_exhausted));
	dispatcher_browse_failed_.connect(sigc::mem_fun(*this, &AvahiDispatcher::on_browse_failed));
	dispatcher_service_added_.connect(sigc::mem_fun(*this, &AvahiDispatcher::on_service_added));
	dispatcher_service_removed_.connect(sigc::mem_fun(*this, &AvahiDispatcher::on_service_removed));
}

/** Get "all for now" signal.
 * @return "all for now" signal
 */
sigc::signal<void>
AvahiDispatcher::signal_all_for_now()
{
	return signal_all_for_now_;
}

/** Get "cache exhausted" signal.
 * @return "cache exhausted" signal
 */
sigc::signal<void>
AvahiDispatcher::signal_cache_exhausted()
{
	return signal_cache_exhausted_;
}

/** Get "browse failed" signal.
 * @return "browse failed" signal
 */
sigc::signal<void>
AvahiDispatcher::signal_browse_failed()
{
	return signal_browse_failed_;
}

/** Get "service added" signal.
 * @return "service added" signal
 */
sigc::signal<void, NetworkService *>
AvahiDispatcher::signal_service_added()
{
	return signal_service_added_;
}

/** Get "service remove" signal.
 * @return "service remove" signal
 */
sigc::signal<void, NetworkService *>
AvahiDispatcher::signal_service_removed()
{
	return signal_service_removed_;
}

void
AvahiDispatcher::all_for_now()
{
	dispatcher_all_for_now_();
}

void
AvahiDispatcher::cache_exhausted()
{
	dispatcher_cache_exhausted_();
}

void
AvahiDispatcher::browse_failed(const char *name, const char *type, const char *domain)
{
	dispatcher_browse_failed_();
}

void
AvahiDispatcher::service_added(const char *            name,
                               const char *            type,
                               const char *            domain,
                               const char *            host_name,
                               const char *            interface,
                               const struct sockaddr * addr,
                               const socklen_t         addr_size,
                               uint16_t                port,
                               std::list<std::string> &txt,
                               int                     flags)
{
	NetworkService *s = new NetworkService(name, type, domain, host_name, port, addr, addr_size, txt);
	queue_service_added_.push_locked(s);
	dispatcher_service_added_();
}

void
AvahiDispatcher::service_removed(const char *name, const char *type, const char *domain)
{
	NetworkService *s = new NetworkService(name, type, domain);
	queue_service_removed_.push_locked(s);
	dispatcher_service_removed_();
}

void
AvahiDispatcher::on_all_for_now()
{
	signal_all_for_now_.emit();
}

void
AvahiDispatcher::on_cache_exhausted()
{
	signal_cache_exhausted_.emit();
}

void
AvahiDispatcher::on_browse_failed()
{
	signal_browse_failed_.emit();
}

void
AvahiDispatcher::on_service_added()
{
	queue_service_added_.lock();
	while (!queue_service_added_.empty()) {
		NetworkService *s = queue_service_added_.front();
		signal_service_added_.emit(s);
		delete s;
		queue_service_added_.pop();
	}
	queue_service_added_.unlock();
}

void
AvahiDispatcher::on_service_removed()
{
	queue_service_removed_.lock();
	while (!queue_service_removed_.empty()) {
		NetworkService *s = queue_service_removed_.front();
		signal_service_removed_.emit(s);
		delete s;
		queue_service_removed_.pop();
	}
	queue_service_removed_.unlock();
}

} // end namespace fawkes
