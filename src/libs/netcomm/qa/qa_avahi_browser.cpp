
/***************************************************************************
 *  qa_avahi_browser.cpp - QA for AvahiBrowser
 *
 *  Created: Fri Nov 10 10:19:39 2006 (recreated after stupid delete)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

/// @cond QA

#include <netcomm/dns-sd/avahi_thread.h>
#include <netcomm/dns-sd/avahi_browse_handler.h>

#include <core/exception.h>
#include <utils/system/signal.h>

#include <stdio.h>

class QAAvahiBrowserMain : public SignalHandler, public AvahiBrowseHandler
{
 public:
  QAAvahiBrowserMain()
  {
    at = new AvahiThread();;
    at->watch("_fawkes._udp", this);
  }

  ~QAAvahiBrowserMain()
  {
    delete at;
  }

  void handle_signal(int signum)
  {
    at->cancel();
  }

  void run()
  {
    at->start();
    at->join();
  }

  virtual void all_for_now()
  {
    printf("ALL_FOR_NOW\n");
  }

  virtual void cache_exhausted()
  {
    printf("CACHE_EXHAUSTED\n");
  }

  virtual void failed(const char *name,
		      const char *type,
		      const char *domain)
  {
    printf("FAILED: name=%s  type=%s  domain=%s\n", name, type, domain);
  }

  virtual void service_added(const char *name,
			     const char *type,
			     const char *domain,
			     const char *host_name,
			     const AvahiAddress *address,
			     uint16_t port,
			     std::list<std::string> &txt,
			     AvahiLookupResultFlags flags
			     )
  {
    printf("SERVICE_ADDED: name=%s  type=%s  domain=%s  hostname=%s\n",
	   name, type, domain, host_name);
  }

  virtual void service_removed(const char *name,
			       const char *type,
			       const char *domain)
  {
    printf("SERVICE_REMOVED: name=%s  type=%s  domain=%s\n", name, type, domain);
  }

 private:
  AvahiThread *at;

};

int
main(int argc, char *argv)
{
  try {

    QAAvahiBrowserMain m;
    SignalManager::register_handler(SIGINT, &m);

    m.run();

  } catch (Exception &e) {
    e.printTrace();
  }

  SignalManager::finalize();
}

/// @endcond
