
/***************************************************************************
 *  qa_avahi_publisher.cpp - QA for AvahiServicePublisher
 *
 *  Generated: Tue Nov 07 18:41:46 2006
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

#include <utils/system/signal.h>

class QAAvahiPublisherMain : public SignalHandler
{
 public:
  QAAvahiPublisherMain()
  {
    NetworkService *as = new NetworkService("Fawkes QA", "_fawkes._udp", 1910);
    at = new AvahiThread();
    at->publish_service(as);
    delete as;
  }

  ~QAAvahiPublisherMain()
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

 private:
  AvahiThread *at;

};

int
main(int argc, char **argv)
{
  QAAvahiPublisherMain m;
  SignalManager::register_handler(SIGINT, &m);

  m.run();

  SignalManager::finalize();
}

/// @endcond
