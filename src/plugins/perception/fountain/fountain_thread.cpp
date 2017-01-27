
/***************************************************************************
 *  fountain_thread.h - Fountain main thread
 *
 *  Created: Fri Nov 16 11:22:30 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include "fountain_thread.h"

#include <core/exceptions/software.h>
#include <fvutils/net/fuse_server.h>

#include <string>
#include <cstdio>

using namespace fawkes;
using namespace firevision;

/** @class FountainThread "fountain_thread.h"
 * Fountain main thread.
 * @author Tim Niemueller
 */

/** Constructor. */
FountainThread::FountainThread()
  : Thread("FountainThread", OPMODE_WAITFORWAKEUP)
{
  __fuse_server = NULL;
  __service = NULL;
}


/** Destructor. */
FountainThread::~FountainThread()
{
  if ( __fuse_server ) {
    thread_collector->remove(__fuse_server);
    delete __fuse_server;
    __fuse_server = NULL;
  }
  delete __service;
  __service = NULL;
}


void
FountainThread::init()
{
  // Start FUSE server
  unsigned int port = 0;
  try {
    port = config->get_uint("/firevision/fountain/tcp_port");
    if ( port > 0xFFFF ) {
      throw OutOfBoundsException("Network port out of bounds", port, 0, 0xFFFF);
    }

    bool enable_ipv4 = true;
    bool enable_ipv6 = true;
    std::string listen_ipv4;
    std::string listen_ipv6;

    try {
	    enable_ipv4 = config->get_bool("/network/ipv4/enable");
    } catch (Exception &e) {}  // ignore, we stick with the default
    try {
	    enable_ipv6 = config->get_bool("/network/ipv6/enable");
    } catch (Exception &e) {}  // ignore, we stick with the default

    try {
	    listen_ipv4 = config->get_string("/network/ipv4/listen");
    } catch (Exception &e) {}  // ignore, we stick with the default
    try {
	    listen_ipv6 = config->get_string("/network/ipv6/listen");
    } catch (Exception &e) {}  // ignore, we stick with the default

    __fuse_server = new FuseServer(enable_ipv4, enable_ipv6,
                                   listen_ipv4, listen_ipv6,
                                   port, thread_collector);
    thread_collector->add(__fuse_server);
  } catch (Exception &e) {
    e.print_trace();
    throw;
  }

  // Announce service
  std::string sname = "Fountain on ";
  sname += nnresolver->short_hostname();
  __service = new NetworkService(sname.c_str(), "_fountain._tcp", port);
  service_publisher->publish_service(__service);
}


void
FountainThread::finalize()
{
  service_publisher->unpublish_service(__service);

  thread_collector->remove(__fuse_server);
  delete __fuse_server;
  __fuse_server = NULL;
  delete __service;
  __service = NULL;
}


void
FountainThread::loop()
{
  // do nothing, but implement to not exit
  printf("Sucker Loop\n");
}
