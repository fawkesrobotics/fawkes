
/***************************************************************************
 *  net_thread.h - Fawkes Example Plugin Network Thread
 *
 *  Generated: Tue May 08 17:48:23 2007
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_EXAMPLE_NET_THREAD_H_
#define __PLUGINS_EXAMPLE_NET_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/fawkes_network.h>
#include <netcomm/fawkes/handler.h>

class ExampleNetworkThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::FawkesNetworkAspect,
  public fawkes::FawkesNetworkHandler
{

 public:
  ExampleNetworkThread(const char *name);
  virtual ~ExampleNetworkThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  /* from FawkesNetworkHandler interface */
  virtual void handle_network_message(fawkes::FawkesNetworkMessage *msg);
  virtual void client_connected(unsigned int clid);
  virtual void client_disconnected(unsigned int clid);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

};


#endif
