
/***************************************************************************
 *  net_thread.h - Fawkes Example Plugin Network Thread
 *
 *  Generated: Tue May 08 17:48:23 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __PLUGINS_EXAMPLE_NET_THREAD_H_
#define __PLUGINS_EXAMPLE_NET_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/fawkes_network.h>
#include <netcomm/fawkes/handler.h>

class ExampleNetworkThread : public Thread, public LoggingAspect, public FawkesNetworkAspect,
  public FawkesNetworkHandler
{

 public:
  ExampleNetworkThread(const char *name);
  virtual ~ExampleNetworkThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  /* from FawkesNetworkHandler interface */
  virtual void handle_network_message(FawkesNetworkMessage *msg);
  virtual void client_connected(unsigned int clid);
  virtual void client_disconnected(unsigned int clid);
  virtual void process_after_loop();

};


#endif
