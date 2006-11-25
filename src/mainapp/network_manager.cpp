
/***************************************************************************
 *  network_manager.cpp - Fawkes network manager
 *
 *  Created: Wed Nov 16 00:05:18 2006
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

#include <mainapp/network_manager.h>

#include <netcomm/fawkes/network_thread.h>
#include <netcomm/fawkes/handler.h>
#include <core/threading/thread_manager.h>

/** @class FawkesNetworkManager mainapp/network_manager.h
 * Fawkes Network Manager.
 * This class provides a manager for network connections used in Fawkes.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param thread_manager thread manager that threads shall be registered to
 * @param fawkes_port port to listen on for Fawkes network connections
 */
FawkesNetworkManager::FawkesNetworkManager(ThreadManager *thread_manager,
					   unsigned short int fawkes_port)
{
  this->fawkes_port    = fawkes_port;
  this->thread_manager = thread_manager;
  fawkes_network_thread = new FawkesNetworkThread(thread_manager, fawkes_port);
  thread_manager->add(fawkes_network_thread);
}


/** Destructor. */
FawkesNetworkManager::~FawkesNetworkManager()
{
  thread_manager->remove(fawkes_network_thread);
  delete fawkes_network_thread;
}


/** Add a handler.
 * The handler is registered on the Fawkes network thread and the emitter on the
 * handler is set appropriately.
 * @param handler to add.
 */
void
FawkesNetworkManager::add_handler(FawkesNetworkHandler *handler)
{
  handler->setEmitter(fawkes_network_thread);
  fawkes_network_thread->add_handler(handler);
}


/** Remove handler.
 * @param handler handler to remove
 */
void
FawkesNetworkManager::remove_handler(FawkesNetworkHandler *handler)
{
  fawkes_network_thread->remove_handler(handler);
}
