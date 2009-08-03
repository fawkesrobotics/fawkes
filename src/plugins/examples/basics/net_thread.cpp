
/***************************************************************************
 *  net_thread.cpp - Fawkes Example Plugin Network Thread
 *
 *  Generated: Tue May 08 17:49:56 2006-2007
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

#include <plugins/examples/basics/net_thread.h>
#include <netcomm/fawkes/component_ids.h>

#include <cstdlib>
#include <unistd.h>

using namespace fawkes;

/** @class ExampleNetworkThread net_thread.h <plugins/examples/basics/net_thread.h>
 * Network thread of example plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param name thread name
 */
ExampleNetworkThread::ExampleNetworkThread(const char *name)
  : Thread(name, Thread::OPMODE_WAITFORWAKEUP),
    FawkesNetworkHandler(FAWKES_CID_EXAMPLE_PLUGIN)
{
}


/** Destructor. */
ExampleNetworkThread::~ExampleNetworkThread()
{
}


/** Initialize thread.
 * This method is called just after all aspects have been initialized but before
 * the thread is run. Here we add this thread as a handler to the Fawkes network
 * hub. This cannot happen in the constructor as fnethandler has not been
 * initialized at that time.
 * @see Thread::init()
 * @see Aspects
 */
void
ExampleNetworkThread::init()
{
  fnethub->add_handler( this );
}


void
ExampleNetworkThread::finalize()
{
  logger->log_info("ExampleNetworkThread", "Removing this thread from list of Fawkes network hub handlers");
  fnethub->remove_handler( this );
}


/** Thread loop.
 * Nothing to do here since nobody will every wake us up (we do not have the
 * BlockedTimingAspect nor does any other thread wake us up). This is ok because
 * everything is done in the network handler call.
 *
 * Note that in general incoming messages should be parsed and appropriate
 * actions enqueued. Then in the next loop iteration you process these
 * incoming messages. This is the best way to avoid strange behavior and low
 * latencies in network message handling.
 *
 * As an example for this see the FawkesConfigManager.
 *
 * @see FawkesConfigManager
 */
void
ExampleNetworkThread::loop()
{
}


/** Handle network message.
 * The message is put into the inbound queue and processed in processAfterLoop().
 * @param msg message
 */
void
ExampleNetworkThread::handle_network_message(FawkesNetworkMessage *msg)
{
  if ( msg->payload_size() == sizeof(unsigned int) ) {
    unsigned int *u = (unsigned int *)msg->payload();
    logger->log_info("ExamplePlugin", "Message of type %u with payload u=%u received, sending reply", msg->msgid(), *u);
    unsigned int *ru = (unsigned int *)malloc(sizeof(unsigned int));
    *ru = *u;
    fnethub->send(msg->clid(), FAWKES_CID_EXAMPLE_PLUGIN, msg->msgid(),
		  ru, sizeof(unsigned int));
    // ru is now owned by the generated message and will be automatically freed
  } else {
    logger->log_error("ExamplePlugin", "Message of invalid size received");
  }
}


/** Client connected.
 * Ignored.
 * @param clid client ID
 */
void
ExampleNetworkThread::client_connected(unsigned int clid)
{
  logger->log_info("ExamplePlugin", "Client %u connected", clid);
}


/** Client disconnected.
 * If the client was a subscriber it is removed.
 * @param clid client ID
 */
void
ExampleNetworkThread::client_disconnected(unsigned int clid)
{
  logger->log_info("ExamplePlugin", "Client %u disconnected", clid);
}
