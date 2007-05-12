
/***************************************************************************
 *  avahi_thread.cpp - Avahi thread
 *
 *  Created: Wed Nov 08 11:19:25 2006
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

#include <netcomm/dns-sd/avahi_thread.h>
#include <netcomm/dns-sd/avahi_service_publisher.h>
#include <netcomm/dns-sd/avahi_browser.h>
#include <netcomm/dns-sd/avahi_resolver.h>

#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>
#include <core/exceptions/software.h>

#include <avahi-common/alternative.h>
#include <avahi-common/simple-watch.h>
#include <avahi-common/malloc.h>
#include <avahi-common/error.h>
#include <avahi-common/timeval.h>

#include <cstddef>

/** @class AvahiThread netcomm/dns-sd/avahi_thread.h
 * Avahi main thread.
 * This thread handles all tasks related to avahi. This is the single
 * interaction point with the Avahi adapter.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
AvahiThread::AvahiThread()
  : Thread("AvahiThread")
{
  service_publisher = new AvahiServicePublisher();
  browser           = new AvahiBrowser();
  _resolver         = new AvahiResolver();
  simple_poll = NULL;
  client = NULL;

  init_mutex        = new Mutex();

  init_mutex->lock();
}


/** Destructor. */
AvahiThread::~AvahiThread()
{
  delete service_publisher;
  delete browser;
  delete _resolver;
  delete init_mutex;

  if ( client )
    avahi_client_free( client );

  if ( simple_poll )
    avahi_simple_poll_free( simple_poll );

}


/** Publish a service.
 * The given service is announced on the network.
 * @param service service to announce
 */
void
AvahiThread::publish(AvahiService *service)
{
  service_publisher->publish(service);
}


/** Watch for a given service.
 * The network is browsed for the given service and the given handler
 * is informed if a service matching the service type joins or leaves
 * the network.
 * @param service_type service type to look for
 * @param h handler to call for events
 */
void
AvahiThread::watch(const char *service_type, AvahiBrowseHandler *h)
{
  browser->add_handler(service_type, h);
}


/** Stop watching for a given service.
 * @param service_type service type to look for
 * @param h handler to no longer call for events
 */
void
AvahiThread::unwatch(const char *service_type, AvahiBrowseHandler *h)
{
  browser->add_handler(service_type, h);
}


/** Get Avahi Resolver.
 * This method returns a pointer to an AvahiResolver.
 * @return initialized resolver
 */
AvahiResolver *
AvahiThread::resolver()
{
  return _resolver;
}


/** Avahi thread loop.
 * The avahi thread calls the simple poll iterate to poll with an infinite
 * timeout. This way the loop blocks until an event occurs.
 */
void
AvahiThread::loop()
{
  if ( ! simple_poll ) {
    // Init
    int error;

    if (! (simple_poll = avahi_simple_poll_new())) {
      throw NullPointerException("Could not create AvahiSimplePoll instance");
    }
    client = avahi_client_new( avahi_simple_poll_get(simple_poll), (AvahiClientFlags)0, //AVAHI_CLIENT_NO_FAIL,
			       AvahiThread::client_callback, this, &error );

    if ( ! client ) {
      avahi_simple_poll_free( simple_poll );
      throw NullPointerException("Could not create avahi client");
    }
  }

  avahi_simple_poll_iterate( simple_poll, -1);
  usleep(0);
}


/** Recover froma broken Avahi connection.
 * This will erase all service browsers and announced service groups
 * and will try to reconnect in the next loop.
 */
void
AvahiThread::recover()
{
  // if someone already gathered the lock we don't care, at least
  // no waiting thread could have aquired it.
  init_mutex->tryLock();

  service_publisher->group_erase();
  browser->erase_browsers();

  if ( client ) {
    avahi_client_free( client );
    client = NULL;
  }

  if ( simple_poll ) {
    avahi_simple_poll_free( simple_poll );
    simple_poll = NULL;
  }

  // next loop() call will now try to reconnnect
}


/* Called whenever the client or server state changes.
 * @param c Avahi client
 * @param state new state
 * @param instance Instance of AvahiThread that triggered the event.
 */
void
AvahiThread::client_callback(AvahiClient *c, AvahiClientState state,
				       void *instance)
{
  AvahiThread *at = static_cast<AvahiThread *>(instance);
  at->client = c;
  at->service_publisher->client = c;
  at->browser->client = c;
  at->_resolver->client = c;

  switch (state) {
  case AVAHI_CLIENT_S_RUNNING:        
    /* The server has startup successfully and registered its host
     * name on the network, so it's time to create our services */
    //printf("(Client): RUNNING\n");
    at->service_publisher->create_services();
    at->browser->create_browsers();
    at->_resolver->set_available( true );
    at->init_unlock();
    break;

  case AVAHI_CLIENT_S_COLLISION:
    //printf("(Client): COLLISION\n");
    /* Let's drop our registered services. When the server is back
     * in AVAHI_SERVER_RUNNING state we will register them
     * again with the new host name. */
    at->service_publisher->group_reset();
    break;
            
  case AVAHI_CLIENT_FAILURE:          
    // Doh!
    //printf("(Client): FAILURE\n");
    at->recover();
    at->_resolver->set_available( false );
    break;

  case AVAHI_CLIENT_CONNECTING:
    //printf("(Client): CONNECTING\n");
    break;

  case AVAHI_CLIENT_S_REGISTERING:
    // Ignored
    //printf("(Client): REGISTERING\n");
    break;
  }
}


/** Unlocks init lock.
 * Only to be called by client_callback().
 */
void
AvahiThread::init_unlock()
{
  init_mutex->unlock();
}


/** Waits for the AvahiThread to be initialized.
 * You can use this if you want to wait until the thread has been
 * fully initialized and may be used. Since the happens in this thread
 * it is in general not immediately ready after start().
 * This will block the calling thread until the AvahiThread has
 * been initialized. This is done by waiting for a release of an
 * initialization mutex.
 */
void
AvahiThread::wait_initialized()
{
  init_mutex->lock();
  init_mutex->unlock();
}
