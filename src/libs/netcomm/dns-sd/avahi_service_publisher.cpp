
/***************************************************************************
 *  avahi_service_publisher.cpp - publish services via avahi
 *
 *  Created: Tue Nov 07 16:42:00 2006
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

#include <netcomm/dns-sd/avahi_service_publisher.h>
#include <core/exceptions/software.h>

#include <avahi-common/alternative.h>
#include <avahi-common/simple-watch.h>
#include <avahi-common/malloc.h>
#include <avahi-common/error.h>
#include <avahi-common/timeval.h>

#include <cstddef>

/** @class AvahiServicePublisher netcomm/dns-sd/avahi_service_publisher.h
 * Announce services on the network.
 * This class gives access to Avahi's possibility to announce services on
 * the network.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor. */
AvahiServicePublisher::AvahiServicePublisher()
{
  client = NULL;
  group = NULL;
  services.clear();
}


/** Destructor. */
AvahiServicePublisher::~AvahiServicePublisher()
{
  for (std::list<NetworkService *>::iterator i = services.begin(); i != services.end(); ++i) {
    delete *i;
  }
  services.clear();

  group_erase();
}


/** Publish service.
 * @param service service to publish.
 */
void
AvahiServicePublisher::publish(NetworkService *service)
{
  services.push_back( service );
  if ( group ) {
    // create service with update flag
    int ret = 0;
    // only IPv4 for now
    AvahiStringList *al = NULL;
    std::list<std::string> l = service->txt();
    for (std::list<std::string>::iterator i = l.begin(); i != l.end(); ++i) {
      al = avahi_string_list_add(al, (*i).c_str());
    }
    if ( (ret = avahi_entry_group_add_service_strlst(group, AVAHI_IF_UNSPEC, AVAHI_PROTO_INET,
						     AVAHI_PUBLISH_UPDATE,
						     service->name(), service->type(),
						     service->domain(), service->host(),
						     service->port(), al)) < 0) {
      avahi_string_list_free(al);
      throw Exception("Adding Avahi services failed");
    }
    avahi_string_list_free(al);
  }
}


/** Create services. */
void
AvahiServicePublisher::create_services()
{
  if ( ! client )  return;
  if ( group ) return;
  if ( services.size() == 0) return;

  if ( ! (group = avahi_entry_group_new(client,
					AvahiServicePublisher::entry_group_callback,
					this))) {
    throw NullPointerException("Cannot create service group");
  }

  int ret = 0;

  for (std::list<NetworkService *>::iterator i = services.begin(); i != services.end(); ++i) {
    // only IPv4 for now
    AvahiStringList *al = NULL;
    std::list<std::string> l = (*i)->txt();
    for (std::list<std::string>::iterator j = l.begin(); j != l.end(); ++j) {
      al = avahi_string_list_add(al, (*j).c_str());
    }
    if ( (ret = avahi_entry_group_add_service_strlst(group, AVAHI_IF_UNSPEC, AVAHI_PROTO_INET,
						     AVAHI_PUBLISH_USE_MULTICAST,
						     (*i)->name(), (*i)->type(),
						     (*i)->domain(), (*i)->host(),
						     (*i)->port(), al)) < 0) {
      avahi_string_list_free(al);
      throw Exception("Adding Avahi services failed");
    }
    avahi_string_list_free(al);
  }

  /* Tell the server to register the service */
  if ((ret = avahi_entry_group_commit(group)) < 0) {
    throw Exception("Registering Avahi services failed");
  }
}


/** Drop our registered services.
 * When the server is back in AVAHI_SERVER_RUNNING state we will register them
 * again with the new host name (triggered by AvahiThread).
 */
void
AvahiServicePublisher::group_reset()
{
  if ( group )
    avahi_entry_group_reset(group);
}


/** Erase service group. */
void
AvahiServicePublisher::group_erase()
{
  if ( group ) {
    avahi_entry_group_reset( group );
    avahi_entry_group_free( group );
    group = NULL;
  }
}


/** Called if there was a name collision. */
void
AvahiServicePublisher::name_collision()
{
  // give all services a new name, can't decide which service caused the problem

  for (std::list<NetworkService *>::iterator i = services.begin(); i != services.end(); ++i) {
    char *n;
    /* A service name collision happened. Let's pick a new name */
    n = avahi_alternative_service_name((*i)->name());
    (*i)->set_name(n);
    avahi_free(n);
  }

  create_services();
}


/** Callback for Avahi.
 * @param g entry group
 * @param state new state
 * @param instance instance of AvahiServicePublisher that triggered the event.
 */
void
AvahiServicePublisher::entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state,
					    void *instance)
{
  AvahiServicePublisher *asp = static_cast<AvahiServicePublisher *>(instance);

  switch (state) {
  case AVAHI_ENTRY_GROUP_ESTABLISHED :
    /* The entry group has been established successfully */
    //fprintf(stderr, "Service '%s' successfully established.\n", name);
    break;

  case AVAHI_ENTRY_GROUP_COLLISION : {
    asp->name_collision();
    break;
  }
  
  case AVAHI_ENTRY_GROUP_FAILURE :
    /* Some kind of failure happened while we were registering our services */
    break;

  case AVAHI_ENTRY_GROUP_UNCOMMITED:
  case AVAHI_ENTRY_GROUP_REGISTERING:
    ;
  }
}
