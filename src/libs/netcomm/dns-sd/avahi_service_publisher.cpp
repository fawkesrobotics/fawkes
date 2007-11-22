
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
  __services.clear();

  __published = false;
}


/** Destructor. */
AvahiServicePublisher::~AvahiServicePublisher()
{
  erase_groups();
  for (__sit = __services.begin(); __sit != __services.end(); ++__sit) {
    delete (*__sit).first;
  }
  __services.clear();
}


/** Publish service.
 * @param service service to publish.
 */
void
AvahiServicePublisher::publish(NetworkService *service)
{
  for (__sit = __services.begin(); __sit != __services.end(); ++__sit) {
    if ( *((*__sit).first) == service ) {
      throw Exception("Service already registered");
    }
  }
  __services[service] = NULL;

  create_services();
}


void
AvahiServicePublisher::unpublish(NetworkService *service)
{
  for (__sit = __services.begin(); __sit != __services.end(); ++__sit) {
    if ( *((*__sit).first) == service ) {
      group_erase((*__sit).second);
      __services.erase(__sit);
      break;
    }
  }
}


void
AvahiServicePublisher::set_published(bool published)
{
  __published = published;
}


/** Create services. */
void
AvahiServicePublisher::create_services()
{
  // the following errors are non-fatal, they can happen since Avahi is started
  // asynchronously, just ignore them by bailing out
  if ( ! client )  return;
  if ( __services.size() == 0) return;
  if ( ! __published) return;

  int ret = 0;

  for ( __sit = __services.begin(); __sit != __services.end(); ++__sit) {

    if ( (*__sit).second != NULL ) {
      continue;
    }

    AvahiEntryGroup *group;
    if ( ! (group = avahi_entry_group_new(client,
					  AvahiServicePublisher::entry_group_callback,
					  this))) {
      throw NullPointerException("Cannot create service group");
    }

    (*__sit).second = group;

    // only IPv4 for now
    AvahiStringList *al = NULL;
    const std::list<std::string> &l = (*__sit).first->txt();
    for (std::list<std::string>::const_iterator j = l.begin(); j != l.end(); ++j) {
      al = avahi_string_list_add(al, (*j).c_str());
    }
    if ( (ret = avahi_entry_group_add_service_strlst(group, AVAHI_IF_UNSPEC, AVAHI_PROTO_INET,
						     AVAHI_PUBLISH_USE_MULTICAST,
						     (*__sit).first->name(), (*__sit).first->type(),
						     (*__sit).first->domain(), (*__sit).first->host(),
						     (*__sit).first->port(), al)) < 0) {
      avahi_string_list_free(al);
      throw Exception("Adding Avahi services failed");
    }
    avahi_string_list_free(al);

    /* Tell the server to register the service */
    if ((ret = avahi_entry_group_commit(group)) < 0) {
      throw Exception("Registering Avahi services failed");
    }
  }

}


/** Drop our registered services.
 * When the server is back in AVAHI_SERVER_RUNNING state we will register them
 * again with the new host name (triggered by AvahiThread).
 */
void
AvahiServicePublisher::group_reset(AvahiEntryGroup *g)
{
  if ( g ) {
    avahi_entry_group_reset(g);
  }
}


/** Erase service group. */
void
AvahiServicePublisher::group_erase(AvahiEntryGroup *g)
{
  if ( g ) {
    avahi_entry_group_reset( g );
    avahi_entry_group_free( g );
  }
}


void
AvahiServicePublisher::erase_groups()
{
  for (__sit = __services.begin(); __sit != __services.end(); ++__sit) {
    group_erase((*__sit).second);
    (*__sit).second = NULL;
  }
}


void
AvahiServicePublisher::reset_groups()
{
  for (__sit = __services.begin(); __sit != __services.end(); ++__sit) {
    group_reset((*__sit).second);
  }
}


/** Called if there was a name collision. */
void
AvahiServicePublisher::name_collision(AvahiEntryGroup *g)
{
  for (__sit = __services.begin(); __sit != __services.end(); ++__sit) {
    if ( (*__sit).second == g ) {
      char *n;
      /* A service name collision happened. Let's pick a new name */
      n = avahi_alternative_service_name((*__sit).first->name());
      (*__sit).first->set_name(n);
      avahi_free(n);
    }
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
    asp->name_collision(g);
    break;
  }
  
  case AVAHI_ENTRY_GROUP_FAILURE :
    /* Some kind of failure happened while we were registering our services */
    break;

  case AVAHI_ENTRY_GROUP_UNCOMMITED:
  case AVAHI_ENTRY_GROUP_REGISTERING:
    break;
  }
}
