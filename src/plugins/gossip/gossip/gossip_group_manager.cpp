
/***************************************************************************
 *  gossip_group_manager.cpp - Fawkes Gossip group manager
 *
 *  Created: Fri Feb 28 16:55:24 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <plugins/gossip/gossip/gossip_group_manager.h>
#include <plugins/gossip/gossip/gossip_group.h>
#include <core/exception.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class GossipGroupConfiguration <plugins/gossip/gossip/gossip_group_manager.h>
 * Group configuration for initial groups.
 */


/** Constructor. */
GossipGroupConfiguration::GossipGroupConfiguration()
  : send_port(0), recv_port(0)
{
}

/** Constructor.
 * @param name name of the group
 * @param broadcast_address IPv4 address to broadcast to
 * @param broadcast_port UDP port to listen on for the group
 */
GossipGroupConfiguration::GossipGroupConfiguration(std::string &name,
						   std::string &broadcast_address,
						   unsigned short broadcast_port)
  : name(name), broadcast_addr(broadcast_address),
    send_port(broadcast_port), recv_port(broadcast_port)
{
}

/** Constructor.
 * @param name name of the group
 * @param broadcast_address IPv4 address to broadcast to
 * @param send_port UDP port to send messages to
 * @param recv_port UDP port to listen on for the group
 */
GossipGroupConfiguration::GossipGroupConfiguration(std::string &name,
						   std::string &broadcast_address,
						   unsigned short send_port,
						   unsigned short recv_port)
  : name(name), broadcast_addr(broadcast_address),
    send_port(send_port), recv_port(recv_port)
{
}

/** Copy contructor.
 * @param c group configuration to copy
 */
GossipGroupConfiguration::GossipGroupConfiguration(const GossipGroupConfiguration &c)
  : name(c.name), broadcast_addr(c.broadcast_addr), send_port(c.send_port),
    recv_port(c.recv_port), crypto_key(c.crypto_key), crypto_cipher(c.crypto_cipher)
{
}


/** @class GossipGroupManager <plugins/gossip/gossip/gossip_group_manager.h>
 * Abstract class for a Gossip group manager.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param service_name service name to announce for each group we join, this
 * must be unique in the group and should identify the robot
 * @param service_publisher service discovery publisher to announce groups
 * @param initial_groups initial group configurations to join
 */
GossipGroupManager::GossipGroupManager(std::string &service_name,
				       ServicePublisher *service_publisher,
				       std::map<std::string, GossipGroupConfiguration> &initial_groups)
  : service_name_(service_name), service_publisher_(service_publisher)
{
#if defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6))
  for (auto g : initial_groups) {
    create_group(g.second);
  }
#else
  std::map<std::string, GossipGroupConfiguration>::iterator g;
  for (g = initial_groups.begin(); g != initial_groups.end(); ++g) {
    create_group(g->second);
  }
#endif
}


/** Destructor. */
GossipGroupManager::~GossipGroupManager()
{
}


/** Join a group.
 * @param name the name of the group to join
 * @return a shared object to communicate with the group.
 */
fawkes::RefPtr<fawkes::GossipGroup>
GossipGroupManager::join_group(const std::string &name)
{
  if (groups_.find(name) == groups_.end()) {
    // try to join group
  }

  if (groups_.find(name) != groups_.end()) {
    return groups_[name];
  } else {
    // still not registered -> fail
    throw Exception("Cannot register to group %s", name.c_str());
  }
}

/** Leave a gossip group.
 * @param group the gossip group to leave, the handle becomes invalid after this call.
 */
void
GossipGroupManager::leave_group(fawkes::RefPtr<fawkes::GossipGroup> &group)
{
  group.reset();

  /*
  if (groups_.find(name) != groups_.end()) {
    if (groups_[name].use_count() == 1) {
      // only us, leave?
    }
  }
  */
}


void
GossipGroupManager::create_group(GossipGroupConfiguration &gc)
{
  if (gc.send_port == gc.recv_port) {
    groups_[gc.name] = new GossipGroup(gc.name, service_name_,
				       gc.broadcast_addr, gc.recv_port,
				       service_publisher_,
				       gc.crypto_key, gc.crypto_cipher);
  } else {
    groups_[gc.name] = new GossipGroup(gc.name, service_name_,
				       gc.broadcast_addr, gc.send_port,
				       gc.recv_port, service_publisher_,
				       gc.crypto_key, gc.crypto_cipher);
  }
}


} // end namespace fawkes
