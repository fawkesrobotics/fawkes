
/***************************************************************************
 *  gossip_group_manager.h - Fawkes Gossip group manager
 *
 *  Created: Fri Feb 28 16:54:34 2014
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

#ifndef __PLUGINS_GOSSIP_GOSSIP_GOSSIP_GROUP_MANAGER_H_
#define __PLUGINS_GOSSIP_GOSSIP_GOSSIP_GROUP_MANAGER_H_

#include <core/utils/refptr.h>

#include <map>
#include <list>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class GossipGroup;
class ServicePublisher;
class GossipAspectIniFin;

class GossipGroupConfiguration
{
 public:
  GossipGroupConfiguration();
  GossipGroupConfiguration(const GossipGroupConfiguration &c);
  GossipGroupConfiguration(std::string &name,
			   std::string &broadcast_address,
			   unsigned short broadcast_port);
  GossipGroupConfiguration(std::string &name,
			   std::string &broadcast_address,
			   unsigned short send_port, unsigned short recv_port);

  std::string    name;			///< name of the group
  std::string    broadcast_addr;	///< Broadcast IP Addr
  unsigned short send_port;		///< UDP port to send messages to
  unsigned short recv_port;		///< UDP port to list on for messages
  std::string    crypto_key;		///< encryption key
  std::string    crypto_cipher;		///< encryption cipher
};

class GossipGroupManager
{
  friend GossipAspectIniFin;
 public:
  GossipGroupManager(std::string &service_name, ServicePublisher *service_publisher,
		     std::map<std::string, GossipGroupConfiguration> &initial_groups);
  virtual ~GossipGroupManager();

  virtual RefPtr<GossipGroup> join_group(const std::string &name);
  virtual void                leave_group(RefPtr<GossipGroup> &group);

 private:
  void create_group(GossipGroupConfiguration &gc);

 private:
  std::string                                  service_name_;
  ServicePublisher                            *service_publisher_;
  std::map<std::string, RefPtr<GossipGroup> >  groups_;
};

} // end namespace fawkes

#endif

