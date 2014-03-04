
/***************************************************************************
 *  gossip_group.h - Robot Group Communication - Gossip Group
 *
 *  Created: Tue Mar 04 10:56:48 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_GOSSIP_GOSSIP_GOSSIP_GROUP_H_
#define __PLUGINS_GOSSIP_GOSSIP_GOSSIP_GROUP_H_

#include <google/protobuf/message.h>
#include <memory>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ServicePublisher;
class NetworkService;

class GossipGroup {
  friend class GossipGroupManager;
 public:
  ~GossipGroup();

  void send(std::string &peer,
	    google::protobuf::Message &m);

  void broadcast(google::protobuf::Message &m);

  /** Get group name.
   * @return group name. */
  const std::string &  name() const { return name_; }
  /** Get local UDP port.
   * @return local UDP port. */
  unsigned short       port() const { return port_; }

 private:
  GossipGroup(std::string &group_name,
	      std::string &peer_name, unsigned short port,
	      ServicePublisher *service_publisher);

 private:
  std::string     name_;
  unsigned short  port_;

  ServicePublisher *service_publisher_;
  std::auto_ptr<NetworkService> service_;
};


} // end namespace fawkes


#endif
