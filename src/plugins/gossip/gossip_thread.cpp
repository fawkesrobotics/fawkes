
/***************************************************************************
 *  gossip_thread.cpp -  Robot Group Communication Plugin
 *
 *  Created: Fri Feb 28 11:11:20 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include "gossip_thread.h"

#include <plugins/gossip/gossip/gossip_group_manager.h>

#include <set>

using namespace fawkes;

#define CFG_PREFIX "/gossip/"

/** @class GossipThread "clips-protobuf-thread.h"
 * Robot Group Communication.
 * @author Tim Niemueller
 */

/** Constructor. */
GossipThread::GossipThread()
  : Thread("GossipThread", Thread::OPMODE_WAITFORWAKEUP),
    AspectProviderAspect(&gossip_aspect_inifin_)
{
}


/** Destructor. */
GossipThread::~GossipThread()
{
}


void
GossipThread::init()
{
  cfg_service_name_ = config->get_string(CFG_PREFIX"name");

  // gather static group configurations
  std::map<std::string, GossipGroupConfiguration> groups;
  std::set<std::string> ignored_groups;

  std::string prefix = CFG_PREFIX"groups/";

  std::shared_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
  while (i->next()) {
    std::string cfg_name = std::string(i->path()).substr(prefix.length());
    cfg_name = cfg_name.substr(0, cfg_name.find("/"));

    if ( (groups.find(cfg_name) == groups.end()) &&
	 (ignored_groups.find(cfg_name) == ignored_groups.end()) ) {

      std::string cfg_prefix = prefix + cfg_name + "/";

      bool active = true;
      try {
	active = config->get_bool((cfg_prefix + "active").c_str());
      } catch (Exception &e) {} // ignored, assume enabled

      try {
	if (active) {
	  std::string  addr = config->get_string((cfg_prefix + "broadcast-address").c_str());

	  if (config->exists((cfg_prefix + "broadcast-send-port").c_str()) &&
	      config->exists((cfg_prefix + "broadcast-recv-port").c_str()) )
	  {
	    unsigned int send_port =
	      config->get_uint((cfg_prefix + "broadcast-send-port").c_str());
	    unsigned int recv_port =
	      config->get_uint((cfg_prefix + "broadcast-recv-port").c_str());

	    if (send_port > 0xFFFF) {
	      throw Exception("Port number too high: %u > %u", send_port, 0xFFFF);
	    }
	    if (recv_port > 0xFFFF) {
	      throw Exception("Port number too high: %u > %u", recv_port, 0xFFFF);
	    }
	    groups[cfg_name] = GossipGroupConfiguration(cfg_name, addr, send_port, recv_port);

	  } else {
	    unsigned int port = config->get_uint((cfg_prefix + "broadcast-port").c_str());

	    if (port > 0xFFFF) {
	      throw Exception("Port number too high: %u > %u", port, 0xFFFF);
	    }

	    groups[cfg_name] = GossipGroupConfiguration(cfg_name, addr, port);
	  }

	  try {
	    groups[cfg_name].crypto_key =
	      config->get_string((cfg_prefix + "encryption-key").c_str());
	    try {
	      groups[cfg_name].crypto_cipher =
		config->get_string((cfg_prefix + "encryption-cipher").c_str());
	    } catch (Exception &e) {
	      // ignore, use default
	      groups[cfg_name].crypto_cipher = "aes-128-ecb";
	    }
	    logger->log_debug(name(), "Setup encryption of type %s for group '%s'",
			      groups[cfg_name].crypto_cipher.c_str(), cfg_name.c_str());
	  } catch (Exception &e) {} // ignored, no encryption

	} else {
	  //printf("Ignoring laser config %s\n", cfg_name.c_str());
	  ignored_groups.insert(cfg_name);
	}
      } catch(Exception &e) {
	throw;
      }
    }
  }

  group_mgr_ =
    std::shared_ptr<GossipGroupManager>(new GossipGroupManager(cfg_service_name_,
							     service_publisher,
							     groups));
  gossip_aspect_inifin_.set_manager(group_mgr_.get());
}


void
GossipThread::finalize()
{
  gossip_aspect_inifin_.set_manager(NULL);
  group_mgr_.reset();
}


void
GossipThread::loop()
{
}
