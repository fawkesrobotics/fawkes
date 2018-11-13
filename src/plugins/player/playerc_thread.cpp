
/***************************************************************************
 *  playerc_thread.cpp - Thread that connects to Player server
 *
 *  Created: Mon Aug 11 22:45:00 2008
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

#define __STDC_LIMIT_MACROS

#include "playerc_thread.h"
#include "mapper_factory.h"

#include <core/exceptions/software.h>
#include <utils/time/time.h>
#include <interfaces/ObjectPositionInterface.h>

#include <libplayerc++/playerc++.h>

#include <stdint.h>

using namespace PlayerCc;
using namespace fawkes;

/** @class PlayerClientThread "playerc_thread.h"
 * Player Client Thread.
 * This thread connects to the player server and handles messages
 * @author Tim Niemueller
 */


/** Constructor. */
PlayerClientThread::PlayerClientThread()
  : Thread("PlayerClientThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
  client_ = NULL;
}


void
PlayerClientThread::init()
{
  client_ = NULL;

  try {
    cfg_player_host_ = config->get_string("/player/host");
    cfg_player_port_ = config->get_uint("/player/port");
  } catch (Exception &e) {
    e.append("Could not read all required config values for %s", name());
    throw;
  }

  try {
    client_ = new PlayerClient(cfg_player_host_.c_str(), cfg_player_port_);

    client_->SetDataMode(PLAYER_DATAMODE_PULL);
    client_->SetReplaceRule(/* replace */ true);
  } catch (PlayerError &pe) {
    finalize();
    throw Exception("Failed to connect to Player. Error was '%s'",
		    pe.GetErrorStr().c_str());
  }

  client_->RequestDeviceList();

  /* shows all available interfaces
  std::list<playerc_device_info_t> devices = client_->GetDeviceList();

  for (std::list<playerc_device_info_t>::iterator i = devices.begin(); i != devices.end(); ++i) {
    logger->log_debug(name(), "Interface of type %u (%s), index %u, host %u, "
		      "robot %u, driver %s",
		      i->addr.interf, client_->LookupName(i->addr.interf).c_str(),
		      i->addr.index, i->addr.host, i->addr.robot, i->drivername);
  }
  */

  try {
    open_fawkes_interfaces();
    open_player_proxies();
    create_mappers();
  } catch (Exception &e) {
    finalize();
    throw;
  }
}

void
PlayerClientThread::open_fawkes_interfaces()
{
  std::string prefix = "/player/interfaces/fawkes/";
  Configuration::ValueIterator *vi = config->search(prefix.c_str());
  while (vi->next()) {
    if (strcmp(vi->type(), "string") != 0) {
      TypeMismatchException e("Only values of type string may occur in %s, "
			      "but found value of type %s",
			      prefix.c_str(), vi->type());
      delete vi;
      throw e;
    }
    std::string uid = vi->get_string();
    std::string varname = std::string(vi->path()).substr(prefix.length());
    std::string iftype = uid.substr(0, uid.find("::"));
    std::string ifname = uid.substr(uid.find("::") + 2);
    logger->log_info(name(), "Adding interface %s::%s with name %s writing",
		     iftype.c_str(), ifname.c_str(), varname.c_str());
    try {
      Interface *iface;
      iface = blackboard->open_for_writing(iftype.c_str(), ifname.c_str());
      imap_[varname] = iface;
    } catch (Exception &e) {
      delete vi;
      throw;
    }
  }
  delete vi;
}


void
PlayerClientThread::open_player_proxies()
{
  std::list<playerc_device_info_t> devices = client_->GetDeviceList();

  sockaddr_in *addr;
  socklen_t    addrlen = sizeof(sockaddr_in);

  if ( ! nnresolver->resolve_name_blocking(cfg_player_host_.c_str(), (sockaddr **)&addr, &addrlen) ) {
    throw Exception("Could not lookup IP of %s (player host)", cfg_player_host_.c_str());
  }

  unsigned int host  = addr->sin_addr.s_addr;
  unsigned int robot = cfg_player_port_;

  std::string prefix = "/player/interfaces/player/";
  Configuration::ValueIterator *vi = config->search(prefix.c_str());
  while (vi->next()) {
    if (strcmp(vi->type(), "string") != 0) {
      TypeMismatchException e("Only values of type string may occur in %s, "
			      "but found value of type %s",
			      prefix.c_str(), vi->type());
      delete vi;
      throw e;
    }
    std::string uid = vi->get_string();
    std::string varname = std::string(vi->path()).substr(prefix.length());
    std::string iftype = uid.substr(0, uid.find(":"));
    long int    ifindexl = atol(uid.substr(uid.find(":") + 1).c_str());
    if ( ifindexl > (long int)UINT32_MAX ) {
      throw Exception("Player interface index is out of range (%li > %u)", ifindexl, UINT32_MAX);
    } else if ( ifindexl < 0 ) {
      throw Exception("Player interface index is out of range (%li < 0)", ifindexl);
    }
    unsigned int ifindex = ifindexl;
    logger->log_info(name(), "Adding Player interface %s:%u with name %s",
		     iftype.c_str(), ifindex, varname.c_str());

    ClientProxy *proxy = NULL;
    for (std::list<playerc_device_info_t>::iterator i = devices.begin();
	 (proxy == NULL) && (i != devices.end()); ++i) {
      if ( (i->addr.host == host) &&
	   (i->addr.robot == robot) &&
	   (i->addr.index == ifindex) &&
	   (iftype == client_->LookupName(i->addr.interf)) ) {
	// positive match
	logger->log_debug(name(), "Opening Player interface of type %u (%s), "
			  "index %u, host %u, robot %u, driver %s",
			  i->addr.interf, client_->LookupName(i->addr.interf).c_str(),
			  i->addr.index, i->addr.host, i->addr.robot, i->drivername);

	if ( iftype == "position2d" ) {
	  proxy = new Position2dProxy(client_, i->addr.index);
	} else if ( iftype == "bumper" ) {
	  proxy = new BumperProxy(client_, i->addr.index);
	} else if ( iftype == "laser" ) {
	  proxy = new LaserProxy(client_, i->addr.index);
	} else {
	  logger->log_warn(name(), "Unknown interface type %s, ignoring", iftype.c_str());
	}
      }
    }
    if ( proxy != NULL ) {
      pmap_[varname] = proxy;
    } else {
      logger->log_warn(name(), "No matching interface found for %s=%s:%u, ignoring",
		       varname.c_str(), iftype.c_str(), ifindex);
    }
  }
  delete vi;
}


void
PlayerClientThread::create_mappers()
{
  for (InterfaceMap::iterator i = imap_.begin(); i != imap_.end(); ++i) {
    if ( pmap_.find(i->first) != pmap_.end() ) {
      logger->log_debug(name(), "Creating mapping for %s from %s to %s",
			i->first.c_str(), i->second->uid(),
			pmap_[i->first]->GetInterfaceStr().c_str());
      mappers_.push_back(PlayerMapperFactory::create_mapper(i->first, i->second,
							     pmap_[i->first]));
    } else {
      throw Exception("No matching proxy found for interface %s (%s)",
		      i->first.c_str(), i->second->uid());
    }
  }

  for (ProxyMap::iterator p = pmap_.begin(); p != pmap_.end(); ++p) {
    if ( imap_.find(p->first) == imap_.end() ) {
      throw Exception("No matching interface found for proxy %s", p->first.c_str());
    }
  }
}


void
PlayerClientThread::finalize()
{
  for (MapperList::iterator m = mappers_.begin(); m != mappers_.end(); ++m) {
    delete *m;
  }
  mappers_.clear();
  
  close_fawkes_interfaces();
  close_player_proxies();

  delete client_;
}


void
PlayerClientThread::close_fawkes_interfaces()
{
  for (InterfaceMap::iterator i = imap_.begin(); i != imap_.end(); ++i) {
    blackboard->close(i->second);
  }
  imap_.clear();
}


void
PlayerClientThread::close_player_proxies()
{
  for (ProxyMap::iterator p = pmap_.begin(); p != pmap_.end(); ++p) {
    // dtor is protected, seems to be a Player bug, will discuss upstream
    // this is a memleak atm
    //delete p->second;
  }
  pmap_.clear();
}


/** Sync Fawkes to player.
 * This will call all mappers to sync Fawkes interfaces to Player proxies. This
 * is meant to be called by the PlayerF2PThread.
 */
void
PlayerClientThread::sync_fawkes_to_player()
{
  //logger->log_debug(name(), "Syncing fawkes to player");
  try {
    for (MapperList::iterator m = mappers_.begin(); m != mappers_.end(); ++m) {
      (*m)->sync_fawkes_to_player();
    }
  } catch (PlayerCc::PlayerError &e) {
    logger->log_warn(name(), "Failed to update player proxies: %s", e.GetErrorStr().c_str());
  }
}

void
PlayerClientThread::loop()
{
  try {
    if ( client_->Peek() ) {
      client_->Read();

      //logger->log_debug(name(), "Syncing player to fawkes");
      for (MapperList::iterator m = mappers_.begin(); m != mappers_.end(); ++m) {
	(*m)->sync_player_to_fawkes();
      }
    } else {
      //logger->log_warn(name(), "Nothing to sync from player to fawkes");
    }
  } catch (PlayerCc::PlayerError &e) {
    logger->log_warn(name(), "Failed to peek/read data: %s", e.GetErrorStr().c_str());
  }
}
