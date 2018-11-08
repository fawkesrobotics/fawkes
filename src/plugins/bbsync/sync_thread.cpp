
/***************************************************************************
 *  sync_thread.cpp - Fawkes BlackBoard Synchronization Thread
 *
 *  Created: Thu Jun 04 18:13:06 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include "sync_thread.h"

#include <blackboard/remote.h>
#include <core/threading/mutex_locker.h>
#include <utils/time/wait.h>

#include <cstring>

using namespace std;
using namespace fawkes;

/** @class BlackBoardSynchronizationThread "sync_thread.h"
 * Thread to synchronize two BlackBoards.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param bbsync_cfg_prefix Configuration prefix for the whole bbsync plugin
 * @param peer_cfg_prefix The configuration prefix for the peer this sync thread
 * has been created for.
 * @param peer name of the peer configuration for this thread
 */
BlackBoardSynchronizationThread::BlackBoardSynchronizationThread(std::string &bbsync_cfg_prefix,
								 std::string &peer_cfg_prefix,
								 std::string &peer)
  : Thread("", Thread::OPMODE_CONTINUOUS)
{
  set_name("BBSyncThread[%s]", peer.c_str());
  set_prepfin_conc_loop(true);

  bbsync_cfg_prefix_ = bbsync_cfg_prefix;
  peer_cfg_prefix_   = peer_cfg_prefix;
  peer_              = peer;

  remote_bb_ = NULL;
}


/** Destructor. */
BlackBoardSynchronizationThread::~BlackBoardSynchronizationThread()
{
}

void
BlackBoardSynchronizationThread::init()
{
  logger->log_debug(name(), "Initializing");
  unsigned int check_interval = 0;
  try {
    host_ = config->get_string((peer_cfg_prefix_ + "host").c_str());
    port_ = config->get_uint((peer_cfg_prefix_ + "port").c_str());

    check_interval = config->get_uint((bbsync_cfg_prefix_ + "check_interval").c_str());
  } catch (Exception &e) {
    e.append("Host or port not specified for peer");
    throw;
  }

  try {
    check_interval = config->get_uint((peer_cfg_prefix_ + "check_interval").c_str());
    logger->log_debug(name(), "Peer check interval set, overriding default.");
  } catch (Exception &e) {
    logger->log_debug(name(), "No per-peer check interval set, using default");
  }

  read_config_combos(peer_cfg_prefix_ + "reading/", /* writing */ false);
  read_config_combos(peer_cfg_prefix_ + "writing/", /* writing */ true);

  for (ComboMap::iterator i = combos_.begin(); i != combos_.end(); ++i) {
    logger->log_debug(name(), "Combo: %s, %s (%s, R) -> %s (%s, W)", i->second.type.c_str(),
		      i->second.reader_id.c_str(), i->second.remote_writer ? "local" : "remote",
		      i->second.writer_id.c_str(), i->second.remote_writer ? "remote" : "local");
  }

  wsl_local_  = new SyncWriterInterfaceListener(this, logger, (peer_ + "/local").c_str());
  wsl_remote_ = new SyncWriterInterfaceListener(this, logger, (peer_ + "/remote").c_str());

  if (! check_connection()) {
    logger->log_warn(name(), "Remote peer not reachable, will keep trying");
  }

  logger->log_debug(name(), "Checking for remote aliveness every %u ms", check_interval);
  timewait_ = new TimeWait(clock, check_interval * 1000);
}


void
BlackBoardSynchronizationThread::finalize()
{

  delete timewait_;

  close_interfaces();

  delete wsl_local_;
  delete wsl_remote_;
  delete remote_bb_;
  remote_bb_ = NULL;
}


void
BlackBoardSynchronizationThread::loop()
{
  timewait_->mark_start();
  check_connection();
  timewait_->wait_systime();
}


bool
BlackBoardSynchronizationThread::check_connection()
{
  if (! remote_bb_ || ! remote_bb_->is_alive()) {
    if (remote_bb_) {
      logger->log_warn(name(), "Lost connection via remote BB to %s (%s:%u), will try to re-establish",
		       peer_.c_str(), host_.c_str(), port_);
      blackboard->unregister_listener(wsl_local_);
      remote_bb_->unregister_listener(wsl_remote_);
      close_interfaces();
      delete remote_bb_;
      remote_bb_ = NULL;
    }

    try {
      remote_bb_ = new RemoteBlackBoard(host_.c_str(), port_);
      logger->log_info(name(), "Successfully connected via remote BB to %s (%s:%u)",
		       peer_.c_str(), host_.c_str(), port_);

      open_interfaces();
      blackboard->register_listener(wsl_local_, BlackBoard::BBIL_FLAG_WRITER);
      remote_bb_->register_listener(wsl_remote_, BlackBoard::BBIL_FLAG_WRITER);
    } catch (Exception &e) {
      e.print_trace();
      return false;
    }
  }
  return true;
}

void
BlackBoardSynchronizationThread::read_config_combos(std::string prefix, bool writing)
{
  Configuration::ValueIterator *i = config->search(prefix.c_str());
  while (i->next()) {
    if (strcmp(i->type(), "string") != 0) {
      TypeMismatchException e("Only values of type string may occur in %s, "
			      "but found value of type %s",
			      prefix.c_str(), i->type());
      delete i;
      throw e;
    }

    std::string varname = std::string(i->path()).substr(prefix.length());
    std::string uid     = i->get_string();
    size_t sf;

    if ((sf = uid.find("::")) == std::string::npos) {
      delete i;
      throw Exception("Interface UID '%s' at %s is not valid, missing double colon",
		      uid.c_str(), i->path());
    }

    std::string type = uid.substr(0, sf);
    std::string id = uid.substr(sf + 2);
    combo_t combo = {  type, id, id, writing };

    if ( (sf = id.find("=")) != std::string::npos) {
      // we got a mapping
      combo.reader_id = id.substr(0, sf);
      combo.writer_id = id.substr(sf + 1);
    }

    combos_[varname] = combo;
  }
  delete i;
}


void
BlackBoardSynchronizationThread::open_interfaces()
{
  logger->log_debug(name(), "Opening interfaces");
  MutexLocker lock(interfaces_.mutex());

  ComboMap::iterator i;
  for (i = combos_.begin(); i != combos_.end(); ++i) {
    Interface *iface_reader = NULL, *iface_writer = NULL;

    BlackBoard *writer_bb = i->second.remote_writer ? remote_bb_ : blackboard;
    BlackBoard *reader_bb = i->second.remote_writer ? blackboard  : remote_bb_;

    try {
      logger->log_debug(name(), "Opening reading %s (%s:%s)",
			i->second.remote_writer ? "locally" : "remotely",
			i->second.type.c_str(), i->second.reader_id.c_str());
      iface_reader = reader_bb->open_for_reading(i->second.type.c_str(),
						 i->second.reader_id.c_str());

      if (iface_reader->has_writer()) {
	logger->log_debug(name(), "Opening writing on %s (%s:%s)",
			  i->second.remote_writer ? "remotely" : "locally",
			  i->second.type.c_str(), i->second.writer_id.c_str());
	iface_writer = writer_bb->open_for_writing(i->second.type.c_str(),
						   i->second.writer_id.c_str());
      }

      InterfaceInfo ii(&i->second, iface_writer, reader_bb, writer_bb);
      interfaces_[iface_reader] = ii;

    } catch (Exception &e) {
      reader_bb->close(iface_reader);
      writer_bb->close(iface_writer);
      throw;
    }

    SyncInterfaceListener *sync_listener = NULL;
    if (iface_writer) {
      logger->log_debug(name(), "Creating sync listener");
      sync_listener = new SyncInterfaceListener(logger, iface_reader, iface_writer,
						reader_bb, writer_bb);
    }
    sync_listeners_[iface_reader] = sync_listener;

    if (i->second.remote_writer) {
      wsl_local_->add_interface(iface_reader);
    } else {
      wsl_remote_->add_interface(iface_reader);
    }
  }
}


void
BlackBoardSynchronizationThread::close_interfaces()
{
  SyncListenerMap::iterator s;
  for (s = sync_listeners_.begin(); s != sync_listeners_.end(); ++s) {
    if (s->second) {
      logger->log_debug(name(), "Closing sync listener %s", s->second->bbil_name());
      delete s->second;
    }
  }
  MutexLocker lock(interfaces_.mutex());
  InterfaceMap::iterator i;
  for (i = interfaces_.begin(); i != interfaces_.end(); ++i) {
    logger->log_debug(name(), "Closing %s reading interface %s",
		      i->second.combo->remote_writer ? "local" : "remote",
		      i->first->uid());
    if (i->second.combo->remote_writer) {
      wsl_local_->remove_interface(i->first);
      blackboard->close(i->first);
    } else {
      wsl_remote_->remove_interface(i->first);
      remote_bb_->close(i->first);
    }
    if (i->second.writer) {
      logger->log_debug(name(), "Closing %s writing interface %s",
			i->second.combo->remote_writer ? "remote" : "local",
			i->second.writer->uid());
      if (i->second.combo->remote_writer) {
	remote_bb_->close(i->second.writer);
      } else {
	blackboard->close(i->second.writer);
      }
    }
  }
  interfaces_.clear();
  sync_listeners_.clear();
}


/** A writer has been added for an interface.
 * To be called only by SyncWriterInterfaceListener.
 * @param interface the interface a writer has been added for.
 */
void
BlackBoardSynchronizationThread::writer_added(fawkes::Interface *interface) throw()
{
  MutexLocker lock(interfaces_.mutex());

  if (interfaces_[interface].writer) {
    // There exists a writer!?
    logger->log_warn(name(), "Writer added for %s, but relay exists already. Bug?", interface->uid());
  } else {
    logger->log_warn(name(), "Writer added for %s, opening relay writer", interface->uid());

    Interface *iface = NULL;
    SyncInterfaceListener *sync_listener = NULL;
    InterfaceInfo &ii = interfaces_[interface];
    try {
      iface = ii.writer_bb->open_for_writing(ii.combo->type.c_str(),
					     ii.combo->writer_id.c_str());
      
      logger->log_debug(name(), "Creating sync listener for %s:%s-%s",
			ii.combo->type.c_str(), ii.combo->reader_id.c_str(),
			ii.combo->writer_id.c_str());

      sync_listener = new SyncInterfaceListener(logger, interface, iface,
						ii.reader_bb, ii.writer_bb);

      sync_listeners_[interface] = sync_listener;
      ii.writer = iface;

    } catch (Exception &e) {
      delete sync_listener;
      ii.writer_bb->close(iface);
      logger->log_error(name(), "Failed to open writer for %s:%s-%s, sync broken",
			ii.combo->type.c_str(), ii.combo->reader_id.c_str(),
			ii.combo->writer_id.c_str());
      logger->log_error(name(), e);
    }
  }
}


/** A writer has been removed for an interface.
 * To be called only by SyncWriterInterfaceListener.
 * @param interface the interface a writer has been removed for.
 */
void
BlackBoardSynchronizationThread::writer_removed(fawkes::Interface *interface) throw()
{
  MutexLocker lock(interfaces_.mutex());

  if (! interfaces_[interface].writer) {
    // We do not have a writer!?
    logger->log_warn(name(), "Writer removed for %s, but no relay exists. Bug?", interface->uid());
  } else {
    logger->log_warn(name(), "Writer removed for %s, closing relay writer", interface->uid());

    InterfaceInfo &ii = interfaces_[interface];
    try {
      delete sync_listeners_[interface];
      sync_listeners_[interface] = NULL;

      ii.writer_bb->close(ii.writer);
      ii.writer = NULL;

    } catch (Exception &e) {
      logger->log_error(name(), "Failed to close writer for %s:%s-%s, sync broken",
			ii.combo->type.c_str(), ii.combo->reader_id.c_str(),
			ii.combo->writer_id.c_str());
      logger->log_error(name(), e);
    }
  }
}
