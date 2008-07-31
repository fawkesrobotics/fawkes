
/***************************************************************************
 *  liaison_thread.cpp - Fawkes Skiller: Liaison Thread
 *
 *  Created: Mon Feb 18 10:24:46 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <plugins/skiller/liaison_thread.h>
#include <plugins/skiller/exec_thread.h>

#include <core/threading/barrier.h>
#include <interfaces/skiller.h>

#include <cstring>

using namespace fawkes;

/** @class SkillerLiaisonThread <plugins/skiller/liaison_thread.h>
 * Skiller Liaison Thread.
 * This threads connects the skill module to the Fawkes main loop. It gathers
 * data from the interface and passes commands to the real skill execution
 * thread.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param liaison_exec_barrier Barrier used to synchronize liaison and exec thread
 */
SkillerLiaisonThread::SkillerLiaisonThread(fawkes::Barrier *liaison_exec_barrier)
  : Thread("SkillerLiaisonThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL),
    BlackBoardInterfaceListener("SkillerLiaisonThread")
{
  __liaison_exec_barrier = liaison_exec_barrier;
  bbio_add_interface_create_type("ObjectPositionInterface");
}


/** Destructor. */
SkillerLiaisonThread::~SkillerLiaisonThread()
{
}


/** Set execution thread.
 * @param set skiller execution thread
 */
void
SkillerLiaisonThread::set_execthread(SkillerExecutionThread *set)
{
  __exec_thread = set;
}


/** Get map of reading interfaces.
 * @return map with variable name as key and interface as value of interfaces
 * opened for reading.
 */
SkillerLiaisonThread::InterfaceMap &
SkillerLiaisonThread::reading_interfaces()
{
  return __reading_ifs;
}


/** Get map of writing interfaces.
 * @return map with variable name as key and interface as value of interfaces
 * opened for writing.
 */
SkillerLiaisonThread::InterfaceMap &
SkillerLiaisonThread::writing_interfaces()
{
  return __writing_ifs;
}

/** Clean up when init failed.
 * You may only call this from init(). Never ever call it from anywhere
 * else!
 */
void
SkillerLiaisonThread::init_failure_cleanup()
{
  try {
    if ( skiller )    blackboard->close(skiller);

    for (InterfaceMap::iterator i = __reading_ifs.begin(); i != __reading_ifs.end(); ++i) {
      blackboard->close(i->second);
    }
    for (InterfaceMap::iterator i = __writing_ifs.begin(); i != __writing_ifs.end(); ++i) {
      blackboard->close(i->second);
    }
  } catch (...) {
    // we really screwed up, can't do anything about it, ignore error, logger is
    // initialized since this method is only called from init() which is only called if
    // all aspects had been initialized successfully
    logger->log_error(name(), "Really screwed up while finalizing, aborting cleanup. "
		              "Fawkes is no longer in a clean state. Restart!");
  }
}


void
SkillerLiaisonThread::open_interfaces(std::string &prefix, InterfaceMap &imap, bool write)
{
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
    logger->log_info(name(), "Adding %s interface %s::%s with name %s",
		     write ? "writing" : "reading",
		     iftype.c_str(), ifname.c_str(), varname.c_str());
    try {
      Interface *iface;
      if (write) {
	iface = blackboard->open_for_writing(iftype.c_str(), ifname.c_str());
      } else {
	iface = blackboard->open_for_reading(iftype.c_str(), ifname.c_str());
      }
      imap[varname] = iface;
    } catch (Exception &e) {
      delete vi;
      throw;
    }
  }
  delete vi;
}

void
SkillerLiaisonThread::init()
{
  skiller = NULL;

  try {
    skiller   = blackboard->open_for_writing<SkillerInterface>("Skiller");

    std::string skillspace  = config->get_string("/skiller/skillspace");

    std::string reading_prefix = "/skiller/interfaces/" + skillspace + "/reading/";
    std::string writing_prefix = "/skiller/interfaces/" + skillspace + "/writing/";
    open_interfaces(reading_prefix, __reading_ifs, /* write */ false);
    open_interfaces(writing_prefix, __writing_ifs, /* write */ true);

  } catch (Exception &e) {
    e.print_trace();
    init_failure_cleanup();
    e.append("SkillerLiaisonThread::init() failed");
    throw;
  }

  /*
  // we only watch create events, since we never ever close an interface while
  // running, we only open newly created ones. We have memory an do not want
  // "oscillating" open/close loops
  blackboard->register_observer(this, BlackBoard::BBIO_FLAG_CREATED);
  */

  // We want to know if our reader leaves and closes the interface
  bbil_add_reader_interface(skiller);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_READER);
}


void
SkillerLiaisonThread::finalize()
{
  blackboard->unregister_listener(this);

  blackboard->close(skiller);

  for (InterfaceMap::iterator i = __reading_ifs.begin(); i != __reading_ifs.end(); ++i) {
    blackboard->close(i->second);
  }
  for (InterfaceMap::iterator i = __writing_ifs.begin(); i != __writing_ifs.end(); ++i) {
    blackboard->close(i->second);
  }
}


void
SkillerLiaisonThread::bb_interface_created(const char *type, const char *id) throw()
{
  /*
  if ( strncmp(id, "WM Obstacle", strlen("WM Obstacle")) == 0 ) {
    // It's a new obstacle in WM, open it
    try {
      ObjectPositionInterface *oi = blackboard->open_for_reading<ObjectPositionInterface>(id);
      wm_obstacles.push_back_locked(oi);
    } catch (Exception &e) {
      logger->log_error("SkillerLiaisonThread", "Tried to open new %s interface instance "
			"'%s', failed, ignoring this interface.", type, id);
      logger->log_error("SkillerLiaisonThread", e);
    }
  }
  */
}


void
SkillerLiaisonThread::bb_interface_reader_removed(Interface *interface,
						  unsigned int instance_serial) throw()
{
  __exec_thread->skiller_reader_removed(instance_serial);
}


void
SkillerLiaisonThread::loop()
{
  for (InterfaceMap::iterator i = __reading_ifs.begin(); i != __reading_ifs.end(); ++i) {
    i->second->read();
  }

  // This barrier wait triggers exec thread execution
  __liaison_exec_barrier->wait();

  // This barrier wait makes us wait until the exec thread is finished
  __liaison_exec_barrier->wait();

  for (InterfaceMap::iterator i = __writing_ifs.begin(); i != __writing_ifs.end(); ++i) {
    try {
      i->second->write();
    } catch (Exception &e) {
      e.append("Failed to write interface %s, ignoring.", i->second->uid());
      e.print_trace();
    }
  }
}
