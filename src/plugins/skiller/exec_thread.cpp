
/***************************************************************************
 *  exec_thread.cpp - Fawkes Skiller: Execution Thread
 *
 *  Created: Mon Feb 18 10:30:17 2008
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

#include <plugins/skiller/exec_thread.h>
#include <plugins/skiller/liaison_thread.h>

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex.h>
#include <core/threading/barrier.h>
#include <utils/logging/component.h>

#include <lua/context.h>

#include <interfaces/skiller.h>

#include <lua.hpp>
#include <tolua++.h>

#include <string>
#include <cstring>

using namespace std;
using namespace fawkes;

/** @class SkillerExecutionThread <plugins/skiller/exec_thread.h>
 * Skiller Execution Thread.
 * This thread runs and controls the Lua interpreter and passes data into the
 * execution engine. It can act upon signals from the liaison thread if
 * necessary.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param liaison_exec_barrier Barrier used to synchronize liaison and exec thread
 * @param slt Skiller liaison thread
 */
SkillerExecutionThread::SkillerExecutionThread(Barrier *liaison_exec_barrier,
					       SkillerLiaisonThread *slt)
  : Thread("SkillerExecutionThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
  __liaison_exec_barrier = liaison_exec_barrier;
  __slt = slt;

  __continuous_run = false;

  __lua = NULL;
}


/** Destructor. */
SkillerExecutionThread::~SkillerExecutionThread()
{
}


/** Determines the skill status and writes it to the BB.
 * This method assumes that it is called from within loop() and lua_mutex is locked.
 * @param curss current skill string
 */
void
SkillerExecutionThread::publish_skill_status(std::string &curss)
{
  __slt->skiller->set_skill_string(curss.c_str());
  __slt->skiller->set_continuous(__continuous_run);

  const char *sst = "Unknown";
  LUA_INTEGER running = 0, final = 0, failed = 0;

  try {
    __lua->do_string("return skills.common.skillenv.get_status()");
    running = __lua->to_integer(-3);
    final   = __lua->to_integer(-2);
    failed  = __lua->to_integer(-1);

    if ( failed > 0 ) {
      sst = "S_FAILED";
      __slt->skiller->set_status(SkillerInterface::S_FAILED);
    } else if ( (final > 0) && (running == 0) ) {
      sst = "S_FINAL";
      __slt->skiller->set_status(SkillerInterface::S_FINAL);
    } else if ( running > 0 ) {
      sst = "S_RUNNING";
      __slt->skiller->set_status(SkillerInterface::S_RUNNING);
    } else {
      // all zero
      sst = "S_INACTIVE";
      __slt->skiller->set_status(SkillerInterface::S_INACTIVE);
    }

    /*
    logger->log_debug("SkillerExecutionThread", "Status is %s "
		      "(running %i, final: %i, failed: %i)",
		      sst, running, final, failed);
    */

  } catch (Exception &e) {
    logger->log_error("SkillerExecutionThread", "Failed to retrieve skill status");
    logger->log_error("SkillerExecutionThread", e);
    __slt->skiller->set_status(SkillerInterface::S_FAILED);
  }

  __slt->skiller->write();
}


void
SkillerExecutionThread::init()
{
  __last_exclusive_controller = 0;
  __reader_just_left = false;

  try {
    __cfg_skillspace  = config->get_string("/skiller/skillspace");
    __cfg_watch_files = config->get_bool("/skiller/watch_files");
  } catch (Exception &e) {
    e.append("Insufficient configuration for Skiller");
    throw;
  }

  logger->log_debug("SkillerExecutionThread", "Skill space: %s", __cfg_skillspace.c_str());

  __clog = new ComponentLogger(logger, "SkillerLua");
  __lua  = new LuaContext(__cfg_watch_files);

  __lua->add_package_dir(LUADIR);
  __lua->add_cpackage_dir(LUALIBDIR);

  __lua->add_package("fawkesutils");
  __lua->add_package("fawkesconfig");
  __lua->add_package("fawkesinterface");
  __lua->add_package("fawkesinterfaces");

  __lua->set_string("SKILLSPACE", __cfg_skillspace.c_str());
  __lua->set_usertype("config", config, "Configuration", "fawkes");
  __lua->set_usertype("logger", __clog, "ComponentLogger", "fawkes");
  __lua->set_usertype("clock", clock, "Clock", "fawkes");

  SkillerLiaisonThread::InterfaceMap::iterator imi;
  SkillerLiaisonThread::InterfaceMap &rimap = __slt->reading_interfaces();
  for (imi = rimap.begin(); imi != rimap.end(); ++imi) {
    __lua->set_usertype(("interface_" + imi->first).c_str(), imi->second, imi->second->type(), "fawkes");
  }

  SkillerLiaisonThread::InterfaceMap &wimap = __slt->writing_interfaces();
  for (imi = wimap.begin(); imi != wimap.end(); ++imi) {
    __lua->set_usertype(("interface_" + imi->first).c_str(), imi->second, imi->second->type(), "fawkes");
  }

  __lua->set_start_script(LUADIR"/skills/common/start.lua");
}


void
SkillerExecutionThread::finalize()
{
  delete __lua;
  __lua = NULL;
  delete __clog;
  __clog = NULL;
}


/** Called when reader has been removed from skiller interface.
 * @param instance_serial instance serial of the interface that triggered the event.
 */
void
SkillerExecutionThread::skiller_reader_removed(unsigned int instance_serial)
{
  if ( instance_serial == __slt->skiller->exclusive_controller() ) {
    logger->log_debug("SkillerExecutionThread", "Controlling interface instance was closed, "
		      "revoking exclusive control");

    __last_exclusive_controller = instance_serial;
    __reader_just_left = true;

    __slt->skiller->set_exclusive_controller(0);
    __slt->skiller->write();
  }
}


void
SkillerExecutionThread::loop()
{
  __liaison_exec_barrier->wait();
  __lua->process_fam_events();

  // Current skill string
  std::string curss = "";

  unsigned int excl_ctrl  = __slt->skiller->exclusive_controller();
  bool continuous_reset   = false;

  while ( ! __slt->skiller->msgq_empty() ) {
    if ( __slt->skiller->msgq_first_is<SkillerInterface::AcquireControlMessage>() ) {
      Message *m = __slt->skiller->msgq_first();
      if ( excl_ctrl == 0 ) {
	logger->log_debug("SkillerExecutionThread", "%s is new exclusive controller",
			  m->sender_thread_name());
	__slt->skiller->set_exclusive_controller(m->sender_id());
	excl_ctrl = m->sender_id();
      } else {
	logger->log_warn("SkillerExecutionThread", "%s tried to acquire exclusive control, "
			 "but another controller exists already", m->sender_thread_name());
      }

    } else if ( __slt->skiller->msgq_first_is<SkillerInterface::ReleaseControlMessage>() ) {
      Message *m = __slt->skiller->msgq_first();
      if ( excl_ctrl == m->sender_id() ) {
	logger->log_debug("SkillerExecutionThread", "%s releases exclusive control",
			  m->sender_thread_name());
	
	if ( __continuous_run ) {
	  __continuous_run = false;
	  continuous_reset = true;
	}
	__last_exclusive_controller = __slt->skiller->exclusive_controller();
	__slt->skiller->set_exclusive_controller(0);
	excl_ctrl = 0;
    } else {
	if ( !__reader_just_left || (m->sender_id() != __last_exclusive_controller)) {
	  logger->log_warn("SkillerExecutionThread", "%s tried to release exclusive control, "
			   "it's not the controller", m->sender_thread_name());
	}
      }
    } else if ( __slt->skiller->msgq_first_is<SkillerInterface::ExecSkillMessage>() ) {
      SkillerInterface::ExecSkillMessage *m = __slt->skiller->msgq_first<SkillerInterface::ExecSkillMessage>();

      if ( m->sender_id() == excl_ctrl ) {
	if ( curss != "" ) {
	  logger->log_warn("SkillerExecutionThread", "More than one skill string enqueued, "
			   "ignoring successive string (%s).", m->skill_string());
	} else {	  
	  logger->log_debug("SkillerExecutionThread", "%s wants me to execute '%s'",
			    m->sender_thread_name(), m->skill_string());

	  if ( __continuous_run ) {
	    __continuous_run = false;
	    continuous_reset = true;
	  }
	  curss = m->skill_string();
	}
      } else {
	logger->log_debug("SkillerExecutionThread", "%s tries to exec while not controller",
			  m->sender_thread_name());
      }

    } else if ( __slt->skiller->msgq_first_is<SkillerInterface::ExecSkillContinuousMessage>() ) {
      SkillerInterface::ExecSkillContinuousMessage *m = __slt->skiller->msgq_first<SkillerInterface::ExecSkillContinuousMessage>();

      if ( m->sender_id() == excl_ctrl ) {
	if ( curss != "" ) {
	  logger->log_warn("SkillerExecutionThread", "More than one skill string enqueued, "
			   "ignoring successive string (%s).", m->skill_string());
	} else {	  
	  logger->log_debug("SkillerExecutionThread", "%s wants me to execute '%s'",
			    m->sender_thread_name(), m->skill_string());

	  curss = m->skill_string();
	  continuous_reset = __continuous_run; // reset if cont exec was in progress
	  __continuous_run = true;
	}
      } else {
	logger->log_debug("SkillerExecutionThread", "%s tries to exec while not controller",
			  m->sender_thread_name());
      }

    } else if ( __slt->skiller->msgq_first_is<SkillerInterface::StopExecMessage>() ) {
      SkillerInterface::StopExecMessage *m = __slt->skiller->msgq_first<SkillerInterface::StopExecMessage>();

      if ( (m->sender_id() == excl_ctrl) ||
	   (__reader_just_left && (m->sender_id() == __last_exclusive_controller)) ) {
	logger->log_debug("SkillerExecutionThread", "Stopping continuous execution");
	if ( __continuous_run ) {
	  __continuous_run = false;
	  continuous_reset = true;
	}
      } else {
	logger->log_debug("SkillerExecutionThread", "%s tries to stop exec while not controller",
			  m->sender_thread_name());
      }
    } else {
      logger->log_warn("SkillerExecutionThread", "Unhandled message of type %s in "
		       "skiller interface", __slt->skiller->msgq_first()->type());
    }

    __slt->skiller->msgq_pop();
  }

  if ( __continuous_run && (curss == "") ) {
    curss = __slt->skiller->skill_string();
  }

  if ( continuous_reset ) {
    logger->log_debug("SkillerExecutionThread", "Continuous reset forced");
    __lua->do_string("skills.common.skillenv.reset_all()");
  }

  if ( curss != "" ) {
    // We've got something to execute

    // we're in continuous mode, reset status for this new loop
    if ( __continuous_run && ! continuous_reset) {
      // was continuous execution, status has to be cleaned up anyway
      //logger->log_debug("SkillerExecutionThread", "Resetting skill status in continuous mode");
      __lua->do_string("skills.common.skillenv.reset_status()");
    }

    try {
                                          // Stack:
      __lua->load_string(curss.c_str());  // sksf (skill string function)
      __lua->do_string("return skills.common.skillenv.gensandbox()"); // sksf, sandbox
      __lua->setfenv();                   // sksf
      __lua->pcall();                     // ---

    } catch (Exception &e) {
      logger->log_error("SkillerExecutionThread", e);
    }

    publish_skill_status(curss);

    if ( ! __continuous_run ) {
      // was one-shot execution, cleanup
      logger->log_debug("SkillerExecutionThread", "Resetting skills");
      __lua->do_string("skills.common.skillenv.reset_all()");
    }
  } // end if (curss != "")

  __reader_just_left = false;

  __liaison_exec_barrier->wait();
}

