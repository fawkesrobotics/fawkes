
/***************************************************************************
 *  exec_thread.cpp - Fawkes Skiller: Execution Thread
 *
 *  Created: Mon Feb 18 10:30:17 2008
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

#include "exec_thread.h"
#include "skiller_feature.h"

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex.h>
#include <logging/component.h>
#ifdef SKILLER_TIMETRACKING
#  include <utils/time/tracker.h>
#endif

#include <lua/context.h>

#include <interfaces/SkillerInterface.h>
#include <interfaces/SkillerDebugInterface.h>

#include <lua.hpp>
#include <tolua++.h>

#include <string>
#include <cstring>

using namespace std;
using namespace fawkes;

/** @class SkillerExecutionThread "exec_thread.h"
 * Skiller Execution Thread.
 * This thread runs and controls the Lua interpreter and passes data into the
 * execution engine.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
SkillerExecutionThread::SkillerExecutionThread()
  : Thread("SkillerExecutionThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL),
    BlackBoardInterfaceListener("SkillerExecutionThread")
{
}


/** Destructor. */
SkillerExecutionThread::~SkillerExecutionThread()
{
}


void
SkillerExecutionThread::init()
{
  try {
    __cfg_skillspace  = config->get_string("/skiller/skillspace");
    __cfg_watch_files = config->get_bool("/skiller/watch_files");
  } catch (Exception &e) {
    e.append("Insufficient configuration for Skiller");
    throw;
  }

  logger->log_debug("SkillerExecutionThread", "Skill space: %s", __cfg_skillspace.c_str());
  __clog = new ComponentLogger(logger, "SkillerLua");

  __lua = NULL;
  __bbo = NULL;
  __skiller_if = NULL;

  try {
	  __skiller_if = 	  blackboard->open_for_reading<SkillerInterface>("Skiller");

    __lua  = new LuaContext();
    if (__cfg_watch_files) {
      __lua->setup_fam(/* auto restart */ true, /* conc thread */ false);
    }

    __lua->add_package_dir(LUADIR, /* prefix */ true);
    __lua->add_cpackage_dir(LUALIBDIR, /* prefix */ true);

    __lua->add_package("fawkesutils");
    __lua->add_package("fawkesconfig");
    __lua->add_package("fawkeslogging");
    __lua->add_package("fawkesinterface");
    __lua->add_package("fawkesblackboard");
#ifdef HAVE_TF
    __lua->add_package("fawkestf");
#endif

    __bbo = new BlackBoardWithOwnership(blackboard, "SkillerLua");
    
    __lua->set_string("SKILLSPACE", __cfg_skillspace.c_str());
    __lua->set_string("LUADIR", LUADIR);
    __lua->set_usertype("config", config, "Configuration", "fawkes");
    __lua->set_usertype("logger", __clog, "ComponentLogger", "fawkes");
    __lua->set_usertype("clock", clock, "Clock", "fawkes");
    __lua->set_usertype("blackboard", __bbo, "BlackBoard", "fawkes");
#ifdef HAVE_TF
    __lua->set_usertype("tf", tf_listener, "Transformer", "fawkes::tf");
#endif

    __lua->create_table();
    __lua->set_global("features_env_template");

    std::list<SkillerFeature *>::iterator f;
    for (f = __features.begin(); f != __features.end(); ++f) {
      (*f)->init_lua_context(__lua);
    }

    __lua->set_finalization_calls("skiller.fawkes.finalize()",
                                  "skiller.fawkes.finalize_prepare()",
                                  "skiller.fawkes.finalize_cancel()");
    
    __lua->set_start_script(LUADIR"/skiller/fawkes/start.lua");

    __lua->add_watcher(this);
  
  } catch (Exception &e) {
	  blackboard->close(__skiller_if);
	  delete __lua;
	  delete __bbo;
	  delete __clog;
	  throw;
  }

  // We want to know if our reader leaves and closes the interface
  bbil_add_reader_interface(__skiller_if);
  blackboard->register_listener(this);

}


void
SkillerExecutionThread::finalize()
{
  __lua->remove_watcher(this);

  blackboard->unregister_listener(this);
  blackboard->close(__skiller_if);

  std::list<SkillerFeature *>::iterator f;
  for (f = __features.begin(); f != __features.end(); ++f) {
    (*f)->finalize_lua_context(__lua);
  }

  delete __lua;
  delete __clog;
  delete __bbo;
}


/** Add a skiller feature.
 * Note that this has to be done before the SkillerExecutionThread is initialized
 * at this time (an extension to do this at run-time might follow later).
 * @param feature feature to add
 */
void
SkillerExecutionThread::add_skiller_feature(SkillerFeature *feature)
{
  __features.push_back(feature);
}


void
SkillerExecutionThread::lua_restarted(LuaContext *context)
{
	context->create_table();
  context->set_global("features_env_template");

  std::list<SkillerFeature *>::iterator f;
  for (f = __features.begin(); f != __features.end(); ++f) {
    (*f)->init_lua_context(context);
  }

  // move writing interfaces
  __lua->do_string("return fawkes.interface_initializer.finalize_prepare()");

  context->create_table();

  __lua->push_nil();
  while (__lua->table_next(-2) ) {
	  void * udata = __lua->to_usertype(-1);
	  if (udata) {
		  std::string type, id;
		  Interface::parse_uid(__lua->to_string(-2), type, id);
		  context->do_string("require(\"interfaces.%s\")", type.c_str());
		  context->push_string(__lua->to_string(-2));
		  context->push_usertype(udata, type.c_str(), "fawkes");
		  context->set_table(-3);
		  __lua->pop(1);
	  }
  }

  context->set_global("interfaces_writing_preload");
}


void
SkillerExecutionThread::bb_interface_reader_removed(Interface *interface,
                                                    unsigned int instance_serial) throw()
{
	__skiller_if_removed_readers.push_locked(instance_serial);
}


void
SkillerExecutionThread::loop()
{
#ifdef HAVE_INOTIFY
  __lua->process_fam_events();
#endif

  __skiller_if_removed_readers.lock();
  while (! __skiller_if_removed_readers.empty()) {
	  __lua->do_string("skiller.fawkes.notify_reader_removed(%u)", __skiller_if_removed_readers.front());
	  __skiller_if_removed_readers.pop();
  }
  __skiller_if_removed_readers.unlock();

  __lua->do_string("skillenv.loop()");
}
