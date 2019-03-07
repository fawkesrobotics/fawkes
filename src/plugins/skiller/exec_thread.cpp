
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
#	include <utils/time/tracker.h>
#endif

#include <interfaces/SkillerDebugInterface.h>
#include <interfaces/SkillerInterface.h>
#include <lua/context.h>

#include <cstring>
#include <lua.hpp>
#include <string>
#include <tolua++.h>

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
		cfg_skillspace_  = config->get_string("/skiller/skillspace");
		cfg_watch_files_ = config->get_bool("/skiller/watch_files");
	} catch (Exception &e) {
		e.append("Insufficient configuration for Skiller");
		throw;
	}

	logger->log_debug("SkillerExecutionThread", "Skill space: %s", cfg_skillspace_.c_str());
	clog_ = new ComponentLogger(logger, "SkillerLua");

	lua_        = NULL;
	bbo_        = NULL;
	skiller_if_ = NULL;

	try {
		skiller_if_ = blackboard->open_for_reading<SkillerInterface>("Skiller");

		lua_ = new LuaContext();
		if (cfg_watch_files_) {
			lua_->setup_fam(/* auto restart */ true, /* conc thread */ false);
		}

		lua_->add_package_dir(LUADIR, /* prefix */ true);
		lua_->add_cpackage_dir(LUALIBDIR, /* prefix */ true);

		lua_->add_package("fawkesutils");
		lua_->add_package("fawkesconfig");
		lua_->add_package("fawkeslogging");
		lua_->add_package("fawkesinterface");
		lua_->add_package("fawkesblackboard");
#ifdef HAVE_TF
		lua_->add_package("fawkestf");
#endif

		bbo_ = new BlackBoardWithOwnership(blackboard, "SkillerLua");

		lua_->set_string("SKILLSPACE", cfg_skillspace_.c_str());
		lua_->set_string("LUADIR", LUADIR);
		lua_->set_usertype("config", config, "Configuration", "fawkes");
		lua_->set_usertype("logger", clog_, "ComponentLogger", "fawkes");
		lua_->set_usertype("clock", clock, "Clock", "fawkes");
		lua_->set_usertype("blackboard", bbo_, "BlackBoard", "fawkes");
#ifdef HAVE_TF
		lua_->set_usertype("tf", tf_listener, "Transformer", "fawkes::tf");
#endif

		lua_->create_table();
		lua_->set_global("features_env_template");

		std::list<SkillerFeature *>::iterator f;
		for (f = features_.begin(); f != features_.end(); ++f) {
			(*f)->init_lua_context(lua_);
		}

		lua_->set_finalization_calls("skiller.fawkes.finalize()",
		                             "skiller.fawkes.finalize_prepare()",
		                             "skiller.fawkes.finalize_cancel()");

		lua_->set_start_script(LUADIR "/skiller/fawkes/start.lua");

		lua_->add_watcher(this);

	} catch (Exception &e) {
		blackboard->close(skiller_if_);
		delete lua_;
		delete bbo_;
		delete clog_;
		throw;
	}

	// We want to know if our reader leaves and closes the interface
	bbil_add_reader_interface(skiller_if_);
	blackboard->register_listener(this);
}

void
SkillerExecutionThread::finalize()
{
	lua_->remove_watcher(this);

	blackboard->unregister_listener(this);
	blackboard->close(skiller_if_);

	std::list<SkillerFeature *>::iterator f;
	for (f = features_.begin(); f != features_.end(); ++f) {
		(*f)->finalize_lua_context(lua_);
	}

	delete lua_;
	delete clog_;
	delete bbo_;
}

/** Add a skiller feature.
 * Note that this has to be done before the SkillerExecutionThread is initialized
 * at this time (an extension to do this at run-time might follow later).
 * @param feature feature to add
 */
void
SkillerExecutionThread::add_skiller_feature(SkillerFeature *feature)
{
	features_.push_back(feature);
}

void
SkillerExecutionThread::lua_restarted(LuaContext *context)
{
	context->create_table();
	context->set_global("features_env_template");

	std::list<SkillerFeature *>::iterator f;
	for (f = features_.begin(); f != features_.end(); ++f) {
		(*f)->init_lua_context(context);
	}

	// move writing interfaces
	lua_->do_string("return fawkes.interface_initializer.finalize_prepare()");

	context->create_table();

	lua_->push_nil();
	while (lua_->table_next(-2)) {
		void *udata = lua_->to_usertype(-1);
		if (udata) {
			std::string type, id;
			Interface::parse_uid(lua_->to_string(-2), type, id);
			context->do_string("require(\"interfaces.%s\")", type.c_str());
			context->push_string(lua_->to_string(-2));
			context->push_usertype(udata, type.c_str(), "fawkes");
			context->set_table(-3);
			lua_->pop(1);
		}
	}

	context->set_global("interfaces_writing_preload");
}

void
SkillerExecutionThread::bb_interface_reader_removed(Interface *  interface,
                                                    unsigned int instance_serial) throw()
{
	skiller_if_removed_readers_.push_locked(instance_serial);
}

void
SkillerExecutionThread::loop()
{
#ifdef HAVE_INOTIFY
	lua_->process_fam_events();
#endif

	skiller_if_removed_readers_.lock();
	while (!skiller_if_removed_readers_.empty()) {
		lua_->do_string("skiller.fawkes.notify_reader_removed(%u)",
		                skiller_if_removed_readers_.front());
		skiller_if_removed_readers_.pop();
	}
	skiller_if_removed_readers_.unlock();

	lua_->do_string("skillenv.loop()");
}
