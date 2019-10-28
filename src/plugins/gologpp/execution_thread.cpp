/***************************************************************************
 *  execution_thread.cpp - Execution thread for Golog++
 *
 *  Created: Mon 26 Aug 2019 CEST 15:38
 *  Copyright  2019  Victor Mataré <matare@fh-aachen.de>
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

#include "execution_thread.h"

#include "exog_manager.h"
#include "gologpp_fawkes_backend.h"

#include <golog++/model/procedural.h>
#include <golog++/parser/parser.h>
#include <golog++/semantics/readylog/execution.h>

namespace fawkes_gpp {

using namespace fawkes;
using namespace gologpp;

const std::string cfg_prefix("/plugins/gologpp");

/** @class GologppThread
 * Main golog++ thread that handles program execution, i.e. spawns an execution
 * context, loads a program and executes it.
 */

GologppThread::GologppThread() : fawkes::Thread("gologpp_agent", Thread::OPMODE_WAITFORWAKEUP)
{
	set_prepfin_conc_loop(true);
}

void
GologppThread::init()
{
	std::string prog_file = config->get_string(cfg_prefix + "/program_path");
	if (prog_file[0] != '/')
		prog_file = SRCDIR "/" + prog_file;

	// Clear the global scope because it's a static variable that may already contain
	// things from a previous (unsuccessful) attempt to initialize this plugin.
	gologpp::global_scope().clear();

	// We know that lists of elementary types will most likely occur, so simply
	// define the types unconditionally. The elementary types themselves are
	// already defined.
	global_scope().register_type(new ListType(*global_scope().lookup_type(BoolType::name())));
	global_scope().register_type(new ListType(*global_scope().lookup_type(NumberType::name())));
	global_scope().register_type(new ListType(*global_scope().lookup_type(SymbolType::name())));

	logger->log_info(name(), "Parsing %s...", prog_file.c_str());
	main_prog_ = gologpp::parser::parse_file(prog_file);
	logger->log_info(name(), "... parsing done");

	logger->log_info(name(), "Initializing ReadyLog context...");

	exog_mgr_ = new ExogManager(this, config, blackboard, logger);

	logger->log_info(name(), "... initialization done");
}

void
GologppThread::once()
{
	gologpp::ReadylogContext::init(
	  {}, std::make_unique<GologppFawkesBackend>(config, logger, blackboard));

	std::lock_guard<std::mutex> l{run_mutex_};
	gologpp::ReadylogContext::instance().run(
	  gologpp::Block{new gologpp::Scope{gologpp::global_scope()}, {main_prog_.release()}});
	logger->log_info(name(), "golog++ main program has ended");
}

bool
GologppThread::prepare_finalize_user()
{
	gologpp::ReadylogContext::instance().terminate();
	std::lock_guard<std::mutex> l{run_mutex_};
	return true;
}

void
GologppThread::finalize()
{
	main_prog_.reset();
	delete exog_mgr_;
	gologpp::global_scope().clear();
	gologpp::ReadylogContext::shutdown();
}

/**
 * @brief GologppThread::gologpp_context
 * @return the currently used golog++ execution context.
 */
gologpp::ExecutionContext &
GologppThread::gologpp_context()
{
	return gologpp::ReadylogContext::instance();
}

} // namespace fawkes_gpp
