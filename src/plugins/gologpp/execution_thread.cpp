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

#include "gologpp_fawkes_backend.h"

#include <golog++/model/procedural.h>
#include <golog++/parser/parser.h>
#include <golog++/semantics/readylog/execution.h>

namespace fawkes_gpp {

using namespace fawkes;

const std::string cfg_prefix("/plugins/gologpp");

/** @class GologppThread
 *  The thread that runs Golog++.
 */

GologppThread::GologppThread() : fawkes::Thread("gologpp_agent", Thread::OPMODE_WAITFORWAKEUP)
{
}

void
GologppThread::init()
{
	std::string prog_file = config->get_string(cfg_prefix + "/program_path");
	if (prog_file[0] != '/')
		prog_file = SRCDIR "/" + prog_file;

	logger->log_info(name(), "Parsing %s...", prog_file.c_str());
	main_prog_ = gologpp::parser::parse_file(prog_file);
	logger->log_info(name(), "... parsing done");

	logger->log_info(name(), "Initializing ReadyLog context...");
	gologpp::ReadylogContext::init({},
	                               std::make_unique<GologppFawkesBackend>(this, logger, blackboard));
	logger->log_info(name(), "... initialization done");
}

void
GologppThread::once()
{
	gologpp::ReadylogContext::instance().run(
	  gologpp::Block{new gologpp::Scope{gologpp::global_scope()}, {main_prog_.release()}});
}

void
GologppThread::finalize()
{
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
