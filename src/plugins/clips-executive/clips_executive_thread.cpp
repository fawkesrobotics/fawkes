
/***************************************************************************
 *  clips_executive_thread.cpp -  CLIPS executive
 *
 *  Created: Tue Sep 19 12:00:06 2017
 *  Copyright  2006-2017  Tim Niemueller [www.niemueller.de]
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

#include "clips_executive_thread.h"

#include <utils/misc/string_conversions.h>
#include <utils/misc/string_split.h>
#include <interfaces/SwitchInterface.h>
#include <core/threading/mutex_locker.h>

using namespace fawkes;

/** @class ClipsExecutiveThread "clips_executive_thread.h"
 * Main thread of CLIPS-based executive.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsExecutiveThread::ClipsExecutiveThread()
	: Thread("ClipsExecutiveThread", Thread::OPMODE_WAITFORWAKEUP),
	  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK),
	  CLIPSAspect("executive", "CLIPS (executive)")
{
}


/** Destructor. */
ClipsExecutiveThread::~ClipsExecutiveThread()
{
}


void
ClipsExecutiveThread::init()
{
	cfg_assert_time_each_loop_ = false;
	try {
		cfg_assert_time_each_loop_ = config->get_bool("/clips-executive/assert-time-each-loop");
	} catch (Exception &e) {} // ignore, use default

	std::vector<std::string> clips_dirs;
	try {
		clips_dirs = config->get_strings("/clips-executive/clips-dirs");
		for (size_t i = 0; i < clips_dirs.size(); ++i) {
			if (clips_dirs[i][clips_dirs[i].size()-1] != '/') {
				clips_dirs[i] += "/";
			}
			logger->log_debug(name(), "DIR: %s", clips_dirs[i].c_str());
		}
	} catch (Exception &e) {} // ignore, use default
	clips_dirs.insert(clips_dirs.begin(), std::string(SRCDIR) + "/clips/");

	MutexLocker lock(clips.objmutex_ptr());

	clips->evaluate(std::string("(path-add-subst \"@BASEDIR@\" \"") + BASEDIR + "\")");
	clips->evaluate(std::string("(path-add-subst \"@FAWKES_BASEDIR@\" \"") +
	                FAWKES_BASEDIR + "\")");
	clips->evaluate(std::string("(path-add-subst \"@RESDIR@\" \"") + RESDIR + "\")");
	clips->evaluate(std::string("(path-add-subst \"@CONFDIR@\" \"") + CONFDIR + "\")");

	for (size_t i = 0; i < clips_dirs.size(); ++i) {
		clips->evaluate("(path-add \"" + clips_dirs[i] + "\")");
	}

	clips->evaluate("(ff-feature-request \"config\")");

	bool cfg_req_redefwarn_feature = true;
	try {
		cfg_req_redefwarn_feature =
			config->get_bool("/clips-executive/request-redefine-warning-feature");
	} catch (Exception &e) {} // ignored, use default
	if (cfg_req_redefwarn_feature) {
		logger->log_debug(name(), "Enabling warnings for redefinitions");
		clips->evaluate("(ff-feature-request \"redefine-warning\")");
	}

	if (!clips->batch_evaluate(SRCDIR"/clips/init.clp")) {
		logger->log_error(name(), "Failed to initialize CLIPS environment, "
		                  "batch file failed.");
		throw Exception("Failed to initialize CLIPS environment, batch file failed.");
	}

	clips->assert_fact("(executive-init)");
	clips->refresh_agenda();
	clips->run();

	started_ = false;
}


void
ClipsExecutiveThread::finalize()
{
	clips->assert_fact("(executive-finalize)");
	clips->refresh_agenda();
	clips->run();
}


void
ClipsExecutiveThread::loop()
{
	MutexLocker lock(clips.objmutex_ptr());

	if (! started_) {
		clips->assert_fact("(start)");
		started_ = true;
	}

	// might be used to trigger loop events
	// must be cleaned up each loop from within the CLIPS code
	if (cfg_assert_time_each_loop_) {
		clips->assert_fact("(time (now))");
	}

	clips->refresh_agenda();
	clips->run();
}
