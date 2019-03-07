
/***************************************************************************
 *  logreplay_thread.cpp - BB Log Replay Thread
 *
 *  Created: Wed Feb 17 01:53:00 2010
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
 *             2010  Masrur Doostdar <doostdar@kbsg.rwth-aachen.de>
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

#include "logreplay_thread.h"

#include "file.h"

#include <blackboard/blackboard.h>
#include <blackboard/internal/instance_factory.h>
#include <core/exceptions/system.h>
#include <core/threading/wait_condition.h>
#include <logging/logger.h>
#include <utils/misc/autofree.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <memory>
#ifdef __FreeBSD__
#	include <sys/endian.h>
#elif defined(__MACH__) && defined(__APPLE__)
#	include <sys/_endian.h>
#else
#	include <endian.h>
#endif
#include <arpa/inet.h>
#include <sys/mman.h>

using namespace fawkes;

/** @class BBLogReplayThread "logreplay_thread.h"
 * BlackBoard log Replay thread.
 * Writes the data of the logfile into a blackboard interface, considering the
 * time-step differences between the data.
 * @author Masrur Doostdar
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logfile_name filename of the log to be replayed
 * @param logdir directory containing the logfile
 * @param scenario ID of the log scenario
 * @param grace_period time in seconds that desired offset and loop offset may
 * diverge to still write the new data
 * @param loop_replay specifies if the replay should be looped
 * @param non_blocking do not block the main loop if not enough time has elapsed
 * to replay new data but just wait for the next cycle. This is ignored in
 * continuous thread mode as it could cause busy waiting.
 * @param thread_name initial thread name
 * @param th_opmode thread operation mode
 */
BBLogReplayThread::BBLogReplayThread(const char *           logfile_name,
                                     const char *           logdir,
                                     const char *           scenario,
                                     float                  grace_period,
                                     bool                   loop_replay,
                                     bool                   non_blocking,
                                     const char *           thread_name,
                                     fawkes::Thread::OpMode th_opmode)
: Thread(thread_name, th_opmode)
{
	set_name("BBLogReplayThread(%s)", logfile_name);
	set_prepfin_conc_loop(true);

	logfile_name_     = strdup(logfile_name);
	logdir_           = strdup(logdir);
	scenario_         = strdup(scenario); // dont need this!?
	filename_         = NULL;
	cfg_grace_period_ = grace_period;
	cfg_loop_replay_  = loop_replay;
	if (th_opmode == OPMODE_WAITFORWAKEUP) {
		cfg_non_blocking_ = non_blocking;
	} else {
		// would cause busy waiting
		cfg_non_blocking_ = false;
	}
}

/** Destructor. */
BBLogReplayThread::~BBLogReplayThread()
{
	free(logfile_name_);
	free(logdir_);
	free(scenario_);
}

void
BBLogReplayThread::init()
{
	logfile_   = NULL;
	interface_ = NULL;
	filename_  = NULL;

	if (asprintf(&filename_, "%s/%s", logdir_, logfile_name_) == -1) {
		throw OutOfMemoryException("Cannot re-generate logfile-path");
	}

	try {
		logfile_ = new BBLogFile(filename_, true);
	} catch (Exception &e) {
		finalize();
		throw;
	}

	if (!logfile_->has_next()) {
		finalize();
		throw Exception("Log file %s does not have any entries", filename_);
	}

	interface_ = blackboard->open_for_writing(logfile_->interface_type(), logfile_->interface_id());

	try {
		logfile_->set_interface(interface_);
	} catch (Exception &e) {
		finalize();
		throw;
	}

	logger->log_info(name(), "Replaying from %s:", filename_);
}

void
BBLogReplayThread::finalize()
{
	delete logfile_;
	if (filename_)
		free(filename_);
	blackboard->close(interface_);
}

void
BBLogReplayThread::once()
{
	// Write first immediately, skip first offset
	logfile_->read_next();
	interface_->write();
	last_offset_ = logfile_->entry_offset();
	if (logfile_->has_next()) {
		logfile_->read_next();
		offsetdiff_  = logfile_->entry_offset() - last_offset_;
		last_offset_ = logfile_->entry_offset();
	}
	last_loop_.stamp();
}

void
BBLogReplayThread::loop()
{
	if (logfile_->has_next()) {
		// check if there is time left to wait
		now_.stamp();
		loopdiff_ = now_ - last_loop_;
		if ((offsetdiff_.in_sec() - loopdiff_.in_sec()) > cfg_grace_period_) {
			if (cfg_non_blocking_) {
				// need to keep waiting before posting, but in non-blocking mode
				// just wait for next loop
				return;
			} else {
				waittime_ = offsetdiff_ - loopdiff_;
				waittime_.wait();
			}
		}

		interface_->write();
		logfile_->read_next();

		last_loop_.stamp();
		offsetdiff_  = logfile_->entry_offset() - last_offset_;
		last_offset_ = logfile_->entry_offset();

	} else {
		if (cfg_loop_replay_) {
			logger->log_info(name(), "replay finished, looping");
			logfile_->rewind();
		} else {
			if (opmode() == OPMODE_CONTINUOUS) {
				// block
				logger->log_info(name(), "replay finished, sleeping");
				WaitCondition waitcond;
				waitcond.wait();
			} // else wait will just run once per loop
		}
	}
}
