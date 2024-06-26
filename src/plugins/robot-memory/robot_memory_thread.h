
/***************************************************************************
 *  robot_memory_thread.h - Robot Memory thread
 *
 *  Created: Sun May 01 13:39:52 2016
 *  Copyright  2016  Frederik Zwilling
 *             2017 Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_ROBOT_MEMORY_THREAD_H_
#define _PLUGINS_ROBOT_MEMORY_THREAD_H_

#include "aspect/robot_memory_inifin.h"
#include "computables/blackboard_computable.h"
#include "computables/transform_computable.h"
#include "robot_memory.h"

#include <aspect/aspect_provider.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <core/threading/thread.h>
#include <plugins/mongodb/aspect/mongodb.h>

#include <string>

namespace fawkes {
class Mutex;
class RobotMemoryInterface;
class TimeWait;
#ifdef USE_TIMETRACKER
class TimeTracker;
#endif
} // namespace fawkes

class RobotMemoryThread : public fawkes::Thread,
                          public fawkes::LoggingAspect,
                          public fawkes::ConfigurableAspect,
                          public fawkes::ClockAspect,
                          public fawkes::MongoDBAspect,
                          public fawkes::BlackBoardAspect,
                          public fawkes::TransformAspect,
                          public fawkes::AspectProviderAspect
{
public:
	RobotMemoryThread();
	virtual ~RobotMemoryThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	RobotMemory              *robot_memory;
	fawkes::RobotMemoryIniFin robot_memory_inifin_;
	BlackboardComputable     *blackboard_computable;
	TransformComputable      *transform_computable;

	fawkes::TimeWait *timewait_;

#ifdef USE_TIMETRACKER
	fawkes::TimeTracker *tt_;
	unsigned int         tt_loopcount_;
	unsigned int         ttc_msgproc_;
	unsigned int         ttc_rmloop_;
#endif
};

#endif
