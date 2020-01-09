/***************************************************************************
 *  exec_thread.h - Simulate skill execution
 *
 *  Created: Mon 06 May 2019 08:53:11 CEST 08:53
 *  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#ifndef _PLUGINS_SKILLER_SIMULATOR_EXEC_THREAD_H_
#define _PLUGINS_SKILLER_SIMULATOR_EXEC_THREAD_H_

#include "execution_time_estimator_aspect/execution_time_estimator_aspect.h"
#include "execution_time_estimator_aspect/execution_time_estimator_aspect_inifin.h"

#include <aspect/aspect_provider.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <interfaces/SkillerInterface.h>
#include <plugins/skiller/skiller_feature.h>

class SkillerSimulatorExecutionThread : public fawkes::Thread,
                                        public fawkes::BlockedTimingAspect,
                                        public fawkes::LoggingAspect,
                                        public fawkes::BlackBoardAspect,
                                        public fawkes::ConfigurableAspect,
                                        public fawkes::AspectProviderAspect
{
public:
	SkillerSimulatorExecutionThread();
	virtual void init();
	virtual void loop();
	virtual void finalize();

protected:
	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
	virtual void
	run()
	{
		Thread::run();
	}

private:
	float                     get_skill_runtime(const std::string &skill) const;
	void                      execute_skill(const std::string &skill);
	fawkes::SkillerInterface *skiller_if_;
	float                     default_skill_runtime_;
	fawkes::Time              skill_starttime_;
	fawkes::skiller_simulator::ExecutionTimeEstimatorManager       execution_time_estimator_manager_;
	fawkes::skiller_simulator::ExecutionTimeEstimatorsAspectIniFin provider_inifin_;
};

#endif /* !_PLUGINS_SKILLER_SIMULATOR_EXEC_THREAD_H_ */
