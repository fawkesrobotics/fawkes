
/***************************************************************************
 *  clips_executive_thread.h - CLIPS-based executive plugin
 *
 *  Created: Tue Sep 19 11:59:44 2017
 *  Copyright  2006-2017  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_EXECUTIVE_CLIPS_EXECUTIVE_THREAD_H_
#define __PLUGINS_CLIPS_EXECUTIVE_CLIPS_EXECUTIVE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <plugins/clips/aspect/clips.h>
#include <utils/time/time.h>

#include <clipsmm.h>
#include <memory>

namespace fawkes {
	class ActionSkillMapping;
}

class ClipsExecutiveThread
: public fawkes::Thread,
	public fawkes::BlockedTimingAspect,
	public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::ClockAspect,
	public fawkes::CLIPSAspect
{
 public:
	ClipsExecutiveThread();
	virtual ~ClipsExecutiveThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
	std::string clips_map_skill(std::string name, CLIPS::Values param_names, CLIPS::Values param_values);

 private:
	bool        cfg_assert_time_each_loop_;

	std::shared_ptr<fawkes::ActionSkillMapping> action_skill_mapping_;
};

#endif
