
/***************************************************************************
 *  thread_roomba_500.h - Roomba 500 thread
 *
 *  Created: Sun Jan 02 12:47:35 2011
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_ROOMBA_THREAD_ROOMBA_500_H_
#define _PLUGINS_ROOMBA_THREAD_ROOMBA_500_H_

#include "roomba_500.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/thread_producer.h>
#include <core/threading/thread.h>
#include <core/utils/refptr.h>

namespace fawkes {
class LedInterface;
class SwitchInterface;
class MotorInterface;
class BatteryInterface;
class Roomba500Interface;
} // namespace fawkes

class Roomba500Thread : public fawkes::Thread,
                        public fawkes::BlockedTimingAspect,
                        public fawkes::LoggingAspect,
                        public fawkes::ConfigurableAspect,
                        public fawkes::ClockAspect,
                        public fawkes::BlackBoardAspect
{
public:
	Roomba500Thread();

	virtual void init();
	virtual void finalize();
	virtual void loop();

	void write_blackboard();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void  close_interfaces();
	float led_process(fawkes::LedInterface *iface);
	void  set_mode(Roomba500::Mode mode);

private:
	fawkes::LedInterface *   led_if_debris_;
	fawkes::LedInterface *   led_if_spot_;
	fawkes::LedInterface *   led_if_dock_;
	fawkes::LedInterface *   led_if_check_robot_;
	fawkes::LedInterface *   led_if_clean_color_;
	fawkes::LedInterface *   led_if_clean_intensity_;
	fawkes::SwitchInterface *switch_if_vacuuming_;
	fawkes::SwitchInterface *switch_if_but_clean_;
	fawkes::SwitchInterface *switch_if_but_spot_;
	fawkes::SwitchInterface *switch_if_but_dock_;
	fawkes::SwitchInterface *switch_if_but_minute_;
	fawkes::SwitchInterface *switch_if_but_hour_;
	fawkes::SwitchInterface *switch_if_but_day_;
	fawkes::SwitchInterface *switch_if_but_schedule_;
	fawkes::SwitchInterface *switch_if_but_clock_;
	//fawkes::MotorInterface     *motor_if_;
	fawkes::BatteryInterface *  battery_if_;
	fawkes::Roomba500Interface *roomba500_if_;

	fawkes::RefPtr<Roomba500> roomba_;

	std::string cfg_conntype_;
	std::string cfg_mode_;
	std::string cfg_device_;
	std::string cfg_bttype_;
	bool        cfg_btsave_;
	bool        cfg_btfast_;
	bool        cfg_query_mode_;
	bool        cfg_play_fanfare_;

	unsigned int greeting_loop_count_;

	int battery_percent_;

	class WorkerThread;
	WorkerThread *wt_;
};

#endif
