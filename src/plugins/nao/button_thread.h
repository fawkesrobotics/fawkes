
/***************************************************************************
 *  button_thread.h - Provide Nao buttons to Fawkes
 *
 *  Created: Mon Aug 15 10:48:49 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_NAO_BUTTON_THREAD_H_
#define _PLUGINS_NAO_BUTTON_THREAD_H_

#include <alcommon/alproxy.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <core/utils/lock_vector.h>
#include <interfaces/SwitchInterface.h>
#include <plugins/nao/aspect/naoqi.h>

#include <vector>

namespace AL {
class ALAudioPlayerProxy;
}
namespace fawkes {
class NaoSensorInterface;
class SwitchInterface;
} // namespace fawkes

class NaoQiButtonThread : public fawkes::Thread,
                          public fawkes::LoggingAspect,
                          public fawkes::ConfigurableAspect,
                          public fawkes::BlockedTimingAspect,
                          public fawkes::ClockAspect,
                          public fawkes::BlackBoardAspect,
                          public fawkes::NaoQiAspect
{
public:
	NaoQiButtonThread();
	virtual ~NaoQiButtonThread();

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
	void set_interface(fawkes::SwitchInterface *switch_if,
	                   bool                     enabled,
	                   float                    value,
	                   float                    history,
	                   unsigned int             activations,
	                   unsigned int             short_act,
	                   unsigned int             long_act);

	void process_messages(fawkes::SwitchInterface *switch_if, bool &remote_enabled, float &value);

	void pattern_button_logic(float         value,
	                          float         time_diff_sec,
	                          bool &        enabled,
	                          float &       history,
	                          unsigned int &activations,
	                          unsigned int &short_act,
	                          unsigned int &long_act,
	                          int           sound_short,
	                          int           sound_long);

	void bumpers_logic(float         value,
	                   float         time_diff_sec,
	                   bool &        enabled,
	                   float &       history,
	                   unsigned int &activations,
	                   int           sound_id);

	void process_pattern_button(fawkes::SwitchInterface *switch_if,
	                            float                    sensor_value,
	                            float                    time_diff_sec,
	                            bool &                   remote_enabled,
	                            int                      sound_short = -1,
	                            int                      sound_long  = -1);
	void process_bumpers(fawkes::SwitchInterface *switch_if,
	                     float                    left_value,
	                     float                    right_value,
	                     float                    time_diff_sec,
	                     bool &                   remote_enabled,
	                     int                      sound_id = -1);

private:
	AL::ALPtr<AL::ALAudioPlayerProxy> auplayer_;

	AL::ALProcessSignals::ProcessSignalConnection dcm_sigconn_;

	fawkes::NaoSensorInterface *sensor_if_;
	fawkes::SwitchInterface *   chestbut_if_;
	fawkes::SwitchInterface *   lfoot_bumper_if_;
	fawkes::SwitchInterface *   rfoot_bumper_if_;
	fawkes::SwitchInterface *   head_front_if_;
	fawkes::SwitchInterface *   head_middle_if_;
	fawkes::SwitchInterface *   head_rear_if_;

	fawkes::Time now;
	fawkes::Time last;

	int sound_longpling_;
	int sound_pling_;
	int sound_bumper_left_;
	int sound_bumper_right_;

	bool chestbut_remote_enabled_;
	bool lfoot_bumper_remote_enabled_;
	bool rfoot_bumper_remote_enabled_;
	bool head_front_remote_enabled_;
	bool head_middle_remote_enabled_;
	bool head_rear_remote_enabled_;

	unsigned int last_shutdown_actcount;
	bool         cfg_chest_triple_long_click_shutdown_;
};

#endif
