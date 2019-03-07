
/***************************************************************************
 *  dp_thread.h - DirectedPerception pan/tilt unit act thread
 *
 *  Created: Sun Jun 21 17:26:33 2009
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

#ifndef _PLUGINS_PANTILT_DIRPERC_DP_THREAD_H_
#define _PLUGINS_PANTILT_DIRPERC_DP_THREAD_H_

#include "../act_thread.h"

#include <blackboard/interface_listener.h>

#ifdef USE_TIMETRACKER
#	include <utils/time/tracker.h>
#endif
#include <memory>
#include <string>

namespace fawkes {
class PanTiltInterface;
class JointInterface;
} // namespace fawkes

class DirectedPerceptionPTU;

class PanTiltDirectedPerceptionThread : public PanTiltActThread,
                                        public fawkes::BlackBoardInterfaceListener
{
public:
	PanTiltDirectedPerceptionThread(std::string &pantilt_cfg_prefix,
	                                std::string &ptu_cfg_prefix,
	                                std::string &ptu_name);

	virtual void init();
	virtual void finalize();
	virtual void loop();

	// For BlackBoardInterfaceListener
	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message *  message) throw();

	void update_sensor_values();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::PanTiltInterface *pantilt_if_;
	fawkes::JointInterface *  panjoint_if_;
	fawkes::JointInterface *  tiltjoint_if_;

	fawkes::RefPtr<DirectedPerceptionPTU> ptu_;

	std::string  pantilt_cfg_prefix_;
	std::string  ptu_cfg_prefix_;
	std::string  ptu_name_;
	std::string  cfg_device_;
	unsigned int cfg_read_timeout_ms_;

	class WorkerThread : public fawkes::Thread
	{
	public:
		WorkerThread(std::string                           ptu_name,
		             fawkes::Logger *                      logger,
		             fawkes::RefPtr<DirectedPerceptionPTU> ptu);

		~WorkerThread();
		void goto_pantilt(float pan, float tilt);
		void get_pantilt(float &pan, float &tilt);
		bool is_final();
		void stop_motion();
		bool has_fresh_data();
		void reset();

		virtual void loop();

	private:
		void exec_goto_pantilt(float pan, float tilt);

	private:
		fawkes::RefPtr<DirectedPerceptionPTU> ptu_;
		fawkes::Logger *                      logger_;

		float pan_min_;
		float pan_max_;
		float tilt_min_;
		float tilt_max_;

		fawkes::Mutex *move_mutex_;
		bool           move_pending_;
		float          target_pan_;
		float          target_tilt_;

		float cur_pan_;
		float cur_tilt_;

		bool reset_pending_;
		bool fresh_data_;
	};

	WorkerThread *wt_;
};

#endif
