
/***************************************************************************
 *  evid100p_thread.h - Sony EviD100P pan/tilt unit act thread
 *
 *  Created: Sun Jun 21 12:30:59 2009
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_PANTILT_SONY_EVID100P_THREAD_H_
#define _PLUGINS_PANTILT_SONY_EVID100P_THREAD_H_

#include "../act_thread.h"

#include <blackboard/interface_listener.h>
#include <interfaces/CameraControlInterface.h>

#ifdef USE_TIMETRACKER
#	include <utils/time/tracker.h>
#endif
#include <memory>
#include <string>

namespace fawkes {
class PanTiltInterface;
class JointInterface;
class SwitchInterface;
} // namespace fawkes

class SonyEviD100PVisca;

class PanTiltSonyEviD100PThread : public PanTiltActThread,
                                  public fawkes::BlackBoardInterfaceListener
{
public:
	PanTiltSonyEviD100PThread(std::string &pantilt_cfg_prefix,
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
	fawkes::PanTiltInterface *      pantilt_if_;
	fawkes::JointInterface *        panjoint_if_;
	fawkes::JointInterface *        tiltjoint_if_;
	fawkes::CameraControlInterface *camctrl_if_;
	fawkes::SwitchInterface *       power_if_;

	fawkes::RefPtr<SonyEviD100PVisca> cam_;

	std::string  pantilt_cfg_prefix_;
	std::string  ptu_cfg_prefix_;
	std::string  ptu_name_;
	std::string  cfg_device_;
	unsigned int cfg_read_timeout_ms_;

	class WorkerThread : public fawkes::Thread
	{
	public:
		WorkerThread(std::string                       ptu_name,
		             fawkes::Logger *                  logger,
		             fawkes::RefPtr<SonyEviD100PVisca> cam,
		             const float &                     pan_min,
		             const float &                     pan_max,
		             const float &                     tilt_min,
		             const float &                     tilt_max);

		~WorkerThread();
		void         set_power(bool powered);
		void         goto_pantilt(float pan, float tilt);
		void         get_pantilt(float &pan, float &tilt);
		void         set_velocities(float pan_vel, float tilt_vel);
		void         set_mirror(bool enabled);
		void         set_zoom(unsigned int zoom_value);
		unsigned int get_zoom();
		void         set_effect(fawkes::CameraControlInterface::Effect effect);
		bool         is_final();
		void         stop_motion();
		bool         has_fresh_data();

		virtual void once();
		virtual void loop();

	private:
		void exec_goto_pantilt(float pan, float tilt);
		void exec_set_zoom(unsigned int zoom);
		void exec_set_effect(fawkes::CameraControlInterface::Effect effect);
		void exec_set_mirror(bool mirror);

	private:
		fawkes::RefPtr<SonyEviD100PVisca> cam_;
		fawkes::Logger *                  logger_;

		fawkes::Mutex *power_mutex_;
		bool           powered_;
		bool           power_pending_;
		bool           power_desired_;

		float pan_min_;
		float pan_max_;
		float tilt_min_;
		float tilt_max_;

		fawkes::Mutex *move_mutex_;
		bool           move_pending_;
		float          target_pan_;
		float          target_tilt_;
		bool           velo_pending_;
		float          pan_vel_;
		float          tilt_vel_;

		fawkes::Mutex *zoom_mutex_;
		bool           zoom_pending_;
		float          target_zoom_;

		fawkes::Mutex *                        effect_mutex_;
		bool                                   effect_pending_;
		fawkes::CameraControlInterface::Effect target_effect_;

		fawkes::Mutex *mirror_mutex_;
		bool           mirror_pending_;
		bool           target_mirror_;

		float cur_pan_;
		float cur_tilt_;

		unsigned int cur_zoom_;

		bool fresh_data_;
	};

	WorkerThread *wt_;
};

#endif
