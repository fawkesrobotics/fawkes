
/***************************************************************************
 *  act_thread.h - Katana plugin act thread
 *
 *  Created: Mon Jun 08 17:59:57 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *             2010-2014  Bahram Maleki-Fard
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

#ifndef _PLUGINS_KATANA_ACT_THREAD_H_
#define _PLUGINS_KATANA_ACT_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <core/threading/thread.h>
#ifdef HAVE_OPENRAVE
#	include <plugins/openrave/aspect/openrave.h>
#endif
#include <blackboard/interface_listener.h>
#include <core/utils/refptr.h>
#ifdef USE_TIMETRACKER
#	include <utils/time/tracker.h>
#endif
#include <memory>
#include <string>
#include <vector>

namespace fawkes {
class KatanaInterface;
class JointInterface;
class Time;
class KatanaController;
} // namespace fawkes

class KatanaSensorAcquisitionThread;
class KatanaMotionThread;
class KatanaCalibrationThread;
class KatanaGotoThread;
class KatanaGripperThread;
class KatanaMotorControlThread;
class KatanaGotoOpenRaveThread;

class KatanaActThread : public fawkes::Thread,
                        public fawkes::ClockAspect,
                        public fawkes::BlockedTimingAspect,
                        public fawkes::LoggingAspect,
                        public fawkes::ConfigurableAspect,
                        public fawkes::BlackBoardAspect,
                        public fawkes::TransformAspect,
#ifdef HAVE_OPENRAVE
                        public fawkes::OpenRaveAspect,
#endif
                        public fawkes::BlackBoardInterfaceListener
{
public:
	KatanaActThread();
	~KatanaActThread();

	virtual void init();
	virtual void finalize();
	virtual void once();
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
	void stop_motion();
	void update_position(bool refresh);
	void update_sensors(bool refresh);
	void update_motors(bool refresh);
	void start_motion(fawkes::RefPtr<KatanaMotionThread> motion_thread,
	                  unsigned int                       msgid,
	                  const char *                       logmsg,
	                  ...);

private:
	fawkes::KatanaInterface *              katana_if_;
	std::vector<fawkes::JointInterface *> *joint_ifs_;

	std::string  cfg_controller_;
	std::string  cfg_device_;
	std::string  cfg_kni_conffile_;
	bool         cfg_auto_calibrate_;
	unsigned int cfg_defmax_speed_;
	unsigned int cfg_read_timeout_;
	unsigned int cfg_write_timeout_;
	unsigned int cfg_gripper_pollint_;
	unsigned int cfg_goto_pollint_;
	float        cfg_park_x_;
	float        cfg_park_y_;
	float        cfg_park_z_;
	float        cfg_park_phi_;
	float        cfg_park_theta_;
	float        cfg_park_psi_;

	float cfg_distance_scale_;

	float cfg_update_interval_;

	std::string cfg_frame_kni_;
	std::string cfg_frame_gripper_;
	std::string cfg_frame_openrave_;

	bool cfg_OR_enabled_;
#ifdef HAVE_OPENRAVE
	bool        cfg_OR_use_viewer_;
	bool        cfg_OR_auto_load_ik_;
	std::string cfg_OR_robot_file_;
	std::string cfg_OR_arm_model_;
#endif

	fawkes::RefPtr<KatanaSensorAcquisitionThread> sensacq_thread_;
	fawkes::RefPtr<KatanaMotionThread>            actmot_thread_;
	fawkes::RefPtr<KatanaCalibrationThread>       calib_thread_;
	fawkes::RefPtr<KatanaGotoThread>              goto_thread_;
	fawkes::RefPtr<KatanaGripperThread>           gripper_thread_;
	fawkes::RefPtr<KatanaMotorControlThread>      motor_control_thread_;
#ifdef HAVE_OPENRAVE
	fawkes::RefPtr<KatanaGotoOpenRaveThread> goto_openrave_thread_;
#endif

	fawkes::RefPtr<fawkes::KatanaController> katana_;

	fawkes::Time *last_update_;

#ifdef USE_TIMETRACKER
	fawkes::RefPtr<fawkes::TimeTracker> tt_;
	unsigned int                        tt_count_;
	unsigned int                        ttc_read_sensor_;
#endif
};

#endif
