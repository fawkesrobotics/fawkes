
/***************************************************************************
 *  act_thread.h - Robotino act thread
 *
 *  Created: Sun Nov 13 16:05:34 2011
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
 *             2014       Sebastian Reuter
 *             2014       Tobias Neumann
 *             2017       Till Hofmann
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

#ifndef __PLUGINS_ROBOTINO_ACT_THREAD_H_
#define __PLUGINS_ROBOTINO_ACT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>
#ifdef HAVE_TF
#  include <aspect/tf.h>
#endif

#include <string>

namespace fawkes {
	class MotorInterface;
	class GripperInterface;
	class IMUInterface;
}

class RobotinoComThread;

class RobotinoActThread
: public fawkes::Thread,
	public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::ClockAspect,
#ifdef HAVE_TF
	public fawkes::TransformAspect,
#endif
	public fawkes::BlockedTimingAspect,
	public fawkes::BlackBoardAspect
{
 public:
	RobotinoActThread(RobotinoComThread *com_thread);

	virtual void init();
	virtual void once();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
	typedef enum {
		ODOM_COPY,
		ODOM_CALC
	} OdometryMode;

	void publish_odometry();
	void publish_gripper();

 private:
	RobotinoComThread              *com_;

	unsigned int                    last_seqnum_;
	fawkes::MotorInterface         *motor_if_;
	fawkes::GripperInterface       *gripper_if_;
	fawkes::IMUInterface           *imu_if_;
	unsigned int                    imu_if_nochange_loops_;
	bool                            imu_if_writer_warning_printed_;
	bool                            imu_if_invquat_warning_printed_;
	bool                            imu_if_changed_warning_printed_;
	bool        			  msg_received_;
	bool        			  msg_zero_vel_;
	fawkes::Time 			  last_msg_time_;

	float        			              cfg_deadman_threshold_;
	float        			              cfg_odom_time_offset_;
	bool 				                    cfg_gripper_enabled_;
	std::string                     cfg_odom_frame_;
	std::string                     cfg_base_frame_;
	OdometryMode                    cfg_odom_mode_;
	unsigned int                    cfg_imu_deadman_loops_;
	float                           cfg_odom_corr_phi_;
	float                           cfg_odom_corr_trans_;
	bool                            cfg_bumper_estop_enabled_;	
	float                           cfg_rb_;
	float                           cfg_rw_;
	float                           cfg_gear_;
	float                           cfg_trans_accel_;
	float                           cfg_trans_decel_;
	float                           cfg_rot_accel_;
	float                           cfg_rot_decel_;

#ifdef HAVE_TF
	bool                            cfg_publish_transform_;
#endif

	bool                            gripper_close_;

	float                           odom_x_;
	float                           odom_y_;
	float                           odom_phi_;
	float                           odom_gyro_origin_;
	fawkes::Time                   *odom_time_;

	float                           des_vx_;
	float                           des_vy_;
	float                           des_omega_;
	std::string                     last_transrot_sender_;
};


#endif
