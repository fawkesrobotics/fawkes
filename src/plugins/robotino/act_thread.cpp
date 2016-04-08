
/***************************************************************************
 *  act_thread.cpp - Robotino act thread
 *
 *  Created: Sun Nov 13 16:07:40 2011
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
 *             2014       Sebastian Reuter
 *             2014       Tobias Neumann
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

#include "act_thread.h"
#include "com_thread.h"

#include <interfaces/MotorInterface.h>
#include <interfaces/GripperInterface.h>
#include <interfaces/IMUInterface.h>
#include <utils/math/angle.h>

using namespace fawkes;

/** @class RobotinoActThread "act_thread.h"
 * Robotino act hook integration thread.
 * This thread integrates into the Fawkes main loop at the ACT hook and
 * executes motion commands.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param com_thread thread that communicates with the hardware base
 */
RobotinoActThread::RobotinoActThread(RobotinoComThread *com_thread)
	: Thread("RobotinoActThread", Thread::OPMODE_WAITFORWAKEUP),
#ifdef HAVE_TF
	  TransformAspect(TransformAspect::ONLY_PUBLISHER, "Robotino Odometry"),
#endif
	  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
	com_ = com_thread;
}


void
RobotinoActThread::init()
{
	last_seqnum_ = 0;
	last_msg_time_ = clock->now();

	//get config values
	cfg_deadman_threshold_    = config->get_float("/hardware/robotino/deadman_time_threshold");
	cfg_gripper_enabled_      = config->get_bool("/hardware/robotino/gripper/enable_gripper");
	cfg_bumper_estop_enabled_ = config->get_bool("/hardware/robotino/bumper/estop_enabled");
	cfg_odom_time_offset_     = config->get_float("/hardware/robotino/odometry/time_offset");
	cfg_odom_frame_           = config->get_string("/hardware/robotino/odometry/frame");
	cfg_base_frame_           = config->get_string("/hardware/robotino/base_frame");
	std::string odom_mode     = config->get_string("/hardware/robotino/odometry/mode");
	cfg_odom_corr_phi_        =
		config->get_float("/hardware/robotino/odometry/calc/correction/phi");
	cfg_odom_corr_trans_      =
		config->get_float("/hardware/robotino/odometry/calc/correction/trans");

	cfg_rb_   = config->get_float("/hardware/robotino/motor-layout/rb");
	cfg_rw_   = config->get_float("/hardware/robotino/motor-layout/rw");
	cfg_gear_ = config->get_float("/hardware/robotino/motor-layout/gear");

	std::string imu_if_id;

	imu_if_ = NULL;
	if (odom_mode == "copy") {
		cfg_odom_mode_ = ODOM_COPY;
	} else if (odom_mode == "calc") {
		cfg_odom_mode_ = ODOM_CALC;
		imu_if_id =
			config->get_string("/hardware/robotino/odometry/calc/imu_interface_id");
		cfg_imu_deadman_loops_ =
			config->get_uint("/hardware/robotino/odometry/calc/imu_deadman_loops");
	} else {
		throw Exception("Invalid odometry mode '%s', must be calc or copy", odom_mode.c_str());
	}

	gripper_close_ = false;

	msg_received_ = false;
	msg_zero_vel_ = false;

	des_vx_    = 0.;
	des_vy_    = 0.;
	des_omega_ = 0.;

	odom_x_ = odom_y_ = odom_phi_ = 0.;
	odom_time_ = new Time(clock);

	motor_if_ = blackboard->open_for_writing<MotorInterface>("Robotino");
	gripper_if_ = blackboard->open_for_writing<GripperInterface>("Robotino");

	if (cfg_odom_mode_ == ODOM_CALC) {
		imu_if_ = blackboard->open_for_reading<IMUInterface>(imu_if_id.c_str());
		imu_if_writer_warning_printed_  = false;
		imu_if_changed_warning_printed_ = false;
		imu_if_invquat_warning_printed_ = false;
		imu_if_nochange_loops_          = 0;
	}

	//state_->emergencyStop.isEnabled = cfg_bumper_estop_enabled_;

	motor_if_->set_motor_state(MotorInterface::MOTOR_ENABLED);
	motor_if_->write();
}


void
RobotinoActThread::finalize()
{
	blackboard->close(imu_if_);
	blackboard->close(motor_if_);
	blackboard->close(gripper_if_);
	com_->set_speed_points(0., 0., 0.);
	com_ = NULL;
	delete odom_time_;
}


void
RobotinoActThread::once()
{
	try {
		com_->set_bumper_estop_enabled(cfg_bumper_estop_enabled_);
	} catch (Exception &e) {}
}

void
RobotinoActThread::loop()
{
	if (! com_->is_connected()) {
		if (! motor_if_->msgq_empty()) {
			logger->log_warn(name(), "Motor commands received while not connected");
			motor_if_->msgq_flush();
		}
		if (! gripper_if_->msgq_empty()) {
			logger->log_warn(name(), "Gripper commands received while not connected");
			gripper_if_->msgq_flush();
		}
		return;
	}

	bool set_speed_points = false;
	float s1 = 0., s2 = 0., s3 = 0.;
	bool reset_odometry = false;

	while (! motor_if_->msgq_empty()) {
		if (MotorInterface::SetMotorStateMessage *msg = motor_if_->msgq_first_safe(msg))
		{
			logger->log_info(name(), "%sabling motor on request",
			                 msg->motor_state() == MotorInterface::MOTOR_ENABLED
			                 ? "En" : "Dis");
			motor_if_->set_motor_state(msg->motor_state());
			motor_if_->write();
		}

		else if (MotorInterface::TransRotMessage *msg = motor_if_->msgq_first_safe(msg))
		{
			project(&s1, &s2, &s3, msg->vx(), msg->vy(), msg->omega());

			des_vx_    = msg->vx();
			des_vy_    = msg->vy();
			des_omega_ = msg->omega();

			set_speed_points = true;
        
			last_msg_time_ = clock->now();
			msg_received_ = true;
	
			msg_zero_vel_ = (s1 == 0.0 && s2 == 0.0 && s3 == 0.0);
		}

		else if (motor_if_->msgq_first_is<MotorInterface::ResetOdometryMessage>())
		{
			odom_x_ = odom_y_ = odom_phi_ = 0.;
			if (imu_if_) {
				imu_if_->read();
				odom_gyro_origin_ = tf::get_yaw(imu_if_->orientation());
			}

			reset_odometry = true;
		}

		motor_if_->msgq_pop();
	} // while motor msgq

	if (cfg_gripper_enabled_) {
		bool open_gripper = false, close_gripper = false;
		while (! gripper_if_->msgq_empty()) {
			if (gripper_if_->msgq_first_is<GripperInterface::OpenGripperMessage>()) {
				close_gripper = false;
				open_gripper  = true;
			} else if (gripper_if_->msgq_first_is<GripperInterface::CloseGripperMessage>()) {
				close_gripper = true;
				open_gripper  = false;
			}
      
			gripper_if_->msgq_pop(); 
		}

		if (close_gripper || open_gripper) {
			gripper_close_ = close_gripper;
			com_->set_gripper(open_gripper);
		}
	} else {
		if (! gripper_if_->msgq_empty())  gripper_if_->msgq_flush();
	}

	// deadman switch to set the velocities to zero if no new message arrives
	double diff =  ( clock->now() - (&last_msg_time_) );
	if (diff >= cfg_deadman_threshold_ && msg_received_ && !msg_zero_vel_) {
		logger->log_error(name(), "Time-Gap between TransRotMsgs too large "
		                  "(%f sec.), motion planner alive?", diff);
		s1 = s2 = s3 = 0.;
		set_speed_points = true;
		msg_received_ = false;
	}

	if (motor_if_->motor_state() == MotorInterface::MOTOR_DISABLED) {
		if (set_speed_points && ((s1 != 0.0) || (s2 != 0.0) || (s3 != 0.0))) {
			logger->log_warn(name(), "Motor command received while disabled, ignoring");
		}
		s1 = s2 = s3 = 0.;
		set_speed_points = true;
	}

	if (set_speed_points)  com_->set_speed_points(s1, s2, s3);
	if (reset_odometry)    com_->reset_odometry();

	publish_odometry();

	if (cfg_gripper_enabled_) {
		publish_gripper();
	}

}


void
RobotinoActThread::publish_odometry()
{
	fawkes::Time sensor_time;
	float a1 = 0., a2 = 0., a3 = 0.;
	unsigned int seq = 0;
	com_->get_act_velocity(a1, a2, a3, seq, sensor_time);

	if (seq != last_seqnum_) {
		last_seqnum_ = seq;

		float vx = 0., vy = 0., omega = 0.;
		unproject(&vx, &vy, &omega, a1, a2, a3);

		motor_if_->set_vx(vx);
		motor_if_->set_vy(vy);
		motor_if_->set_omega(omega);

		motor_if_->set_des_vx(des_vx_);
		motor_if_->set_des_vy(des_vy_);
		motor_if_->set_des_omega(des_omega_);

		if (cfg_odom_mode_ == ODOM_COPY) {
			double x, y, phi;
			com_->get_odometry(x, y, phi);
			odom_x_   = x;
			odom_y_   = y;
			odom_phi_ = phi;
		} else {
			float diff_sec = sensor_time - odom_time_;
			*odom_time_ = sensor_time;

			// velocity-based method
			if (imu_if_ && imu_if_->has_writer()) {
				imu_if_->read();
				if (imu_if_->changed()) {
					//float imu_age = now - imu_if_->timestamp();
					//logger->log_debug(name(), "IMU age: %f sec", imu_age);
					float *ori_q = imu_if_->orientation();
					try {
						tf::Quaternion q(ori_q[0], ori_q[1], ori_q[2], ori_q[3]);
						tf::assert_quaternion_valid(q);

						// reset no change loop count
						imu_if_nochange_loops_ = 0;

						if (imu_if_writer_warning_printed_ ||
						    imu_if_invquat_warning_printed_ ||
						    imu_if_changed_warning_printed_)
						{
							float old_odom_gyro_origin = odom_gyro_origin_;

							// adjust gyro angle for continuity if we used
							// wheel odometry for some time
							// Note that we use the _updated_ odometry, i.e.  we
							// use wheel odometry for the last time frame because
							// we do not have any point of reference for the gyro, yet
							float wheel_odom_phi = normalize_mirror_rad(odom_phi_ + omega * diff_sec);
							odom_gyro_origin_ = tf::get_yaw(q) - wheel_odom_phi;

							if (imu_if_writer_warning_printed_) {
								imu_if_writer_warning_printed_ = false;
								logger->log_info(name(), "IMU writer is back again, "
								                 "adjusted origin to %f (was %f)",
								                 odom_gyro_origin_, old_odom_gyro_origin);
							}

							if (imu_if_changed_warning_printed_) {
								imu_if_changed_warning_printed_ = false;
								logger->log_info(name(), "IMU interface changed again, "
								                 "adjusted origin to %f (was %f)",
								                 odom_gyro_origin_, old_odom_gyro_origin);
							}
							if (imu_if_invquat_warning_printed_) {
								imu_if_invquat_warning_printed_ = false;

								logger->log_info(name(), "IMU quaternion valid again, "
								                 "adjusted origin to %f (was %f)",
								                 odom_gyro_origin_, old_odom_gyro_origin);
							}
						}

						// Yaw taken as asbolute value from IMU, the origin is used
						// to smooth recovery of IMU data (see above) or for
						// odometry reset requests (see message processing)
						odom_phi_ =
							normalize_mirror_rad(tf::get_yaw(q) - odom_gyro_origin_);

					} catch (Exception &e) {
						if (! imu_if_invquat_warning_printed_) {
							imu_if_invquat_warning_printed_ = true;
							logger->log_warn(name(), "Invalid gyro quaternion (%f,%f,%f,%f), "
							                 "falling back to wheel odometry",
							                 ori_q[0], ori_q[1], ori_q[2], ori_q[3]);
						}
						odom_phi_ =
							normalize_mirror_rad(odom_phi_ + omega * diff_sec * cfg_odom_corr_phi_);
					}
				} else {
					if (++imu_if_nochange_loops_ > cfg_imu_deadman_loops_) {
						if (! imu_if_changed_warning_printed_) {
							imu_if_changed_warning_printed_ = true;
							logger->log_warn(name(), "IMU interface not changed, "
							                 "falling back to wheel odometry");
						}
						odom_phi_ =
							normalize_mirror_rad(odom_phi_ + omega * diff_sec * cfg_odom_corr_phi_);
					} // else use previous odometry yaw value
				}
			} else {
				if (! imu_if_writer_warning_printed_) {
					logger->log_warn(name(), "No writer for IMU interface, "
					                 "using wheel odometry only");
					imu_if_writer_warning_printed_ = true;
				}

				odom_phi_ =
					normalize_mirror_rad(odom_phi_ + omega * diff_sec * cfg_odom_corr_phi_);
			}

			odom_x_ += cos(odom_phi_) * vx * diff_sec * cfg_odom_corr_trans_ - sin(odom_phi_) * vy * diff_sec * cfg_odom_corr_trans_;
			odom_y_ += sin(odom_phi_) * vx * diff_sec * cfg_odom_corr_trans_ + cos(odom_phi_) * vy * diff_sec * cfg_odom_corr_trans_;
		}


		motor_if_->set_odometry_position_x(odom_x_);
		motor_if_->set_odometry_position_y(odom_y_);
		motor_if_->set_odometry_orientation(odom_phi_);
		motor_if_->write();

#ifdef HAVE_TF
		tf::Transform t(tf::Quaternion(tf::Vector3(0,0,1), odom_phi_),
		                tf::Vector3(odom_x_, odom_y_, 0.));

		try {
			tf_publisher->send_transform(t, sensor_time + cfg_odom_time_offset_,
			                             cfg_odom_frame_, cfg_base_frame_);
		} catch (Exception &e) {
			logger->log_warn(name(), "Failed to publish odometry transform for "
			                 "(%f,%f,%f), exception follows",
			                 odom_x_, odom_y_, odom_phi_);
			logger->log_warn(name(), e);
		}
#endif
	}
}

void
RobotinoActThread::publish_gripper()
{
	if (com_->is_gripper_open()) {
		gripper_if_->set_gripper_state(GripperInterface::CLOSED);
		gripper_if_->write();
	} else {
		gripper_if_->set_gripper_state(GripperInterface::OPEN);
		gripper_if_->write();
	}
}

/** Project the velocity of the robot in cartesian coordinates to single motor speeds.
 *
 * From OpenRobotino API2 (C) REC Robotics Equipment Corporation GmbH, Planegg, Germany.
 * The code has been released under a 2-clause BSD license.
 *
 * @param m1		The resulting speed of motor 1 in rpm
 * @param m2		The resulting speed of motor 2 in rpm
 * @param m3		The resulting speed of motor 3 in rpm
 * @param vx		Velocity in x-direction in m/s
 * @param vy		Velocity in y-direction in m/s
 * @param omega	Angular velocity in rad/s
 */
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//1) Redistributions of source code must retain the above copyright
//notice, this list of conditions and the following disclaimer.
//2) Redistributions in binary form must reproduce the above copyright
//notice, this list of conditions and the following disclaimer in the
//documentation and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY REC ROBOTICS EQUIPMENT CORPORATION GMBH
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REC
//ROBOTICS EQUIPMENT CORPORATION GMBH BE LIABLE FOR ANY DIRECT,
//INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
//STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
//OF THE POSSIBILITY OF SUCH DAMAGE.
void
RobotinoActThread::project( float* m1, float* m2, float* m3, float vx, float vy, float omega ) const
{
	//Projection matrix
	static const double v0[2] = { -0.5 * sqrt( 3.0 ),  0.5 };
	static const double v1[2] = {  0.0              , -1.0 };
	static const double v2[2] = {  0.5 * sqrt( 3.0 ),  0.5 };

	//Scale omega with the radius of the robot
	double vOmegaScaled = cfg_rb_ * (double)omega ;

	//Convert from m/s to RPM
	const double k = 60.0 * cfg_gear_ / ( 2.0 * M_PI * cfg_rw_ );

	//Compute the desired velocity
	*m1 = static_cast<float>( ( v0[0] * (double)vx + v0[1] * (double)vy + vOmegaScaled ) * k );
	*m2 = static_cast<float>( ( v1[0] * (double)vx + v1[1] * (double)vy + vOmegaScaled ) * k );
	*m3 = static_cast<float>( ( v2[0] * (double)vx + v2[1] * (double)vy + vOmegaScaled ) * k );
}

/** Project single motor speeds to velocity in cartesian coordinates.
 *
 * From OpenRobotino API2 (C) REC Robotics Equipment Corporation GmbH, Planegg, Germany.
 * The code has been released under a 2-clause BSD license.
 *
 * @param vx		The resulting speed in x-direction in m/s
 * @param vy		The resulting speed in y-direction in m/s
 * @param omega	The resulting angular velocity in rad/s
 * @param m1		Speed of motor 1 in rpm
 * @param m2		Speed of motor 2 in rpm
 * @param m3		Speed of motor 3 in rpm
 * @throws		RobotinoException if no valid drive layout parameters are available.
 */
void
RobotinoActThread::unproject( float* vx, float* vy, float* omega, float m1, float m2, float m3 ) const
{
	//Convert from RPM to mm/s
	const double k = 60.0 * cfg_gear_ / ( 2.0 * M_PI * cfg_rw_ );

	*vx = static_cast<float>( ( (double)m3 - (double)m1 ) / sqrt( 3.0 ) / k );
	*vy = static_cast<float>( 2.0 / 3.0 * ( (double)m1 + 0.5 * ( (double)m3 - (double)m1 ) - (double)m2 ) / k );

	double vw = (double)*vy + (double)m2 / k;

	*omega = static_cast<float>( vw / cfg_rb_ );
}
