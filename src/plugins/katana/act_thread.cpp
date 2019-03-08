
/***************************************************************************
 *  act_thread.cpp - Katana plugin act thread
 *
 *  Created: Mon Jun 08 18:00:56 2009
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

#include "act_thread.h"

#include "calib_thread.h"
#include "controller_kni.h"
#include "controller_openrave.h"
#include "goto_openrave_thread.h"
#include "goto_thread.h"
#include "gripper_thread.h"
#include "motion_thread.h"
#include "motor_control_thread.h"
#include "sensacq_thread.h"

#include <core/threading/mutex_locker.h>
#include <interfaces/JointInterface.h>
#include <interfaces/KatanaInterface.h>
#include <utils/math/angle.h>
#include <utils/time/time.h>

#ifdef HAVE_OPENRAVE
#	include <plugins/openrave/manipulator.h>
#	include <plugins/openrave/robot.h>
#endif

#include <algorithm>
#include <cstdarg>
#include <cstring>

using namespace fawkes;
#ifdef HAVE_TF
using namespace fawkes::tf;
#endif

/** @class KatanaActThread "act_thread.h"
 * Katana act thread.
 * This thread integrates into the Fawkes main loop at the ACT_EXEC hook and
 * interacts with a given controller for the Katana.
 * @author Tim Niemueller
 */

/** Constructor. */
KatanaActThread::KatanaActThread()
: Thread("KatanaActThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC),
  TransformAspect(TransformAspect::BOTH, "Katana"),
  BlackBoardInterfaceListener("KatanaActThread")
{
	last_update_ = new Time();
}

/** Destructor. */
KatanaActThread::~KatanaActThread()
{
	delete last_update_;
}

void
KatanaActThread::init()
{
	// Note: due to the use of auto_ptr and RefPtr resources are automatically
	// freed on destruction, therefore no special handling is necessary in init()
	// itself!

	std::string cfg_prefix = "/hardware/katana/";
	cfg_controller_        = config->get_string((cfg_prefix + "controller").c_str());
	cfg_device_            = config->get_string((cfg_prefix + "device").c_str());
	cfg_kni_conffile_      = config->get_string((cfg_prefix + "kni_conffile").c_str());
	cfg_auto_calibrate_    = config->get_bool((cfg_prefix + "auto_calibrate").c_str());
	cfg_defmax_speed_      = config->get_uint((cfg_prefix + "default_max_speed").c_str());
	cfg_read_timeout_      = config->get_uint((cfg_prefix + "read_timeout_msec").c_str());
	cfg_write_timeout_     = config->get_uint((cfg_prefix + "write_timeout_msec").c_str());
	cfg_gripper_pollint_   = config->get_uint((cfg_prefix + "gripper_pollint_msec").c_str());
	cfg_goto_pollint_      = config->get_uint((cfg_prefix + "goto_pollint_msec").c_str());

	cfg_park_x_     = config->get_float((cfg_prefix + "park_x").c_str());
	cfg_park_y_     = config->get_float((cfg_prefix + "park_y").c_str());
	cfg_park_z_     = config->get_float((cfg_prefix + "park_z").c_str());
	cfg_park_phi_   = config->get_float((cfg_prefix + "park_phi").c_str());
	cfg_park_theta_ = config->get_float((cfg_prefix + "park_theta").c_str());
	cfg_park_psi_   = config->get_float((cfg_prefix + "park_psi").c_str());

	cfg_distance_scale_ = config->get_float((cfg_prefix + "distance_scale").c_str());

	cfg_update_interval_ = config->get_float((cfg_prefix + "update_interval").c_str());

	cfg_frame_kni_      = config->get_string((cfg_prefix + "frame/kni").c_str());
	cfg_frame_gripper_  = config->get_string((cfg_prefix + "frame/gripper").c_str());
	cfg_frame_openrave_ = config->get_string((cfg_prefix + "frame/openrave").c_str());

#ifdef HAVE_OPENRAVE
	cfg_OR_enabled_      = config->get_bool((cfg_prefix + "openrave/enabled").c_str());
	cfg_OR_use_viewer_   = config->get_bool((cfg_prefix + "openrave/use_viewer").c_str());
	cfg_OR_auto_load_ik_ = config->get_bool((cfg_prefix + "openrave/auto_load_ik").c_str());
	cfg_OR_robot_file_   = config->get_string((cfg_prefix + "openrave/robot_file").c_str());
	cfg_OR_arm_model_    = config->get_string((cfg_prefix + "openrave/arm_model").c_str());
#else
	cfg_OR_enabled_ = false;
#endif

	// see if config entries for joints exist
	std::string joint_name;
	for (long long i = 0; i < 5; ++i) {
		joint_name = config->get_string((cfg_prefix + "joints/" + std::to_string(i)).c_str());
		joint_name.clear();
	}
	joint_name = config->get_string((cfg_prefix + "joints/finger_l").c_str());
	joint_name.clear();
	joint_name = config->get_string((cfg_prefix + "joints/finger_r").c_str());
	joint_name.clear();

	last_update_->set_clock(clock);
	last_update_->set_time(0, 0);

	// Load katana controller
	if (cfg_controller_ == "kni") {
		KatanaControllerKni *kat_ctrl = new KatanaControllerKni();
		katana_                       = kat_ctrl;
		try {
			kat_ctrl->setup(cfg_device_, cfg_kni_conffile_, cfg_read_timeout_, cfg_write_timeout_);
		} catch (fawkes::Exception &e) {
			logger->log_warn(name(), "Setup KatanaControllerKni failed. Ex: %s", e.what());
		}
		kat_ctrl = NULL;

	} else if (cfg_controller_ == "openrave") {
#ifdef HAVE_OPENRAVE
		if (!cfg_OR_enabled_) {
			throw fawkes::Exception(
			  "Cannot use controller 'openrave', OpenRAVE is deactivated by config flag!");
		}
		katana_ = new KatanaControllerOpenrave(openrave);
#else
		throw fawkes::Exception("Cannot use controller 'openrave', OpenRAVE not installed!");
#endif

	} else {
		throw fawkes::Exception("Invalid controller given: '%s'", cfg_controller_.c_str());
	}

	// If you have more than one interface: catch exception and close them!
	try {
		katana_if_               = blackboard->open_for_writing<KatanaInterface>("Katana");
		joint_ifs_               = new std::vector<JointInterface *>();
		JointInterface *joint_if = NULL;
		for (long long i = 0; i < 5; ++i) {
			joint_name = config->get_string((cfg_prefix + "joints/" + std::to_string(i)).c_str());
			joint_if   = blackboard->open_for_writing<JointInterface>(joint_name.c_str());
			joint_ifs_->push_back(joint_if);
			joint_name.clear();
		}

		joint_name = config->get_string((cfg_prefix + "joints/finger_l").c_str());
		joint_if   = blackboard->open_for_writing<JointInterface>(joint_name.c_str());
		joint_ifs_->push_back(joint_if);
		joint_name.clear();
		joint_name = config->get_string((cfg_prefix + "joints/finger_r").c_str());
		joint_if   = blackboard->open_for_writing<JointInterface>(joint_name.c_str());
		joint_ifs_->push_back(joint_if);
		joint_name.clear();

		joint_if = NULL;
	} catch (Exception &e) {
		finalize();
		throw;
	}

	// Create all other threads
	sensacq_thread_       = new KatanaSensorAcquisitionThread(katana_, logger);
	calib_thread_         = new KatanaCalibrationThread(katana_, logger);
	gripper_thread_       = new KatanaGripperThread(katana_, logger, cfg_gripper_pollint_);
	motor_control_thread_ = new KatanaMotorControlThread(katana_, logger, cfg_goto_pollint_);
	goto_thread_          = new KatanaGotoThread(katana_, logger, cfg_goto_pollint_);
#ifdef HAVE_OPENRAVE
	goto_openrave_thread_ = new KatanaGotoOpenRaveThread(katana_,
	                                                     logger,
	                                                     openrave,
	                                                     cfg_goto_pollint_,
	                                                     cfg_OR_robot_file_,
	                                                     cfg_OR_arm_model_,
	                                                     cfg_OR_auto_load_ik_,
	                                                     cfg_OR_use_viewer_);
	if (cfg_OR_enabled_) {
		goto_openrave_thread_->init();
	}
#endif

	// Intialize katana controller
	try {
		katana_->init();
		katana_->set_max_velocity(cfg_defmax_speed_);
		logger->log_debug(name(), "Katana successfully initialized");
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(), "Initializing controller failed. Ex: %s", e.what());
		finalize();
		throw;
	}

	sensacq_thread_->start();

	bbil_add_message_interface(katana_if_);
	blackboard->register_listener(this);

#ifdef USE_TIMETRACKER
	tt_.reset(new TimeTracker());
	tt_count_        = 0;
	ttc_read_sensor_ = tt_->add_class("Read Sensor");
#endif
}

void
KatanaActThread::finalize()
{
	try {
		if (actmot_thread_) {
			actmot_thread_->cancel();
			actmot_thread_->join();
			actmot_thread_ = NULL;
		}
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(), "failed finalizing motion thread. Ex:%s", e.what());
	}

	try {
		sensacq_thread_->cancel();
		sensacq_thread_->join();
		sensacq_thread_.reset();
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(), "failed finalizing sensor_aquisition thread. Ex:%s", e.what());
	}

	// Setting to NULL also deletes instance (RefPtr)
	calib_thread_         = NULL;
	goto_thread_          = NULL;
	gripper_thread_       = NULL;
	motor_control_thread_ = NULL;
#ifdef HAVE_OPENRAVE
	if (cfg_OR_enabled_ && goto_openrave_thread_)
		goto_openrave_thread_->finalize();
	goto_openrave_thread_ = NULL;
#endif

	try {
		katana_->stop();
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(), "failed stopping katana. Ex:%s", e.what());
	}
	katana_ = NULL;

	try {
		blackboard->unregister_listener(this);
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(), "failed unregistering blackboard listener. Ex:%s", e.what());
	}

	if (katana_if_)
		blackboard->close(katana_if_);
	for (std::vector<JointInterface *>::iterator it = joint_ifs_->begin(); it != joint_ifs_->end();
	     ++it) {
		blackboard->close(*it);
	}
}

void
KatanaActThread::once()
{
	if (cfg_auto_calibrate_) {
		start_motion(calib_thread_, 0, "Auto calibration enabled, calibrating");
		katana_if_->set_enabled(true);
		katana_if_->write();
	}
}

/** Update position data in BB interface.
 * @param refresh recv new data from arm
 */
void
KatanaActThread::update_position(bool refresh)
{
	try {
		katana_->read_coordinates(refresh);
		if (cfg_controller_ == "kni") {
			katana_if_->set_x(cfg_distance_scale_ * katana_->x());
			katana_if_->set_y(cfg_distance_scale_ * katana_->y());
			katana_if_->set_z(cfg_distance_scale_ * katana_->z());
		} else if (cfg_controller_ == "openrave") {
			if (!tf_listener->frame_exists(cfg_frame_openrave_)) {
				logger->log_warn(name(),
				                 "tf frames not existing: '%s'. Skipping transform to kni coordinates.",
				                 cfg_frame_openrave_.c_str());
			} else {
				Stamped<Point> target;
				Stamped<Point> target_local(Point(katana_->x(), katana_->y(), katana_->z()),
				                            fawkes::Time(0, 0),
				                            cfg_frame_openrave_);

				tf_listener->transform_point(cfg_frame_kni_, target_local, target);

				katana_if_->set_x(target.getX());
				katana_if_->set_y(target.getY());
				katana_if_->set_z(target.getZ());
			}
		}
		katana_if_->set_phi(katana_->phi());
		katana_if_->set_theta(katana_->theta());
		katana_if_->set_psi(katana_->psi());
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(), "Updating position values failed: %s", e.what());
	}

	float *a = katana_if_->angles();

	joint_ifs_->at(0)->set_position(a[0] + M_PI);
	joint_ifs_->at(1)->set_position(a[1]); // + M_PI/2);
	joint_ifs_->at(2)->set_position(a[2] + M_PI);
	joint_ifs_->at(3)->set_position(a[3] - M_PI);
	joint_ifs_->at(4)->set_position(a[4]);
	joint_ifs_->at(5)->set_position(-a[5] / 2.f - 0.5f);
	joint_ifs_->at(6)->set_position(-a[5] / 2.f - 0.5f);
	for (unsigned int i = 0; i < joint_ifs_->size(); ++i) {
		joint_ifs_->at(i)->write();
	}
	/*
  fawkes::Time now(clock);

  static const float  p90 = deg2rad(90);
  static const float p180 = deg2rad(180);

  Transform bs_j1(Quaternion(a[0],      0,        0), Vector3(0, 0, 0.141));
  Transform j1_j2(Quaternion(0,      a[1] -  p90, 0), Vector3(0, 0, 0.064));
  Transform j2_j3(Quaternion(0,      a[2] + p180, 0), Vector3(0, 0, 0.190));
  Transform j3_j4(Quaternion(0,     -a[3] - p180, 0), Vector3(0, 0, 0.139));
  Transform j4_j5(Quaternion(-a[4],     0,        0), Vector3(0, 0, 0.120));
  Transform j5_gr(Quaternion(0,      -p90,        0), Vector3(0, 0, 0.065));

  tf_publisher->send_transform(bs_j1, now, "/katana/base", "/katana/j1");
  tf_publisher->send_transform(j1_j2, now, "/katana/j1", "/katana/j2");
  tf_publisher->send_transform(j2_j3, now, "/katana/j2", "/katana/j3");
  tf_publisher->send_transform(j3_j4, now, "/katana/j3", "/katana/j4");
  tf_publisher->send_transform(j4_j5, now, "/katana/j4", "/katana/j5");
  tf_publisher->send_transform(j5_gr, now, "/katana/j5", "/katana/gripper"); //remember to adjust name in message-processing on change
*/
}

/** Update sensor values as necessary.
 * To be called only from KatanaSensorThread. Makes the local decision whether
 * sensor can be written (calibration is not running) and whether the data
 * needs to be refreshed (no active motion).
 */
void
KatanaActThread::update_sensor_values()
{
	MutexLocker lock(loop_mutex);
	if (actmot_thread_ != calib_thread_) {
		update_sensors(!actmot_thread_);
	}
}

/** Update sensor value in BB interface.
 * @param refresh recv new data from arm
 */
void
KatanaActThread::update_sensors(bool refresh)
{
	try {
		std::vector<int> sensors;
		katana_->get_sensors(sensors, false);

		const int num_sensors = std::min(sensors.size(), katana_if_->maxlenof_sensor_value());
		for (int i = 0; i < num_sensors; ++i) {
			if (sensors.at(i) <= 0) {
				katana_if_->set_sensor_value(i, 0);
			} else if (sensors.at(i) >= 255) {
				katana_if_->set_sensor_value(i, 255);
			} else {
				katana_if_->set_sensor_value(i, sensors.at(i));
			}
		}
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(), "Updating sensor values failed: %s", e.what());
	}

	if (refresh)
		sensacq_thread_->wakeup();
}

/** Update motor encoder and angle data in BB interface.
 * @param refresh recv new data from arm
 */
void
KatanaActThread::update_motors(bool refresh)
{
	try {
		if (katana_->joint_encoders()) {
			std::vector<int> encoders;
			katana_->get_encoders(encoders, refresh);
			for (unsigned int i = 0; i < encoders.size(); i++) {
				katana_if_->set_encoders(i, encoders.at(i));
			}
		}

		if (katana_->joint_angles()) {
			std::vector<float> angles;
			katana_->get_angles(angles, false);
			for (unsigned int i = 0; i < angles.size(); i++) {
				katana_if_->set_angles(i, angles.at(i));
			}
		}
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(), "Updating motor values failed. Ex:%s", e.what());
	}
}

/** Start a motion.
 * @param motion_thread motion thread to start
 * @param msgid BB message  ID of message that caused the motion
 * @param logmsg message to print, format for following arguments
 */
void
KatanaActThread::start_motion(RefPtr<KatanaMotionThread> motion_thread,
                              unsigned int               msgid,
                              const char *               logmsg,
                              ...)
{
	va_list arg;
	va_start(arg, logmsg);
	logger->vlog_debug(name(), logmsg, arg);
	sensacq_thread_->set_enabled(false);
	actmot_thread_ = motion_thread;
	actmot_thread_->start(/* wait */ false);
	katana_if_->set_msgid(msgid);
	katana_if_->set_final(false);
	va_end(arg);
}

/** Stop any running motion. */
void
KatanaActThread::stop_motion()
{
	logger->log_info(name(), "Stopping arm movement");
	loop_mutex->lock();
	if (actmot_thread_) {
		actmot_thread_->cancel();
		actmot_thread_->join();
		actmot_thread_ = NULL;
	}
	try {
		katana_->stop();
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(), "Failed to freeze robot on stop: %s", e.what());
	}
	loop_mutex->unlock();
}

void
KatanaActThread::loop()
{
	if (actmot_thread_) {
		update_motors(/* refresh */ false);
		update_position(/* refresh */ false);
		katana_if_->write();
		if (!actmot_thread_->finished()) {
			return;
		} else {
			logger->log_debug(name(), "Motion thread %s finished, collecting", actmot_thread_->name());
			actmot_thread_->join();
			katana_if_->set_final(true);
			katana_if_->set_error_code(actmot_thread_->error_code());
			if (actmot_thread_ == calib_thread_) {
				katana_if_->set_calibrated(true);
			}
			actmot_thread_->reset();
			actmot_thread_ = NULL;
			logger->log_debug(name(), "Motion thread collected");
			sensacq_thread_->set_enabled(true);

			update_motors(/* refresh */ true);
			update_position(/* refresh */ true);

#ifdef HAVE_OPENRAVE
			if (cfg_OR_enabled_) {
				goto_openrave_thread_->update_openrave_data();
			}
#endif
		}
	} else if (!katana_if_->is_enabled()) {
		update_position(/* refresh */ true);
		update_motors(/* refresh */ true);

	} else {
		// update every once in a while to keep transforms updated
		fawkes::Time now(clock);
		if ((now - last_update_) >= cfg_update_interval_) {
			last_update_->stamp();
			update_position(/* refresh */ false);
			update_motors(/* refresh */ false);
		}
	}

	while (!katana_if_->msgq_empty() && !actmot_thread_) {
		if (katana_if_->msgq_first_is<KatanaInterface::CalibrateMessage>()) {
			KatanaInterface::CalibrateMessage *msg = katana_if_->msgq_first(msg);
			start_motion(calib_thread_, msg->id(), "Calibrating arm");

		} else if (katana_if_->msgq_first_is<KatanaInterface::LinearGotoMessage>()) {
			KatanaInterface::LinearGotoMessage *msg = katana_if_->msgq_first(msg);

			bool trans_frame_exists = tf_listener->frame_exists(msg->trans_frame());
			bool rot_frame_exists   = tf_listener->frame_exists(msg->rot_frame());
			if (!trans_frame_exists || !rot_frame_exists) {
				logger->log_warn(name(),
				                 "tf frames not existing: '%s%s%s'. Skipping message.",
				                 trans_frame_exists ? "" : msg->trans_frame(),
				                 trans_frame_exists ? "" : "', '",
				                 rot_frame_exists ? "" : msg->rot_frame());
			} else {
				Stamped<Point> target;
				Stamped<Point> target_local(Point(msg->x(), msg->y(), msg->z()),
				                            fawkes::Time(0, 0),
				                            msg->trans_frame());
				if (cfg_OR_enabled_) {
#ifdef HAVE_OPENRAVE
					tf_listener->transform_point(cfg_frame_openrave_, target_local, target);
					if (msg->offset_xy() != 0.f) {
						Vector3 offset(target.getX(), target.getY(), 0.f);
						offset = (offset / offset.length())
						         * msg->offset_xy(); // Vector3 does not support a set_length method
						target += offset;
					}
					// TODO: how to transform euler rotation to quaternion, to be used for tf??
					if (strcmp(msg->trans_frame(), cfg_frame_gripper_.c_str()) == 0) {
						goto_openrave_thread_->set_target(
						  msg->x(), msg->y(), msg->z(), msg->phi(), msg->theta(), msg->psi());
						goto_openrave_thread_->set_arm_extension(true);
					} else {
						goto_openrave_thread_->set_target(
						  target.getX(), target.getY(), target.getZ(), msg->phi(), msg->theta(), msg->psi());
					}
					goto_openrave_thread_->set_theta_error(msg->theta_error());
					goto_openrave_thread_->set_move_straight(msg->is_straight());
#	ifdef EARLY_PLANNING
					goto_openrave_thread_->plan_target();
#	endif
					start_motion(
					  goto_openrave_thread_,
					  msg->id(),
					  "Linear movement to (%f,%f,%f, %f,%f,%f), frame '%s', theta_error:%f, straight:%u",
					  target.getX(),
					  target.getY(),
					  target.getZ(),
					  msg->phi(),
					  msg->theta(),
					  msg->psi(),
					  cfg_frame_openrave_.c_str(),
					  msg->theta_error(),
					  msg->is_straight());
#endif
				} else {
					tf_listener->transform_point(cfg_frame_kni_, target_local, target);
					if (msg->offset_xy() != 0.f) {
						Vector3 offset(target.getX(), target.getY(), 0.f);
						offset = (offset / offset.length())
						         * msg->offset_xy(); // Vector3 does not support a set_length method
						target += offset;
					}
					goto_thread_->set_target(target.getX() / cfg_distance_scale_,
					                         target.getY() / cfg_distance_scale_,
					                         target.getZ() / cfg_distance_scale_,
					                         msg->phi(),
					                         msg->theta(),
					                         msg->psi());
					start_motion(goto_thread_,
					             msg->id(),
					             "Linear movement to (%f,%f,%f, %f,%f,%f), frame '%s'",
					             target.getX(),
					             target.getY(),
					             target.getZ(),
					             msg->phi(),
					             msg->theta(),
					             msg->psi(),
					             cfg_frame_kni_.c_str());
				}
			}

		} else if (katana_if_->msgq_first_is<KatanaInterface::LinearGotoKniMessage>()) {
			KatanaInterface::LinearGotoKniMessage *msg = katana_if_->msgq_first(msg);

			float x = msg->x() * cfg_distance_scale_;
			float y = msg->y() * cfg_distance_scale_;
			float z = msg->z() * cfg_distance_scale_;

			tf::Stamped<Point> target;
			tf::Stamped<Point> target_local(tf::Point(x, y, z), fawkes::Time(0, 0), cfg_frame_kni_);

			if (cfg_OR_enabled_) {
#ifdef HAVE_OPENRAVE
				tf_listener->transform_point(cfg_frame_openrave_, target_local, target);
				goto_openrave_thread_->set_target(
				  target.getX(), target.getY(), target.getZ(), msg->phi(), msg->theta(), msg->psi());
#	ifdef EARLY_PLANNING
				goto_openrave_thread_->plan_target();
#	endif
				start_motion(goto_openrave_thread_,
				             msg->id(),
				             "Linear movement to (%f,%f,%f, %f,%f,%f), frame '%s'",
				             target.getX(),
				             target.getY(),
				             target.getZ(),
				             msg->phi(),
				             msg->theta(),
				             msg->psi(),
				             cfg_frame_openrave_.c_str());
#endif
			} else {
				goto_thread_->set_target(
				  msg->x(), msg->y(), msg->z(), msg->phi(), msg->theta(), msg->psi());

				start_motion(goto_thread_,
				             msg->id(),
				             "Linear movement to (%f,%f,%f, %f,%f,%f), frame '%s'",
				             x,
				             y,
				             z,
				             msg->phi(),
				             msg->theta(),
				             msg->psi(),
				             cfg_frame_kni_.c_str());
			}

#ifdef HAVE_OPENRAVE
		} else if (katana_if_->msgq_first_is<KatanaInterface::ObjectGotoMessage>() && cfg_OR_enabled_) {
			KatanaInterface::ObjectGotoMessage *msg = katana_if_->msgq_first(msg);

			float rot_x = 0.f;
			if (msg->rot_x()) {
				rot_x = msg->rot_x();
			}

			goto_openrave_thread_->set_target(msg->object(), rot_x);
#	ifdef EARLY_PLANNING
			goto_openrave_thread_->plan_target();
#	endif
			start_motion(goto_openrave_thread_,
			             msg->id(),
			             "Linear movement to object (%s, %f)",
			             msg->object(),
			             msg->rot_x());
#endif

		} else if (katana_if_->msgq_first_is<KatanaInterface::ParkMessage>()) {
			KatanaInterface::ParkMessage *msg = katana_if_->msgq_first(msg);

			if (cfg_OR_enabled_) {
#ifdef HAVE_OPENRAVE
				tf::Stamped<Point> target;
				tf::Stamped<Point> target_local(tf::Point(cfg_park_x_ * cfg_distance_scale_,
				                                          cfg_park_y_ * cfg_distance_scale_,
				                                          cfg_park_z_ * cfg_distance_scale_),
				                                fawkes::Time(0, 0),
				                                cfg_frame_kni_);
				tf_listener->transform_point(cfg_frame_openrave_, target_local, target);
				goto_openrave_thread_->set_target(target.getX(),
				                                  target.getY(),
				                                  target.getZ(),
				                                  cfg_park_phi_,
				                                  cfg_park_theta_,
				                                  cfg_park_psi_);
#	ifdef EARLY_PLANNING
				goto_openrave_thread_->plan_target();
#	endif
				start_motion(goto_openrave_thread_, msg->id(), "Parking arm");
#endif
			} else {
				goto_thread_->set_target(
				  cfg_park_x_, cfg_park_y_, cfg_park_z_, cfg_park_phi_, cfg_park_theta_, cfg_park_psi_);
				start_motion(goto_thread_, msg->id(), "Parking arm");
			}

		} else if (katana_if_->msgq_first_is<KatanaInterface::OpenGripperMessage>()) {
			KatanaInterface::OpenGripperMessage *msg = katana_if_->msgq_first(msg);
			gripper_thread_->set_mode(KatanaGripperThread::OPEN_GRIPPER);
			start_motion(gripper_thread_, msg->id(), "Opening gripper");

		} else if (katana_if_->msgq_first_is<KatanaInterface::CloseGripperMessage>()) {
			KatanaInterface::CloseGripperMessage *msg = katana_if_->msgq_first(msg);
			gripper_thread_->set_mode(KatanaGripperThread::CLOSE_GRIPPER);
			start_motion(gripper_thread_, msg->id(), "Closing gripper");

		} else if (katana_if_->msgq_first_is<KatanaInterface::SetEnabledMessage>()) {
			KatanaInterface::SetEnabledMessage *msg = katana_if_->msgq_first(msg);

			try {
				if (msg->is_enabled()) {
					logger->log_debug(name(), "Turning ON the arm");
					katana_->turn_on();
					update_position(/* refresh */ true);
					update_motors(/* refresh */ true);
#ifdef HAVE_OPENRAVE
					if (cfg_OR_enabled_)
						goto_openrave_thread_->update_openrave_data();
#endif
				} else {
					logger->log_debug(name(), "Turning OFF the arm");
					katana_->turn_off();
				}
				katana_if_->set_enabled(msg->is_enabled());
			} catch (/*KNI*/ ::Exception &e) {
				logger->log_warn(name(), "Failed enable/disable arm: %s", e.what());
			}

		} else if (katana_if_->msgq_first_is<KatanaInterface::SetMaxVelocityMessage>()) {
			KatanaInterface::SetMaxVelocityMessage *msg = katana_if_->msgq_first(msg);

			unsigned int max_vel = msg->max_velocity();
			if (max_vel == 0)
				max_vel = cfg_defmax_speed_;

			try {
				katana_->set_max_velocity(max_vel);
			} catch (fawkes::Exception &e) {
				logger->log_warn(name(), "Failed setting max velocity. Ex:%s", e.what());
			}
			katana_if_->set_max_velocity(max_vel);

		} else if (katana_if_->msgq_first_is<KatanaInterface::SetPlannerParamsMessage>()) {
			KatanaInterface::SetPlannerParamsMessage *msg = katana_if_->msgq_first(msg);

			if (cfg_OR_enabled_) {
#ifdef HAVE_OPENRAVE
				goto_openrave_thread_->set_plannerparams(msg->plannerparams());
#endif
			}

		} else if (katana_if_->msgq_first_is<KatanaInterface::SetMotorEncoderMessage>()) {
			KatanaInterface::SetMotorEncoderMessage *msg = katana_if_->msgq_first(msg);

			motor_control_thread_->set_encoder(msg->nr(), msg->enc(), false);
			start_motion(motor_control_thread_, msg->id(), "Moving motor");

		} else if (katana_if_->msgq_first_is<KatanaInterface::MoveMotorEncoderMessage>()) {
			KatanaInterface::MoveMotorEncoderMessage *msg = katana_if_->msgq_first(msg);

			motor_control_thread_->set_encoder(msg->nr(), msg->enc(), true);
			start_motion(motor_control_thread_, msg->id(), "Moving motor");

		} else if (katana_if_->msgq_first_is<KatanaInterface::SetMotorAngleMessage>()) {
			KatanaInterface::SetMotorAngleMessage *msg = katana_if_->msgq_first(msg);

			motor_control_thread_->set_angle(msg->nr(), msg->angle(), false);
			start_motion(motor_control_thread_, msg->id(), "Moving motor");

		} else if (katana_if_->msgq_first_is<KatanaInterface::MoveMotorAngleMessage>()) {
			KatanaInterface::MoveMotorAngleMessage *msg = katana_if_->msgq_first(msg);

			motor_control_thread_->set_angle(msg->nr(), msg->angle(), true);
			start_motion(motor_control_thread_, msg->id(), "Moving motor");

		} else {
			logger->log_warn(name(), "Unknown message received");
		}

		katana_if_->msgq_pop();
	}

	katana_if_->write();

#ifdef USE_TIMETRACKER
	if (++tt_count_ > 100) {
		tt_count_ = 0;
		tt_->print_to_stdout();
	}
#endif
}

bool
KatanaActThread::bb_interface_message_received(Interface *interface, Message *message) throw()
{
	if (message->is_of_type<KatanaInterface::StopMessage>()) {
		stop_motion();
		return false; // do not enqueue StopMessage
	} else if (message->is_of_type<KatanaInterface::FlushMessage>()) {
		stop_motion();
		logger->log_info(name(), "Flushing message queue");
		katana_if_->msgq_flush();
		return false;
	} else {
		logger->log_info(name(), "Received message of type %s, enqueueing", message->type());
		return true;
	}
}
