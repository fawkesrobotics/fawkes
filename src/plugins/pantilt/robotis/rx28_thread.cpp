
/***************************************************************************
 *  rx28_thread.cpp - RX28 pan/tilt unit act thread
 *
 *  Created: Thu Jun 18 09:53:49 2009
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "rx28_thread.h"
#include "rx28.h"

#include <utils/math/angle.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/read_write_lock.h>
#include <core/threading/scoped_rwlock.h>
#include <core/threading/wait_condition.h>
#include <interfaces/PanTiltInterface.h>
#include <interfaces/LedInterface.h>
#include <interfaces/JointInterface.h>

#include <cstdarg>
#include <cmath>
#include <unistd.h>

using namespace fawkes;

/** @class PanTiltRX28Thread "rx28_thread.h"
 * PanTilt act thread for RX28 PTU.
 * This thread integrates into the Fawkes main loop at the ACT_EXEC hook and
 * interacts with the controller of the RX28 PTU.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param pantilt_cfg_prefix pantilt plugin configuration prefix
 * @param ptu_cfg_prefix configuration prefix specific for the PTU
 * @param ptu_name name of the PTU configuration
 */
PanTiltRX28Thread::PanTiltRX28Thread(std::string &pantilt_cfg_prefix,
				     std::string &ptu_cfg_prefix,
				     std::string &ptu_name)
  : PanTiltActThread("PanTiltRX28Thread"),
#ifdef HAVE_TF
    TransformAspect(TransformAspect::ONLY_PUBLISHER,
                    (std::string("PTU ") + ptu_name).c_str()),
#endif
    BlackBoardInterfaceListener("PanTiltRX28Thread(%s)", ptu_name.c_str())
{
  set_name("PanTiltRX28Thread(%s)", ptu_name.c_str());

  pantilt_cfg_prefix_ = pantilt_cfg_prefix;
  ptu_cfg_prefix_     = ptu_cfg_prefix;
  ptu_name_           = ptu_name;

  rx28_ = NULL;
}


void
PanTiltRX28Thread::init()
{
  last_pan_ = last_tilt_ = 0.f;
  float init_pan_velocity = 0.f;
  float init_tilt_velocity = 0.f;

  // Note: due to the use of auto_ptr and RefPtr resources are automatically
  // freed on destruction, therefore no special handling is necessary in init()
  // itself!

  cfg_device_           = config->get_string((ptu_cfg_prefix_ + "device").c_str());
  cfg_read_timeout_ms_  = config->get_uint((ptu_cfg_prefix_ + "read_timeout_ms").c_str());
  cfg_disc_timeout_ms_  = config->get_uint((ptu_cfg_prefix_ + "discover_timeout_ms").c_str());
  cfg_pan_servo_id_     = config->get_uint((ptu_cfg_prefix_ + "pan_servo_id").c_str());
  cfg_tilt_servo_id_    = config->get_uint((ptu_cfg_prefix_ + "tilt_servo_id").c_str());
  cfg_pan_offset_       = deg2rad(config->get_float((ptu_cfg_prefix_ + "pan_offset").c_str()));
  cfg_tilt_offset_      = deg2rad(config->get_float((ptu_cfg_prefix_ + "tilt_offset").c_str()));
  cfg_goto_zero_start_  = config->get_bool((ptu_cfg_prefix_ + "goto_zero_start").c_str());
  cfg_turn_off_         = config->get_bool((ptu_cfg_prefix_ + "turn_off").c_str());
  cfg_cw_compl_margin_  = config->get_uint((ptu_cfg_prefix_ + "cw_compl_margin").c_str());
  cfg_ccw_compl_margin_ = config->get_uint((ptu_cfg_prefix_ + "ccw_compl_margin").c_str());
  cfg_cw_compl_slope_   = config->get_uint((ptu_cfg_prefix_ + "cw_compl_slope").c_str());
  cfg_ccw_compl_slope_  = config->get_uint((ptu_cfg_prefix_ + "ccw_compl_slope").c_str());
  cfg_pan_min_          = config->get_float((ptu_cfg_prefix_ + "pan_min").c_str());
  cfg_pan_max_          = config->get_float((ptu_cfg_prefix_ + "pan_max").c_str());
  cfg_tilt_min_         = config->get_float((ptu_cfg_prefix_ + "tilt_min").c_str());
  cfg_tilt_max_         = config->get_float((ptu_cfg_prefix_ + "tilt_max").c_str());
  cfg_pan_margin_       = config->get_float((ptu_cfg_prefix_ + "pan_margin").c_str());
  cfg_tilt_margin_      = config->get_float((ptu_cfg_prefix_ + "tilt_margin").c_str());
  cfg_pan_start_        = config->get_float((ptu_cfg_prefix_ + "pan_start").c_str());
  cfg_tilt_start_       = config->get_float((ptu_cfg_prefix_ + "tilt_start").c_str());
#ifdef HAVE_TF
  cfg_publish_transforms_=config->get_bool((ptu_cfg_prefix_ + "publish_transforms").c_str());
#endif

#ifdef HAVE_TF
  if (cfg_publish_transforms_) {
    float pan_trans_x  =
        config->get_float((ptu_cfg_prefix_ + "pan_trans_x").c_str());
    float pan_trans_y  =
        config->get_float((ptu_cfg_prefix_ + "pan_trans_y").c_str());
    float pan_trans_z  =
        config->get_float((ptu_cfg_prefix_ + "pan_trans_z").c_str());
    float tilt_trans_x =
        config->get_float((ptu_cfg_prefix_ + "tilt_trans_x").c_str());
    float tilt_trans_y =
        config->get_float((ptu_cfg_prefix_ + "tilt_trans_y").c_str());
    float tilt_trans_z =
        config->get_float((ptu_cfg_prefix_ + "tilt_trans_z").c_str());


    std::string frame_id_prefix = std::string("") + ptu_name_;
    try {
      frame_id_prefix =
          config->get_string((ptu_cfg_prefix_ + "frame_id_prefix").c_str());
    } catch (Exception &e) {} // ignore, use default

    cfg_base_frame_ = frame_id_prefix + "/base";
    cfg_pan_link_   = frame_id_prefix + "/pan";
    cfg_tilt_link_  = frame_id_prefix + "/tilt";

    translation_pan_.setValue(pan_trans_x, pan_trans_y, pan_trans_z);
    translation_tilt_.setValue(tilt_trans_x, tilt_trans_y, tilt_trans_z);
  }
#endif

  bool pan_servo_found = false, tilt_servo_found = false;

  rx28_ = new RobotisRX28(cfg_device_.c_str(), cfg_read_timeout_ms_);
  RobotisRX28::DeviceList devl = rx28_->discover();
  for (RobotisRX28::DeviceList::iterator i = devl.begin(); i != devl.end(); ++i) {
    if (cfg_pan_servo_id_ == *i) {
      pan_servo_found  = true;
    } else if (cfg_tilt_servo_id_ == *i) {
      tilt_servo_found = true;
    } else {
      logger->log_warn(name(), "Servo %u in PTU servo chain, but neither "
		       "configured as pan nor as tilt servo", *i);
    }
  }

  // We only want responses to be sent on explicit READ to speed up communication
  rx28_->set_status_return_level(RobotisRX28::BROADCAST_ID, RobotisRX28::SRL_RESPOND_READ);
  // set compliance values
  rx28_->set_compliance_values(RobotisRX28::BROADCAST_ID,
				cfg_cw_compl_margin_, cfg_cw_compl_slope_,
				cfg_ccw_compl_margin_, cfg_ccw_compl_slope_);
  rx28_->set_led_enabled(cfg_pan_servo_id_, false);


  if (! (pan_servo_found && tilt_servo_found)) {
    throw Exception("Pan and/or tilt servo not found: pan: %i  tilt: %i",
		    pan_servo_found, tilt_servo_found);
  }

  // If you have more than one interface: catch exception and close them!
  std::string bbid = "PanTilt " + ptu_name_;
  pantilt_if_ = blackboard->open_for_writing<PanTiltInterface>(bbid.c_str());
  pantilt_if_->set_calibrated(true);
  pantilt_if_->set_min_pan(cfg_pan_min_);
  pantilt_if_->set_max_pan(cfg_pan_max_);
  pantilt_if_->set_min_tilt(cfg_tilt_min_);
  pantilt_if_->set_max_tilt(cfg_tilt_max_);
  pantilt_if_->set_pan_margin(cfg_pan_margin_);
  pantilt_if_->set_tilt_margin(cfg_tilt_margin_);
  pantilt_if_->set_max_pan_velocity(rx28_->get_max_supported_speed(cfg_pan_servo_id_));
  pantilt_if_->set_max_tilt_velocity(rx28_->get_max_supported_speed(cfg_tilt_servo_id_));
  pantilt_if_->set_pan_velocity(init_pan_velocity);
  pantilt_if_->set_tilt_velocity(init_tilt_velocity);
  pantilt_if_->write();

  led_if_ = blackboard->open_for_writing<LedInterface>(bbid.c_str());

  std::string panid = ptu_name_ + " pan";
  panjoint_if_ = blackboard->open_for_writing<JointInterface>(panid.c_str());
  panjoint_if_->set_position(last_pan_);
  panjoint_if_->set_velocity(init_pan_velocity);
  panjoint_if_->write();

  std::string tiltid = ptu_name_ + " tilt";
  tiltjoint_if_ = blackboard->open_for_writing<JointInterface>(tiltid.c_str());
  tiltjoint_if_->set_position(last_tilt_);
  tiltjoint_if_->set_velocity(init_tilt_velocity);
  tiltjoint_if_->write();

  wt_ = new WorkerThread(ptu_name_, logger, rx28_,
			  cfg_pan_servo_id_, cfg_tilt_servo_id_,
			  cfg_pan_min_, cfg_pan_max_, cfg_tilt_min_, cfg_tilt_max_,
			  cfg_pan_offset_, cfg_tilt_offset_);
  wt_->set_margins(cfg_pan_margin_, cfg_tilt_margin_);
  wt_->start();
  wt_->set_enabled(true);
  if ( cfg_goto_zero_start_ ) {
    wt_->goto_pantilt_timed(cfg_pan_start_, cfg_tilt_start_, 3.0);
  }

  bbil_add_message_interface(pantilt_if_);
  bbil_add_message_interface(panjoint_if_);
  bbil_add_message_interface(tiltjoint_if_);
  blackboard->register_listener(this);

#ifdef USE_TIMETRACKER
  tt_.reset(new TimeTracker());
  tt_count_ = 0;
  ttc_read_sensor_ = tt_->add_class("Read Sensor");
#endif  

}


bool
PanTiltRX28Thread::prepare_finalize_user()
{
  if (cfg_turn_off_) {
    logger->log_info(name(), "Moving to park position");
    wt_->goto_pantilt_timed(0, cfg_tilt_max_, 2.0);
    // we need to wait twice, because the first wakeup is likely to happen
    // before the command is actually send
    wt_->wait_for_fresh_data();
    wt_->wait_for_fresh_data();

    while (! wt_->is_final()) {
      //wt_->wakeup();
      wt_->wait_for_fresh_data();
    }
  }
  return true;
}

void
PanTiltRX28Thread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->close(pantilt_if_);
  blackboard->close(led_if_);
  blackboard->close(panjoint_if_);
  blackboard->close(tiltjoint_if_);

  wt_->cancel();
  wt_->join();
  delete wt_;

  if (cfg_turn_off_) {
    logger->log_info(name(), "Turning off PTU");
    try {
      rx28_->set_led_enabled(cfg_pan_servo_id_,  false);
      rx28_->set_led_enabled(cfg_tilt_servo_id_, false);
      rx28_->set_torques_enabled(false, 2, cfg_pan_servo_id_, cfg_tilt_servo_id_);
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to turn of PTU: %s", e.what());
    }
  }
  
  // Setting to NULL deletes instance (RefPtr)
  rx28_ = NULL;
}


/** Update sensor values as necessary.
 * To be called only from PanTiltSensorThread. Writes the current pan/tilt
 * data into the interface.
 */
void
PanTiltRX28Thread::update_sensor_values()
{
  if (wt_->has_fresh_data()) {
    float pan = 0, tilt = 0, panvel=0, tiltvel=0;
    fawkes::Time time;
    wt_->get_pantilt(pan, tilt, time);
    wt_->get_velocities(panvel, tiltvel);

    // poor man's filter: only update if we get a change of least half a degree
    if (fabs(last_pan_ - pan) >= 0.009 || fabs(last_tilt_ - tilt) >= 0.009) {
      last_pan_  = pan;
      last_tilt_ = tilt;
    } else {
      pan  = last_pan_;
      tilt = last_tilt_;
    }

    pantilt_if_->set_pan(pan);
    pantilt_if_->set_tilt(tilt);
    pantilt_if_->set_pan_velocity(panvel);
    pantilt_if_->set_tilt_velocity(tiltvel);
    pantilt_if_->set_enabled(wt_->is_enabled());
    pantilt_if_->set_final(wt_->is_final());
    pantilt_if_->write();

    panjoint_if_->set_position(pan);
    panjoint_if_->set_velocity(panvel);
    panjoint_if_->write();

    tiltjoint_if_->set_position(tilt);
    tiltjoint_if_->set_velocity(tiltvel);
    tiltjoint_if_->write();

#ifdef HAVE_TF
    if (cfg_publish_transforms_) {
      // Always publish updated transforms
      tf::Quaternion pr;  pr.setEulerZYX(pan, 0, 0);
      tf::Transform ptr(pr, translation_pan_);
      tf_publisher->send_transform(ptr, time, cfg_base_frame_, cfg_pan_link_);

      tf::Quaternion tr; tr.setEulerZYX(0, tilt, 0);
      tf::Transform ttr(tr, translation_tilt_);
      tf_publisher->send_transform(ttr, time, cfg_pan_link_, cfg_tilt_link_);
    }
#endif
  }
}


void
PanTiltRX28Thread::loop()
{
  pantilt_if_->set_final(wt_->is_final());

  while (! pantilt_if_->msgq_empty() ) {
    if (pantilt_if_->msgq_first_is<PanTiltInterface::CalibrateMessage>()) {
      // ignored

    } else if (pantilt_if_->msgq_first_is<PanTiltInterface::GotoMessage>()) {
      PanTiltInterface::GotoMessage *msg = pantilt_if_->msgq_first(msg);

      wt_->goto_pantilt(msg->pan(), msg->tilt());
      pantilt_if_->set_msgid(msg->id());
      pantilt_if_->set_final(false);

    } else if (pantilt_if_->msgq_first_is<PanTiltInterface::TimedGotoMessage>()) {
      PanTiltInterface::TimedGotoMessage *msg = pantilt_if_->msgq_first(msg);

      wt_->goto_pantilt_timed(msg->pan(), msg->tilt(), msg->time_sec());
      pantilt_if_->set_msgid(msg->id());
      pantilt_if_->set_final(false);

    } else if (pantilt_if_->msgq_first_is<PanTiltInterface::ParkMessage>()) {
      PanTiltInterface::ParkMessage *msg = pantilt_if_->msgq_first(msg);

      wt_->goto_pantilt(0, 0);
      pantilt_if_->set_msgid(msg->id());
      pantilt_if_->set_final(false);

    } else if (pantilt_if_->msgq_first_is<PanTiltInterface::SetEnabledMessage>()) {
      PanTiltInterface::SetEnabledMessage *msg = pantilt_if_->msgq_first(msg);

      wt_->set_enabled(msg->is_enabled());

    } else if (pantilt_if_->msgq_first_is<PanTiltInterface::SetVelocityMessage>()) {
      PanTiltInterface::SetVelocityMessage *msg = pantilt_if_->msgq_first(msg);

      if (msg->pan_velocity() > pantilt_if_->max_pan_velocity()) {
	logger->log_warn(name(), "Desired pan velocity %f too high, max is %f",
			 msg->pan_velocity(), pantilt_if_->max_pan_velocity());
      } else if (msg->tilt_velocity() > pantilt_if_->max_tilt_velocity()) {
	logger->log_warn(name(), "Desired tilt velocity %f too high, max is %f",
			 msg->tilt_velocity(), pantilt_if_->max_tilt_velocity());
      } else {
	wt_->set_velocities(msg->pan_velocity(), msg->tilt_velocity());
      }

    } else if (pantilt_if_->msgq_first_is<PanTiltInterface::SetMarginMessage>()) {
      PanTiltInterface::SetMarginMessage *msg = pantilt_if_->msgq_first(msg);

      wt_->set_margins(msg->pan_margin(), msg->tilt_margin());
      pantilt_if_->set_pan_margin(msg->pan_margin());
      pantilt_if_->set_tilt_margin(msg->tilt_margin());

    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    pantilt_if_->msgq_pop();
  }

  pantilt_if_->write();

  bool write_led_if = false;
  while (! led_if_->msgq_empty() ) {
    write_led_if = true;
    if (led_if_->msgq_first_is<LedInterface::SetIntensityMessage>()) {
      LedInterface::SetIntensityMessage *msg = led_if_->msgq_first(msg);
      wt_->set_led_enabled((msg->intensity() >= 0.5));
      led_if_->set_intensity((msg->intensity() >= 0.5) ? LedInterface::ON : LedInterface::OFF);
    } else if (led_if_->msgq_first_is<LedInterface::TurnOnMessage>()) {
      wt_->set_led_enabled(true);
      led_if_->set_intensity(LedInterface::ON);
    } else if (led_if_->msgq_first_is<LedInterface::TurnOffMessage>()) {
      wt_->set_led_enabled(false);
      led_if_->set_intensity(LedInterface::OFF);
    }

    led_if_->msgq_pop();
  }
  if (write_led_if)  led_if_->write();

  //wt_->wakeup();
}


bool
PanTiltRX28Thread::bb_interface_message_received(Interface *interface,
						 Message *message) throw()
{
  if (message->is_of_type<PanTiltInterface::StopMessage>()) {
    wt_->stop_motion();
    return false; // do not enqueue StopMessage
  } else if (message->is_of_type<PanTiltInterface::FlushMessage>()) {
    wt_->stop_motion();
    logger->log_info(name(), "Flushing message queue");
    pantilt_if_->msgq_flush();
    return false;
  } else {
    logger->log_info(name(), "Received message of type %s, enqueueing", message->type());
    return true;
  }
}


/** @class PanTiltRX28Thread::WorkerThread "robotis/rx28_thread.h"
 * Worker thread for the PanTiltRX28Thread.
 * This continuous thread issues commands to the RX28 chain. In each loop it
 * will first execute pending operations, and then update the sensor data (lengthy
 * operation). Sensor data will only be updated while either a servo in the chain
 * is still moving or torque is disabled (so the motor can be move manually).
 * @author Tim Niemueller
 */


/** Constructor.
 * @param ptu_name name of the pan/tilt unit
 * @param logger logger
 * @param rx28 RX28 chain
 * @param pan_servo_id servo ID of the pan servo
 * @param tilt_servo_id servo ID of the tilt servo
 * @param pan_min minimum pan in rad
 * @param pan_max maximum pan in rad
 * @param tilt_min minimum tilt in rad
 * @param tilt_max maximum tilt in rad
 * @param pan_offset pan offset from zero in servo ticks
 * @param tilt_offset tilt offset from zero in servo ticks
 */
PanTiltRX28Thread::WorkerThread::WorkerThread(std::string ptu_name,
					      fawkes::Logger *logger,
					      fawkes::RefPtr<RobotisRX28> rx28,
					      unsigned char pan_servo_id,
					      unsigned char tilt_servo_id,
					      float &pan_min, float &pan_max,
					      float &tilt_min, float &tilt_max,
					      float &pan_offset, float &tilt_offset)
  : Thread("", Thread::OPMODE_WAITFORWAKEUP)
{
  set_name("RX28WorkerThread(%s)", ptu_name.c_str());
  set_coalesce_wakeups(true);

  logger_           = logger;

  value_rwlock_     = new ReadWriteLock();
  rx28_rwlock_      = new ReadWriteLock();
  fresh_data_mutex_ = new Mutex();
  update_waitcond_  = new WaitCondition();

  rx28_ = rx28;
  move_pending_     = false;
  target_pan_       = 0;
  target_tilt_      = 0;
  pan_servo_id_     = pan_servo_id;
  tilt_servo_id_    = tilt_servo_id;

  pan_min_          = pan_min;
  pan_max_          = pan_max;
  tilt_min_         = tilt_min;
  tilt_max_         = tilt_max;
  pan_offset_       = pan_offset;
  tilt_offset_      = tilt_offset;
  enable_           = false;
  disable_          = false;
  led_enable_       = false;
  led_disable_      = false;

  max_pan_speed_    = rx28_->get_max_supported_speed(pan_servo_id_);
  max_tilt_speed_   = rx28_->get_max_supported_speed(tilt_servo_id_);
}


/** Destructor. */
PanTiltRX28Thread::WorkerThread::~WorkerThread()
{
  delete value_rwlock_;
  delete rx28_rwlock_;
  delete fresh_data_mutex_;
  delete update_waitcond_;
}


/** Enable or disable servo.
 * @param enabled true to enable servos, false to turn them off
 */
void
PanTiltRX28Thread::WorkerThread::set_enabled(bool enabled)
{
  ScopedRWLock lock(value_rwlock_);
  if (enabled) {
    enable_  = true;
    disable_ = false;
  } else {
    enable_  = false;
    disable_ = true;
  }
  wakeup();
}


/** Enable or disable LED.
 * @param enabled true to enable LED, false to turn it off
 */
void
PanTiltRX28Thread::WorkerThread::set_led_enabled(bool enabled)
{
  ScopedRWLock lock(value_rwlock_);
  if (enabled) {
    led_enable_  = true;
    led_disable_ = false;
  } else {
    led_enable_  = false;
    led_disable_ = true;
  }
  wakeup();
}


/** Stop currently running motion. */
void
PanTiltRX28Thread::WorkerThread::stop_motion()
{
  float pan = 0, tilt = 0;
  get_pantilt(pan, tilt);
  goto_pantilt(pan, tilt);
}


/** Goto desired pan/tilt values.
 * @param pan pan in radians
 * @param tilt tilt in radians
 */
void
PanTiltRX28Thread::WorkerThread::goto_pantilt(float pan, float tilt)
{
  ScopedRWLock lock(value_rwlock_);
  target_pan_   = pan;
  target_tilt_  = tilt;
  move_pending_ = true;
  wakeup();
}


/** Goto desired pan/tilt values in a specified time.
 * @param pan pan in radians
 * @param tilt tilt in radians
 * @param time_sec time when to reach the desired pan/tilt values
 */
void
PanTiltRX28Thread::WorkerThread::goto_pantilt_timed(float pan, float tilt, float time_sec)
{
  target_pan_   = pan;
  target_tilt_  = tilt;
  move_pending_ = true;

  float cpan=0, ctilt=0;
  get_pantilt(cpan, ctilt);

  float pan_diff  = fabs(pan - cpan);
  float tilt_diff = fabs(tilt - ctilt);

  float req_pan_vel  = pan_diff  / time_sec;
  float req_tilt_vel = tilt_diff / time_sec;

  //logger_->log_debug(name(), "Current: %f/%f Des: %f/%f  Time: %f  Diff: %f/%f  ReqVel: %f/%f",
  //		      cpan, ctilt, pan, tilt, time_sec, pan_diff, tilt_diff, req_pan_vel, req_tilt_vel);


  if (req_pan_vel > max_pan_speed_) {
    logger_->log_warn(name(), "Requested move to (%f, %f) in %f sec requires a "
		       "pan speed of %f rad/s, which is greater than the maximum "
		       "of %f rad/s, reducing to max", pan, tilt, time_sec,
		       req_pan_vel, max_pan_speed_);
    req_pan_vel = max_pan_speed_;
  }

  if (req_tilt_vel > max_tilt_speed_) {
    logger_->log_warn(name(), "Requested move to (%f, %f) in %f sec requires a "
		       "tilt speed of %f rad/s, which is greater than the maximum of "
		       "%f rad/s, reducing to max", pan, tilt, time_sec,
		       req_tilt_vel, max_tilt_speed_);
    req_tilt_vel = max_tilt_speed_;
  }

  set_velocities(req_pan_vel, req_tilt_vel);

  wakeup();
}


/** Set desired velocities.
 * @param pan_vel pan velocity
 * @param tilt_vel tilt velocity
 */
void
PanTiltRX28Thread::WorkerThread::set_velocities(float pan_vel, float tilt_vel)
{
  ScopedRWLock lock(value_rwlock_);
  float pan_tmp  = roundf((pan_vel  / max_pan_speed_)  * RobotisRX28::MAX_SPEED);
  float tilt_tmp = roundf((tilt_vel / max_tilt_speed_) * RobotisRX28::MAX_SPEED);

  //logger_->log_debug(name(), "old speed: %u/%u new speed: %f/%f", pan_vel_,
  //		      tilt_vel_, pan_tmp, tilt_tmp);

  if ((pan_tmp >= 0) && (pan_tmp <= RobotisRX28::MAX_SPEED)) {
    pan_vel_ = (unsigned int)pan_tmp;
    velo_pending_ = true;
  } else {
    logger_->log_warn(name(), "Calculated pan value out of bounds, min: 0  max: %u  des: %u",
		       RobotisRX28::MAX_SPEED, (unsigned int)pan_tmp);
  }

  if ((tilt_tmp >= 0) && (tilt_tmp <= RobotisRX28::MAX_SPEED)) {
    tilt_vel_ = (unsigned int)tilt_tmp;
    velo_pending_ = true;
  } else {
    logger_->log_warn(name(), "Calculated tilt value out of bounds, min: 0  max: %u  des: %u",
		       RobotisRX28::MAX_SPEED, (unsigned int)tilt_tmp);
  }
}


/** Get current velocities.
 * @param pan_vel upon return contains current pan velocity
 * @param tilt_vel upon return contains current tilt velocity
 */
void
PanTiltRX28Thread::WorkerThread::get_velocities(float &pan_vel, float &tilt_vel)
{
  unsigned int pan_velticks  = rx28_->get_goal_speed(pan_servo_id_);
  unsigned int tilt_velticks = rx28_->get_goal_speed(tilt_servo_id_);

  pan_velticks  = (unsigned int)(((float)pan_velticks  / (float)RobotisRX28::MAX_SPEED) * max_pan_speed_);
  tilt_velticks = (unsigned int)(((float)tilt_velticks / (float)RobotisRX28::MAX_SPEED) * max_tilt_speed_);
}


/** Set desired velocities.
 * @param pan_margin pan margin
 * @param tilt_margin tilt margin
 */
void
PanTiltRX28Thread::WorkerThread::set_margins(float pan_margin, float tilt_margin)
{
  if (pan_margin  > 0.0)  pan_margin_  = pan_margin;
  if (tilt_margin > 0.0)  tilt_margin_ = tilt_margin;
  //logger_->log_warn(name(), "Margins set to %f, %f", pan_margin_, tilt_margin_);
}


/** Get pan/tilt value.
 * @param pan upon return contains the current pan value
 * @param tilt upon return contains the current tilt value
 */
void
PanTiltRX28Thread::WorkerThread::get_pantilt(float &pan, float &tilt)
{
  ScopedRWLock lock(rx28_rwlock_, ScopedRWLock::LOCK_READ);

  int pan_ticks  = ((int)rx28_->get_position(pan_servo_id_)  - (int)RobotisRX28::CENTER_POSITION);
  int tilt_ticks = ((int)rx28_->get_position(tilt_servo_id_) - (int)RobotisRX28::CENTER_POSITION);

  pan  = pan_ticks *  RobotisRX28::RAD_PER_POS_TICK + pan_offset_;
  tilt = tilt_ticks * RobotisRX28::RAD_PER_POS_TICK + tilt_offset_;
}


/** Get pan/tilt value with time.
 * @param pan upon return contains the current pan value
 * @param tilt upon return contains the current tilt value
 * @param time upon return contains the time the pan and tilt values were read
 */
void
PanTiltRX28Thread::WorkerThread::get_pantilt(float &pan, float &tilt,
                                             fawkes::Time &time)
{
  get_pantilt(pan, tilt);
  time = pantilt_time_;
}


/** Check if motion is final.
 * @return true if motion is final, false otherwise
 */
bool
PanTiltRX28Thread::WorkerThread::is_final()
{
  float pan, tilt;
  get_pantilt(pan, tilt);

  /*
  logger_->log_debug(name(), "P: %f  T: %f  TP: %f  TT: %f  PM: %f  TM: %f  PMov: %i  TMov: %i  Final: %s",
                      pan, tilt, target_pan_, target_tilt_, pan_margin_, tilt_margin_,
                      rx28_->is_moving(pan_servo_id_), rx28_->is_moving(tilt_servo_id_),
                      (( (fabs(pan  - target_pan_)  <= pan_margin_) &&
                         (fabs(tilt - target_tilt_) <= tilt_margin_) ) ||
                       (! rx28_->is_moving(pan_servo_id_) &&
                        ! rx28_->is_moving(tilt_servo_id_))) ? "YES" : "NO");
  */

  ScopedRWLock lock(rx28_rwlock_, ScopedRWLock::LOCK_READ);

  return  ( (fabs(pan  - target_pan_)  <= pan_margin_) &&
	    (fabs(tilt - target_tilt_) <= tilt_margin_) ) ||
          (! rx28_->is_moving(pan_servo_id_) &&
	   ! rx28_->is_moving(tilt_servo_id_));
}


/** Check if PTU is enabled.
 * @return true if torque is enabled for both servos, false otherwise
 */
bool
PanTiltRX28Thread::WorkerThread::is_enabled()
{
  return (rx28_->is_torque_enabled(pan_servo_id_) &&
	  rx28_->is_torque_enabled(tilt_servo_id_));
}


/** Check is fresh sensor data is available.
 * Note that this method will return true at once per sensor update cycle.
 * @return true if fresh data is available, false otherwise
 */
bool
PanTiltRX28Thread::WorkerThread::has_fresh_data()
{
  MutexLocker lock(fresh_data_mutex_);

  bool rv = fresh_data_;
  fresh_data_ = false;
  return rv;
}


void
PanTiltRX28Thread::WorkerThread::loop()
{
  if (enable_) {
    value_rwlock_->lock_for_write();
    enable_  = false;
    value_rwlock_->unlock();
    ScopedRWLock lock(rx28_rwlock_);
    rx28_->set_led_enabled(tilt_servo_id_, true);
    rx28_->set_torques_enabled(true, 2, pan_servo_id_, tilt_servo_id_);
  } else if (disable_) {
    value_rwlock_->lock_for_write();
    disable_ = false;
    value_rwlock_->unlock();
    ScopedRWLock lock(rx28_rwlock_);
    if (led_enable_ || led_disable_ || velo_pending_ || move_pending_) usleep(3000);
  }

  if (led_enable_) {
    value_rwlock_->lock_for_write();
    led_enable_ = false;
    value_rwlock_->unlock();    
    ScopedRWLock lock(rx28_rwlock_);
    rx28_->set_led_enabled(pan_servo_id_, true);
    if (velo_pending_ || move_pending_) usleep(3000);
  } else if (led_disable_) {
    value_rwlock_->lock_for_write();
    led_disable_ = false;
    value_rwlock_->unlock();    
    ScopedRWLock lock(rx28_rwlock_);
    rx28_->set_led_enabled(pan_servo_id_, false);    
    if (velo_pending_ || move_pending_) usleep(3000);
  }

  if (velo_pending_) {
    value_rwlock_->lock_for_write();
    velo_pending_ = false;
    unsigned int pan_vel  = pan_vel_;
    unsigned int tilt_vel = tilt_vel_;
    value_rwlock_->unlock();
    ScopedRWLock lock(rx28_rwlock_);
    rx28_->set_goal_speeds(2, pan_servo_id_, pan_vel, tilt_servo_id_, tilt_vel);
    if (move_pending_) usleep(3000);
  }

  if (move_pending_) {
    value_rwlock_->lock_for_write();
    move_pending_    = false;
    float target_pan  = target_pan_;
    float target_tilt = target_tilt_;
    value_rwlock_->unlock();
    exec_goto_pantilt(target_pan, target_tilt);
  }

  try {
    ScopedRWLock lock(rx28_rwlock_, ScopedRWLock::LOCK_READ);
    rx28_->read_table_values(pan_servo_id_);
    rx28_->read_table_values(tilt_servo_id_);
    {
      MutexLocker lock_fresh_data(fresh_data_mutex_);
      fresh_data_ = true;
      pantilt_time_.stamp();
    }
  } catch (Exception &e) {
    // usually just a timeout, too noisy
    //logger_->log_warn(name(), "Error while reading table values from servos, exception follows");
    //logger_->log_warn(name(), e);
  }

  //if (! is_final() ||
  //    ! rx28_->is_torque_enabled(pan_servo_id_) ||
  //    ! rx28_->is_torque_enabled(tilt_servo_id_)) {
    // while moving, and while the motor is off, wake us up to get new servo
    // position data
    //wakeup();
    //}

  update_waitcond_->wake_all();

  // Wakeup ourselves for faster updates
  wakeup();
}


/** Execute pan/tilt motion.
 * @param pan_rad pan in rad to move to
 * @param tilt_rad tilt in rad to move to
 */
void
PanTiltRX28Thread::WorkerThread::exec_goto_pantilt(float pan_rad, float tilt_rad)
{
  if ( (pan_rad < pan_min_) || (pan_rad > pan_max_) ) {
    logger_->log_warn(name(), "Pan value out of bounds, min: %f  max: %f  des: %f",
		       pan_min_, pan_max_, pan_rad);
    return;
  }
  if ( (tilt_rad < tilt_min_) || (tilt_rad > tilt_max_) ) {
    logger_->log_warn(name(), "Tilt value out of bounds, min: %f  max: %f  des: %f",
		       tilt_min_, tilt_max_, tilt_rad);
    return;
  }

  unsigned int pan_min = 0, pan_max = 0, tilt_min = 0, tilt_max = 0;

  rx28_->get_angle_limits(pan_servo_id_, pan_min, pan_max);
  rx28_->get_angle_limits(tilt_servo_id_, tilt_min, tilt_max);


  int pan_pos  = (int)roundf(RobotisRX28::POS_TICKS_PER_RAD * (pan_rad - pan_offset_))
                 + RobotisRX28::CENTER_POSITION;
  int tilt_pos = (int)roundf(RobotisRX28::POS_TICKS_PER_RAD * (tilt_rad - tilt_offset_))
                 + RobotisRX28::CENTER_POSITION;

  if ( (pan_pos < 0) || ((unsigned int)pan_pos < pan_min) || ((unsigned int)pan_pos > pan_max) ) {
    logger_->log_warn(name(), "Pan position out of bounds, min: %u  max: %u  des: %i",
		       pan_min, pan_max, pan_pos);
    return;
  }

  if ( (tilt_pos < 0) || ((unsigned int)tilt_pos < tilt_min) || ((unsigned int)tilt_pos > tilt_max) ) {
    logger_->log_warn(name(), "Tilt position out of bounds, min: %u  max: %u  des: %i",
		       tilt_min, tilt_max, tilt_pos);
    return;
  }

  ScopedRWLock lock(rx28_rwlock_);
  rx28_->goto_positions(2, pan_servo_id_, pan_pos, tilt_servo_id_, tilt_pos);
}


/** Wait for fresh data to be received.
 * Blocks the calling thread.
 */
void
PanTiltRX28Thread::WorkerThread::wait_for_fresh_data()
{
  update_waitcond_->wait();
}
