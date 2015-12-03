
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

  __pantilt_cfg_prefix = pantilt_cfg_prefix;
  __ptu_cfg_prefix     = ptu_cfg_prefix;
  __ptu_name           = ptu_name;

  __rx28 = NULL;
}


void
PanTiltRX28Thread::init()
{
  __last_pan = __last_tilt = 0.f;
  float init_pan_velocity = 0.f;
  float init_tilt_velocity = 0.f;

  // Note: due to the use of auto_ptr and RefPtr resources are automatically
  // freed on destruction, therefore no special handling is necessary in init()
  // itself!

  __cfg_device           = config->get_string((__ptu_cfg_prefix + "device").c_str());
  __cfg_read_timeout_ms  = config->get_uint((__ptu_cfg_prefix + "read_timeout_ms").c_str());
  __cfg_disc_timeout_ms  = config->get_uint((__ptu_cfg_prefix + "discover_timeout_ms").c_str());
  __cfg_pan_servo_id     = config->get_uint((__ptu_cfg_prefix + "pan_servo_id").c_str());
  __cfg_tilt_servo_id    = config->get_uint((__ptu_cfg_prefix + "tilt_servo_id").c_str());
  __cfg_pan_offset       = deg2rad(config->get_float((__ptu_cfg_prefix + "pan_offset").c_str()));
  __cfg_tilt_offset      = deg2rad(config->get_float((__ptu_cfg_prefix + "tilt_offset").c_str()));
  __cfg_goto_zero_start  = config->get_bool((__ptu_cfg_prefix + "goto_zero_start").c_str());
  __cfg_turn_off         = config->get_bool((__ptu_cfg_prefix + "turn_off").c_str());
  __cfg_cw_compl_margin  = config->get_uint((__ptu_cfg_prefix + "cw_compl_margin").c_str());
  __cfg_ccw_compl_margin = config->get_uint((__ptu_cfg_prefix + "ccw_compl_margin").c_str());
  __cfg_cw_compl_slope   = config->get_uint((__ptu_cfg_prefix + "cw_compl_slope").c_str());
  __cfg_ccw_compl_slope  = config->get_uint((__ptu_cfg_prefix + "ccw_compl_slope").c_str());
  __cfg_pan_min          = config->get_float((__ptu_cfg_prefix + "pan_min").c_str());
  __cfg_pan_max          = config->get_float((__ptu_cfg_prefix + "pan_max").c_str());
  __cfg_tilt_min         = config->get_float((__ptu_cfg_prefix + "tilt_min").c_str());
  __cfg_tilt_max         = config->get_float((__ptu_cfg_prefix + "tilt_max").c_str());
  __cfg_pan_margin       = config->get_float((__ptu_cfg_prefix + "pan_margin").c_str());
  __cfg_tilt_margin      = config->get_float((__ptu_cfg_prefix + "tilt_margin").c_str());
  __cfg_pan_start        = config->get_float((__ptu_cfg_prefix + "pan_start").c_str());
  __cfg_tilt_start       = config->get_float((__ptu_cfg_prefix + "tilt_start").c_str());
#ifdef HAVE_TF
  __cfg_publish_transforms=config->get_bool((__ptu_cfg_prefix + "publish_transforms").c_str());
#endif

#ifdef HAVE_TF
  if (__cfg_publish_transforms) {
    float pan_trans_x  =
        config->get_float((__ptu_cfg_prefix + "pan_trans_x").c_str());
    float pan_trans_y  =
        config->get_float((__ptu_cfg_prefix + "pan_trans_y").c_str());
    float pan_trans_z  =
        config->get_float((__ptu_cfg_prefix + "pan_trans_z").c_str());
    float tilt_trans_x =
        config->get_float((__ptu_cfg_prefix + "tilt_trans_x").c_str());
    float tilt_trans_y =
        config->get_float((__ptu_cfg_prefix + "tilt_trans_y").c_str());
    float tilt_trans_z =
        config->get_float((__ptu_cfg_prefix + "tilt_trans_z").c_str());


    std::string frame_id_prefix = std::string("") + __ptu_name;
    try {
      frame_id_prefix =
          config->get_string((__ptu_cfg_prefix + "frame_id_prefix").c_str());
    } catch (Exception &e) {} // ignore, use default

    __cfg_base_frame = frame_id_prefix + "/base";
    __cfg_pan_link   = frame_id_prefix + "/pan";
    __cfg_tilt_link  = frame_id_prefix + "/tilt";

    __translation_pan.setValue(pan_trans_x, pan_trans_y, pan_trans_z);
    __translation_tilt.setValue(tilt_trans_x, tilt_trans_y, tilt_trans_z);
  }
#endif

  bool pan_servo_found = false, tilt_servo_found = false;

  __rx28 = new RobotisRX28(__cfg_device.c_str(), __cfg_read_timeout_ms);
  RobotisRX28::DeviceList devl = __rx28->discover();
  for (RobotisRX28::DeviceList::iterator i = devl.begin(); i != devl.end(); ++i) {
    if (__cfg_pan_servo_id == *i) {
      pan_servo_found  = true;
    } else if (__cfg_tilt_servo_id == *i) {
      tilt_servo_found = true;
    } else {
      logger->log_warn(name(), "Servo %u in PTU servo chain, but neither "
		       "configured as pan nor as tilt servo", *i);
    }
  }

  // We only want responses to be sent on explicit READ to speed up communication
  __rx28->set_status_return_level(RobotisRX28::BROADCAST_ID, RobotisRX28::SRL_RESPOND_READ);
  // set compliance values
  __rx28->set_compliance_values(RobotisRX28::BROADCAST_ID,
				__cfg_cw_compl_margin, __cfg_cw_compl_slope,
				__cfg_ccw_compl_margin, __cfg_ccw_compl_slope);
  __rx28->set_led_enabled(__cfg_pan_servo_id, false);


  if (! (pan_servo_found && tilt_servo_found)) {
    throw Exception("Pan and/or tilt servo not found: pan: %i  tilt: %i",
		    pan_servo_found, tilt_servo_found);
  }

  // If you have more than one interface: catch exception and close them!
  std::string bbid = "PanTilt " + __ptu_name;
  __pantilt_if = blackboard->open_for_writing<PanTiltInterface>(bbid.c_str());
  __pantilt_if->set_calibrated(true);
  __pantilt_if->set_min_pan(__cfg_pan_min);
  __pantilt_if->set_max_pan(__cfg_pan_max);
  __pantilt_if->set_min_tilt(__cfg_tilt_min);
  __pantilt_if->set_max_tilt(__cfg_tilt_max);
  __pantilt_if->set_pan_margin(__cfg_pan_margin);
  __pantilt_if->set_tilt_margin(__cfg_tilt_margin);
  __pantilt_if->set_max_pan_velocity(__rx28->get_max_supported_speed(__cfg_pan_servo_id));
  __pantilt_if->set_max_tilt_velocity(__rx28->get_max_supported_speed(__cfg_tilt_servo_id));
  __pantilt_if->set_pan_velocity(init_pan_velocity);
  __pantilt_if->set_tilt_velocity(init_tilt_velocity);
  __pantilt_if->write();

  __led_if = blackboard->open_for_writing<LedInterface>(bbid.c_str());

  std::string panid = __ptu_name + " pan";
  __panjoint_if = blackboard->open_for_writing<JointInterface>(panid.c_str());
  __panjoint_if->set_position(__last_pan);
  __panjoint_if->set_velocity(init_pan_velocity);
  __panjoint_if->write();

  std::string tiltid = __ptu_name + " tilt";
  __tiltjoint_if = blackboard->open_for_writing<JointInterface>(tiltid.c_str());
  __tiltjoint_if->set_position(__last_tilt);
  __tiltjoint_if->set_velocity(init_tilt_velocity);
  __tiltjoint_if->write();

  __wt = new WorkerThread(__ptu_name, logger, __rx28,
			  __cfg_pan_servo_id, __cfg_tilt_servo_id,
			  __cfg_pan_min, __cfg_pan_max, __cfg_tilt_min, __cfg_tilt_max,
			  __cfg_pan_offset, __cfg_tilt_offset);
  __wt->set_margins(__cfg_pan_margin, __cfg_tilt_margin);
  __wt->start();
  __wt->set_enabled(true);
  if ( __cfg_goto_zero_start ) {
    __wt->goto_pantilt_timed(__cfg_pan_start, __cfg_tilt_start, 3.0);
  }

  bbil_add_message_interface(__pantilt_if);
  bbil_add_message_interface(__panjoint_if);
  bbil_add_message_interface(__tiltjoint_if);
  blackboard->register_listener(this);

#ifdef USE_TIMETRACKER
  __tt.reset(new TimeTracker());
  __tt_count = 0;
  __ttc_read_sensor = __tt->add_class("Read Sensor");
#endif  

}


bool
PanTiltRX28Thread::prepare_finalize_user()
{
  if (__cfg_turn_off) {
    logger->log_info(name(), "Moving to park position");
    __wt->goto_pantilt_timed(0, __cfg_tilt_max, 2.0);
    // we need to wait twice, because the first wakeup is likely to happen
    // before the command is actually send
    __wt->wait_for_fresh_data();
    __wt->wait_for_fresh_data();

    while (! __wt->is_final()) {
      //__wt->wakeup();
      __wt->wait_for_fresh_data();
    }
  }
  return true;
}

void
PanTiltRX28Thread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->close(__pantilt_if);
  blackboard->close(__led_if);
  blackboard->close(__panjoint_if);
  blackboard->close(__tiltjoint_if);

  __wt->cancel();
  __wt->join();
  delete __wt;

  if (__cfg_turn_off) {
    logger->log_info(name(), "Turning off PTU");
    try {
      __rx28->set_led_enabled(__cfg_pan_servo_id,  false);
      __rx28->set_led_enabled(__cfg_tilt_servo_id, false);
      __rx28->set_torques_enabled(false, 2, __cfg_pan_servo_id, __cfg_tilt_servo_id);
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to turn of PTU: %s", e.what());
    }
  }
  
  // Setting to NULL deletes instance (RefPtr)
  __rx28 = NULL;
}


/** Update sensor values as necessary.
 * To be called only from PanTiltSensorThread. Writes the current pan/tilt
 * data into the interface.
 */
void
PanTiltRX28Thread::update_sensor_values()
{
  if (__wt->has_fresh_data()) {
    float pan = 0, tilt = 0, panvel=0, tiltvel=0;
    fawkes::Time time;
    __wt->get_pantilt(pan, tilt, time);
    __wt->get_velocities(panvel, tiltvel);

    // poor man's filter: only update if we get a change of least half a degree
    if (fabs(__last_pan - pan) >= 0.009 || fabs(__last_tilt - tilt) >= 0.009) {
      __last_pan  = pan;
      __last_tilt = tilt;
    } else {
      pan  = __last_pan;
      tilt = __last_tilt;
    }

    __pantilt_if->set_pan(pan);
    __pantilt_if->set_tilt(tilt);
    __pantilt_if->set_pan_velocity(panvel);
    __pantilt_if->set_tilt_velocity(tiltvel);
    __pantilt_if->set_enabled(__wt->is_enabled());
    __pantilt_if->set_final(__wt->is_final());
    __pantilt_if->write();

    __panjoint_if->set_position(pan);
    __panjoint_if->set_velocity(panvel);
    __panjoint_if->write();

    __tiltjoint_if->set_position(tilt);
    __tiltjoint_if->set_velocity(tiltvel);
    __tiltjoint_if->write();

#ifdef HAVE_TF
    if (__cfg_publish_transforms) {
      // Always publish updated transforms
      tf::Quaternion pr;  pr.setEulerZYX(pan, 0, 0);
      tf::Transform ptr(pr, __translation_pan);
      tf_publisher->send_transform(ptr, time, __cfg_base_frame, __cfg_pan_link);

      tf::Quaternion tr; tr.setEulerZYX(0, tilt, 0);
      tf::Transform ttr(tr, __translation_tilt);
      tf_publisher->send_transform(ttr, time, __cfg_pan_link, __cfg_tilt_link);
    }
#endif
  }
}


void
PanTiltRX28Thread::loop()
{
  __pantilt_if->set_final(__wt->is_final());

  while (! __pantilt_if->msgq_empty() ) {
    if (__pantilt_if->msgq_first_is<PanTiltInterface::CalibrateMessage>()) {
      // ignored

    } else if (__pantilt_if->msgq_first_is<PanTiltInterface::GotoMessage>()) {
      PanTiltInterface::GotoMessage *msg = __pantilt_if->msgq_first(msg);

      __wt->goto_pantilt(msg->pan(), msg->tilt());
      __pantilt_if->set_msgid(msg->id());
      __pantilt_if->set_final(false);

    } else if (__pantilt_if->msgq_first_is<PanTiltInterface::TimedGotoMessage>()) {
      PanTiltInterface::TimedGotoMessage *msg = __pantilt_if->msgq_first(msg);

      __wt->goto_pantilt_timed(msg->pan(), msg->tilt(), msg->time_sec());
      __pantilt_if->set_msgid(msg->id());
      __pantilt_if->set_final(false);

    } else if (__pantilt_if->msgq_first_is<PanTiltInterface::ParkMessage>()) {
      PanTiltInterface::ParkMessage *msg = __pantilt_if->msgq_first(msg);

      __wt->goto_pantilt(0, 0);
      __pantilt_if->set_msgid(msg->id());
      __pantilt_if->set_final(false);

    } else if (__pantilt_if->msgq_first_is<PanTiltInterface::SetEnabledMessage>()) {
      PanTiltInterface::SetEnabledMessage *msg = __pantilt_if->msgq_first(msg);

      __wt->set_enabled(msg->is_enabled());

    } else if (__pantilt_if->msgq_first_is<PanTiltInterface::SetVelocityMessage>()) {
      PanTiltInterface::SetVelocityMessage *msg = __pantilt_if->msgq_first(msg);

      if (msg->pan_velocity() > __pantilt_if->max_pan_velocity()) {
	logger->log_warn(name(), "Desired pan velocity %f too high, max is %f",
			 msg->pan_velocity(), __pantilt_if->max_pan_velocity());
      } else if (msg->tilt_velocity() > __pantilt_if->max_tilt_velocity()) {
	logger->log_warn(name(), "Desired tilt velocity %f too high, max is %f",
			 msg->tilt_velocity(), __pantilt_if->max_tilt_velocity());
      } else {
	__wt->set_velocities(msg->pan_velocity(), msg->tilt_velocity());
      }

    } else if (__pantilt_if->msgq_first_is<PanTiltInterface::SetMarginMessage>()) {
      PanTiltInterface::SetMarginMessage *msg = __pantilt_if->msgq_first(msg);

      __wt->set_margins(msg->pan_margin(), msg->tilt_margin());
      __pantilt_if->set_pan_margin(msg->pan_margin());
      __pantilt_if->set_tilt_margin(msg->tilt_margin());

    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    __pantilt_if->msgq_pop();
  }

  __pantilt_if->write();

  bool write_led_if = false;
  while (! __led_if->msgq_empty() ) {
    write_led_if = true;
    if (__led_if->msgq_first_is<LedInterface::SetIntensityMessage>()) {
      LedInterface::SetIntensityMessage *msg = __led_if->msgq_first(msg);
      __wt->set_led_enabled((msg->intensity() >= 0.5));
      __led_if->set_intensity((msg->intensity() >= 0.5) ? LedInterface::ON : LedInterface::OFF);
    } else if (__led_if->msgq_first_is<LedInterface::TurnOnMessage>()) {
      __wt->set_led_enabled(true);
      __led_if->set_intensity(LedInterface::ON);
    } else if (__led_if->msgq_first_is<LedInterface::TurnOffMessage>()) {
      __wt->set_led_enabled(false);
      __led_if->set_intensity(LedInterface::OFF);
    }

    __led_if->msgq_pop();
  }
  if (write_led_if)  __led_if->write();

  //__wt->wakeup();
}


bool
PanTiltRX28Thread::bb_interface_message_received(Interface *interface,
						 Message *message) throw()
{
  if (message->is_of_type<PanTiltInterface::StopMessage>()) {
    __wt->stop_motion();
    return false; // do not enqueue StopMessage
  } else if (message->is_of_type<PanTiltInterface::FlushMessage>()) {
    __wt->stop_motion();
    logger->log_info(name(), "Flushing message queue");
    __pantilt_if->msgq_flush();
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

  __logger           = logger;

  __value_rwlock     = new ReadWriteLock();
  __rx28_rwlock      = new ReadWriteLock();
  __fresh_data_mutex = new Mutex();
  __update_waitcond  = new WaitCondition();

  __rx28 = rx28;
  __move_pending     = false;
  __target_pan       = 0;
  __target_tilt      = 0;
  __pan_servo_id     = pan_servo_id;
  __tilt_servo_id    = tilt_servo_id;

  __pan_min          = pan_min;
  __pan_max          = pan_max;
  __tilt_min         = tilt_min;
  __tilt_max         = tilt_max;
  __pan_offset       = pan_offset;
  __tilt_offset      = tilt_offset;
  __enable           = false;
  __disable          = false;
  __led_enable       = false;
  __led_disable      = false;

  __max_pan_speed    = __rx28->get_max_supported_speed(__pan_servo_id);
  __max_tilt_speed   = __rx28->get_max_supported_speed(__tilt_servo_id);
}


/** Destructor. */
PanTiltRX28Thread::WorkerThread::~WorkerThread()
{
  delete __value_rwlock;
  delete __rx28_rwlock;
  delete __fresh_data_mutex;
  delete __update_waitcond;
}


/** Enable or disable servo.
 * @param enabled true to enable servos, false to turn them off
 */
void
PanTiltRX28Thread::WorkerThread::set_enabled(bool enabled)
{
  ScopedRWLock lock(__value_rwlock);
  if (enabled) {
    __enable  = true;
    __disable = false;
  } else {
    __enable  = false;
    __disable = true;
  }
  wakeup();
}


/** Enable or disable LED.
 * @param enabled true to enable LED, false to turn it off
 */
void
PanTiltRX28Thread::WorkerThread::set_led_enabled(bool enabled)
{
  ScopedRWLock lock(__value_rwlock);
  if (enabled) {
    __led_enable  = true;
    __led_disable = false;
  } else {
    __led_enable  = false;
    __led_disable = true;
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
  ScopedRWLock lock(__value_rwlock);
  __target_pan   = pan;
  __target_tilt  = tilt;
  __move_pending = true;
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
  __target_pan   = pan;
  __target_tilt  = tilt;
  __move_pending = true;

  float cpan=0, ctilt=0;
  get_pantilt(cpan, ctilt);

  float pan_diff  = fabs(pan - cpan);
  float tilt_diff = fabs(tilt - ctilt);

  float req_pan_vel  = pan_diff  / time_sec;
  float req_tilt_vel = tilt_diff / time_sec;

  //__logger->log_debug(name(), "Current: %f/%f Des: %f/%f  Time: %f  Diff: %f/%f  ReqVel: %f/%f",
  //		      cpan, ctilt, pan, tilt, time_sec, pan_diff, tilt_diff, req_pan_vel, req_tilt_vel);


  if (req_pan_vel > __max_pan_speed) {
    __logger->log_warn(name(), "Requested move to (%f, %f) in %f sec requires a "
		       "pan speed of %f rad/s, which is greater than the maximum "
		       "of %f rad/s, reducing to max", pan, tilt, time_sec,
		       req_pan_vel, __max_pan_speed);
    req_pan_vel = __max_pan_speed;
  }

  if (req_tilt_vel > __max_tilt_speed) {
    __logger->log_warn(name(), "Requested move to (%f, %f) in %f sec requires a "
		       "tilt speed of %f rad/s, which is greater than the maximum of "
		       "%f rad/s, reducing to max", pan, tilt, time_sec,
		       req_tilt_vel, __max_tilt_speed);
    req_tilt_vel = __max_tilt_speed;
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
  ScopedRWLock lock(__value_rwlock);
  float pan_tmp  = roundf((pan_vel  / __max_pan_speed)  * RobotisRX28::MAX_SPEED);
  float tilt_tmp = roundf((tilt_vel / __max_tilt_speed) * RobotisRX28::MAX_SPEED);

  //__logger->log_debug(name(), "old speed: %u/%u new speed: %f/%f", __pan_vel,
  //		      __tilt_vel, pan_tmp, tilt_tmp);

  if ((pan_tmp >= 0) && (pan_tmp <= RobotisRX28::MAX_SPEED)) {
    __pan_vel = (unsigned int)pan_tmp;
    __velo_pending = true;
  } else {
    __logger->log_warn(name(), "Calculated pan value out of bounds, min: 0  max: %u  des: %u",
		       RobotisRX28::MAX_SPEED, (unsigned int)pan_tmp);
  }

  if ((tilt_tmp >= 0) && (tilt_tmp <= RobotisRX28::MAX_SPEED)) {
    __tilt_vel = (unsigned int)tilt_tmp;
    __velo_pending = true;
  } else {
    __logger->log_warn(name(), "Calculated tilt value out of bounds, min: 0  max: %u  des: %u",
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
  unsigned int pan_velticks  = __rx28->get_goal_speed(__pan_servo_id);
  unsigned int tilt_velticks = __rx28->get_goal_speed(__tilt_servo_id);

  pan_velticks  = (unsigned int)(((float)pan_velticks  / (float)RobotisRX28::MAX_SPEED) * __max_pan_speed);
  tilt_velticks = (unsigned int)(((float)tilt_velticks / (float)RobotisRX28::MAX_SPEED) * __max_tilt_speed);
}


/** Set desired velocities.
 * @param pan_margin pan margin
 * @param tilt_margin tilt margin
 */
void
PanTiltRX28Thread::WorkerThread::set_margins(float pan_margin, float tilt_margin)
{
  if (pan_margin  > 0.0)  __pan_margin  = pan_margin;
  if (tilt_margin > 0.0)  __tilt_margin = tilt_margin;
  //__logger->log_warn(name(), "Margins set to %f, %f", __pan_margin, __tilt_margin);
}


/** Get pan/tilt value.
 * @param pan upon return contains the current pan value
 * @param tilt upon return contains the current tilt value
 */
void
PanTiltRX28Thread::WorkerThread::get_pantilt(float &pan, float &tilt)
{
  ScopedRWLock lock(__rx28_rwlock, ScopedRWLock::LOCK_READ);

  int pan_ticks  = ((int)__rx28->get_position(__pan_servo_id)  - (int)RobotisRX28::CENTER_POSITION);
  int tilt_ticks = ((int)__rx28->get_position(__tilt_servo_id) - (int)RobotisRX28::CENTER_POSITION);

  pan  = pan_ticks *  RobotisRX28::RAD_PER_POS_TICK + __pan_offset;
  tilt = tilt_ticks * RobotisRX28::RAD_PER_POS_TICK + __tilt_offset;
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
  time = __pantilt_time;
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
  __logger->log_debug(name(), "P: %f  T: %f  TP: %f  TT: %f  PM: %f  TM: %f  PMov: %i  TMov: %i  Final: %s",
                      pan, tilt, __target_pan, __target_tilt, __pan_margin, __tilt_margin,
                      __rx28->is_moving(__pan_servo_id), __rx28->is_moving(__tilt_servo_id),
                      (( (fabs(pan  - __target_pan)  <= __pan_margin) &&
                         (fabs(tilt - __target_tilt) <= __tilt_margin) ) ||
                       (! __rx28->is_moving(__pan_servo_id) &&
                        ! __rx28->is_moving(__tilt_servo_id))) ? "YES" : "NO");
  */

  ScopedRWLock lock(__rx28_rwlock, ScopedRWLock::LOCK_READ);

  return  ( (fabs(pan  - __target_pan)  <= __pan_margin) &&
	    (fabs(tilt - __target_tilt) <= __tilt_margin) ) ||
          (! __rx28->is_moving(__pan_servo_id) &&
	   ! __rx28->is_moving(__tilt_servo_id));
}


/** Check if PTU is enabled.
 * @return true if torque is enabled for both servos, false otherwise
 */
bool
PanTiltRX28Thread::WorkerThread::is_enabled()
{
  return (__rx28->is_torque_enabled(__pan_servo_id) &&
	  __rx28->is_torque_enabled(__tilt_servo_id));
}


/** Check is fresh sensor data is available.
 * Note that this method will return true at once per sensor update cycle.
 * @return true if fresh data is available, false otherwise
 */
bool
PanTiltRX28Thread::WorkerThread::has_fresh_data()
{
  MutexLocker lock(__fresh_data_mutex);

  bool rv = __fresh_data;
  __fresh_data = false;
  return rv;
}


void
PanTiltRX28Thread::WorkerThread::loop()
{
  if (__enable) {
    __value_rwlock->lock_for_write();
    __enable  = false;
    __value_rwlock->unlock();
    ScopedRWLock lock(__rx28_rwlock);
    __rx28->set_led_enabled(__tilt_servo_id, true);
    __rx28->set_torques_enabled(true, 2, __pan_servo_id, __tilt_servo_id);
  } else if (__disable) {
    __value_rwlock->lock_for_write();
    __disable = false;
    __value_rwlock->unlock();
    ScopedRWLock lock(__rx28_rwlock);
    if (__led_enable || __led_disable || __velo_pending || __move_pending) usleep(3000);
  }

  if (__led_enable) {
    __value_rwlock->lock_for_write();
    __led_enable = false;
    __value_rwlock->unlock();    
    ScopedRWLock lock(__rx28_rwlock);
    __rx28->set_led_enabled(__pan_servo_id, true);
    if (__velo_pending || __move_pending) usleep(3000);
  } else if (__led_disable) {
    __value_rwlock->lock_for_write();
    __led_disable = false;
    __value_rwlock->unlock();    
    ScopedRWLock lock(__rx28_rwlock);
    __rx28->set_led_enabled(__pan_servo_id, false);    
    if (__velo_pending || __move_pending) usleep(3000);
  }

  if (__velo_pending) {
    __value_rwlock->lock_for_write();
    __velo_pending = false;
    unsigned int pan_vel  = __pan_vel;
    unsigned int tilt_vel = __tilt_vel;
    __value_rwlock->unlock();
    ScopedRWLock lock(__rx28_rwlock);
    __rx28->set_goal_speeds(2, __pan_servo_id, pan_vel, __tilt_servo_id, tilt_vel);
    if (__move_pending) usleep(3000);
  }

  if (__move_pending) {
    __value_rwlock->lock_for_write();
    __move_pending    = false;
    float target_pan  = __target_pan;
    float target_tilt = __target_tilt;
    __value_rwlock->unlock();
    exec_goto_pantilt(target_pan, target_tilt);
  }

  try {
    ScopedRWLock lock(__rx28_rwlock, ScopedRWLock::LOCK_READ);
    __rx28->read_table_values(__pan_servo_id);
    __rx28->read_table_values(__tilt_servo_id);
    {
      MutexLocker lock_fresh_data(__fresh_data_mutex);
      __fresh_data = true;
      __pantilt_time.stamp();
    }
  } catch (Exception &e) {
    // usually just a timeout, too noisy
    //__logger->log_warn(name(), "Error while reading table values from servos, exception follows");
    //__logger->log_warn(name(), e);
  }

  //if (! is_final() ||
  //    ! __rx28->is_torque_enabled(__pan_servo_id) ||
  //    ! __rx28->is_torque_enabled(__tilt_servo_id)) {
    // while moving, and while the motor is off, wake us up to get new servo
    // position data
    //wakeup();
    //}

  __update_waitcond->wake_all();

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
  if ( (pan_rad < __pan_min) || (pan_rad > __pan_max) ) {
    __logger->log_warn(name(), "Pan value out of bounds, min: %f  max: %f  des: %f",
		       __pan_min, __pan_max, pan_rad);
    return;
  }
  if ( (tilt_rad < __tilt_min) || (tilt_rad > __tilt_max) ) {
    __logger->log_warn(name(), "Tilt value out of bounds, min: %f  max: %f  des: %f",
		       __tilt_min, __tilt_max, tilt_rad);
    return;
  }

  unsigned int pan_min = 0, pan_max = 0, tilt_min = 0, tilt_max = 0;

  __rx28->get_angle_limits(__pan_servo_id, pan_min, pan_max);
  __rx28->get_angle_limits(__tilt_servo_id, tilt_min, tilt_max);


  int pan_pos  = (int)roundf(RobotisRX28::POS_TICKS_PER_RAD * (pan_rad - __pan_offset))
                 + RobotisRX28::CENTER_POSITION;
  int tilt_pos = (int)roundf(RobotisRX28::POS_TICKS_PER_RAD * (tilt_rad - __tilt_offset))
                 + RobotisRX28::CENTER_POSITION;

  if ( (pan_pos < 0) || ((unsigned int)pan_pos < pan_min) || ((unsigned int)pan_pos > pan_max) ) {
    __logger->log_warn(name(), "Pan position out of bounds, min: %u  max: %u  des: %i",
		       pan_min, pan_max, pan_pos);
    return;
  }

  if ( (tilt_pos < 0) || ((unsigned int)tilt_pos < tilt_min) || ((unsigned int)tilt_pos > tilt_max) ) {
    __logger->log_warn(name(), "Tilt position out of bounds, min: %u  max: %u  des: %i",
		       tilt_min, tilt_max, tilt_pos);
    return;
  }

  ScopedRWLock lock(__rx28_rwlock);
  __rx28->goto_positions(2, __pan_servo_id, pan_pos, __tilt_servo_id, tilt_pos);
}


/** Wait for fresh data to be received.
 * Blocks the calling thread.
 */
void
PanTiltRX28Thread::WorkerThread::wait_for_fresh_data()
{
  __update_waitcond->wait();
}
