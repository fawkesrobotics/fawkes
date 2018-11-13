
/***************************************************************************
 *  evid100p_thread.h - Sony EviD100P pan/tilt unit act thread
 *
 *  Created: Sun Jun 21 12:38:34 2009
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

#include "evid100p_thread.h"
#include "evid100p.h"

#include <core/threading/mutex_locker.h>
#include <interfaces/PanTiltInterface.h>
#include <interfaces/JointInterface.h>
#include <interfaces/SwitchInterface.h>

#include <cstdarg>
#include <cmath>

using namespace fawkes;

/** @class PanTiltSonyEviD100PThread "evid100p_thread.h"
 * PanTilt act thread for the PTU part of the Sony EviD100P camera.
 * This thread integrates into the Fawkes main loop at the ACT_EXEC hook and
 * interacts via the Visca protocol with the controller of the Sony EviD100P.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param pantilt_cfg_prefix pantilt plugin configuration prefix
 * @param ptu_cfg_prefix configuration prefix specific for the PTU
 * @param ptu_name name of the PTU configuration
 */
PanTiltSonyEviD100PThread::PanTiltSonyEviD100PThread(std::string &pantilt_cfg_prefix,
						     std::string &ptu_cfg_prefix,
						     std::string &ptu_name)
  : PanTiltActThread("PanTiltSonyEviD100PThread"),
    BlackBoardInterfaceListener("PanTiltSonyEviD100PThread")
{
  set_name("PanTiltSonyEviD100PThread(%s)", ptu_name.c_str());

  pantilt_cfg_prefix_ = pantilt_cfg_prefix;
  ptu_cfg_prefix_     = ptu_cfg_prefix;
  ptu_name_           = ptu_name;

  cam_ = NULL;
}


void
PanTiltSonyEviD100PThread::init()
{
  // Note: due to the use of auto_ptr and RefPtr resources are automatically
  // freed on destruction, therefore no special handling is necessary in init()
  // itself!

  cfg_device_           = config->get_string((ptu_cfg_prefix_ + "device").c_str());
  cfg_read_timeout_ms_  = config->get_uint((ptu_cfg_prefix_ + "read_timeout_ms").c_str());

  try {
    cam_ = new SonyEviD100PVisca(cfg_device_.c_str(), cfg_read_timeout_ms_,
				  /* blocking */ false);
  } catch (Exception &e) {
    e.print_trace();
    e.print_backtrace();
    throw;
  }

  bool power_up = true;
  try {
    power_up = config->get_bool((ptu_cfg_prefix_ + "power-up").c_str());
  } catch (Exception &e) {} // ignore, use default
  if (power_up)  cam_->set_power(true);

  float init_pan = 0.f;
  float init_tilt = 0.f;
  float init_pan_velocity = 0.f;
  float init_tilt_velocity = 0.f;

  // If you have more than one interface: catch exception and close them!
  std::string bbid = "PanTilt " + ptu_name_;
  pantilt_if_ = blackboard->open_for_writing<PanTiltInterface>(bbid.c_str());
  pantilt_if_->set_calibrated(true);
  pantilt_if_->set_min_pan(SonyEviD100PVisca::MIN_PAN_RAD);
  pantilt_if_->set_max_pan(SonyEviD100PVisca::MAX_PAN_RAD);
  pantilt_if_->set_min_tilt(SonyEviD100PVisca::MIN_TILT_RAD);
  pantilt_if_->set_max_tilt(SonyEviD100PVisca::MAX_TILT_RAD);
  pantilt_if_->set_enabled(true); // Cannot be turned off

  float pan_smin, pan_smax, tilt_smin, tilt_smax;
  cam_->get_speed_limits(pan_smin, pan_smax, tilt_smin, tilt_smax);
  pantilt_if_->set_max_pan_velocity(pan_smax);
  pantilt_if_->set_max_tilt_velocity(tilt_smax);
  pantilt_if_->set_pan_velocity(init_pan_velocity);
  pantilt_if_->set_tilt_velocity(init_tilt_velocity);
  pantilt_if_->write();

  std::string panid = ptu_name_ + " pan";
  panjoint_if_ = blackboard->open_for_writing<JointInterface>(panid.c_str());
  panjoint_if_->set_position(init_pan);
  panjoint_if_->set_velocity(init_pan_velocity);
  panjoint_if_->write();

  std::string tiltid = ptu_name_ + " tilt";
  tiltjoint_if_ = blackboard->open_for_writing<JointInterface>(tiltid.c_str());
  tiltjoint_if_->set_position(init_tilt);
  tiltjoint_if_->set_velocity(init_tilt_velocity);
  tiltjoint_if_->write();

  camctrl_if_ = blackboard->open_for_writing<CameraControlInterface>(bbid.c_str());
  camctrl_if_->set_effect(CameraControlInterface::EFF_NONE);
  camctrl_if_->set_effect_supported(true);
  camctrl_if_->set_zoom_supported(true);
  camctrl_if_->set_zoom_min(0);
  camctrl_if_->set_zoom_max(13);

  power_if_ = blackboard->open_for_writing<SwitchInterface>(bbid.c_str());
  power_if_->set_enabled(cam_->is_powered());
  power_if_->write();

  bool mirror = false;
  try {
    mirror = config->get_bool((ptu_cfg_prefix_ + "mirror").c_str());
  } catch (Exception &e) {} // ignore, use default
  if (power_if_->is_enabled()) {
    cam_->set_mirror(mirror);
  }

  camctrl_if_->set_mirror(mirror);
  camctrl_if_->set_mirror_supported(true);
  camctrl_if_->write();

  wt_ = new WorkerThread(ptu_name_, logger, cam_,
			  SonyEviD100PVisca::MIN_PAN_RAD, SonyEviD100PVisca::MAX_PAN_RAD,
			  SonyEviD100PVisca::MIN_TILT_RAD, SonyEviD100PVisca::MAX_TILT_RAD);
  wt_->start();
  // Wakeup once to get values
  wt_->wakeup();

  wt_->set_velocities(pan_smax, tilt_smax);

  bbil_add_message_interface(pantilt_if_);
  blackboard->register_listener(this);

#ifdef USE_TIMETRACKER
  tt_.reset(new TimeTracker());
  tt_count_ = 0;
  ttc_read_sensor_ = tt_->add_class("Read Sensor");
#endif  

}


void
PanTiltSonyEviD100PThread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->close(pantilt_if_);
  blackboard->close(panjoint_if_);
  blackboard->close(tiltjoint_if_);
  blackboard->close(camctrl_if_);
  blackboard->close(power_if_);

  wt_->cancel();
  wt_->join();
  delete wt_;

  bool power_down = true;
  try {
    power_down = config->get_bool((ptu_cfg_prefix_ + "power-down").c_str());
  } catch (Exception &e) {} // ignore, use default
  if (power_down)  cam_->set_power(false);

  // Setting to NULL deletes instance (RefPtr)
  cam_ = NULL;
}


/** Update sensor values as necessary.
 * To be called only from PanTiltSensorThread. Writes the current pan/tilt
 * data into the interface.
 */
void
PanTiltSonyEviD100PThread::update_sensor_values()
{
  if (wt_->has_fresh_data()) {
    float pan = 0, tilt = 0;
    wt_->get_pantilt(pan, tilt);
    pantilt_if_->set_pan(pan);
    pantilt_if_->set_tilt(tilt);
    pantilt_if_->set_final(wt_->is_final());
    pantilt_if_->write();

    panjoint_if_->set_position(pan);
    panjoint_if_->write();

    tiltjoint_if_->set_position(tilt);
    tiltjoint_if_->write();

    unsigned int zoom = wt_->get_zoom();
    if (camctrl_if_->zoom() != zoom) {
      camctrl_if_->set_zoom(zoom);
      camctrl_if_->write();
    }
  }
}


void
PanTiltSonyEviD100PThread::loop()
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

    } else if (pantilt_if_->msgq_first_is<PanTiltInterface::ParkMessage>()) {
      PanTiltInterface::ParkMessage *msg = pantilt_if_->msgq_first(msg);

      wt_->goto_pantilt(0, 0);
      pantilt_if_->set_msgid(msg->id());
      pantilt_if_->set_final(false);

    } else if (pantilt_if_->msgq_first_is<PanTiltInterface::SetEnabledMessage>()) {
      PanTiltInterface::SetEnabledMessage *msg = pantilt_if_->msgq_first(msg);

      logger->log_warn(name(), "SetEnabledMessage ignored for Sony EviD100P");

    } else if (pantilt_if_->msgq_first_is<PanTiltInterface::SetVelocityMessage>()) {
      PanTiltInterface::SetVelocityMessage *msg = pantilt_if_->msgq_first(msg);

      if ((msg->pan_velocity() < 0) || (msg->tilt_velocity() < 0) ) {
	logger->log_warn(name(), "Ignoring pan/tilt velocities %f/%f, at least one "
			 " is negative", msg->pan_velocity(), msg->tilt_velocity());
      } else if (msg->pan_velocity() > pantilt_if_->max_pan_velocity()) {
	logger->log_warn(name(), "Desired pan velocity %f too high, max is %f",
			 msg->pan_velocity(), pantilt_if_->max_pan_velocity());
      } else if (msg->tilt_velocity() > pantilt_if_->max_tilt_velocity()) {
	logger->log_warn(name(), "Desired tilt velocity %f too high, max is %f",
			 msg->tilt_velocity(), pantilt_if_->max_tilt_velocity());
      } else {
	wt_->set_velocities(msg->pan_velocity(), msg->tilt_velocity());
	pantilt_if_->set_pan_velocity(msg->pan_velocity());
	pantilt_if_->set_tilt_velocity(msg->tilt_velocity());
	panjoint_if_->set_velocity(msg->pan_velocity());
	panjoint_if_->write();
	tiltjoint_if_->set_velocity(msg->tilt_velocity());
	tiltjoint_if_->write();
      }

    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    pantilt_if_->msgq_pop();
  }
  pantilt_if_->write();

  while (! camctrl_if_->msgq_empty() ) {
    if (camctrl_if_->msgq_first_is<CameraControlInterface::SetMirrorMessage>()) {
      CameraControlInterface::SetMirrorMessage *msg = camctrl_if_->msgq_first(msg);
      wt_->set_mirror(msg->is_mirror());
      camctrl_if_->set_mirror(msg->is_mirror());
      camctrl_if_->write();
    } else if (camctrl_if_->msgq_first_is<CameraControlInterface::SetEffectMessage>()) {
      CameraControlInterface::SetEffectMessage *msg = camctrl_if_->msgq_first(msg);
      wt_->set_effect(msg->effect());
      camctrl_if_->set_effect(msg->effect());
      camctrl_if_->write();
    } else if (camctrl_if_->msgq_first_is<CameraControlInterface::SetZoomMessage>()) {
      CameraControlInterface::SetZoomMessage *msg = camctrl_if_->msgq_first(msg);
      wt_->set_zoom(msg->zoom());
    } else {
      logger->log_warn(name(), "Unhandled message %s ignored",
		       camctrl_if_->msgq_first()->type());
    }
    camctrl_if_->msgq_pop();
  }

  while (! power_if_->msgq_empty() ) {
    if (power_if_->msgq_first_is<SwitchInterface::EnableSwitchMessage>()) {
      // must be re-established
      wt_->set_mirror(camctrl_if_->is_mirror());
      wt_->set_effect(camctrl_if_->effect());
      wt_->set_power(true);
      power_if_->set_enabled(true);
      power_if_->write();
    } else if (power_if_->msgq_first_is<SwitchInterface::DisableSwitchMessage>()) {
      wt_->set_power(false);
      power_if_->set_enabled(false);
      power_if_->write();
    } else if (power_if_->msgq_first_is<SwitchInterface::SetMessage>()) {
      SwitchInterface::SetMessage *msg = power_if_->msgq_first(msg);
      wt_->set_power(msg->is_enabled() || msg->value() > 0.5);
      power_if_->set_enabled(msg->is_enabled() || msg->value() > 0.5);
      power_if_->write();
    } else {
      logger->log_warn(name(), "Unhandled message %s ignored",
		       power_if_->msgq_first()->type());
    }
    power_if_->msgq_pop();
  }
}


bool
PanTiltSonyEviD100PThread::bb_interface_message_received(Interface *interface,
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
    //logger->log_info(name(), "Received message of type %s, enqueueing", message->type());
    return true;
  }
}


/** @class PanTiltSonyEviD100PThread::WorkerThread "sony/evid100p_thread.h"
 * Worker thread for the PanTiltSonyEviD100PThread.
 * This continuous thread issues commands to the camera. In each loop it
 * will first execute pending operations, and then update the sensor data (lengthy
 * operation). Sensor data will only be updated while either a servo in the chain
 * is still moving or torque is disabled (so the motor can be move manually).
 * @author Tim Niemueller
 */


/** Constructor.
 * @param ptu_name name of the pan/tilt unit
 * @param logger logger
 * @param cam Visca controller object
 * @param pan_min minimum pan in rad
 * @param pan_max maximum pan in rad
 * @param tilt_min minimum tilt in rad
 * @param tilt_max maximum tilt in rad
 */
PanTiltSonyEviD100PThread::WorkerThread::WorkerThread(std::string ptu_name,
						      fawkes::Logger *logger,
						      fawkes::RefPtr<SonyEviD100PVisca> cam,
						      const float &pan_min,
						      const float &pan_max,
						      const float &tilt_min,
						      const float &tilt_max)
  : Thread("", Thread::OPMODE_WAITFORWAKEUP)
{
  set_name("SonyEviD100PWorkerThread(%s)", ptu_name.c_str());
  set_coalesce_wakeups(true);

  logger_           = logger;

  move_mutex_       = new Mutex();
  effect_mutex_     = new Mutex();
  zoom_mutex_       = new Mutex();
  mirror_mutex_     = new Mutex();
  power_mutex_      = new Mutex();

  cam_              = cam;
  move_pending_     = false;
  target_pan_       = 0;
  target_tilt_      = 0;
  fresh_data_       = false;

  velo_pending_     = false;
  pan_vel_          = 0;
  tilt_vel_         = 0;

  pan_min_          = pan_min;
  pan_max_          = pan_max;
  tilt_min_         = tilt_min;
  tilt_max_         = tilt_max;

  zoom_pending_     = false;
  target_zoom_      = 0;

  mirror_pending_   = false;
  power_pending_    = false;
  effect_pending_   = false;

  powered_          = cam_->is_powered();
}


/** Destructor. */
PanTiltSonyEviD100PThread::WorkerThread::~WorkerThread()
{
  delete move_mutex_;
  delete zoom_mutex_;
  delete effect_mutex_;
  delete mirror_mutex_;
  delete power_mutex_;
}


/** Stop currently running motion. */
void
PanTiltSonyEviD100PThread::WorkerThread::stop_motion()
{
  if (powered_) {
    float pan = 0, tilt = 0;
    get_pantilt(pan, tilt);
    goto_pantilt(pan, tilt);
  }
}


/** Goto desired pan/tilt values.
 * @param pan pan in radians
 * @param tilt tilt in radians
 */
void
PanTiltSonyEviD100PThread::WorkerThread::goto_pantilt(float pan, float tilt)
{
  MutexLocker lock(move_mutex_);
  target_pan_   = pan;
  target_tilt_  = tilt;
  move_pending_ = true;
  if (powered_)  wakeup();
}

/** Set desired zoom value.
 * @param zoom_value desired zoom
 */
void
PanTiltSonyEviD100PThread::WorkerThread::set_zoom(unsigned int zoom_value)
{
  MutexLocker lock(zoom_mutex_);
  zoom_pending_ = true;

  switch (zoom_value) {
  case  0: target_zoom_ = Visca::VISCA_ZOOM_VALUE_WIDE;    break;
  case  1: target_zoom_ = Visca::VISCA_ZOOM_VALUE_1X;      break;
  case  2: target_zoom_ = Visca::VISCA_ZOOM_VALUE_2X;      break;
  case  3: target_zoom_ = Visca::VISCA_ZOOM_VALUE_3X;      break;
  case  4: target_zoom_ = Visca::VISCA_ZOOM_VALUE_4X;      break;
  case  5: target_zoom_ = Visca::VISCA_ZOOM_VALUE_5X;      break;
  case  6: target_zoom_ = Visca::VISCA_ZOOM_VALUE_6X;      break;
  case  7: target_zoom_ = Visca::VISCA_ZOOM_VALUE_7X;      break;
  case  8: target_zoom_ = Visca::VISCA_ZOOM_VALUE_8X;      break;
  case  9: target_zoom_ = Visca::VISCA_ZOOM_VALUE_9X;      break;
  case 10: target_zoom_ = Visca::VISCA_ZOOM_VALUE_10X;     break;
  case 11: target_zoom_ = Visca::VISCA_ZOOM_VALUE_DIG_20X; break;
  case 12: target_zoom_ = Visca::VISCA_ZOOM_VALUE_DIG_30X; break;
  case 13: target_zoom_ = Visca::VISCA_ZOOM_VALUE_DIG_40X; break;
  default:
    logger_->log_warn(name(), "Illegal zoom value %u ignored", zoom_value);
    zoom_pending_ = false;
  }
  if (powered_)  wakeup();
}


/** Set desired effect.
 * @param effect effect value
 */
void
PanTiltSonyEviD100PThread::WorkerThread::set_effect(CameraControlInterface::Effect effect)
{
  MutexLocker lock(effect_mutex_);
  target_effect_ = effect;
  effect_pending_ = true;
  if (powered_)  wakeup();
}

/** Set mirror state.
 * @param enabled true to enable mirroring, false to disable
 */
void
PanTiltSonyEviD100PThread::WorkerThread::set_mirror(bool enabled)
{
  MutexLocker lock(effect_mutex_);
  target_mirror_ = enabled;
  mirror_pending_ = true;
  if (powered_)  wakeup();
}


/** Set power for camera.
 * @param powered true to turn on, false to turn off
 */
void
PanTiltSonyEviD100PThread::WorkerThread::set_power(bool powered)
{
  MutexLocker lock(power_mutex_);
  power_desired_ = powered;
  power_pending_ = true;
  wakeup();
}

/** Get pan/tilt value.
 * @param pan upon return contains the current pan value
 * @param tilt upon return contains the current tilt value
 */
void
PanTiltSonyEviD100PThread::WorkerThread::get_pantilt(float &pan, float &tilt)
{
  pan  = cur_pan_;
  tilt = cur_tilt_;
}


/** Get zoom value.
 * @return current zoom value
 */
unsigned int
PanTiltSonyEviD100PThread::WorkerThread::get_zoom()
{
  switch (cur_zoom_) {
  case Visca::VISCA_ZOOM_VALUE_1X:      return 1;
  case Visca::VISCA_ZOOM_VALUE_2X:      return 2;
  case Visca::VISCA_ZOOM_VALUE_3X:      return 3;
  case Visca::VISCA_ZOOM_VALUE_4X:      return 4;
  case Visca::VISCA_ZOOM_VALUE_5X:      return 5;
  case Visca::VISCA_ZOOM_VALUE_6X:      return 6;
  case Visca::VISCA_ZOOM_VALUE_7X:      return 7;
  case Visca::VISCA_ZOOM_VALUE_8X:      return 8;
  case Visca::VISCA_ZOOM_VALUE_9X:      return 9;
  case Visca::VISCA_ZOOM_VALUE_10X:     return 10;
  case Visca::VISCA_ZOOM_VALUE_DIG_20X: return 11;
  case Visca::VISCA_ZOOM_VALUE_DIG_30X: return 12;
  case Visca::VISCA_ZOOM_VALUE_DIG_40X: return 13;
  default: return 0;
  }
}


/** Set desired velocities.
 * @param pan_vel pan velocity
 * @param tilt_vel tilt velocity
 */
void
PanTiltSonyEviD100PThread::WorkerThread::set_velocities(float pan_vel, float tilt_vel)
{
  pan_vel_      = pan_vel;
  tilt_vel_     = tilt_vel;
  velo_pending_ = true;
}


/** Check if motion is final.
 * @return true if motion is final, false otherwise
 */
bool
PanTiltSonyEviD100PThread::WorkerThread::is_final()
{
  MutexLocker lock(move_mutex_);
  return powered_ && cam_->is_nonblocking_finished(SonyEviD100PVisca::NONBLOCKING_PANTILT)
    &&   cam_->is_nonblocking_finished(SonyEviD100PVisca::NONBLOCKING_ZOOM);
}


/** Check is fresh sensor data is available.
 * Note that this method will return true at once per sensor update cycle.
 * @return true if fresh data is available, false otherwise
 */
bool
PanTiltSonyEviD100PThread::WorkerThread::has_fresh_data()
{
  bool rv = fresh_data_;
  fresh_data_ = false;
  return rv;
}


void
PanTiltSonyEviD100PThread::WorkerThread::once()
{
  // do some process cycles to process data returning back from set_address()
  // and clear calls
  for (int i = 0; i < 20; ++i) {
    try {
      cam_->process();
    } catch (Exception &e) { /* ignored */ }
  }
}


void
PanTiltSonyEviD100PThread::WorkerThread::loop()
{
  try {
    cam_->process();
  } catch (Exception &e) {
    logger_->log_warn(name(), "Data processing failed, exception follows");
    logger_->log_warn(name(), e);
  }

  if (power_pending_) {
    power_mutex_->lock();
    logger_->log_debug(name(), "Powering %s the PTU", power_desired_ ? "up" : "down");
    power_pending_ = false;
    cam_->set_power(power_desired_);
    powered_ = power_desired_;
    power_mutex_->unlock();
  }

  if (velo_pending_) {
    try {
      if (powered_)  cam_->set_speed_radsec(pan_vel_, tilt_vel_);
    } catch (Exception &e) {
      logger_->log_warn(name(), "Setting pan/tilt values failed, exception follows");
      logger_->log_warn(name(), e);
    }
    velo_pending_ = false;
  }

  if (move_pending_) {
    move_mutex_->lock();
    logger_->log_debug(name(), "Executing goto to %f, %f", target_pan_, target_tilt_);
    if (powered_) exec_goto_pantilt(target_pan_, target_tilt_);
    move_pending_ = false;
    move_mutex_->unlock();
  }

  if (zoom_pending_) {
    zoom_mutex_->lock();
    if (powered_) exec_set_zoom(target_zoom_);
    zoom_pending_ = false;
    zoom_mutex_->unlock();
  }

  if (effect_pending_) {
    effect_mutex_->lock();
    if (powered_) exec_set_effect(target_effect_);
    effect_pending_ = false;
    effect_mutex_->unlock();
  }

  if (mirror_pending_) {
    mirror_mutex_->lock();
    logger_->log_debug(name(), "%sabling mirroring", target_mirror_ ? "En" : "Dis");
    if (powered_) exec_set_mirror(target_mirror_);
    mirror_pending_ = false;
    mirror_mutex_->unlock();
  }

  //cam_->start_get_pan_tilt();
  try {
    if (powered_) {
      cam_->get_pan_tilt_rad(cur_pan_, cur_tilt_);
      fresh_data_ = true;
    }
  } catch (Exception &e) {
    logger_->log_warn(name(), "Failed to get new pan/tilt data, exception follows");
    logger_->log_warn(name(), e);
  }

  try {
    if (powered_) {
      unsigned int new_zoom = 0;
      cam_->get_zoom(new_zoom);
      if (new_zoom != cur_zoom_) {
	cur_zoom_ = new_zoom;
	fresh_data_ = true;
      }
    }
  } catch (Exception &e) {
    logger_->log_warn(name(), "Failed to get new zoom data, exception follows");
    logger_->log_warn(name(), e);
  }

  if (powered_ && (! is_final() || ! fresh_data_)) {
    // while moving or if data reception failed wake us up to get new servo data
    wakeup();
  }
}


/** Execute pan/tilt motion.
 * @param pan_rad pan in rad to move to
 * @param tilt_rad tilt in rad to move to
 */
void
PanTiltSonyEviD100PThread::WorkerThread::exec_goto_pantilt(float pan_rad, float tilt_rad)
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

  try {
    cam_->set_pan_tilt_rad(pan_rad, tilt_rad);
  } catch (Exception &e) {
    logger_->log_warn(name(), "Failed to execute pan/tilt to %f, %f, exception "
		       "follows", pan_rad, tilt_rad);
    logger_->log_warn(name(), e);
  }
}

/** Execute zoom setting.
 * @param zoom Zoom value to set
 */
void
PanTiltSonyEviD100PThread::WorkerThread::exec_set_zoom(unsigned int zoom)
{
  try {
    cam_->set_zoom(zoom);
  } catch (Exception &e) {
    logger_->log_warn(name(), "Failed to execute zoom to %u, exception "
		       "follows", zoom);
    logger_->log_warn(name(), e);
  }
}

/** Execute mirror setting.
 * @param mirror true to enable monitoring, false to disable
 */
void
PanTiltSonyEviD100PThread::WorkerThread::exec_set_mirror(bool mirror)
{
  try {
    cam_->set_mirror(mirror);
  } catch (Exception &e) {
    logger_->log_warn(name(), "Failed to %sabling mirror mod, exception follows",
		       mirror ? "En" : "Dis");
    logger_->log_warn(name(), e);
  }
}


/** Execute effect setting.
 * @param effect Target effect value to set
 */
void
PanTiltSonyEviD100PThread::WorkerThread::exec_set_effect(CameraControlInterface::Effect effect)
{
  try {
    switch (effect) {
    case CameraControlInterface::EFF_NEGATIVE:
      cam_->apply_effect_neg_art(); break;
    case CameraControlInterface::EFF_PASTEL:
      cam_->apply_effect_pastel(); break;
    case CameraControlInterface::EFF_BW:
      cam_->apply_effect_bnw(); break;
    case CameraControlInterface::EFF_SOLARIZE:
      cam_->apply_effect_solarize(); break;
    default:
      cam_->reset_effect(); break;
    }
  } catch (Exception &e) {
    logger_->log_warn(name(), "Failed to set effect, exception follows");
    logger_->log_warn(name(), e);
  }
}
