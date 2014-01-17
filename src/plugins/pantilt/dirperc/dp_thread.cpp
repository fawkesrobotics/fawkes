
/***************************************************************************
 *  dp_thread.h - DirectedPerception pan/tilt unit act thread
 *
 *  Created: Sun Jun 21 17:31:50 2009
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

#include "dp_thread.h"
#include "dp_ptu.h"

#include <core/threading/mutex_locker.h>
#include <interfaces/PanTiltInterface.h>
#include <interfaces/JointInterface.h>

#include <cstdarg>
#include <cmath>

using namespace fawkes;

/** @class PanTiltDirectedPerceptionThread "dp_thread.h"
 * PanTilt act thread for PTUs from DirectedPerception employing the ASCII
 * protocol.
 * This thread integrates into the Fawkes main loop at the ACT_EXEC hook and
 * interacts via the Visca protocol with the controller of the Sony EviD100P.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param pantilt_cfg_prefix pantilt plugin configuration prefix
 * @param ptu_cfg_prefix configuration prefix specific for the PTU
 * @param ptu_name name of the PTU configuration
 */
PanTiltDirectedPerceptionThread::PanTiltDirectedPerceptionThread(std::string &pantilt_cfg_prefix,
								 std::string &ptu_cfg_prefix,
								 std::string &ptu_name)
  : PanTiltActThread("PanTiltDirectedPerceptionThread"),
    BlackBoardInterfaceListener("PanTiltDirectedPerceptionThread")
{
  set_name("PanTiltDirectedPerceptionThread(%s)", ptu_name.c_str());

  __pantilt_cfg_prefix = pantilt_cfg_prefix;
  __ptu_cfg_prefix     = ptu_cfg_prefix;
  __ptu_name           = ptu_name;
}


void
PanTiltDirectedPerceptionThread::init()
{
  // Note: due to the use of auto_ptr and RefPtr resources are automatically
  // freed on destruction, therefore no special handling is necessary in init()
  // itself!

  __cfg_device           = config->get_string((__ptu_cfg_prefix + "device").c_str());
  __cfg_read_timeout_ms  = config->get_uint((__ptu_cfg_prefix + "read_timeout_ms").c_str());

  __ptu = new DirectedPerceptionPTU(__cfg_device.c_str(), __cfg_read_timeout_ms);

  // If you have more than one interface: catch exception and close them!
  std::string bbid = "PanTilt " + __ptu_name;
  __pantilt_if = blackboard->open_for_writing<PanTiltInterface>(bbid.c_str());

  float min_pan=0, max_pan=0, min_tilt=0, max_tilt=0;
  __ptu->get_limits(min_pan, max_pan, min_tilt, max_tilt);

  __pantilt_if->set_calibrated(true);
  __pantilt_if->set_min_pan(min_pan);
  __pantilt_if->set_max_pan(max_pan);
  __pantilt_if->set_min_tilt(min_tilt);
  __pantilt_if->set_max_tilt(max_tilt);
  __pantilt_if->set_enabled(true); // Cannot be turned off
  //__pantilt_if->set_max_pan_velocity(0);
  //__pantilt_if->set_max_tilt_velocity(0);
  //__pantilt_if->set_pan_velocity(0);
  //__pantilt_if->set_tilt_velocity(0);
  __pantilt_if->write();

  float init_pan = 0.f;
  float init_tilt = 0.f;
  float init_pan_velocity = 0.f;
  float init_tilt_velocity = 0.f;

  std::string panid = __ptu_name + " pan";
  __panjoint_if = blackboard->open_for_writing<JointInterface>(panid.c_str());
  __panjoint_if->set_position(init_pan);
  __panjoint_if->set_velocity(init_pan_velocity);
  __panjoint_if->write();

  std::string tiltid = __ptu_name + " tilt";
  __tiltjoint_if = blackboard->open_for_writing<JointInterface>(tiltid.c_str());
  __tiltjoint_if->set_position(init_tilt);
  __tiltjoint_if->set_velocity(init_tilt_velocity);
  __tiltjoint_if->write();

  __wt = new WorkerThread(__ptu_name, logger, __ptu);
  __wt->start();

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


void
PanTiltDirectedPerceptionThread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->close(__pantilt_if);
  blackboard->close(__panjoint_if);
  blackboard->close(__tiltjoint_if);

  __wt->cancel();
  __wt->join();
  delete __wt;

  // Setting to NULL deletes instance (RefPtr)
  __ptu = NULL;
}


/** Update sensor values as necessary.
 * To be called only from PanTiltSensorThread. Writes the current pan/tilt
 * data into the interface.
 */
void
PanTiltDirectedPerceptionThread::update_sensor_values()
{
  if (__wt->has_fresh_data()) {
    float pan = 0, tilt = 0;
    __wt->get_pantilt(pan, tilt);
    __pantilt_if->set_pan(pan);
    __pantilt_if->set_tilt(tilt);
    __pantilt_if->set_final(__wt->is_final());
    __pantilt_if->write();

    __panjoint_if->set_position(pan);
    __panjoint_if->write();

    __tiltjoint_if->set_position(tilt);
    __tiltjoint_if->write();
  }
}


void
PanTiltDirectedPerceptionThread::loop()
{
  __pantilt_if->set_final(__wt->is_final());

  while (! __pantilt_if->msgq_empty() ) {
    if (__pantilt_if->msgq_first_is<PanTiltInterface::CalibrateMessage>()) {
      __wt->reset();

    } else if (__pantilt_if->msgq_first_is<PanTiltInterface::GotoMessage>()) {
      PanTiltInterface::GotoMessage *msg = __pantilt_if->msgq_first(msg);

      __wt->goto_pantilt(msg->pan(), msg->tilt());
      __pantilt_if->set_msgid(msg->id());
      __pantilt_if->set_final(false);

    } else if (__pantilt_if->msgq_first_is<PanTiltInterface::ParkMessage>()) {
      PanTiltInterface::ParkMessage *msg = __pantilt_if->msgq_first(msg);

      __wt->goto_pantilt(0, 0);
      __pantilt_if->set_msgid(msg->id());
      __pantilt_if->set_final(false);

    } else if (__pantilt_if->msgq_first_is<PanTiltInterface::SetEnabledMessage>()) {
      PanTiltInterface::SetEnabledMessage *msg = __pantilt_if->msgq_first(msg);

      logger->log_warn(name(), "SetEnabledMessage ignored for Sony EviD100P");

    } else if (__pantilt_if->msgq_first_is<PanTiltInterface::SetVelocityMessage>()) {
      PanTiltInterface::SetVelocityMessage *msg = __pantilt_if->msgq_first(msg);

      logger->log_warn(name(), "SetVelocityMessage ignored for Sony EviD100P");

      /* ignored for now
      if (msg->pan_velocity() > __pantilt_if->max_pan_velocity()) {
	logger->log_warn(name(), "Desired pan velocity %f too high, max is %f",
			 msg->pan_velocity(), __pantilt_if->max_pan_velocity());
      } else if (msg->tilt_velocity() > __pantilt_if->max_tilt_velocity()) {
	logger->log_warn(name(), "Desired tilt velocity %f too high, max is %f",
			 msg->tilt_velocity(), __pantilt_if->max_tilt_velocity());
      } else {
	__wt->set_velocities(msg->pan_velocity(), msg->tilt_velocity());
	__pantilt_if->set_pan_velocity(msg->pan_velocity());
	__pantilt_if->set_tilt_velocity(msg->tilt_velocity());
	__panjoint_if->set_velocity(msg->pan_velocity());
	__panjoint_if->write();
	__tiltjoint_if->set_velocity(msg->tilt_velocity());
	__tiltjoint_if->write();
      }
      */

    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    __pantilt_if->msgq_pop();
  }

  __pantilt_if->write();

}


bool
PanTiltDirectedPerceptionThread::bb_interface_message_received(Interface *interface,
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


/** @class PanTiltDirectedPerceptionThread::WorkerThread "sony/evid100p_thread.h"
 * Worker thread for the PanTiltDirectedPerceptionThread.
 * This continuous thread issues commands to the camera. In each loop it
 * will first execute pending operations, and then update the sensor data (lengthy
 * operation). Sensor data will only be updated while either a servo in the chain
 * is still moving or torque is disabled (so the motor can be move manually).
 * @author Tim Niemueller
 */


/** Constructor.
 * @param ptu_name name of the pan/tilt unit
 * @param logger logger
 * @param ptu ptu controller
 */
PanTiltDirectedPerceptionThread::WorkerThread::WorkerThread(std::string ptu_name,
						      fawkes::Logger *logger,
						      fawkes::RefPtr<DirectedPerceptionPTU> ptu)
  : Thread("", Thread::OPMODE_WAITFORWAKEUP)
{
  set_name("SonyDirectedPerceptionWorkerThread(%s)", ptu_name.c_str());
  set_coalesce_wakeups(true);

  __logger           = logger;

  __move_mutex       = new Mutex();

  __ptu              = ptu;
  __move_pending     = false;
  __reset_pending    = false;
  __target_pan       = 0;
  __target_tilt      = 0;

  __ptu->get_limits(__pan_min, __pan_max, __tilt_min, __tilt_max);
}


/** Destructor. */
PanTiltDirectedPerceptionThread::WorkerThread::~WorkerThread()
{
  delete __move_mutex;
}


/** Stop currently running motion. */
void
PanTiltDirectedPerceptionThread::WorkerThread::stop_motion()
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
PanTiltDirectedPerceptionThread::WorkerThread::goto_pantilt(float pan, float tilt)
{
  MutexLocker lock(__move_mutex);
  __target_pan   = pan;
  __target_tilt  = tilt;
  __move_pending = true;
  wakeup();
}


/** Get pan/tilt value.
 * @param pan upon return contains the current pan value
 * @param tilt upon return contains the current tilt value
 */
void
PanTiltDirectedPerceptionThread::WorkerThread::get_pantilt(float &pan, float &tilt)
{
  pan  = __cur_pan;
  tilt = __cur_tilt;
}


/** Trigger a reset of the PTU. */
void
PanTiltDirectedPerceptionThread::WorkerThread::reset()
{
  __reset_pending = true;
}


/** Check if motion is final.
 * @return true if motion is final, false otherwise
 */
bool
PanTiltDirectedPerceptionThread::WorkerThread::is_final()
{
  MutexLocker lock(__move_mutex);
  return ( (fabs(__cur_pan  - __target_pan)  < 0.01) &&
	   (fabs(__cur_tilt - __target_tilt) < 0.01));
}


/** Check is fresh sensor data is available.
 * Note that this method will return true at once per sensor update cycle.
 * @return true if fresh data is available, false otherwise
 */
bool
PanTiltDirectedPerceptionThread::WorkerThread::has_fresh_data()
{
  bool rv = __fresh_data;
  __fresh_data = false;
  return rv;
}


void
PanTiltDirectedPerceptionThread::WorkerThread::loop()
{
  if (__move_pending) {
    __move_mutex->lock();
    exec_goto_pantilt(__target_pan, __target_tilt);
    __move_mutex->unlock();
  }

  if (__reset_pending) {
    __move_mutex->lock();
    __reset_pending = false;
    __move_mutex->unlock();
    __ptu->reset();
  }

  try {
    __ptu->get_pan_tilt_rad(__cur_pan, __cur_tilt);
    __fresh_data = true;
  } catch (Exception &e) {
    __logger->log_warn(name(), "Failed to get new pan/tilt data, exception follows");
    __logger->log_warn(name(), e);
  }

  if (! is_final()) {
    // while moving wake us up to get new servo position data
    wakeup();
  }
}


/** Execute pan/tilt motion.
 * @param pan_rad pan in rad to move to
 * @param tilt_rad tilt in rad to move to
 */
void
PanTiltDirectedPerceptionThread::WorkerThread::exec_goto_pantilt(float pan_rad, float tilt_rad)
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

  __ptu->set_pan_tilt_rad(pan_rad, tilt_rad);
  __move_pending = false;
}
