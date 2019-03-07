
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
#include <interfaces/JointInterface.h>
#include <interfaces/PanTiltInterface.h>

#include <cmath>
#include <cstdarg>

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

	pantilt_cfg_prefix_ = pantilt_cfg_prefix;
	ptu_cfg_prefix_     = ptu_cfg_prefix;
	ptu_name_           = ptu_name;
}

void
PanTiltDirectedPerceptionThread::init()
{
	// Note: due to the use of auto_ptr and RefPtr resources are automatically
	// freed on destruction, therefore no special handling is necessary in init()
	// itself!

	cfg_device_          = config->get_string((ptu_cfg_prefix_ + "device").c_str());
	cfg_read_timeout_ms_ = config->get_uint((ptu_cfg_prefix_ + "read_timeout_ms").c_str());

	ptu_ = new DirectedPerceptionPTU(cfg_device_.c_str(), cfg_read_timeout_ms_);

	// If you have more than one interface: catch exception and close them!
	std::string bbid = "PanTilt " + ptu_name_;
	pantilt_if_      = blackboard->open_for_writing<PanTiltInterface>(bbid.c_str());

	float min_pan = 0, max_pan = 0, min_tilt = 0, max_tilt = 0;
	ptu_->get_limits(min_pan, max_pan, min_tilt, max_tilt);

	pantilt_if_->set_calibrated(true);
	pantilt_if_->set_min_pan(min_pan);
	pantilt_if_->set_max_pan(max_pan);
	pantilt_if_->set_min_tilt(min_tilt);
	pantilt_if_->set_max_tilt(max_tilt);
	pantilt_if_->set_enabled(true); // Cannot be turned off
	//pantilt_if_->set_max_pan_velocity(0);
	//pantilt_if_->set_max_tilt_velocity(0);
	//pantilt_if_->set_pan_velocity(0);
	//pantilt_if_->set_tilt_velocity(0);
	pantilt_if_->write();

	float init_pan           = 0.f;
	float init_tilt          = 0.f;
	float init_pan_velocity  = 0.f;
	float init_tilt_velocity = 0.f;

	std::string panid = ptu_name_ + " pan";
	panjoint_if_      = blackboard->open_for_writing<JointInterface>(panid.c_str());
	panjoint_if_->set_position(init_pan);
	panjoint_if_->set_velocity(init_pan_velocity);
	panjoint_if_->write();

	std::string tiltid = ptu_name_ + " tilt";
	tiltjoint_if_      = blackboard->open_for_writing<JointInterface>(tiltid.c_str());
	tiltjoint_if_->set_position(init_tilt);
	tiltjoint_if_->set_velocity(init_tilt_velocity);
	tiltjoint_if_->write();

	wt_ = new WorkerThread(ptu_name_, logger, ptu_);
	wt_->start();

	bbil_add_message_interface(pantilt_if_);
	bbil_add_message_interface(panjoint_if_);
	bbil_add_message_interface(tiltjoint_if_);
	blackboard->register_listener(this);

#ifdef USE_TIMETRACKER
	tt_.reset(new TimeTracker());
	tt_count_        = 0;
	ttc_read_sensor_ = tt_->add_class("Read Sensor");
#endif
}

void
PanTiltDirectedPerceptionThread::finalize()
{
	blackboard->unregister_listener(this);
	blackboard->close(pantilt_if_);
	blackboard->close(panjoint_if_);
	blackboard->close(tiltjoint_if_);

	wt_->cancel();
	wt_->join();
	delete wt_;

	// Setting to NULL deletes instance (RefPtr)
	ptu_ = NULL;
}

/** Update sensor values as necessary.
 * To be called only from PanTiltSensorThread. Writes the current pan/tilt
 * data into the interface.
 */
void
PanTiltDirectedPerceptionThread::update_sensor_values()
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
	}
}

void
PanTiltDirectedPerceptionThread::loop()
{
	pantilt_if_->set_final(wt_->is_final());

	while (!pantilt_if_->msgq_empty()) {
		if (pantilt_if_->msgq_first_is<PanTiltInterface::CalibrateMessage>()) {
			wt_->reset();

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

			logger->log_warn(name(), "SetVelocityMessage ignored for Sony EviD100P");

			/* ignored for now
      if (msg->pan_velocity() > pantilt_if_->max_pan_velocity()) {
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
      */

		} else {
			logger->log_warn(name(), "Unknown message received");
		}

		pantilt_if_->msgq_pop();
	}

	pantilt_if_->write();
}

bool
PanTiltDirectedPerceptionThread::bb_interface_message_received(Interface *interface,
                                                               Message *  message) throw()
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
PanTiltDirectedPerceptionThread::WorkerThread::WorkerThread(
  std::string                           ptu_name,
  fawkes::Logger *                      logger,
  fawkes::RefPtr<DirectedPerceptionPTU> ptu)
: Thread("", Thread::OPMODE_WAITFORWAKEUP)
{
	set_name("SonyDirectedPerceptionWorkerThread(%s)", ptu_name.c_str());
	set_coalesce_wakeups(true);

	logger_ = logger;

	move_mutex_ = new Mutex();

	ptu_           = ptu;
	move_pending_  = false;
	reset_pending_ = false;
	target_pan_    = 0;
	target_tilt_   = 0;

	ptu_->get_limits(pan_min_, pan_max_, tilt_min_, tilt_max_);
}

/** Destructor. */
PanTiltDirectedPerceptionThread::WorkerThread::~WorkerThread()
{
	delete move_mutex_;
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
	MutexLocker lock(move_mutex_);
	target_pan_   = pan;
	target_tilt_  = tilt;
	move_pending_ = true;
	wakeup();
}

/** Get pan/tilt value.
 * @param pan upon return contains the current pan value
 * @param tilt upon return contains the current tilt value
 */
void
PanTiltDirectedPerceptionThread::WorkerThread::get_pantilt(float &pan, float &tilt)
{
	pan  = cur_pan_;
	tilt = cur_tilt_;
}

/** Trigger a reset of the PTU. */
void
PanTiltDirectedPerceptionThread::WorkerThread::reset()
{
	reset_pending_ = true;
}

/** Check if motion is final.
 * @return true if motion is final, false otherwise
 */
bool
PanTiltDirectedPerceptionThread::WorkerThread::is_final()
{
	MutexLocker lock(move_mutex_);
	return ((fabs(cur_pan_ - target_pan_) < 0.01) && (fabs(cur_tilt_ - target_tilt_) < 0.01));
}

/** Check is fresh sensor data is available.
 * Note that this method will return true at once per sensor update cycle.
 * @return true if fresh data is available, false otherwise
 */
bool
PanTiltDirectedPerceptionThread::WorkerThread::has_fresh_data()
{
	bool rv     = fresh_data_;
	fresh_data_ = false;
	return rv;
}

void
PanTiltDirectedPerceptionThread::WorkerThread::loop()
{
	if (move_pending_) {
		move_mutex_->lock();
		exec_goto_pantilt(target_pan_, target_tilt_);
		move_mutex_->unlock();
	}

	if (reset_pending_) {
		move_mutex_->lock();
		reset_pending_ = false;
		move_mutex_->unlock();
		ptu_->reset();
	}

	try {
		ptu_->get_pan_tilt_rad(cur_pan_, cur_tilt_);
		fresh_data_ = true;
	} catch (Exception &e) {
		logger_->log_warn(name(), "Failed to get new pan/tilt data, exception follows");
		logger_->log_warn(name(), e);
	}

	if (!is_final()) {
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
	if ((pan_rad < pan_min_) || (pan_rad > pan_max_)) {
		logger_->log_warn(
		  name(), "Pan value out of bounds, min: %f  max: %f  des: %f", pan_min_, pan_max_, pan_rad);
		return;
	}
	if ((tilt_rad < tilt_min_) || (tilt_rad > tilt_max_)) {
		logger_->log_warn(name(),
		                  "Tilt value out of bounds, min: %f  max: %f  des: %f",
		                  tilt_min_,
		                  tilt_max_,
		                  tilt_rad);
		return;
	}

	ptu_->set_pan_tilt_rad(pan_rad, tilt_rad);
	move_pending_ = false;
}
