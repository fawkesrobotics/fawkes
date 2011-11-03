
/***************************************************************************
 *  act_thread.cpp - Katana plugin act thread
 *
 *  Created: Mon Jun 08 18:00:56 2009
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

#include "act_thread.h"

#include <core/threading/mutex_locker.h>
#include <interfaces/KatanaInterface.h>
#include <utils/math/angle.h>
#include <utils/time/time.h>

#include <algorithm>
#include <cstdarg>
#include <kniBase.h>
#include <common/MathHelperFunctions.h>

using namespace fawkes;
using namespace fawkes::tf;

/** @class KatanaActThread "act_thread.h"
 * Katana act thread.
 * This thread integrates into the Fawkes main loop at the ACT_EXEC hook and
 * interacts with the controller of the Katana arm via the KNI library.
 * @author Tim Niemueller
 */

/** Constructor. */
KatanaActThread::KatanaActThread()
  : Thread("KatanaActThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC),
#ifdef HAVE_TF
    TransformAspect(TransformAspect::ONLY_PUBLISHER, "Katana"),
#endif
    BlackBoardInterfaceListener("KatanaActThread")
{
  __last_update = new Time();
}

/** Destructor. */
KatanaActThread::~KatanaActThread()
{
  delete __last_update;
}


void
KatanaActThread::init()
{
  // Note: due to the use of auto_ptr and RefPtr resources are automatically
  // freed on destruction, therefore no special handling is necessary in init()
  // itself!

  __cfg_device           = config->get_string("/hardware/katana/device");
  __cfg_kni_conffile     = config->get_string("/hardware/katana/kni_conffile");
  __cfg_auto_calibrate   = config->get_bool("/hardware/katana/auto_calibrate");
  __cfg_defmax_speed     = config->get_uint("/hardware/katana/default_max_speed");
  __cfg_read_timeout     = config->get_uint("/hardware/katana/read_timeout_msec");
  __cfg_write_timeout    = config->get_uint("/hardware/katana/write_timeout_msec");
  __cfg_gripper_pollint  = config->get_uint("/hardware/katana/gripper_pollint_msec");
  __cfg_goto_pollint     = config->get_uint("/hardware/katana/goto_pollint_msec");

  __cfg_park_x           = config->get_float("/hardware/katana/park_x");
  __cfg_park_y           = config->get_float("/hardware/katana/park_y");
  __cfg_park_z           = config->get_float("/hardware/katana/park_z");
  __cfg_park_phi         = config->get_float("/hardware/katana/park_phi");
  __cfg_park_theta       = config->get_float("/hardware/katana/park_theta");
  __cfg_park_psi         = config->get_float("/hardware/katana/park_psi");

  __cfg_offset_x         = config->get_float("/hardware/katana/offset_x");
  __cfg_offset_y         = config->get_float("/hardware/katana/offset_y");
  __cfg_offset_z         = config->get_float("/hardware/katana/offset_z");
  __cfg_distance_scale   = config->get_float("/hardware/katana/distance_scale");

  __cfg_update_interval  = config->get_float("/hardware/katana/update_interval");

#ifdef HAVE_OPENRAVE
  __cfg_OR_enabled       = config->get_bool("/hardware/katana/openrave/enabled");
  __cfg_OR_use_viewer    = config->get_bool("/hardware/katana/openrave/use_viewer");
  __cfg_OR_auto_load_ik  = config->get_bool("/hardware/katana/openrave/auto_load_ik");
  __cfg_OR_robot_file    = config->get_string("/hardware/katana/openrave/robot_file");
#else
  __cfg_OR_enabled       = false;
#endif

  __last_update->set_clock(clock);
  __last_update->set_time(0, 0);

  try {
    TCdlCOMDesc ccd = {0, 57600, 8, 'N', 1, __cfg_read_timeout, __cfg_write_timeout};
    __device.reset(new CCdlCOM(ccd, __cfg_device.c_str()));

    __protocol.reset(new CCplSerialCRC());
    __protocol->init(__device.get());

    __katana = RefPtr<CLMBase>(new CLMBase());
    __katana->create(__cfg_kni_conffile.c_str(), __protocol.get());
    __katbase = __katana->GetBase();
    __sensor_ctrl = &__katbase->GetSCT()->arr[0];

    __katana->setRobotVelocityLimit(__cfg_defmax_speed);

    __katbase->recvECH();
    logger->log_debug(name(), "Katana successfully initialized");

  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception(e.what());
  }

  try {
    __motor_init.resize(__katana->getNumberOfMotors());
    for(unsigned int i=0; i<__motor_init.size(); i++) {
      __motor_init.at(i) = *(__katbase->GetMOT()->arr[i].GetInitialParameters());
    }

    logger->log_debug(name(), "Katana initial motor parameters successfully loaded");
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception(e.what());
  }

  // If you have more than one interface: catch exception and close them!
  __katana_if = blackboard->open_for_writing<KatanaInterface>("Katana");

  __sensacq_thread.reset(new KatanaSensorAcquisitionThread(__katana, logger));
  __calib_thread   = new KatanaCalibrationThread(__katana, logger);
  __gripper_thread = new KatanaGripperThread(__katana, logger,
					     __cfg_gripper_pollint);
  __motor_control_thread = new KatanaMotorControlThread(__katana, logger, __cfg_goto_pollint);
  __goto_thread    = new KatanaGotoThread(__katana, logger, __cfg_goto_pollint);
#ifdef HAVE_OPENRAVE
  __goto_openrave_thread = new KatanaGotoOpenRaveThread(__katana, logger, openrave, __cfg_goto_pollint, __cfg_OR_robot_file, __cfg_OR_auto_load_ik, __cfg_OR_use_viewer);
  if(__cfg_OR_enabled)
    {__goto_openrave_thread->init();}
#endif

  __sensacq_thread->start();

  bbil_add_message_interface(__katana_if);
  blackboard->register_listener(this);

#ifdef USE_TIMETRACKER
  __tt.reset(new TimeTracker());
  __tt_count = 0;
  __ttc_read_sensor = __tt->add_class("Read Sensor");
#endif

}


void
KatanaActThread::finalize()
{
  if ( __actmot_thread ) {
    __actmot_thread->cancel();
    __actmot_thread->join();
    __actmot_thread = NULL;
  }
  __sensacq_thread->cancel();
  __sensacq_thread->join();
  __sensacq_thread.reset();

  // Setting to NULL also deletes instance (RefPtr)
  __calib_thread   = NULL;
  __goto_thread    = NULL;
  __gripper_thread = NULL;
  __motor_control_thread = NULL;
#ifdef HAVE_OPENRAVE
   if(__cfg_OR_enabled)
    {__goto_openrave_thread->finalize();}
  __goto_openrave_thread = NULL;
#endif

  __katana->freezeRobot();
  __katana = NULL;

  __device.reset();
  __protocol.reset();

  blackboard->unregister_listener(this);
  blackboard->close(__katana_if);
}


void
KatanaActThread::once()
{
  if ( __cfg_auto_calibrate ) {
    start_motion(__calib_thread, 0, "Auto calibration enabled, calibrating");
    __katana_if->set_enabled(true);
    __katana_if->write();
  }
}


/** Update position data in BB interface.
 * @param refresh recv new data from arm
 */
void
KatanaActThread::update_position(bool refresh)
{
  double x, y, z, phi, theta, psi;
  try {
    __katana->getCoordinates(x, y, z, phi, theta, psi, refresh);
    __katana_if->set_x(__cfg_offset_x + __cfg_distance_scale * x);
    __katana_if->set_y(__cfg_offset_y + __cfg_distance_scale * y);
    __katana_if->set_z(__cfg_offset_z + __cfg_distance_scale * z);
    __katana_if->set_phi(phi);
    __katana_if->set_theta(theta);
    __katana_if->set_psi(psi);
  } catch (/*KNI*/::Exception &e) {
    logger->log_warn(name(), "Updating position values failed: %s", e.what());
  }

#ifdef HAVE_TF
  float *a = __katana_if->angles();
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
  tf_publisher->send_transform(j5_gr, now, "/katana/j5", "/katana/gripper");
#endif  
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
  if ( __actmot_thread != __calib_thread ) {
    update_sensors(! __actmot_thread);
  }
}


/** Update sensor value in BB interface.
 * @param refresh recv new data from arm
 */
void
KatanaActThread::update_sensors(bool refresh)
{
  try {
    const TSctDAT *sensor_data = __sensor_ctrl->GetDAT();

    unsigned char sensor_values[__katana_if->maxlenof_sensor_value()];
    const int num_sensors = std::min((size_t)sensor_data->cnt, __katana_if->maxlenof_sensor_value());
    for (int i = 0; i < num_sensors; ++i) {
      if (sensor_data->arr[i] <= 0) {
	sensor_values[i] = 0;
      } else if (sensor_data->arr[i] >= 255) {
	sensor_values[i] = 255;
      } else {
	sensor_values[i] = sensor_data->arr[i];
      }
    }

    __katana_if->set_sensor_value(sensor_values);
  } catch (/*KNI*/::Exception &e) {
    logger->log_warn(name(), "Updating sensor values failed: %s", e.what());
  }

  if (refresh) __sensacq_thread->wakeup();
}


/** Update motor encoder and angle data in BB interface.
 * @param refresh recv new data from arm
 */
void
KatanaActThread::update_motors(bool refresh)
{
  try {
    std::vector<int> encoders = __katana->getRobotEncoders(refresh);

    for(unsigned int i=0; i<encoders.size(); i++) {
      __katana_if->set_encoders(i, encoders.at(i));
      __katana_if->set_angles(i, KNI_MHF::enc2rad( encoders.at(i), __motor_init.at(i).angleOffset, __motor_init.at(i).encodersPerCycle, __motor_init.at(i).encoderOffset, __motor_init.at(i).rotationDirection));
    }
  } catch (/*KNI*/::Exception &e) {
    logger->log_warn(name(), "Updating motor values failed: %s", e.what());
  }
}


/** Start a motion.
 * @param motion_thread motion thread to start
 * @param msgid BB message  ID of message that caused the motion
 * @param logmsg message to print, format for following arguments
 */
void
KatanaActThread::start_motion(RefPtr<KatanaMotionThread> motion_thread,
			      unsigned int msgid, const char *logmsg, ...)
{
  va_list arg;
  va_start(arg, logmsg);
  logger->vlog_debug(name(), logmsg, arg);
  __sensacq_thread->set_enabled(false);
  __actmot_thread = motion_thread;
  __actmot_thread->start(/* wait */ false);
  __katana_if->set_msgid(msgid);
  __katana_if->set_final(false);
  va_end(arg);
}


/** Stop any running motion. */
void
KatanaActThread::stop_motion()
{
  logger->log_info(name(), "Stopping arm movement");
  loop_mutex->lock();
  if (__actmot_thread) {
    __actmot_thread->cancel();
    __actmot_thread->join();
    __actmot_thread = NULL;
  }
  try {
    __katana->freezeRobot();
  } catch (/*KNI*/::Exception &e) {
    logger->log_warn(name(), "Failed to freeze robot on stop: %s", e.what());
  }
  loop_mutex->unlock();
}


void
KatanaActThread::loop()
{
  if ( __actmot_thread ) {
    update_motors(/* refresh */ false);
    update_position(/* refresh */ false);
    __katana_if->write();
    if (! __actmot_thread->finished()) {
      return;
    } else {
      logger->log_debug(name(), "Motion thread %s finished, collecting", __actmot_thread->name());
      __actmot_thread->join();
      __katana_if->set_final(true);
      __katana_if->set_error_code(__actmot_thread->error_code());
      if (__actmot_thread == __calib_thread) {
	__katana_if->set_calibrated(true);
      }
      __actmot_thread->reset();
      __actmot_thread = NULL;
      logger->log_debug(name(), "Motion thread collected");
      __sensacq_thread->set_enabled(true);

#ifdef HAVE_OPENRAVE
      if(__cfg_OR_enabled) { __goto_openrave_thread->update_openrave_data(); }
#endif
      update_motors(/* refresh */ true);
      update_position(/* refresh */ true);
    }
  } else if (!__katana_if->is_enabled()) {
      update_position(/* refresh */ true);
      update_motors(/* refresh */ true);

  } else {
    // update every once in a while to keep transforms updated
    fawkes::Time now(clock);
    if ((now - __last_update) >= __cfg_update_interval) {
      __last_update->stamp();
      update_position(/* refresh */ false);
      update_motors(/* refresh */ false);
    }
  }

  while (! __katana_if->msgq_empty() && ! __actmot_thread ) {
    if (__katana_if->msgq_first_is<KatanaInterface::CalibrateMessage>()) {
      KatanaInterface::CalibrateMessage *msg = __katana_if->msgq_first(msg);
      start_motion(__calib_thread, msg->id(), "Calibrating arm");

    } else if (__katana_if->msgq_first_is<KatanaInterface::LinearGotoMessage>()) {
      KatanaInterface::LinearGotoMessage *msg = __katana_if->msgq_first(msg);

      if(__cfg_OR_enabled) {
#ifdef HAVE_OPENRAVE
        __goto_openrave_thread->set_target(msg->x(), msg->y(), msg->z(),
                                           msg->phi(), msg->theta(), msg->psi());
        start_motion(__goto_openrave_thread, msg->id(),
		     "Linear movement to (%f,%f,%f, %f,%f,%f)",
		     msg->x(), msg->y(), msg->z(),
		     msg->phi(), msg->theta(), msg->psi());
#endif
      } else {
        __goto_thread->set_target((msg->x() - __cfg_offset_x)/__cfg_distance_scale,
                                  (msg->y() - __cfg_offset_y)/__cfg_distance_scale,
                                  (msg->z() - __cfg_offset_z)/__cfg_distance_scale,
			 	  msg->phi(), msg->theta(), msg->psi());
        start_motion(__goto_thread, msg->id(),
		     "Linear movement to (%f,%f,%f, %f,%f,%f)",
		     msg->x(), msg->y(), msg->z(),
		     msg->phi(), msg->theta(), msg->psi());
      }

    } else if (__katana_if->msgq_first_is<KatanaInterface::LinearGotoKniMessage>()) {
      KatanaInterface::LinearGotoKniMessage *msg = __katana_if->msgq_first(msg);

      float x = msg->x() * __cfg_distance_scale + __cfg_offset_x;
      float y = msg->y() * __cfg_distance_scale + __cfg_offset_y;
      float z = msg->z() * __cfg_distance_scale + __cfg_offset_z;

      if( __cfg_OR_enabled ) {
#ifdef HAVE_OPENRAVE
          __goto_openrave_thread->set_target(x, y, z,
				  	     msg->phi(), msg->theta(), msg->psi());

          start_motion(__goto_openrave_thread, msg->id(),
		       "Linear movement to (%f,%f,%f, %f,%f,%f)",
		       x, y, z,
		       msg->phi(), msg->theta(), msg->psi());

#endif
        } else {
          __goto_thread->set_target(msg->x(), msg->y(), msg->z(),
				    msg->phi(), msg->theta(), msg->psi());

          start_motion(__goto_thread, msg->id(),
		       "Linear movement to (%f,%f,%f, %f,%f,%f)",
		       x, y, z,
		       msg->phi(), msg->theta(), msg->psi());

        }

#ifdef HAVE_OPENRAVE
    } else if (__katana_if->msgq_first_is<KatanaInterface::ObjectGotoMessage>() && __cfg_OR_enabled) {
      KatanaInterface::ObjectGotoMessage *msg = __katana_if->msgq_first(msg);

      float rot_x = 0.f;
      if( msg->rot_x() )
        { rot_x = msg->rot_x(); }

      __goto_openrave_thread->set_target(msg->object(), rot_x);
      start_motion(__goto_openrave_thread, msg->id(),
		   "Linear movement to object (%s, %f)", msg->object(), msg->rot_x());
#endif

    } else if (__katana_if->msgq_first_is<KatanaInterface::ParkMessage>()) {
      KatanaInterface::ParkMessage *msg = __katana_if->msgq_first(msg);

      if(__cfg_OR_enabled) {
#ifdef HAVE_OPENRAVE
        __goto_openrave_thread->set_target(__cfg_park_x * __cfg_distance_scale + __cfg_offset_x,
                                           __cfg_park_y * __cfg_distance_scale + __cfg_offset_y,
                                  	   __cfg_park_z * __cfg_distance_scale + __cfg_offset_z,
				  	   __cfg_park_phi, __cfg_park_theta, __cfg_park_psi);

        start_motion(__goto_openrave_thread, msg->id(), "Parking arm");
#endif
      } else {
        __goto_thread->set_target(__cfg_park_x, __cfg_park_y, __cfg_park_z,
				  __cfg_park_phi, __cfg_park_theta, __cfg_park_psi);
        start_motion(__goto_thread, msg->id(), "Parking arm");
      }

    } else if (__katana_if->msgq_first_is<KatanaInterface::OpenGripperMessage>()) {
      KatanaInterface::OpenGripperMessage *msg = __katana_if->msgq_first(msg);
      __gripper_thread->set_mode(KatanaGripperThread::OPEN_GRIPPER);
      start_motion(__gripper_thread, msg->id(), "Opening gripper");

    } else if (__katana_if->msgq_first_is<KatanaInterface::CloseGripperMessage>()) {
      KatanaInterface::CloseGripperMessage *msg = __katana_if->msgq_first(msg);
      __gripper_thread->set_mode(KatanaGripperThread::CLOSE_GRIPPER);
      start_motion(__gripper_thread, msg->id(), "Closing gripper");

    } else if (__katana_if->msgq_first_is<KatanaInterface::SetEnabledMessage>()) {
      KatanaInterface::SetEnabledMessage *msg = __katana_if->msgq_first(msg);

      try {
	if (msg->is_enabled()) {
	  logger->log_debug(name(), "Turning ON the arm");
	  __katana->switchRobotOn();
	  update_position(/* refresh */ true);
          update_motors(/* refresh */ true);
#ifdef HAVE_OPENRAVE
	    if(__cfg_OR_enabled)
	      {__goto_openrave_thread->update_openrave_data();}
#endif
	} else {
	  logger->log_debug(name(), "Turning OFF the arm");
	  __katana->switchRobotOff();
	}
	__katana_if->set_enabled(msg->is_enabled());
      } catch (/*KNI*/::Exception &e) {
	logger->log_warn(name(), "Failed enable/disable arm: %s", e.what());
      }

    } else if (__katana_if->msgq_first_is<KatanaInterface::SetMaxVelocityMessage>()) {
      KatanaInterface::SetMaxVelocityMessage *msg = __katana_if->msgq_first(msg);

      unsigned int max_vel = msg->max_velocity();
      if ( max_vel == 0 )  max_vel = __cfg_defmax_speed;

      __katana->setRobotVelocityLimit(max_vel);
      __katana_if->set_max_velocity(max_vel);

    } else if (__katana_if->msgq_first_is<KatanaInterface::SetMotorEncoderMessage>()) {
      KatanaInterface::SetMotorEncoderMessage *msg = __katana_if->msgq_first(msg);

      __motor_control_thread->set_encoder(msg->nr(), msg->enc(), false);
      start_motion(__motor_control_thread, msg->id(), "Moving motor");

    } else if (__katana_if->msgq_first_is<KatanaInterface::MoveMotorEncoderMessage>()) {
      KatanaInterface::MoveMotorEncoderMessage *msg = __katana_if->msgq_first(msg);

      __motor_control_thread->set_encoder(msg->nr(), msg->enc(), true);
      start_motion(__motor_control_thread, msg->id(), "Moving motor");

    } else if (__katana_if->msgq_first_is<KatanaInterface::SetMotorAngleMessage>()) {
      KatanaInterface::SetMotorAngleMessage *msg = __katana_if->msgq_first(msg);

      __motor_control_thread->set_angle(msg->nr(), msg->angle(), false);
      start_motion(__motor_control_thread, msg->id(), "Moving motor");

    } else if (__katana_if->msgq_first_is<KatanaInterface::MoveMotorAngleMessage>()) {
      KatanaInterface::MoveMotorAngleMessage *msg = __katana_if->msgq_first(msg);

      __motor_control_thread->set_angle(msg->nr(), msg->angle(), true);
      start_motion(__motor_control_thread, msg->id(), "Moving motor");

    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    __katana_if->msgq_pop();
  }

  __katana_if->write();

#ifdef USE_TIMETRACKER
  if (++__tt_count > 100) {
    __tt_count = 0;
    __tt->print_to_stdout();
  }
#endif
}


bool
KatanaActThread::bb_interface_message_received(Interface *interface,
					       Message *message) throw()
{
  if (message->is_of_type<KatanaInterface::StopMessage>()) {
    stop_motion();
    return false; // do not enqueue StopMessage
  } else if (message->is_of_type<KatanaInterface::FlushMessage>()) {
    stop_motion();
    logger->log_info(name(), "Flushing message queue");
    __katana_if->msgq_flush();
    return false;
  } else {
    logger->log_info(name(), "Received message of type %s, enqueueing", message->type());
    return true;
  }
}
