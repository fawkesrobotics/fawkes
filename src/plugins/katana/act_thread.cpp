
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
#include "motion_thread.h"
#include "calib_thread.h"
#include "goto_thread.h"
#include "goto_openrave_thread.h"
#include "gripper_thread.h"
#include "sensacq_thread.h"
#include "motor_control_thread.h"
#include "controller_kni.h"
#include "controller_openrave.h"

#include <core/threading/mutex_locker.h>
#include <interfaces/KatanaInterface.h>
#include <interfaces/JointInterface.h>
#include <utils/math/angle.h>
#include <utils/time/time.h>

#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/robot.h>
 #include <plugins/openrave/manipulator.h>
#endif

#include <algorithm>
#include <cstdarg>

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

  std::string cfg_prefix = "/hardware/katana/";
  __cfg_controller       = config->get_string((cfg_prefix + "controller").c_str());
  __cfg_device           = config->get_string((cfg_prefix + "device").c_str());
  __cfg_kni_conffile     = config->get_string((cfg_prefix + "kni_conffile").c_str());
  __cfg_auto_calibrate   = config->get_bool((cfg_prefix + "auto_calibrate").c_str());
  __cfg_defmax_speed     = config->get_uint((cfg_prefix + "default_max_speed").c_str());
  __cfg_read_timeout     = config->get_uint((cfg_prefix + "read_timeout_msec").c_str());
  __cfg_write_timeout    = config->get_uint((cfg_prefix + "write_timeout_msec").c_str());
  __cfg_gripper_pollint  = config->get_uint((cfg_prefix + "gripper_pollint_msec").c_str());
  __cfg_goto_pollint     = config->get_uint((cfg_prefix + "goto_pollint_msec").c_str());

  __cfg_park_x           = config->get_float((cfg_prefix + "park_x").c_str());
  __cfg_park_y           = config->get_float((cfg_prefix + "park_y").c_str());
  __cfg_park_z           = config->get_float((cfg_prefix + "park_z").c_str());
  __cfg_park_phi         = config->get_float((cfg_prefix + "park_phi").c_str());
  __cfg_park_theta       = config->get_float((cfg_prefix + "park_theta").c_str());
  __cfg_park_psi         = config->get_float((cfg_prefix + "park_psi").c_str());

  __cfg_distance_scale   = config->get_float((cfg_prefix + "distance_scale").c_str());

  __cfg_update_interval  = config->get_float((cfg_prefix + "update_interval").c_str());

  __cfg_frame_kni        = config->get_string((cfg_prefix + "frame/kni").c_str());
  __cfg_frame_gripper    = config->get_string((cfg_prefix + "frame/gripper").c_str());
  __cfg_frame_openrave   = config->get_string((cfg_prefix + "frame/openrave").c_str());

#ifdef HAVE_OPENRAVE
  __cfg_OR_enabled       = config->get_bool((cfg_prefix + "openrave/enabled").c_str());
  __cfg_OR_use_viewer    = config->get_bool((cfg_prefix + "openrave/use_viewer").c_str());
  __cfg_OR_auto_load_ik  = config->get_bool((cfg_prefix + "openrave/auto_load_ik").c_str());
  __cfg_OR_robot_file    = config->get_string((cfg_prefix + "openrave/robot_file").c_str());
  __cfg_OR_arm_model     = config->get_string((cfg_prefix + "openrave/arm_model").c_str());
#else
  __cfg_OR_enabled       = false;
#endif

  // see if config entries for joints exist
  std::string joint_name;
  for( long long i = 0; i < 5; ++i) {
    joint_name = config->get_string((cfg_prefix + "joints/" + std::to_string(i)).c_str() );
    joint_name.clear();
  }
  joint_name = config->get_string((cfg_prefix + "joints/finger_l").c_str());
  joint_name.clear();
  joint_name = config->get_string((cfg_prefix + "joints/finger_r").c_str());
  joint_name.clear();

  __last_update->set_clock(clock);
  __last_update->set_time(0, 0);

  // Load katana controller
  if( __cfg_controller == "kni") {
    KatanaControllerKni* kat_ctrl = new KatanaControllerKni();
    __katana = kat_ctrl;
    try {
      kat_ctrl->setup(__cfg_device, __cfg_kni_conffile,
                      __cfg_read_timeout, __cfg_write_timeout);
    } catch(fawkes::Exception &e) {
      logger->log_warn(name(), "Setup KatanaControllerKni failed. Ex: %s", e.what());
    }
    kat_ctrl = NULL;

  } else if( __cfg_controller == "openrave") {
#ifdef HAVE_OPENRAVE
    if(!__cfg_OR_enabled) {
      throw fawkes::Exception("Cannot use controller 'openrave', OpenRAVE is deactivated by config flag!");
    }
    __katana = new KatanaControllerOpenrave(openrave);
#else
    throw fawkes::Exception("Cannot use controller 'openrave', OpenRAVE not installed!");
#endif

  } else {
    throw fawkes::Exception("Invalid controller given: '%s'", __cfg_controller.c_str());
  }

  // If you have more than one interface: catch exception and close them!
  try {
    __katana_if = blackboard->open_for_writing<KatanaInterface>("Katana");
    __joint_ifs = new std::vector<JointInterface*>();
    JointInterface* joint_if = NULL;
    for( long long i = 0; i < 5; ++i) {
      joint_name = config->get_string((cfg_prefix + "joints/" + std::to_string(i)).c_str() );
      joint_if = blackboard->open_for_writing<JointInterface>(joint_name.c_str());
      __joint_ifs->push_back( joint_if );
      joint_name.clear();
    }

    joint_name = config->get_string((cfg_prefix + "joints/finger_l").c_str());
    joint_if = blackboard->open_for_writing<JointInterface>(joint_name.c_str());
    __joint_ifs->push_back( joint_if );
    joint_name.clear();
    joint_name = config->get_string((cfg_prefix + "joints/finger_r").c_str());
    joint_if = blackboard->open_for_writing<JointInterface>(joint_name.c_str());
    __joint_ifs->push_back( joint_if );
    joint_name.clear();

    joint_if = NULL;
  } catch(Exception &e) {
    finalize();
    throw;
  }

  // Create all other threads
  __sensacq_thread = new KatanaSensorAcquisitionThread(__katana, logger);
  __calib_thread   = new KatanaCalibrationThread(__katana, logger);
  __gripper_thread = new KatanaGripperThread(__katana, logger, __cfg_gripper_pollint);
  __motor_control_thread = new KatanaMotorControlThread(__katana, logger, __cfg_goto_pollint);
  __goto_thread    = new KatanaGotoThread(__katana, logger, __cfg_goto_pollint);
#ifdef HAVE_OPENRAVE
  __goto_openrave_thread = new KatanaGotoOpenRaveThread(__katana, logger, openrave, __cfg_goto_pollint, __cfg_OR_robot_file,
                                                        __cfg_OR_arm_model, __cfg_OR_auto_load_ik, __cfg_OR_use_viewer);
  if(__cfg_OR_enabled)
    {__goto_openrave_thread->init();}
#endif

  // Intialize katana controller
  try {
    __katana->init();
    __katana->set_max_velocity(__cfg_defmax_speed);
    logger->log_debug(name(), "Katana successfully initialized");
  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Initializing controller failed. Ex: %s", e.what());
    finalize();
    throw;
  }

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
  try {
    if( __actmot_thread ) {
      __actmot_thread->cancel();
      __actmot_thread->join();
      __actmot_thread = NULL;
    }
  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "failed finalizing motion thread. Ex:%s", e.what());
  }

  try {
    __sensacq_thread->cancel();
    __sensacq_thread->join();
    __sensacq_thread.reset();
  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "failed finalizing sensor_aquisition thread. Ex:%s", e.what());
  }

  // Setting to NULL also deletes instance (RefPtr)
  __calib_thread   = NULL;
  __goto_thread    = NULL;
  __gripper_thread = NULL;
  __motor_control_thread = NULL;
#ifdef HAVE_OPENRAVE
  if( __cfg_OR_enabled && __goto_openrave_thread )
    __goto_openrave_thread->finalize();
  __goto_openrave_thread = NULL;
#endif

  try {
    __katana->stop();
  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "failed stopping katana. Ex:%s", e.what());
  }
  __katana = NULL;

  try {
    blackboard->unregister_listener(this);
  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "failed unregistering blackboard listener. Ex:%s", e.what());
  }

  if(__katana_if)
    blackboard->close(__katana_if);
  for (std::vector<JointInterface*>::iterator it = __joint_ifs->begin(); it != __joint_ifs->end(); ++it) {
    blackboard->close(*it);
  }
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
  try {
    __katana->read_coordinates(refresh);
    if( __cfg_controller == "kni") {
      __katana_if->set_x(__cfg_distance_scale * __katana->x());
      __katana_if->set_y(__cfg_distance_scale * __katana->y());
      __katana_if->set_z(__cfg_distance_scale * __katana->z());
    } else if( __cfg_controller == "openrave") {

      if( !tf_listener->frame_exists(__cfg_frame_openrave) ) {
        logger->log_warn(name(), "tf frames not existing: '%s'. Skipping transform to kni coordinates.",
                         __cfg_frame_openrave.c_str() );
      } else {
        Stamped<Point> target;
        Stamped<Point> target_local(Point(__katana->x(), __katana->y(), __katana->z()),
                                    fawkes::Time(0,0), __cfg_frame_openrave);

        tf_listener->transform_point(__cfg_frame_kni, target_local, target);

        __katana_if->set_x(target.getX());
        __katana_if->set_y(target.getY());
        __katana_if->set_z(target.getZ());
      }
    }
    __katana_if->set_phi(__katana->phi());
    __katana_if->set_theta(__katana->theta());
    __katana_if->set_psi(__katana->psi());
  } catch (fawkes::Exception &e) {
    logger->log_warn(name(), "Updating position values failed: %s", e.what());
  }

  float *a = __katana_if->angles();

  __joint_ifs->at(0)->set_position(  a[0] + M_PI);
  __joint_ifs->at(1)->set_position(  a[1]);// + M_PI/2);
  __joint_ifs->at(2)->set_position(  a[2] + M_PI);
  __joint_ifs->at(3)->set_position(  a[3] - M_PI);
  __joint_ifs->at(4)->set_position(  a[4]);
  __joint_ifs->at(5)->set_position(  -a[5]/2.f - 0.5f);
  __joint_ifs->at(6)->set_position(  -a[5]/2.f - 0.5f);
  for( unsigned int i=0; i<__joint_ifs->size(); ++i) {
    __joint_ifs->at(i)->write();
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
    std::vector<int> sensors;
    __katana->get_sensors(sensors, false);

    const int num_sensors = std::min(sensors.size(), __katana_if->maxlenof_sensor_value());
    for (int i = 0; i < num_sensors; ++i) {
      if (sensors.at(i) <= 0) {
        __katana_if->set_sensor_value(i, 0);
      } else if (sensors.at(i) >= 255) {
        __katana_if->set_sensor_value(i, 255);
      } else {
        __katana_if->set_sensor_value(i, sensors.at(i));
      }
    }
  } catch (fawkes::Exception &e) {
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
    if( __katana->joint_encoders()) {
      std::vector<int> encoders;
      __katana->get_encoders(encoders, refresh);
      for(unsigned int i=0; i<encoders.size(); i++) {
        __katana_if->set_encoders(i, encoders.at(i));
      }
    }

    if( __katana->joint_angles()) {
      std::vector<float> angles;
      __katana->get_angles(angles, false);
      for(unsigned int i=0; i<angles.size(); i++) {
        __katana_if->set_angles(i, angles.at(i));
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
    __katana->stop();
  } catch (fawkes::Exception &e) {
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

      update_motors(/* refresh */ true);
      update_position(/* refresh */ true);

#ifdef HAVE_OPENRAVE
      if(__cfg_OR_enabled) { __goto_openrave_thread->update_openrave_data(); }
#endif
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

      bool trans_frame_exists = tf_listener->frame_exists(msg->trans_frame());
      bool rot_frame_exists   = tf_listener->frame_exists(msg->rot_frame());
      if( !trans_frame_exists || !rot_frame_exists ) {
        logger->log_warn(name(), "tf frames not existing: '%s%s%s'. Skipping message.",
                         trans_frame_exists ? "" : msg->trans_frame(),
                         trans_frame_exists ? "" : "', '",
                         rot_frame_exists   ? "" : msg->rot_frame() );
      } else {
        Stamped<Point> target;
        Stamped<Point> target_local(Point(msg->x(), msg->y(), msg->z()),
                                    fawkes::Time(0,0), msg->trans_frame());
        if( __cfg_OR_enabled ) {
#ifdef HAVE_OPENRAVE
          tf_listener->transform_point(__cfg_frame_openrave, target_local, target);
          if( msg->offset_xy() != 0.f ) {
            Vector3 offset(target.getX(), target.getY(), 0.f);
            offset = (offset/offset.length()) * msg->offset_xy(); // Vector3 does not support a set_length method
            target += offset;
          }
          // TODO: how to transform euler rotation to quaternion, to be used for tf??
          if( strcmp(msg->trans_frame(), __cfg_frame_gripper.c_str())==0 ) {
            __goto_openrave_thread->set_target(msg->x(), msg->y(), msg->z(),
                                               msg->phi(), msg->theta(), msg->psi());
            __goto_openrave_thread->set_arm_extension(true);
          } else {
            __goto_openrave_thread->set_target(target.getX(), target.getY(), target.getZ(),
                                               msg->phi(), msg->theta(), msg->psi());
          }
          __goto_openrave_thread->set_theta_error(msg->theta_error());
          __goto_openrave_thread->set_move_straight(msg->is_straight());
 #ifdef EARLY_PLANNING
          __goto_openrave_thread->plan_target();
 #endif
          start_motion(__goto_openrave_thread, msg->id(),
                       "Linear movement to (%f,%f,%f, %f,%f,%f), frame '%s', theta_error:%f, straight:%u",
                       target.getX(), target.getY(), target.getZ(),
                       msg->phi(), msg->theta(), msg->psi(), __cfg_frame_openrave.c_str(), msg->theta_error(), msg->is_straight());
#endif
        } else {
          tf_listener->transform_point(__cfg_frame_kni, target_local, target);
          if( msg->offset_xy() != 0.f ) {
            Vector3 offset(target.getX(), target.getY(), 0.f);
            offset = (offset/offset.length()) * msg->offset_xy(); // Vector3 does not support a set_length method
            target += offset;
          }
          __goto_thread->set_target(target.getX() / __cfg_distance_scale,
                                    target.getY() / __cfg_distance_scale,
                                    target.getZ() / __cfg_distance_scale,
                                    msg->phi(), msg->theta(), msg->psi());
          start_motion(__goto_thread, msg->id(),
                       "Linear movement to (%f,%f,%f, %f,%f,%f), frame '%s'",
                       target.getX(), target.getY(), target.getZ(),
                       msg->phi(), msg->theta(), msg->psi(), __cfg_frame_kni.c_str());
        }
      }

    } else if (__katana_if->msgq_first_is<KatanaInterface::LinearGotoKniMessage>()) {
      KatanaInterface::LinearGotoKniMessage *msg = __katana_if->msgq_first(msg);

      float x = msg->x() * __cfg_distance_scale;
      float y = msg->y() * __cfg_distance_scale;
      float z = msg->z() * __cfg_distance_scale;

      tf::Stamped<Point> target;
      tf::Stamped<Point> target_local(tf::Point(x, y, z),
                                      fawkes::Time(0,0), __cfg_frame_kni);

      if( __cfg_OR_enabled ) {
#ifdef HAVE_OPENRAVE
        tf_listener->transform_point(__cfg_frame_openrave, target_local, target);
        __goto_openrave_thread->set_target(target.getX(), target.getY(), target.getZ(),
                                           msg->phi(), msg->theta(), msg->psi());
 #ifdef EARLY_PLANNING
        __goto_openrave_thread->plan_target();
 #endif
        start_motion(__goto_openrave_thread, msg->id(),
                     "Linear movement to (%f,%f,%f, %f,%f,%f), frame '%s'",
                     target.getX(), target.getY(), target.getZ(),
                     msg->phi(), msg->theta(), msg->psi(), __cfg_frame_openrave.c_str());
#endif
      } else {
        __goto_thread->set_target(msg->x(), msg->y(), msg->z(),
                                  msg->phi(), msg->theta(), msg->psi());

        start_motion(__goto_thread, msg->id(),
                     "Linear movement to (%f,%f,%f, %f,%f,%f), frame '%s'",
                     x, y, z,
                     msg->phi(), msg->theta(), msg->psi(), __cfg_frame_kni.c_str());
      }

#ifdef HAVE_OPENRAVE
    } else if (__katana_if->msgq_first_is<KatanaInterface::ObjectGotoMessage>() && __cfg_OR_enabled) {
      KatanaInterface::ObjectGotoMessage *msg = __katana_if->msgq_first(msg);

      float rot_x = 0.f;
      if( msg->rot_x() )
        { rot_x = msg->rot_x(); }

      __goto_openrave_thread->set_target(msg->object(), rot_x);
 #ifdef EARLY_PLANNING
      __goto_openrave_thread->plan_target();
 #endif
      start_motion(__goto_openrave_thread, msg->id(),
                   "Linear movement to object (%s, %f)", msg->object(), msg->rot_x());
#endif

    } else if (__katana_if->msgq_first_is<KatanaInterface::ParkMessage>()) {
      KatanaInterface::ParkMessage *msg = __katana_if->msgq_first(msg);

      if(__cfg_OR_enabled) {
#ifdef HAVE_OPENRAVE
        tf::Stamped<Point> target;
        tf::Stamped<Point> target_local(tf::Point(__cfg_park_x * __cfg_distance_scale,
                                                  __cfg_park_y * __cfg_distance_scale,
                                                  __cfg_park_z * __cfg_distance_scale),
                                        fawkes::Time(0,0), __cfg_frame_kni);
        tf_listener->transform_point(__cfg_frame_openrave, target_local, target);
        __goto_openrave_thread->set_target(target.getX(), target.getY(), target.getZ(),
                                           __cfg_park_phi, __cfg_park_theta, __cfg_park_psi);
 #ifdef EARLY_PLANNING
        __goto_openrave_thread->plan_target();
 #endif
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
          __katana->turn_on();
          update_position(/* refresh */ true);
          update_motors(/* refresh */ true);
#ifdef HAVE_OPENRAVE
          if(__cfg_OR_enabled)
            __goto_openrave_thread->update_openrave_data();
#endif
        } else {
          logger->log_debug(name(), "Turning OFF the arm");
          __katana->turn_off();
        }
        __katana_if->set_enabled(msg->is_enabled());
      } catch (/*KNI*/::Exception &e) {
        logger->log_warn(name(), "Failed enable/disable arm: %s", e.what());
      }

    } else if (__katana_if->msgq_first_is<KatanaInterface::SetMaxVelocityMessage>()) {
      KatanaInterface::SetMaxVelocityMessage *msg = __katana_if->msgq_first(msg);

      unsigned int max_vel = msg->max_velocity();
      if ( max_vel == 0 )  max_vel = __cfg_defmax_speed;

      try {
        __katana->set_max_velocity(max_vel);
      } catch (fawkes::Exception &e) {
        logger->log_warn(name(), "Failed setting max velocity. Ex:%s", e.what());
      }
      __katana_if->set_max_velocity(max_vel);

    } else if (__katana_if->msgq_first_is<KatanaInterface::SetPlannerParamsMessage>()) {
      KatanaInterface::SetPlannerParamsMessage *msg = __katana_if->msgq_first(msg);

      if( __cfg_OR_enabled ) {
#ifdef HAVE_OPENRAVE
        __goto_openrave_thread->set_plannerparams(msg->plannerparams());
#endif
      }

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
KatanaActThread::bb_interface_message_received(Interface *interface, Message *message) throw()
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
