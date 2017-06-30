
/***************************************************************************
 *  clips_navgraph_thread.cpp -  NavGraph feature for CLIPS
 *
 *  Created: Wed Oct 09 19:27:41 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include "clips_tf_thread.h"

using namespace fawkes;

/** @class ClipsTFThread "clips-protobuf-thread.h"
 * Provide protobuf functionality to CLIPS environment.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsTFThread::ClipsTFThread()
  : Thread("ClipsTFThread", Thread::OPMODE_WAITFORWAKEUP),
    CLIPSFeature("tf"), CLIPSFeatureAspect(this),
    debug_(true)
{
}


/** Destructor. */
ClipsTFThread::~ClipsTFThread()
{
}


void
ClipsTFThread::init()
{
  try {
    debug_ = config->get_bool("/plugins/clips-tf/debug");
  } catch(...) {}
}


void
ClipsTFThread::finalize()
{
  envs_.clear();
}


void
ClipsTFThread::clips_context_init(const std::string &env_name,
				  LockPtr<CLIPS::Environment> &clips)
{
  envs_[env_name] = clips;
  logger->log_debug(name(), "Called to initialize environment %s", env_name.c_str());

  clips.lock();
  //clips->batch_evaluate(SRCDIR"/clips/navgraph.clp");

  clips->add_function("tf-quat-from-yaw",
    sigc::slot<CLIPS::Values, double>
    (sigc::mem_fun(*this, &ClipsTFThread::clips_tf_quat_from_yaw)));
  clips->add_function("tf-yaw-from-quat",
    sigc::slot<double, CLIPS::Values>
    (sigc::mem_fun(*this, &ClipsTFThread::clips_tf_yaw_from_quat)));

  clips->add_function("tf-frame-exists", sigc::slot<CLIPS::Value, std::string>
    (sigc::mem_fun(*this, &ClipsTFThread::clips_tf_frame_exists)));
  clips->add_function("tf-can-transform",
    sigc::slot<CLIPS::Value, std::string, std::string, CLIPS::Values>
    (sigc::mem_fun(*this, &ClipsTFThread::clips_tf_can_transform)));

  clips->add_function("tf-transform-point",
    sigc::slot<CLIPS::Values, std::string, std::string, CLIPS::Values, CLIPS::Values>
    (sigc::mem_fun(*this, &ClipsTFThread::clips_tf_transform_point)));
  clips->add_function("tf-transform-vector",
    sigc::slot<CLIPS::Values, std::string, std::string, CLIPS::Values, CLIPS::Values>
    (sigc::mem_fun(*this, &ClipsTFThread::clips_tf_transform_vector)));
  clips->add_function("tf-transform-quaternion",
    sigc::slot<CLIPS::Values, std::string, std::string, CLIPS::Values, CLIPS::Values>
    (sigc::mem_fun(*this, &ClipsTFThread::clips_tf_transform_quaternion)));
  clips->add_function("tf-transform-pose",
    sigc::slot<CLIPS::Values, std::string, std::string,
	       CLIPS::Values, CLIPS::Values, CLIPS::Values>
    (sigc::mem_fun(*this, &ClipsTFThread::clips_tf_transform_pose)));

  clips.unlock();
}

void
ClipsTFThread::clips_context_destroyed(const std::string &env_name)
{
  envs_.erase(env_name);
  logger->log_debug(name(), "Removing environment %s", env_name.c_str());
}

CLIPS::Value
ClipsTFThread::clips_tf_frame_exists(std::string frame_id)
{
  return CLIPS::Value(tf_listener->frame_exists(frame_id) ? "TRUE" : "FALSE",
		      CLIPS::TYPE_SYMBOL);
}


CLIPS::Value
ClipsTFThread::clips_tf_can_transform(std::string target_frame, std::string source_frame,
				      CLIPS::Values time)
{
  if (! validate_time(time)) {
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }

  fawkes::Time t(convert_time(time));
  return CLIPS::Value(tf_listener->can_transform(target_frame, source_frame, t)
		      ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}


CLIPS::Values
ClipsTFThread::clips_tf_transform_point(std::string target_frame, std::string source_frame,
					CLIPS::Values time, CLIPS::Values point)
{
  if (! (validate_time(time) && validate_point(point))) {
    return CLIPS::Values(1, CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
  }

  fawkes::Time t(convert_time(time));
  tf::Stamped<tf::Point>
    in(tf::Point(point[0].as_float(), point[1].as_float(), point[2].as_float()),
       t, source_frame);
  tf::Stamped<tf::Point> out;

  try {
    tf_listener->transform_point(target_frame, in, out);

    if (debug_)
      logger->log_debug(name(), "Transformed point %s->%s: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
		      source_frame.c_str(), target_frame.c_str(),
		      in.x(), in.y(), in.z(), out.x(), out.y(), out.z());

    CLIPS::Values rv(3, CLIPS::Value(0.));
    rv[0] = out.x();
    rv[1] = out.y();
    rv[2] = out.z();
    return rv;
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to transform point from %s to %s: %s",
		     source_frame.c_str(), target_frame.c_str(), e.what_no_backtrace());
    return CLIPS::Values(1, CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
  }
}

CLIPS::Values
ClipsTFThread::clips_tf_transform_vector( std::string target_frame, std::string source_frame,
					 CLIPS::Values time, CLIPS::Values vector3)
{
  if (! (validate_time(time) && validate_vector3(vector3))) {
    return CLIPS::Values(1, CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
  }

  fawkes::Time t(convert_time(time));
  tf::Stamped<tf::Vector3>
    in(tf::Vector3(vector3[0].as_float(), vector3[1].as_float(), vector3[2].as_float()),
       t, source_frame);
  tf::Stamped<tf::Vector3> out;

  try {
    tf_listener->transform_vector(target_frame, in, out);

    if (debug_)
      logger->log_debug(name(), "Transformed vector %s->%s: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
		      source_frame.c_str(), target_frame.c_str(),
		      in.x(), in.y(), in.z(), out.x(), out.y(), out.z());

    CLIPS::Values rv(3, CLIPS::Value(0.));
    rv[0] = out.x();
    rv[1] = out.y();
    rv[2] = out.z();
    return rv;
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to transform vector from %s to %s: %s",
		     source_frame.c_str(), target_frame.c_str(), e.what_no_backtrace());
    return CLIPS::Values(1, CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
  }
}

CLIPS::Values
ClipsTFThread::clips_tf_transform_quaternion(std::string target_frame, std::string source_frame,
					     CLIPS::Values time, CLIPS::Values quat)
{
  if (! (validate_time(time) && validate_quat(quat))) {
    return CLIPS::Values(1, CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
  }

  fawkes::Time t(convert_time(time));
  tf::Stamped<tf::Quaternion>
    in(tf::Quaternion(quat[0].as_float(), quat[1].as_float(),
		      quat[2].as_float(), quat[3].as_float()),
       t, source_frame);
  tf::Stamped<tf::Quaternion> out;

  try {
    tf_listener->transform_quaternion(target_frame, in, out);

    if (debug_)
      logger->log_debug(name(), "Transformed quaternion %s->%s: "
		      "(%.2f,%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f,%.2f)",
		      source_frame.c_str(), target_frame.c_str(),
		      in.x(), in.y(), in.z(), in.w(), out.x(), out.y(), out.z(), out.w());

    CLIPS::Values rv(4, CLIPS::Value(0.));
    rv[0] = out.x();
    rv[1] = out.y();
    rv[2] = out.z();
    rv[3] = out.w();
    return rv;
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to transform vector quaternion %s to %s: %s",
		     source_frame.c_str(), target_frame.c_str(), e.what_no_backtrace());
    return CLIPS::Values(1, CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
  }
}


CLIPS::Values
ClipsTFThread::clips_tf_transform_pose(std::string target_frame, std::string source_frame,
				       CLIPS::Values time,
				       CLIPS::Values translation, CLIPS::Values rotation_quat)
{
  if (! (validate_time(time) && validate_vector3(translation) && validate_quat(rotation_quat)))
  {
    return CLIPS::Values(1, CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
  }

  fawkes::Time t(convert_time(time));
  tf::Stamped<tf::Pose>
    in(tf::Pose(tf::Quaternion(rotation_quat[0].as_float(), rotation_quat[1].as_float(),
			       rotation_quat[2].as_float(), rotation_quat[3].as_float()),
		tf::Vector3(translation[0].as_float(), translation[1].as_float(),
			    translation[2].as_float())),
       t, source_frame);
  tf::Stamped<tf::Pose> out;

  try {
    tf_listener->transform_pose(target_frame, in, out);

    tf::Quaternion in_q  = in.getRotation();
    tf::Quaternion out_q = out.getRotation();
    if (debug_)
      logger->log_debug(name(), "Transformed pose %s->%s: "
		      "T(%.2f,%.2f,%.2f) R(%.2f,%.2f,%.2f,%.2f) -> "
		      "T(%.2f,%.2f,%.2f) R(%.2f,%.2f,%.2f,%.2f)",
		      source_frame.c_str(), target_frame.c_str(),
		      in.getOrigin().x(), in.getOrigin().y(), in.getOrigin().z(),
		      in_q.x(), in_q.y(), in_q.z(), in_q.w(),
		      out.getOrigin().x(), out.getOrigin().y(), out.getOrigin().z(),
		      out_q.x(), out_q.y(), out_q.z(), out_q.w());

    CLIPS::Values rv(7, CLIPS::Value(0.));
    rv[0] = out.getOrigin().x();
    rv[1] = out.getOrigin().y();
    rv[2] = out.getOrigin().z();
    rv[3] = out_q.x();
    rv[4] = out_q.y();
    rv[5] = out_q.z();
    rv[6] = out_q.w();
    return rv;
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to transform pose from %s to %s: %s",
		     source_frame.c_str(), target_frame.c_str(), e.what_no_backtrace());
    return CLIPS::Values(1, CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
  }
}


CLIPS::Values
ClipsTFThread::clips_tf_quat_from_yaw(double yaw)
{
  tf::Quaternion q = tf::create_quaternion_from_yaw(yaw);
  CLIPS::Values rv(4, CLIPS::Value(0.));
  rv[0] = q.x();
  rv[1] = q.y();
  rv[2] = q.z();
  rv[3] = q.w();
  return rv;  
}


double
ClipsTFThread::clips_tf_yaw_from_quat(CLIPS::Values quat)
{
  tf::Quaternion q(quat[0].as_float(), quat[1].as_float(),
		   quat[2].as_float(), quat[3].as_float());
  return tf::get_yaw(q);
}


void
ClipsTFThread::loop()
{
}


bool
ClipsTFThread::validate_time(const CLIPS::Values &time)
{
  if (time.size() != 2) {
    logger->log_warn(name(), "Invalid time: must be list of exactly two entries");
    return false;
  }
  for (auto &t : time) {
    CLIPS::Type t_type = t.type();
    if (t_type != CLIPS::TYPE_INTEGER) {
      logger->log_warn(name(), "Invalid time: must be list of integers");
      return false;
    }
  }
  return true;
}

fawkes::Time
ClipsTFThread::convert_time(const CLIPS::Values &time)
{
  if (! validate_time(time)) return fawkes::Time(0,0);

  return fawkes::Time(time[0].as_integer(), time[1].as_integer());
}


bool
ClipsTFThread::validate_point(const CLIPS::Values &point)
{
  if (point.size() != 3) {
    logger->log_warn(name(), "Invalid point: must be list of exactly three entries");
    return false;
  }
  for (auto &c : point) {
    CLIPS::Type c_type = c.type();
    if (c_type != CLIPS::TYPE_FLOAT && c_type != CLIPS::TYPE_INTEGER) {
      logger->log_warn(name(), "Invalid point: must be list of floats or integers");
      return false;
    }
  }
  return true;
}

bool
ClipsTFThread::validate_vector3(const CLIPS::Values &vector3)
{
  if (vector3.size() != 3) {
    logger->log_warn(name(), "Invalid vector: must be list of exactly three entries");
    return false;
  }
  for (auto &c : vector3) {
    CLIPS::Type c_type = c.type();
    if (c_type != CLIPS::TYPE_FLOAT && c_type != CLIPS::TYPE_INTEGER) {
      logger->log_warn(name(), "Invalid vector: must be list of floats or integers");
      return false;
    }
  }
  return true;
}


bool
ClipsTFThread::validate_quat(const CLIPS::Values &quat)
{
  if (quat.size() != 4) {
    logger->log_warn(name(), "Invalid quaternion: must be list of exactly four entries");
    return false;
  }
  for (auto &c : quat) {
    CLIPS::Type c_type = c.type();
    if (c_type != CLIPS::TYPE_FLOAT && c_type != CLIPS::TYPE_INTEGER) {
      logger->log_warn(name(), "Invalid quaternion: must be list of floats or integers");
      return false;
    }
  }
  return true;
}
