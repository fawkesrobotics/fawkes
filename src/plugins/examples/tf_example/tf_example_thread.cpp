
/***************************************************************************
 *  tf_example_thread.cpp - tf example thread
 *
 *  Created: Tue Oct 25 18:01:36 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include "tf_example_thread.h"

#include <tf/time_cache.h>

/** @class TfExampleThread "tf_example_thread.h"
 * Main thread of tf example plugin.
 * @author Tim Niemueller
 */

using namespace fawkes;

/** Constructor. */
TfExampleThread::TfExampleThread()
  : Thread("TfExampleThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK),
    TransformAspect(TransformAspect::BOTH, "test_frame")
{
}


/** Destructor. */
TfExampleThread::~TfExampleThread()
{
}


void
TfExampleThread::init()
{
	angle_ = 0.;
}


void
TfExampleThread::finalize()
{
}


#define SOURCE "rx28/tilt"
#define TARGET "base_link"

void
TfExampleThread::loop()
{
  bool world_frame_exists = tf_listener->frame_exists(SOURCE);
  bool robot_frame_exists = tf_listener->frame_exists(TARGET);

  if (! world_frame_exists || ! robot_frame_exists) {
    logger->log_warn(name(), "Frame missing: %s %s   %s %s",
                     SOURCE, world_frame_exists ? "exists" : "missing",
                     TARGET, robot_frame_exists ? "exists" : "missing");
  } else {
    tf::StampedTransform transform;
    try {
      tf_listener->lookup_transform(TARGET, SOURCE, transform);
    } catch (tf::ExtrapolationException &e) {
      logger->log_debug(name(), "Extrapolation error");
      return;
    } catch (tf::ConnectivityException &e) {
      logger->log_debug(name(), "Connectivity exception: %s", e.what());
      return;	
    }

    fawkes::Time now;
    double diff;
    if (now >= transform.stamp) {
      diff = now - &transform.stamp;
    } else {
      diff = transform.stamp - &now;
    }

    tf::Quaternion q = transform.getRotation();
    tf::Vector3 v   = transform.getOrigin();

    const tf::TimeCacheInterfacePtr world_cache = tf_listener->get_frame_cache(SOURCE);
    const tf::TimeCacheInterfacePtr robot_cache = tf_listener->get_frame_cache(TARGET);

    logger->log_info(name(), "Transform %s -> %s, %f sec old: "
                     "T(%f,%f,%f)  Q(%f,%f,%f,%f)",
                     transform.frame_id.c_str(), transform.child_frame_id.c_str(),
                     diff, v.x(), v.y(), v.z(), q.x(), q.y(), q.z(), q.w());

    logger->log_info(name(), "World cache size: %zu  Robot cache size: %zu",
                     world_cache->get_list_length(),
                     robot_cache->get_list_length());
  }

  angle_ += M_PI / 4.;
  if (angle_ >= 2*M_PI) angle_ = 0.;
  fawkes::Time now;

  tf::Transform t(tf::create_quaternion_from_yaw(angle_));
  tf::StampedTransform st(t, now, "base_link", "test_frame");
  tf_publisher->send_transform(st);
}
