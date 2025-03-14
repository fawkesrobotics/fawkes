
/***************************************************************************
 *  pos3d_publisher_thread.h - A publisher for 3D robot positions
 *
 *  Created: Sat Apr 06 16:20:00 2024
 *  Copyright  2024  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

#include "pos3d_publisher_thread.h"

#include <tf/time_cache.h>

/** @class Pos3dPublisherThread "pos3d_publisher_thread.h"
 * Main thread of tf example plugin.
 */

using namespace fawkes;

/** Constructor. */
Pos3dPublisherThread::Pos3dPublisherThread()
: Thread("Pos3dPublisherThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK),
  TransformAspect(TransformAspect::BOTH_DEFER_PUBLISHER),
  BlackBoardInterfaceListener("POS3DThread")
{
}

/** Destructor. */
Pos3dPublisherThread::~Pos3dPublisherThread()
{
}

void
Pos3dPublisherThread::init()
{
	global_frame_id_       = config->get_string("/frames/fixed");
	cfg_pose_ifname_       = config->get_string("/plugins/amcl/pose_interface_id");
	cfg_pose_ifname_agent_ = config->get_string("/plugins/amcl/pose_interface_id_agent");
	loop_threshold_        = config->get_int("/plugins/amcl/agent_scale_factor");
	blackboard->register_observer(this);

	pos3d_if_ = blackboard->open_for_writing<Position3DInterface>(cfg_pose_ifname_.c_str());
	pos3d_if_->set_frame(global_frame_id_.c_str());
	pos3d_if_->write();
	pos3d_if_agent_ =
	  blackboard->open_for_writing<Position3DInterface>(cfg_pose_ifname_agent_.c_str());
	pos3d_if_agent_->set_frame(global_frame_id_.c_str());
	pos3d_if_agent_->write();
}

void
Pos3dPublisherThread::finalize()
{
	blackboard->unregister_listener(this);
	blackboard->unregister_observer(this);
	blackboard->close(pos3d_if_);
	blackboard->close(pos3d_if_agent_);
}

#define TARGET "map"
#define SOURCE "base_link"

void
Pos3dPublisherThread::loop()
{
	bool world_frame_exists = tf_listener->frame_exists(TARGET);
	bool robot_frame_exists = tf_listener->frame_exists(SOURCE);

	loop_nr_++;

	if (!world_frame_exists || !robot_frame_exists) {
		logger->log_warn(name(),
		                 "Frame missing: %s %s   %s %s",
		                 SOURCE,
		                 world_frame_exists ? "exists" : "missing",
		                 TARGET,
		                 robot_frame_exists ? "exists" : "missing");
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

		tf::Quaternion q = transform.getRotation();
		tf::Vector3    v = transform.getOrigin();

		if (pos3d_if_->visibility_history() >= 0) {
			pos3d_if_->set_visibility_history(pos3d_if_->visibility_history() + 1);
		} else {
			pos3d_if_->set_visibility_history(1);
		}
		pos3d_if_->set_translation(transform.getOrigin());
		pos3d_if_->set_rotation(transform.getRotation());
		pos3d_if_->write();

		if (loop_nr_ > loop_threshold_) {
			if (pos3d_if_agent_->visibility_history() >= 0) {
				pos3d_if_agent_->set_visibility_history(pos3d_if_agent_->visibility_history() + 1);
			} else {
				pos3d_if_agent_->set_visibility_history(1);
			}
			pos3d_if_agent_->set_visibility_history(pos3d_if_agent_->visibility_history() + 1);
			pos3d_if_agent_->set_translation(transform.getOrigin());
			pos3d_if_agent_->set_rotation(transform.getRotation());
			pos3d_if_agent_->write();
		}
	}

	if (loop_nr_ > loop_threshold_) {
		loop_nr_ = 0;
	}
}
