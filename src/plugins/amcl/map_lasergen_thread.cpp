/***************************************************************************
 *  map_lasergen_thread.cpp - Thread to generate laser data from map
 *
 *  Created: Thu Aug 23 18:43:39 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include "map_lasergen_thread.h"
#include "amcl_utils.h"

#include <utils/math/angle.h>

using namespace fawkes;

/** @class MapLaserGenThread "map_lasergen_thread.h"
 * Generate laser data from map and position.
 * @author Tim Niemueller
 */

/** Constructor. */
MapLaserGenThread::MapLaserGenThread()
  : Thread("MapLaserGenThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
    TransformAspect(TransformAspect::BOTH, "Odometry")
{
  map_ = NULL;
}

/** Destructor. */
MapLaserGenThread::~MapLaserGenThread()
{
}

void MapLaserGenThread::init()
{
  fawkes::amcl::read_map_config(config, cfg_map_file_, cfg_resolution_, cfg_origin_x_,
				cfg_origin_y_, cfg_origin_theta_, cfg_occupied_thresh_,
				cfg_free_thresh_);
  cfg_laser_ifname_ = config->get_string(CFG_PREFIX"laser_interface_id");

  std::vector<std::pair<int, int> > free_space_indices;
  map_ = fawkes::amcl::read_map(cfg_map_file_.c_str(),
				cfg_origin_x_, cfg_origin_y_, cfg_resolution_,
				cfg_occupied_thresh_, cfg_free_thresh_, free_space_indices);

  map_width_  = map_->size_x;
  map_height_ = map_->size_y;

  logger->log_info(name(), "Size: %ux%u (%zu/%u free)", map_width_, map_height_,
		   free_space_indices.size(), map_width_ * map_height_);

  laser_if_ = blackboard->open_for_writing<Laser360Interface>(cfg_laser_ifname_.c_str());
  pos3d_if_ = blackboard->open_for_writing<Position3DInterface>("Map LaserGen Groundtruth");

  pos_x = pos_y = 2.3;

  laser_if_->set_frame("/base_laser");

}


void
MapLaserGenThread::loop()
{
  /*
  if (!laser_pose_set_) {
    if (set_laser_pose()) {
      laser_pose_set_ = true;
    } else {
      logger->log_warn(name(), "Could not determine laser pose, skipping loop");
      return;
    }
  }
  */

  float dists[360];
  for (unsigned int i = 0; i < 360; ++i) {
    dists[i] = map_calc_range(map_, pos_x, pos_y, deg2rad(i), 100.);
  }
  laser_if_->set_distances(dists);
  laser_if_->write();

  pos3d_if_->set_translation(0, pos_x);
  pos3d_if_->set_translation(1, pos_y);
  pos3d_if_->write();

  tf::Transform
    tmp_tf(tf::create_quaternion_from_yaw(0), tf::Vector3(0,0,0));

  Time transform_expiration =
    (Time(clock) + 1.0);

  tf::StampedTransform tmp_tf_stamped(tmp_tf,
				      transform_expiration,
				      "/robotino_odometry", "/base_link");

  tf_publisher->send_transform(tmp_tf_stamped);
}

void MapLaserGenThread::finalize()
{
  if (map_) {
    map_free(map_);
    map_ = NULL;
  }

  blackboard->close(laser_if_);
  blackboard->close(pos3d_if_);
}
