
/***************************************************************************
 *  ir_pcl_thread.cpp - Robotino IR point cloud thread
 *
 *  Created: Mon Mar 26 14:06:29 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#include "ir_pcl_thread.h"

#include <interfaces/RobotinoSensorInterface.h>
#include <limits>

using namespace fawkes;

/** @class RobotinoIrPclThread "ir_pcl_thread.h"
 * Robotino IR distances as point cloud.
 * This thread integrates into the Fawkes main loop at the SENSOR_PROCESS
 * hook and converts sensor data to a pointcloud
 * @author Tim Niemueller
 */

/** Constructor. */
RobotinoIrPclThread::RobotinoIrPclThread()
  : Thread("RobotinoIrPclThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE)
{
}


void
RobotinoIrPclThread::init()
{
  sens_if_ = blackboard->open_for_reading<RobotinoSensorInterface>("Robotino");

  sens_if_->read();

  pcl_xyz_ = new pcl::PointCloud<pcl::PointXYZ>();
  pcl_xyz_->is_dense = false;
  pcl_xyz_->width    = sens_if_->maxlenof_distance();
  pcl_xyz_->height   = 1;
  pcl_xyz_->points.resize(pcl_xyz_->width * pcl_xyz_->height);
  pcl_xyz_->header.frame_id = config->get_string("/hardware/robotino/base_frame");

  pcl_manager->add_pointcloud("robotino-ir", pcl_xyz_);

  float angle_offset = (2 * M_PI) / pcl_xyz_->width;
  angle_sines_   = new float[pcl_xyz_->width];
  angle_cosines_ = new float[pcl_xyz_->width];
  for (unsigned int i = 0; i < pcl_xyz_->width; ++i) {
    angle_sines_[i]   = sinf(angle_offset * i);
    angle_cosines_[i] = cosf(angle_offset * i);
  }
}


void
RobotinoIrPclThread::finalize()
{
  pcl_manager->remove_pointcloud("robotino-ir");
  blackboard->close(sens_if_);

  delete angle_sines_;
  delete angle_cosines_;
}

void
RobotinoIrPclThread::loop()
{
  // update sensor values in interface
  sens_if_->read();

  if (sens_if_->changed()) {
    const Time *ct = sens_if_->timestamp();
    const float *distances = sens_if_->distance();

    pcl::PointCloud<pcl::PointXYZ> &pcl = **pcl_xyz_;
    pcl.header.seq += 1;

    pcl_utils::set_time(pcl_xyz_, *ct);

    for (unsigned int i = 0; i < pcl_xyz_->width; ++i) {
      if (distances[i] == 0.) {
        pcl.points[i].x = pcl.points[i].y = pcl.points[i].z =
          std::numeric_limits<float>::quiet_NaN();
      } else {
        pcl.points[i].x = (distances[i] + 0.2) * angle_cosines_[i];
        pcl.points[i].y = (distances[i] + 0.2) * angle_sines_[i];
        pcl.points[i].z = 0.025; // 2.5 cm above ground
      }
    }
  }
}
