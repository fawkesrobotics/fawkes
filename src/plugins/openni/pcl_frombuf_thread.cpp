
/***************************************************************************
 *  pcl_frombuf_thread.cpp - Create PCL point cloud from buffer
 *
 *  Created: Fri Dec 02 19:56:06 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "pcl_frombuf_thread.h"

#include <fvutils/base/types.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/ipc/shm_image.h>
#include <pcl_utils/utils.h>

#include <memory>

using namespace fawkes;
using namespace firevision;

/** @class OpenNiPclOnlyThread "pointcloud_thread.h"
 * OpenNI Point Cloud Provider Thread.
 * This thread provides a point cloud calculated from the depth image
 * acquired via OpenNI and provides it as a
 * SharedMemoryImageBuffer to other FireVision plugins.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
OpenNiPclOnlyThread::OpenNiPclOnlyThread()
: Thread("OpenNiPclOnlyThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE)
{
}

/** Destructor. */
OpenNiPclOnlyThread::~OpenNiPclOnlyThread()
{
}

void
OpenNiPclOnlyThread::init()
{
	pcl_buf_ = new SharedMemoryImageBuffer("openni-pointcloud");

	width_  = pcl_buf_->width();
	height_ = pcl_buf_->height();

	pcl_           = new pcl::PointCloud<pcl::PointXYZ>();
	pcl_->is_dense = false;
	pcl_->width    = width_;
	pcl_->height   = height_;
	pcl_->points.resize((size_t)width_ * (size_t)height_);
	pcl_->header.frame_id = config->get_string("/plugins/openni/frame/depth");

	pcl_manager->add_pointcloud("openni-pointcloud", pcl_);
}

void
OpenNiPclOnlyThread::finalize()
{
	pcl_manager->remove_pointcloud("openni-pointcloud");

	delete pcl_buf_;
}

void
OpenNiPclOnlyThread::loop()
{
	if ((pcl_buf_->num_attached() > 1) || (pcl_.use_count() > 1)) {
		pcl_buf_->lock_for_read();
		fawkes::Time capture_time = pcl_buf_->capture_time();

		if (last_capture_time_ != capture_time) {
			last_capture_time_ = capture_time;

			pcl_point_t *pclbuf = (pcl_point_t *)pcl_buf_->buffer();

			pcl::PointCloud<pcl::PointXYZ> &pcl = **pcl_;
			pcl.header.seq += 1;
			pcl_utils::set_time(pcl_, capture_time);

			unsigned int idx = 0;
			for (unsigned int h = 0; h < height_; ++h) {
				for (unsigned int w = 0; w < width_; ++w, ++idx, ++pclbuf) {
					// Fill in XYZ
					pcl.points[idx].x = pclbuf->x;
					pcl.points[idx].y = pclbuf->y;
					pcl.points[idx].z = pclbuf->z;
				}
			}
		}

		pcl_buf_->unlock();
	}
}
