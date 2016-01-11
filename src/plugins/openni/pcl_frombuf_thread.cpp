
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

#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/base/types.h>
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
  __pcl_buf = new SharedMemoryImageBuffer("openni-pointcloud");


  __width  = __pcl_buf->width();
  __height = __pcl_buf->height();

  __pcl = new pcl::PointCloud<pcl::PointXYZ>();
  __pcl->is_dense = false;
  __pcl->width    = __width;
  __pcl->height   = __height;
  __pcl->points.resize(__width * __height);
  __pcl->header.frame_id = config->get_string("/plugins/openni/frame/depth");

  pcl_manager->add_pointcloud("openni-pointcloud", __pcl);
}


void
OpenNiPclOnlyThread::finalize()
{
  pcl_manager->remove_pointcloud("openni-pointcloud");

  delete __pcl_buf;
}


void
OpenNiPclOnlyThread::loop()
{
  if ((__pcl_buf->num_attached() > 1) || (__pcl.use_count() > 1))
  {
    __pcl_buf->lock_for_read();
    fawkes::Time capture_time = __pcl_buf->capture_time();

    if (__last_capture_time != capture_time) {
      __last_capture_time = capture_time;

      pcl_point_t *pclbuf = (pcl_point_t *)__pcl_buf->buffer();

      pcl::PointCloud<pcl::PointXYZ> &pcl = **__pcl;
      pcl.header.seq += 1;
      pcl_utils::set_time(__pcl, capture_time);

      unsigned int idx = 0;
      for (unsigned int h = 0; h < __height; ++h) {
        for (unsigned int w = 0; w < __width; ++w, ++idx, ++pclbuf) {
          // Fill in XYZ
          pcl.points[idx].x = pclbuf->x;
          pcl.points[idx].y = pclbuf->y;
          pcl.points[idx].z = pclbuf->z;
        }
      }
    }

    __pcl_buf->unlock();
  }
}
