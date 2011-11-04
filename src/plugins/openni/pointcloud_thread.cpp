
/***************************************************************************
 *  pointcloud_thread.cpp - OpenNI point cloud provider thread
 *
 *  Created: Fri Mar 25 23:49:11 2011
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

#include "pointcloud_thread.h"
#include "utils/setup.h"

#include <core/threading/mutex_locker.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/base/types.h>

#include <memory>

using namespace fawkes;
using namespace firevision;

/** @class OpenNiPointCloudThread "pointcloud_thread.h"
 * OpenNI Point Cloud Provider Thread.
 * This thread provides a point cloud calculated from the depth image
 * acquired via OpenNI and provides it as a
 * SharedMemoryImageBuffer to other FireVision plugins.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
OpenNiPointCloudThread::OpenNiPointCloudThread()
  : Thread("OpenNiPointCloudThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}


/** Destructor. */
OpenNiPointCloudThread::~OpenNiPointCloudThread()
{
}


void
OpenNiPointCloudThread::init()
{
  MutexLocker lock(openni.objmutex_ptr());

  __depth_gen = new xn::DepthGenerator();
  std::auto_ptr<xn::DepthGenerator> depthgen_autoptr(__depth_gen);

  XnStatus st;

  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_DEPTH, __depth_gen);
  fawkes::openni::setup_map_generator(*__depth_gen, config);

  __depth_md = new xn::DepthMetaData();
  __depth_gen->GetMetaData(*__depth_md);

  __pcl_buf = new SharedMemoryImageBuffer("openni-pointcloud",
					  CARTESIAN_3D_FLOAT,
					  __depth_md->XRes(), __depth_md->YRes());

  // this is magic from ROS openni_device.cpp, reading code from
  // openni-primesense suggests that SXGA is the base configuration
  XnUInt64 zpd; // zero plane distance
  if ((st = __depth_gen->GetIntProperty("ZPD", zpd)) != XN_STATUS_OK) {
    throw Exception("Failed to get ZPD: %s", xnGetStatusString(st));
  }
  XnDouble pixel_size; // zero plane pixel size
  if ((st = __depth_gen->GetRealProperty("ZPPS", pixel_size)) != XN_STATUS_OK) {
    throw Exception("Failed to get ZPPS: %s", xnGetStatusString(st));
  }

  if ((st = __depth_gen->GetIntProperty("NoSampleValue", __no_sample_value))
      != XN_STATUS_OK)
  {
    throw Exception("Failed to get NoSampleValue: %s", xnGetStatusString(st));
  }
  if ((st = __depth_gen->GetIntProperty("ShadowValue", __shadow_value))
      != XN_STATUS_OK)
  {
    throw Exception("Failed to get ShadowValue: %s", xnGetStatusString(st));
  }


  __width  = __depth_md->XRes();
  __height = __depth_md->YRes();
  float scale = __width / (float)XN_SXGA_X_RES;
  __focal_length = ((float)zpd / pixel_size) * scale;
  __center_x = (__width  / 2.) - .5f;
  __center_y = (__height / 2.) - .5f;

  __depth_gen->StartGenerating();

  depthgen_autoptr.release();
}


void
OpenNiPointCloudThread::finalize()
{
  // we do not stop generating, we don't know if there is no other plugin
  // using the node.
  delete __depth_gen;
  delete __depth_md;
  delete __pcl_buf;
}


void
OpenNiPointCloudThread::loop()
{
  MutexLocker lock(openni.objmutex_ptr());
  bool is_data_new = __depth_gen->IsDataNew();
  const XnDepthPixel * const data = __depth_md->Data();
  // experimental: unlock here as we do not invoke any methods anymore
  // since data has been updated earlier in the sensor hook we should be safe
  lock.unlock();

  if (is_data_new && (__pcl_buf->num_attached() > 1)) {
    // convert depth to points
    register pcl_point_t *pcl = (pcl_point_t *)__pcl_buf->buffer();

    const float constant = 0.001 / __focal_length;

    unsigned int depth_idx = 0;
    for (unsigned int h = 0; h < __height; ++h) {
      for (unsigned int w = 0; w < __width; ++w, ++depth_idx, ++pcl) {
	// Check for invalid measurements
	if (data[depth_idx] == 0 ||
	    data[depth_idx] == __no_sample_value ||
	    data[depth_idx] == __shadow_value)
	{
	  // invalid
          pcl->x = pcl->y = pcl->z = 0.f;
	} else {
	  // Fill in XYZ
	  pcl->x = data[depth_idx] * 0.001f;
	  pcl->y = -(w - __center_x) * data[depth_idx] * constant;
	  pcl->z = -(h - __center_y) * data[depth_idx] * constant;
	}
      }
    }
  }
}
