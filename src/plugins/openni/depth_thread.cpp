
/***************************************************************************
 *  depth_thread.cpp - OpenNI depth provider thread
 *
 *  Created: Thu Dec 22 11:36:31 2011
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

#include "depth_thread.h"
#include "utils/setup.h"

#include <core/threading/mutex_locker.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/colorspaces.h>

#include <memory>

using namespace fawkes;
using namespace firevision;

/** @class OpenNiDepthThread "image_thread.h"
 * OpenNI Depth Provider Thread.
 * This thread provides RGB and depth images from the camera via a
 * SharedMemoryImageBuffer to other FireVision plugins.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
OpenNiDepthThread::OpenNiDepthThread()
  : Thread("OpenNiDepthThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE)
{
}


/** Destructor. */
OpenNiDepthThread::~OpenNiDepthThread()
{
}


void
OpenNiDepthThread::init()
{
  MutexLocker lock(openni.objmutex_ptr());

  depth_gen_ = new xn::DepthGenerator();
#if __cplusplus >= 201103L
  std::unique_ptr<xn::DepthGenerator> depthgen_uniqueptr(depth_gen_);
#else
  std::auto_ptr<xn::DepthGenerator> depthgen_uniqueptr(depth_gen_);
#endif

  XnStatus st;

  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_DEPTH, depth_gen_);
  fawkes::openni::setup_map_generator(*depth_gen_, config);

  depth_md_ = new xn::DepthMetaData();

  depth_gen_->GetMetaData(*depth_md_);

  depth_width_  = depth_md_->XRes();
  depth_height_ = depth_md_->YRes();

  depth_buf_ = new SharedMemoryImageBuffer("openni-depth", RAW16,
  					    depth_md_->XRes(), depth_md_->YRes());
  depth_bufsize_ = colorspace_buffer_size(RAW16,
					   depth_md_->XRes(), depth_md_->YRes());

  depth_gen_->StartGenerating();

  capture_start_ = new Time(clock);
  capture_start_->stamp_systime();
  // Update once to get timestamp
  depth_gen_->WaitAndUpdateData();
  // arbitrarily define the zero reference point,
  // we can't get any closer than this
  *capture_start_ -= (long int)depth_gen_->GetTimestamp();
  
  
  depthgen_uniqueptr.release();
}


void
OpenNiDepthThread::finalize()
{
  // we do not stop generating, we don't know if there is no other plugin
  // using the node.
  delete depth_gen_;
  delete depth_md_;
  delete depth_buf_;
  delete capture_start_;
}


void
OpenNiDepthThread::loop()
{
  MutexLocker lock(openni.objmutex_ptr());
  bool is_depth_new = depth_gen_->IsDataNew();
  depth_gen_->GetMetaData(*depth_md_);
  const XnDepthPixel * const depth_data = depth_md_->Data();
  fawkes::Time ts = *capture_start_ + (long int)depth_gen_->GetTimestamp();
  lock.unlock();

  if (is_depth_new && (depth_buf_->num_attached() > 1)) {
    memcpy(depth_buf_->buffer(), depth_data, depth_bufsize_);
  }

  depth_buf_->set_capture_time(&ts);
}
