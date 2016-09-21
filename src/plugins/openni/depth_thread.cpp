
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

  __depth_gen = new xn::DepthGenerator();
#if __cplusplus >= 201103L
  std::unique_ptr<xn::DepthGenerator> depthgen_uniqueptr(__depth_gen);
#else
  std::auto_ptr<xn::DepthGenerator> depthgen_uniqueptr(__depth_gen);
#endif

  XnStatus st;

  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_DEPTH, __depth_gen);
  fawkes::openni::setup_map_generator(*__depth_gen, config);

  __depth_md = new xn::DepthMetaData();

  __depth_gen->GetMetaData(*__depth_md);

  __depth_width  = __depth_md->XRes();
  __depth_height = __depth_md->YRes();

  __depth_buf = new SharedMemoryImageBuffer("openni-depth", RAW16,
  					    __depth_md->XRes(), __depth_md->YRes());
  __depth_bufsize = colorspace_buffer_size(RAW16,
					   __depth_md->XRes(), __depth_md->YRes());

  __depth_gen->StartGenerating();

  __capture_start = new Time(clock);
  __capture_start->stamp_systime();
  // Update once to get timestamp
  __depth_gen->WaitAndUpdateData();
  // arbitrarily define the zero reference point,
  // we can't get any closer than this
  *__capture_start -= (long int)__depth_gen->GetTimestamp();
  
  
  depthgen_uniqueptr.release();
}


void
OpenNiDepthThread::finalize()
{
  // we do not stop generating, we don't know if there is no other plugin
  // using the node.
  delete __depth_gen;
  delete __depth_md;
  delete __depth_buf;
}


void
OpenNiDepthThread::loop()
{
  MutexLocker lock(openni.objmutex_ptr());
  bool is_depth_new = __depth_gen->IsDataNew();
  __depth_gen->GetMetaData(*__depth_md);
  const XnDepthPixel * const depth_data = __depth_md->Data();
  fawkes::Time ts = *__capture_start + (long int)__depth_gen->GetTimestamp();
  lock.unlock();

  if (is_depth_new && (__depth_buf->num_attached() > 1)) {
    memcpy(__depth_buf->buffer(), depth_data, __depth_bufsize);
  }

  __depth_buf->set_capture_time(&ts);
}
