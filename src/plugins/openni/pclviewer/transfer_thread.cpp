
/***************************************************************************
 *  transfer_thread.cpp - OpenNI Visualization: network transfer thread
 *
 *  Created: Sat Apr 02 20:19:21 2011
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

#include "transfer_thread.h"
#include <core/threading/read_write_lock.h>
#include <fvcams/camera.h>
#include <fvutils/color/colorspaces.h>

#include <cstdlib>
#include <cstring>

using namespace fawkes;
using namespace firevision;

/** @class PclViewerTransferThread "transfer_thread.h"
 * PCL viewer transfer thread.
 * Especially for transmission over slow Wifi networks tranmission
 * must be pushed to its own thread. It captures the frame and copies
 * it to an internal buffer to anytime use.
 * @author Tim Niemueller
 */

/** Constructor. */
PclViewerTransferThread::PclViewerTransferThread()
  : Thread("PclViewerTransferThread", Thread::OPMODE_CONTINUOUS)
{
  rwlock_ = new ReadWriteLock();
}


/** Destructor. */
PclViewerTransferThread::~PclViewerTransferThread()
{
  delete rwlock_;
  std::map<std::string, unsigned char *>::iterator c;
  for (c = buffers_.begin(); c != buffers_.end(); ++c) {
    free(c->second);
  }
}


/** Lock for reading.
 * Images will not be updated while the lock is held. Any number of
 * readers can hold a read lock at the same time. Make sure that the
 * thread does not starve.
 */
void
PclViewerTransferThread::lock_for_read()
{
  rwlock_->lock_for_read();
}


/** Unlock. */
void
PclViewerTransferThread::unlock()
{
  rwlock_->unlock();
}


/** Add a camera from which to pull images.
 * @param name symbolic name, used to access buffers
 * @param cam camera to add
 */
void
PclViewerTransferThread::add_camera(std::string name, firevision::Camera *cam)
{
  cams_[name] = cam;
  buffers_[name] = malloc_buffer(cam->colorspace(), cam->pixel_width(),
				  cam->pixel_height());
  buffer_sizes_[name] = colorspace_buffer_size(cam->colorspace(),
						cam->pixel_width(),
						cam->pixel_height());
}


void
PclViewerTransferThread::loop()
{
  std::map<std::string, firevision::Camera *>::iterator c;
  for (c = cams_.begin(); c != cams_.end(); ++c) {
    c->second->capture();
    rwlock_->lock_for_write();
    memcpy(buffers_[c->first], c->second->buffer(), buffer_sizes_[c->first]);
    rwlock_->unlock();
    c->second->dispose_buffer();
  }
}
