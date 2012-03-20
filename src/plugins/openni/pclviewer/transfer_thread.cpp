
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
  __rwlock = new ReadWriteLock();
}


/** Destructor. */
PclViewerTransferThread::~PclViewerTransferThread()
{
  delete __rwlock;
  std::map<std::string, unsigned char *>::iterator c;
  for (c = __buffers.begin(); c != __buffers.end(); ++c) {
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
  __rwlock->lock_for_read();
}


/** Unlock. */
void
PclViewerTransferThread::unlock()
{
  __rwlock->unlock();
}


/** Add a camera from which to pull images.
 * @param name symbolic name, used to access buffers
 * @param cam camera to add
 */
void
PclViewerTransferThread::add_camera(std::string name, firevision::Camera *cam)
{
  __cams[name] = cam;
  __buffers[name] = malloc_buffer(cam->colorspace(), cam->pixel_width(),
				  cam->pixel_height());
  __buffer_sizes[name] = colorspace_buffer_size(cam->colorspace(),
						cam->pixel_width(),
						cam->pixel_height());
}


void
PclViewerTransferThread::loop()
{
  std::map<std::string, firevision::Camera *>::iterator c;
  for (c = __cams.begin(); c != __cams.end(); ++c) {
    c->second->capture();
    __rwlock->lock_for_write();
    memcpy(__buffers[c->first], c->second->buffer(), __buffer_sizes[c->first]);
    __rwlock->unlock();
    c->second->dispose_buffer();
  }
}
