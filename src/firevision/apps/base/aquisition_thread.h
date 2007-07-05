
/***************************************************************************
 *  aquisition_thread.h - FireVision Aquisition Thread
 *
 *  Created: Wed Jun 06 19:01:10 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_APPS_BASE_AQUISITION_THREAD_H_
#define __FIREVISION_APPS_BASE_AQUISITION_THREAD_H_

#include <core/threading/thread.h>
#include <core/threading/thread_notification_listener.h>

#include <fvutils/color/colorspaces.h>

class Barrier;
class WaitCondition;
class Mutex;
class ThreadList;
class SharedMemoryImageBuffer;
class Camera;
class FvBaseThread;
class Logger;

class FvAquisitionThread
: public Thread, public ThreadNotificationListener
{
 public:
  FvAquisitionThread(FvBaseThread *base_thread, Logger *logger,
		     const char *id, Camera *camera,
		     unsigned int timeout);
  virtual ~FvAquisitionThread();

  virtual void loop();

  bool empty();

  // from ThreadNotificationListener
  virtual void thread_started(Thread *thread);
  virtual void thread_init_failed(Thread *thread);

  // from VisionMaster
  virtual Camera *  camera_instance(bool raw);
  virtual bool      has_thread(Thread *thread);
  virtual void      add_thread(Thread *thread, bool raw);
  virtual void      remove_thread(Thread *thread);

 private:
  Camera                   *_camera;
  SharedMemoryImageBuffer  *_shm;
  SharedMemoryImageBuffer  *_shm_raw;
  char                     *_image_id;
  char                     *_image_id_raw;
  ThreadList               *_running_tl;
  Barrier                  *_running_tl_barrier;
  ThreadList               *_running_raw_tl;
  Barrier                  *_running_raw_tl_barrier;
  ThreadList               *_waiting_tl;
  ThreadList               *_waiting_raw_tl;
  WaitCondition            *_wait_for_threads_cond;
  Mutex                    *_wait_for_threads_mutex;
  unsigned int              _timeout;
  FvBaseThread             *_base_thread;
  Logger                   *_logger;

  colorspace_t              _colorspace;
  unsigned int              _width;
  unsigned int              _height;
  unsigned char            *_buffer;
  unsigned char            *_buffer_raw;
};


#endif
