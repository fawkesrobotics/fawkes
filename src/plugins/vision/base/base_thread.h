
/***************************************************************************
 *  base_thread.h - FireVision Base Thread
 *
 *  Created: Tue May 29 16:40:10 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_APPS_BASE_BASE_THREAD_H_
#define __FIREVISION_APPS_BASE_BASE_THREAD_H_

#include <core/threading/thread.h>
#include <core/threading/thread_notification_listener.h>
#include <core/utils/lock_map.h>
#include <core/utils/lock_list.h>

#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/vision_master.h>
#include <aspect/clock.h>
#include <aspect/thread_producer.h>
#include <aspect/configurable.h>

#include <fvutils/base/vision_master.h>
#include <string>

namespace fawkes {
  class Barrier;
}
class FvAcquisitionThread;

class FvBaseThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::VisionMasterAspect,
  public fawkes::ClockAspect,
  public fawkes::ThreadProducerAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ThreadNotificationListener,
  public firevision::VisionMaster
{
 public:
  FvBaseThread();
  virtual ~FvBaseThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  virtual firevision::VisionMaster *  vision_master();

  virtual firevision::Camera *  register_for_camera(const char *camera_string,
						    fawkes::Thread *thread,
						    firevision::colorspace_t cspace = firevision::YUV422_PLANAR);
  virtual firevision::Camera *  register_for_raw_camera(const char *camera_string,
							fawkes::Thread *thread);
  virtual void      unregister_thread(fawkes::Thread *thread);


  virtual firevision::CameraControl *acquire_camctrl(const char *cam_string);
  virtual void                       release_camctrl(firevision::CameraControl *cc);

  virtual bool thread_started(fawkes::Thread *thread) throw();
  virtual bool thread_init_failed(fawkes::Thread *thread) throw();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 protected:
  virtual firevision::CameraControl *acquire_camctrl(const char *cam_string,
						     const std::type_info &typeinf);

 private:
  void cond_recreate_barrier(unsigned int num_cyclic_threads);
  firevision::CameraControl * create_camctrl(const char *camera_string);

 private:
  fawkes::LockMap<std::string, FvAcquisitionThread *> __aqts;
  fawkes::LockMap<std::string, FvAcquisitionThread *>::iterator __ait;
  unsigned int __aqt_timeout;

  fawkes::LockList<firevision::CameraControl *>  __owned_controls;
  fawkes::LockMap<Thread *, FvAcquisitionThread *> __started_threads;

  fawkes::Barrier *__aqt_barrier;
};


#endif
