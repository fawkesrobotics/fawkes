
/***************************************************************************
 *  acquisition_thread.h - FireVision Acquisition Thread
 *
 *  Created: Wed Jun 06 19:01:10 2007
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

#ifndef __FIREVISION_APPS_BASE_ACQUISITION_THREAD_H_
#define __FIREVISION_APPS_BASE_ACQUISITION_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/logging.h>
#include <aspect/blackboard.h>

#include <fvcams/shmem.h>
#include <fvutils/color/colorspaces.h>
#include <blackboard/interface_listener.h>

#include <map>

namespace fawkes {
  class Logger;
  class Clock;
  class Mutex;
  class WaitCondition;
  class SwitchInterface;
#ifdef FVBASE_TIMETRACKER
  class TimeTracker;
#endif
}
namespace firevision {
  class SharedMemoryImageBuffer;
}
class FvBaseThread;
class FvAqtVisionThreads;

class FvAcquisitionThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  /** Acquisition thread mode. */
  typedef enum {
    AqtCyclic,		/**< cyclic mode, use if there is at least one cyclic thread
			 * for this acquisition thread. */
    AqtContinuous	/**< continuous mode, use if there are only continuous threads
			 * for this acquisition thread. */
  } AqtMode;

  FvAcquisitionThread(const char *id, firevision::Camera *camera,
		      fawkes::Logger *logger, fawkes::Clock *clock);
  virtual ~FvAcquisitionThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  void set_aqtmode(AqtMode mode);
  AqtMode aqtmode();
  firevision::Camera *  camera_instance(firevision::colorspace_t cspace, bool deep_copy);

  firevision::Camera *get_camera();

  void set_vt_prepfin_hold(bool hold);
  void set_enabled(bool enabled);

 public:
  /** Vision threads assigned to this acquisition thread. To be used only by the
   * base thread. */
  FvAqtVisionThreads       *vision_threads;

  /** Vision thread registered for raw camera access on this camera. */
  fawkes::Thread           *raw_subscriber_thread;

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  virtual bool bb_interface_message_received(fawkes::Interface *interface,
                                             fawkes::Message *message) throw();

 private:
  bool                      __enabled;
  fawkes::Mutex            *__enabled_mutex;
  fawkes::WaitCondition    *__enabled_waitcond;

  firevision::Camera       *__camera;
  char                     *__image_id;

  firevision::colorspace_t  __colorspace;
  unsigned int              __width;
  unsigned int              __height;

  AqtMode                   __mode;

  std::map<firevision::colorspace_t, firevision::SharedMemoryImageBuffer *> __shm;
  std::map<firevision::colorspace_t, firevision::SharedMemoryImageBuffer *>::iterator __shmit;

  fawkes::SwitchInterface  *__enabled_if;

#ifdef FVBASE_TIMETRACKER
  fawkes::TimeTracker *__tt;
  unsigned int __loop_count;
  unsigned int __ttc_capture;
  unsigned int __ttc_lock;
  unsigned int __ttc_convert;
  unsigned int __ttc_unlock;
  unsigned int __ttc_dispose;
#endif
};

#endif
