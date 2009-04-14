
/***************************************************************************
 *  acquisition_thread.h - FireVision Acquisition Thread
 *
 *  Created: Wed Jun 06 19:01:10 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <cams/shmem.h>
#include <fvutils/color/colorspaces.h>

#include <map>

class SharedMemoryImageBuffer;
class FvBaseThread;
class FvAqtVisionThreads;
namespace fawkes {
  class Logger;
  class Clock;
#ifdef FVBASE_TIMETRACKER
  class TimeTracker;
#endif
}

class FvAcquisitionThread
: public fawkes::Thread
{
 public:
  /** Acquisition thread mode. */
  typedef enum {
    AqtCyclic,		/**< cyclic mode, use if there is at least one cyclic thread
			 * for this acquisition thread. */
    AqtContinuous	/**< continuous mode, use if there are only continuous threads
			 * for this acquisition thread. */
  } AqtMode;

  FvAcquisitionThread(const char *id, Camera *camera,
		      fawkes::Logger *logger, fawkes::Clock *clock);
  virtual ~FvAcquisitionThread();

  virtual void loop();

  void set_aqtmode(AqtMode mode);
  AqtMode aqtmode();
  SharedMemoryCamera *  camera_instance(colorspace_t cspace, bool deep_copy);

  void set_vt_prepfin_hold(bool hold);
  void set_enabled(bool enabled);

 public:
  /** Vision threads assigned to this acquisition thread. To be used only by the
   * base thread. */
  FvAqtVisionThreads       *vision_threads;

 private:
  bool                      __enabled;

  Camera                   *__camera;

  char                     *__image_id;

  fawkes::Logger           *__logger;

  colorspace_t              __colorspace;
  unsigned int              __width;
  unsigned int              __height;

  AqtMode                   __mode;

  std::map<colorspace_t, SharedMemoryImageBuffer *> __shm;
  std::map<colorspace_t, SharedMemoryImageBuffer *>::iterator __shmit;

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
