
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

#include <cams/shmem.h>
#include <fvutils/color/colorspaces.h>

class SharedMemoryImageBuffer;
class FvBaseThread;
class FvAqtVisionThreads;
class Logger;
class Clock;
#ifdef FVBASE_TIMETRACKER
class TimeTracker;
#endif

class FvAquisitionThread
: public Thread
{
 public:
  friend class FvBaseThread;

  /** Aquisition thread mode. */
  typedef enum {
    AqtCyclic,		/**< cyclic mode, use if there is at least one cyclic thread
			 * for this aquisition thread. */
    AqtContinuous	/**< continuous mode, use if there are only continuous threads
			 * for this aquisition thread. */
  } AqtMode;

  FvAquisitionThread(const char *id, Camera *camera,
		     Logger *logger, Clock *clock);
  virtual ~FvAquisitionThread();

  virtual void loop();

  void set_aqtmode(AqtMode mode);
  AqtMode aqtmode();
  SharedMemoryCamera *  camera_instance(bool raw, bool deep_copy);

 private:
  Camera                   *_camera;
  SharedMemoryImageBuffer  *_shm;
  SharedMemoryImageBuffer  *_shm_raw;
  char                     *_image_id;
  char                     *_image_id_raw;

  Logger                   *_logger;

  colorspace_t              _colorspace;
  unsigned int              _width;
  unsigned int              _height;
  unsigned char            *_buffer;
  unsigned char            *_buffer_raw;

  AqtMode                   _mode;
  FvAqtVisionThreads       *_vision_threads;

#ifdef FVBASE_TIMETRACKER
  TimeTracker *__tt;
  unsigned int __loop_count;
  unsigned int __ttc_capture;
  unsigned int __ttc_lock;
  unsigned int __ttc_convert;
  unsigned int __ttc_unlock;
  unsigned int __ttc_dispose;
#endif
};


#endif
