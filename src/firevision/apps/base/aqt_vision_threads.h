
/***************************************************************************
 *  aqt_vision_threads.h - FireVision Base Vision Camera Data
 *
 *  Created: Mon Sep 24 16:16:05 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_APPS_BASE_AQT_VISION_THREADS_H_
#define __FIREVISION_APPS_BASE_AQT_VISION_THREADS_H_

#include <core/threading/thread_list.h>

class Clock;
class Time;
class Barrier;

class FvAqtVisionThreads
{
 friend class FvBaseThread;
 friend class FvAquisitionThread;
 public:
  FvAqtVisionThreads(Clock *clock);
  ~FvAqtVisionThreads();

  void add_waiting_thread(Thread *thread, bool raw);
  void remove_thread(Thread *thread);
  void remove_waiting_thread(Thread *thread);
  void set_thread_running(Thread *thread);

  bool  empty();
  float empty_time();

  bool  has_cyclic_thread();

  void  wakeup_cyclic_threads();
  void  wait_cyclic_threads();

  bool  has_waiting_thread(Thread *t);

 private:
  ThreadList  *running_threads_cyclic;
  ThreadList  *running_threads_cyclic_raw;
  ThreadList  *running_threads_cont;
  ThreadList  *running_threads_cont_raw;
  ThreadList  *waiting_threads;
  ThreadList  *waiting_threads_raw;

  Barrier     *cyclic_barrier;

  Clock *clock;
  Time  *_empty_time;

};

#endif
