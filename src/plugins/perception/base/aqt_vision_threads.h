
/***************************************************************************
 *  aqt_vision_threads.h - FireVision Base Vision Camera Data
 *
 *  Created: Mon Sep 24 16:16:05 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

namespace fawkes {
  class Clock;
  class Time;
  class Barrier;
}

class FvBaseThread;
class FvAquisitionThread;

class FvAqtVisionThreads
{
 friend FvBaseThread;
 friend FvAquisitionThread;
 public:
  FvAqtVisionThreads(fawkes::Clock *clock);
  ~FvAqtVisionThreads();

  void add_waiting_thread(fawkes::Thread *thread);
  void remove_thread(fawkes::Thread *thread);
  void remove_waiting_thread(fawkes::Thread *thread);
  void set_thread_running(fawkes::Thread *thread);

  bool  empty();
  float empty_time();

  bool  has_cyclic_thread();
  bool  has_cont_thread();

  void  wakeup_and_wait_cyclic_threads();

  void  set_prepfin_hold(bool hold);

  bool  has_waiting_thread(fawkes::Thread *t);

 private:
  fawkes::ThreadList  *running_threads_cyclic;
  fawkes::ThreadList  *running_threads_cont;
  fawkes::ThreadList  *waiting_threads;

  fawkes::Barrier     *cyclic_barrier;

  fawkes::Clock *clock;
  fawkes::Time  *_empty_time;
};

#endif
