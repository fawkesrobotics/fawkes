
/***************************************************************************
 *  wait_condition.h - condition variable implementation
 *
 *  Created: Thu Sep 14 21:34:58 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __CORE_THREADING_WAIT_CONDITION_H_
#define __CORE_THREADING_WAIT_CONDITION_H_

#include <core/threading/mutex.h>

namespace fawkes {

class WaitConditionData;

class WaitCondition {
 public:
  WaitCondition();
  ~WaitCondition();

  void wait(Mutex *mutex);
  inline void wait() { __mutex->lock(); wait(__mutex); __mutex->unlock(); };

  bool abstimed_wait(Mutex *mutex, long int sec, long int nanosec);
  inline bool abstimed_wait(long int sec, long int nanosec)
  { __mutex->lock(); bool rv = abstimed_wait(__mutex, sec, nanosec);
    __mutex->unlock(); return rv; }

  bool reltimed_wait(Mutex *mutex, unsigned int sec, unsigned int nanosec);
  inline bool reltimed_wait(unsigned int sec, unsigned int nanosec)
  { __mutex->lock(); bool rv = reltimed_wait(__mutex, sec, nanosec);
    __mutex->unlock(); return rv; }

  void wake_one();
  void wake_all();

 private:
  WaitConditionData *__cond_data;
  Mutex             *__mutex;
  Mutex             *__active_mutex;
  unsigned int       __waiters;
};


} // end namespace fawkes

#endif
