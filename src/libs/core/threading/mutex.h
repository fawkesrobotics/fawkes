
/***************************************************************************
 *  mutex.h - Mutex
 *
 *  Generated: Thu Sep 14 16:58:49 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_THREADING_MUTEX_H_
#define __CORE_THREADING_MUTEX_H_

namespace fawkes {

class MutexData;
class WaitCondition;

class Mutex
{
  friend class WaitCondition;

 public:
  /** Mutex type. */
  typedef enum {
    NORMAL,	///< This type of mutex does not detect deadlock.
    RECURSIVE	///< A thread attempting to relock this mutex without
    		///< first unlocking it shall succeed in locking the mutex.
  } Type;

  Mutex(Type type = NORMAL);
  ~Mutex();

  void lock();
  bool try_lock();
  void unlock();

  void stopby();

 private:
  MutexData *mutex_data;
};


} // end namespace fawkes

#endif
