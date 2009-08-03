
/***************************************************************************
 *  mutex_data.h - mutex data
 *
 *  Generated: Thu Sep 14 17:44:54 2006
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

#ifndef __CORE_THREADING_MUTEX_DATA_H_
#define __CORE_THREADING_MUTEX_DATA_H_

#include <pthread.h>

#ifdef DEBUG_THREADING
#include <core/threading/thread.h>
#include <core/exception.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#endif

namespace fawkes {


/// @cond INTERNALS
/** Internal class of Mutexes, do not use directly.
 */
class MutexData {
 public:
  pthread_mutex_t mutex;

#ifdef DEBUG_THREADING
  MutexData() {
    lock_holder = strdup("Not locked");
  }

  ~MutexData() {
    if ( lock_holder ) {
      free(lock_holder);
    }
  }

  char *lock_holder;

  void set_lock_holder()
  {
    if ( lock_holder ) {
      free(lock_holder);
    }
    try {
      Thread *ct = Thread::current_thread();
      if ( ct ) {
	lock_holder = strdup(ct->name());
      } else {
	lock_holder = strdup("Unknown");
      }
    } catch (Exception &e) {
      asprintf(&lock_holder, "Unknown: failed to get thread (%s)", e.what());
    }
  }

  void unset_lock_holder() {
    if ( lock_holder ) {
      free(lock_holder);
    }
    lock_holder = strdup("Not locked");
  }

#endif
};
/// @endcond


} // end namespace fawkes

#endif
