
/***************************************************************************
 *  sem_exceptions.h - exceptions thrown in semaphore utils, do NOT put your
 *                     own application specific exceptions here!
 *
 *  Generated: Tue Sep 19 15:19:05 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_IPC_SEM_EXCEPTIONS_H_
#define __UTILS_IPC_SEM_EXCEPTIONS_H_

#include <core/exception.h>

namespace fawkes {


/** Semaphore or semaphore set invalid */
class SemInvalidException : public Exception {
 public:
  /** Constructor */
  SemInvalidException() : Exception("Semaphore or semaphore set invalid")  {}
};

/** Cannot lock semaphore */
class SemCannotLockException : public Exception {
 public:
  /** Constructor */
  SemCannotLockException() : Exception("Cannot lock semaphore")  {}
};

/** Cannot unlock semaphore */
class SemCannotUnlockException : public Exception {
 public:
  /** Constructor */
  SemCannotUnlockException() : Exception("Cannot unlock semaphore")  {}
};

/** Cannot set value on semaphore */
class SemCannotSetValException : public Exception {
 public:
  /** Constructor */
  SemCannotSetValException() : Exception("Cannot set value")  {}
};


} // end namespace fawkes

#endif
