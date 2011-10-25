
/***************************************************************************
 *  mutex.cpp - implementation of mutex, based on pthreads
 *
 *  Generated: Thu Sep 14 17:03:57 2006
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

#include <core/threading/recursive_mutex.h>

namespace fawkes {

/** @class RecursiveMutex <core/threading/recursive_mutex.h>
 * Recursive mutex.
 * This is a mutex which can be locked multiple times by the same thread.
 * Other threads attempting to lock the mutex will block as if this were a
 * regular mutex.
 * This class is just a convenience sub-class of Mutex with its type set
 * to Mutex::RECURSIVE. It is meant to be used to make the actual behavior
 * more obvious.
 *
 * @ingroup Threading
 * @ingroup FCL
 *
 * @author Tim Niemueller
 */

/** Constructor. */
RecursiveMutex::RecursiveMutex()
  : Mutex(Mutex::RECURSIVE)
{
}

} // end namespace fawkes
