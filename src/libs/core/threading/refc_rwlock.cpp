
/***************************************************************************
 *  refc_rwlock.cpp - Read Write Lock with reference counter
 *
 *  Generated: Fri Oct 27 17:28:04 2006
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

#include <core/threading/refc_rwlock.h>

namespace fawkes {

/** @class RefCountRWLock core/threading/refc_rwlock.h
 * Read/write lock with reference counting.
 * This class is a combination of ReadWriteLock and RefCount. A reference count
 * is maintained for the class to allow for automatic destruction when the last
 * user calls unref(). The class otherwise acts as a normal ReadWriteLock, you
 * just should not delete the instance but rather unref() it.
 * @see ReadWriteLock
 * @see RefCount
 * @ingroup Threading
 * @ingroup FCL
 * @author Tim Niemueller
 */

/** Constructor.
 * @param policy Policy, see ReadWriteLock::ReadWriteLock() for more info on this.
 */
RefCountRWLock::RefCountRWLock(ReadWriteLock::ReadWriteLockPolicy policy)
  : ReadWriteLock(policy), RefCount()
{ 
}

/** Destructor */
RefCountRWLock::~RefCountRWLock()
{
}


} // end namespace fawkes
