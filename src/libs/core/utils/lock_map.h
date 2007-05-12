
/***************************************************************************
 *  lock_map.h - Lockable map
 *
 *  Created: Sat May 12 10:45:15 2007
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

#ifndef __CORE_UTILS_LOCK_MAP_H_
#define __CORE_UTILS_LOCK_MAP_H_

#include <core/threading/mutex.h>
#include <map>

template <typename KeyType,
          typename ValueType,
          typename LessKey     = std::less<KeyType> >
class LockMap : public std::map<KeyType, ValueType, LessKey>
{
 public:
  LockMap();
  LockMap(const LockMap<KeyType, ValueType, LessKey> &lm);
  virtual ~LockMap();

  void lock();
  bool tryLock();
  void unlock();

 private:
  Mutex *mutex;

};


/** @class LockMap core/utils/lock_hashmap.h
 * Hash map with a lock.
 * This class provides a hahs map that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see Mutex
 * @ingroup FCL
 * @author Tim Niemueller
 */


/** Constructor. */
template <typename KeyType, typename ValueType, typename LessKey>
LockMap<KeyType, ValueType, LessKey>::LockMap()
{
  mutex = new Mutex();
}


/** Copy constructor.
 * @param lm LockMap to copy
 */
template <typename KeyType, typename ValueType, typename LessKey>
LockMap<KeyType, ValueType, LessKey>::LockMap(const LockMap<KeyType, ValueType, LessKey> &lm)
  : std::map<KeyType, ValueType, LessKey>::map(lm)
{
  mutex = new Mutex();
}


/** Destructor. */
template <typename KeyType, typename ValueType, typename LessKey>
LockMap<KeyType, ValueType, LessKey>::~LockMap()
{
  delete mutex;
}


/** Lock list. */
template <typename KeyType, typename ValueType, typename LessKey>
void
LockMap<KeyType, ValueType, LessKey>::lock()
{
  mutex->lock();
}


/** Try to lock list.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename KeyType, typename ValueType, typename LessKey>
bool
LockMap<KeyType, ValueType, LessKey>::tryLock()
{
  return mutex->tryLock();
}


/** Unlock list. */
template <typename KeyType, typename ValueType, typename LessKey>
void
LockMap<KeyType, ValueType, LessKey>::unlock()
{
  return mutex->unlock();
}

#endif
