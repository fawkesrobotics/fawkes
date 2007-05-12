
/***************************************************************************
 *  lock_hashmap.h - Lockable hash map
 *
 *  Created: Fri May 11 22:40:09 2007
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

#ifndef __CORE_UTILS_LOCK_HASHMAP_H_
#define __CORE_UTILS_LOCK_HASHMAP_H_

#include <core/threading/mutex.h>
#include <ext/hash_map>

template <class KeyType,
          class ValueType,
          class HashFunction = __gnu_cxx::hash<KeyType>,
          class EqualKey     = std::equal_to<KeyType> >
class LockHashMap : public __gnu_cxx::hash_map<KeyType, ValueType, HashFunction, EqualKey>
{
 public:
  LockHashMap();
  LockHashMap(const LockHashMap<KeyType, ValueType, HashFunction, EqualKey> &lh);
  virtual ~LockHashMap();

  void lock();
  bool tryLock();
  void unlock();

 private:
  Mutex *mutex;

};


/** @class LockHashMap core/utils/lock_hashmap.h
 * Hash map with a lock.
 * This class provides a hash map that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see Mutex
 * @ingroup FCL
 * @author Tim Niemueller
 */


/** Constructor. */
template <class KeyType, class ValueType, class HashFunction, class EqualKey>
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::LockHashMap()
{
  mutex = new Mutex();
}


/** Copy constructor.
 * @param lh LockHashMap to copy
 */
template <class KeyType, class ValueType, class HashFunction, class EqualKey>
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::LockHashMap(const LockHashMap<KeyType, ValueType, HashFunction, EqualKey> &lh)
  : __gnu_cxx::hash_map<KeyType, ValueType, HashFunction, EqualKey>::hash_map(lh)
{
  mutex = new Mutex();
}


/** Destructor. */
template <class KeyType, class ValueType, class HashFunction, class EqualKey>
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::~LockHashMap()
{
  delete mutex;
}


/** Lock list. */
template <class KeyType, class ValueType, class HashFunction, class EqualKey>
void
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::lock()
{
  mutex->lock();
}


/** Try to lock list.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <class KeyType, class ValueType, class HashFunction, class EqualKey>
bool
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::tryLock()
{
  return mutex->tryLock();
}


/** Unlock list. */
template <class KeyType, class ValueType, class HashFunction, class EqualKey>
void
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::unlock()
{
  return mutex->unlock();
}

#endif
