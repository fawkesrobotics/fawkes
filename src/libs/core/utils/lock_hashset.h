
/***************************************************************************
 *  lock_hashset.h - Lockable hash set
 *
 *  Created: Sat May 12 13:06:31 2007
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

#ifndef __CORE_UTILS_LOCK_HASHSET_H_
#define __CORE_UTILS_LOCK_HASHSET_H_

#include <core/threading/mutex.h>
#include <ext/hash_set>

template <class KeyType,
          class HashFunction = __gnu_cxx::hash<KeyType>,
          class EqualKey     = std::equal_to<KeyType> >
class LockHashSet : public __gnu_cxx::hash_set<KeyType, HashFunction, EqualKey>
{
 public:
  LockHashSet();
  LockHashSet(const LockHashSet<KeyType, HashFunction, EqualKey> &lh);
  virtual ~LockHashSet();

  void lock();
  bool tryLock();
  void unlock();

  void insert_locked(const KeyType& x);

 private:
  Mutex *mutex;

};


/** @class LockHashSet core/utils/lock_hashset.h
 * Hash set with a lock.
 * This class provides a hash set that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see Mutex
 * @ingroup FCL
 * @author Tim Niemueller
 */


/** Constructor. */
template <class KeyType, class HashFunction, class EqualKey>
LockHashSet<KeyType, HashFunction, EqualKey>::LockHashSet()
{
  mutex = new Mutex();
}


/** Copy constructor.
 * @param lh LockHashSet to copy
 */
template <class KeyType, class HashFunction, class EqualKey>
LockHashSet<KeyType, HashFunction, EqualKey>::LockHashSet(const LockHashSet<KeyType, HashFunction, EqualKey> &lh)
  : __gnu_cxx::hash_set<KeyType, HashFunction, EqualKey>::hash_set(lh)
{
  mutex = new Mutex();
}


/** Destructor. */
template <class KeyType, class HashFunction, class EqualKey>
LockHashSet<KeyType, HashFunction, EqualKey>::~LockHashSet()
{
  delete mutex;
}


/** Lock list. */
template <class KeyType, class HashFunction, class EqualKey>
void
LockHashSet<KeyType, HashFunction, EqualKey>::lock()
{
  mutex->lock();
}


/** Try to lock list.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <class KeyType, class HashFunction, class EqualKey>
bool
LockHashSet<KeyType, HashFunction, EqualKey>::tryLock()
{
  return mutex->tryLock();
}


/** Unlock list. */
template <class KeyType, class HashFunction, class EqualKey>
void
LockHashSet<KeyType, HashFunction, EqualKey>::unlock()
{
  return mutex->unlock();
}


/** Insert element to hash set with lock protection.
 * @param x element to add
 */
template <class KeyType, class HashFunction, class EqualKey>
void
LockHashSet<KeyType, HashFunction, EqualKey>::insert_locked(const KeyType& x)
{
  mutex->lock();
  __gnu_cxx::hash_set<KeyType, HashFunction, EqualKey>::insert(x);
  mutex->unlock();
}


#endif
