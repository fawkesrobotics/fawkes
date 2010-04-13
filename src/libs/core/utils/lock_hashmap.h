
/***************************************************************************
 *  lock_hashmap.h - Lockable hash map
 *
 *  Created: Fri May 11 22:40:09 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_UTILS_LOCK_HASHMAP_H_
#define __CORE_UTILS_LOCK_HASHMAP_H_

#include <core/threading/mutex.h>
#include <core/utils/refptr.h>
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 2)
#  include <tr1/unordered_map>
#else
#  include <ext/hash_map>
#endif

namespace fawkes {


template <class KeyType,
          class ValueType,
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 2)
          class HashFunction = std::tr1::hash<KeyType>,
          class EqualKey     = std::equal_to<KeyType> >
class LockHashMap : public std::tr1::unordered_map<KeyType, ValueType, HashFunction, EqualKey>
#else
          class HashFunction = __gnu_cxx::hash<KeyType>,
          class EqualKey     = std::equal_to<KeyType> >
class LockHashMap : public __gnu_cxx::hash_map<KeyType, ValueType, HashFunction, EqualKey>
#endif
{
 public:
  LockHashMap();
  LockHashMap(const LockHashMap<KeyType, ValueType, HashFunction, EqualKey> &lh);
  virtual ~LockHashMap();

  void          lock() const;
  bool          try_lock() const;
  void          unlock() const;
  RefPtr<Mutex> mutex() const;

  LockHashMap<KeyType, ValueType, HashFunction, EqualKey> &
  operator=(const LockHashMap<KeyType, ValueType, HashFunction, EqualKey> &ll);

 private:
  mutable RefPtr<Mutex> __mutex;

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
  : __mutex(new Mutex())
{
}


/** Copy constructor.
 * @param lh LockHashMap to copy
 */
template <class KeyType, class ValueType, class HashFunction, class EqualKey>
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::LockHashMap(const LockHashMap<KeyType, ValueType, HashFunction, EqualKey> &lh)
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 2)
  : std::tr1::unordered_map<KeyType, ValueType, HashFunction, EqualKey>::unordered_map(lh)
#else
  : __gnu_cxx::hash_map<KeyType, ValueType, HashFunction, EqualKey>::hash_map(lh)
#endif
    , __mutex(new Mutex())
{
}


/** Destructor. */
template <class KeyType, class ValueType, class HashFunction, class EqualKey>
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::~LockHashMap()
{
}


/** Lock map. */
template <class KeyType, class ValueType, class HashFunction, class EqualKey>
void
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::lock() const
{
  __mutex->lock();
}


/** Try to lock map.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <class KeyType, class ValueType, class HashFunction, class EqualKey>
bool
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::try_lock() const
{
  return __mutex->try_lock();
}


/** Unlock map. */
template <class KeyType, class ValueType, class HashFunction, class EqualKey>
void
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::unlock() const
{
  return __mutex->unlock();
}


/** Get access to the internal mutex.
 * Can be used with MutexLocker.
 * @return internal mutex
 */
template <typename KeyType, typename ValueType, class HashFunction, typename EqualKey>
RefPtr<Mutex>
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::mutex() const
{
  return __mutex;
}


/** Copy values from another LockHashMap.
 * Copies the values one by one. Both instances are locked during the copying and
 * this instance is cleared before copying.
 * @param ll hash map to copy
 * @return reference to this instance
 */
template <typename KeyType, typename ValueType, class HashFunction, typename EqualKey>
LockHashMap<KeyType, ValueType, HashFunction, EqualKey> &
LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::operator=(
  const LockHashMap<KeyType, ValueType, HashFunction, EqualKey> &ll)
{
  __mutex->lock();
  ll.lock();
  this->clear();
  typename LockHashMap<KeyType, ValueType, HashFunction, EqualKey>::const_iterator i;
  for (i = ll.begin(); i != ll.end(); ++i) {
    this->insert(*i);
  }
  ll.unlock();
  __mutex->unlock();

  return *this;
}

} // end namespace fawkes

#endif
