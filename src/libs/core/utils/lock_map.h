
/***************************************************************************
 *  lock_map.h - Lockable map
 *
 *  Created: Sat May 12 10:45:15 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_UTILS_LOCK_MAP_H_
#define __CORE_UTILS_LOCK_MAP_H_

#include <core/threading/mutex.h>
#include <core/utils/refptr.h>
#include <map>

namespace fawkes {


template <typename KeyType,
          typename ValueType,
          typename LessKey     = std::less<KeyType> >
class LockMap : public std::map<KeyType, ValueType, LessKey>
{
 public:
  LockMap();
  LockMap(const LockMap<KeyType, ValueType, LessKey> &lm);
  virtual ~LockMap();

  void           lock() const;
  bool           try_lock() const;
  void           unlock() const;
  RefPtr<Mutex>  mutex() const;

  void     erase_locked(const KeyType &key);

  LockMap<KeyType, ValueType, LessKey> &
  operator=(const LockMap<KeyType, ValueType, LessKey> &ll);

  LockMap<KeyType, ValueType, LessKey> &
  operator=(const std::map<KeyType, ValueType, LessKey> &l);

 private:
  mutable RefPtr<Mutex>  __mutex;

};


/** @class LockMap <core/utils/lock_map.h>
 * Map with a lock.
 * This class provides a map that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see Mutex
 * @ingroup FCL
 * @author Tim Niemueller
 */


/** Constructor. */
template <typename KeyType, typename ValueType, typename LessKey>
LockMap<KeyType, ValueType, LessKey>::LockMap()
  : __mutex(new Mutex())
{}


/** Copy constructor.
 * @param lm LockMap to copy
 */
template <typename KeyType, typename ValueType, typename LessKey>
LockMap<KeyType, ValueType, LessKey>::LockMap(const LockMap<KeyType, ValueType, LessKey> &lm)
  : std::map<KeyType, ValueType, LessKey>::map(lm),
    __mutex(new Mutex())
{}


/** Destructor. */
template <typename KeyType, typename ValueType, typename LessKey>
LockMap<KeyType, ValueType, LessKey>::~LockMap()
{}


/** Lock list. */
template <typename KeyType, typename ValueType, typename LessKey>
void
LockMap<KeyType, ValueType, LessKey>::lock() const
{
  __mutex->lock();
}


/** Try to lock list.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename KeyType, typename ValueType, typename LessKey>
bool
LockMap<KeyType, ValueType, LessKey>::try_lock() const
{
  return __mutex->try_lock();
}


/** Unlock list. */
template <typename KeyType, typename ValueType, typename LessKey>
void
LockMap<KeyType, ValueType, LessKey>::unlock() const
{
  return __mutex->unlock();
}


/** Remove item with lock.
 * The map is automatically locked and unlocked during the removal.
 * @param key key of the value to erase
 */
template <typename KeyType, typename ValueType, typename LessKey>
void
LockMap<KeyType, ValueType, LessKey>::erase_locked(const KeyType &key)
{
  __mutex->lock();
  std::map<KeyType, ValueType, LessKey>::erase(key);
  __mutex->unlock();
}


/** Get access to the internal mutex.
 * Can be used with MutexLocker.
 * @return internal mutex
 */
template <typename KeyType, typename ValueType, typename LessKey>
RefPtr<Mutex>
LockMap<KeyType, ValueType, LessKey>::mutex() const
{
  return __mutex;
}



/** Copy values from another LockMap.
 * Copies the values one by one. Both instances are locked during the copying and
 * this instance is cleared before copying.
 * @param ll map to copy
 * @return reference to this instance
 */
template <typename KeyType, typename ValueType, typename LessKey>
LockMap<KeyType, ValueType, LessKey> &
LockMap<KeyType, ValueType, LessKey>::operator=(const LockMap<KeyType, ValueType, LessKey> &ll)
{
  __mutex->lock();
  ll.lock();
  this->clear();
  typename LockMap<KeyType, ValueType, LessKey>::const_iterator i;
  for (i = ll.begin(); i != ll.end(); ++i) {
    this->insert(*i);
  }
  ll.unlock();
  __mutex->unlock();

  return *this;
}


/** Copy values from a standard map.
 * Copies the values one by one. This instance is locked during the copying and
 * cleared.
 * @param l map to copy
 * @return reference to this instance
 */
template <typename KeyType, typename ValueType, typename LessKey>
LockMap<KeyType, ValueType, LessKey> &
LockMap<KeyType, ValueType, LessKey>::operator=(const std::map<KeyType, ValueType, LessKey> &l)
{
  __mutex->lock();
  this->clear();
  typename std::map<KeyType, ValueType, LessKey>::const_iterator i;
  for (i = l.begin(); i != l.end(); ++i) {
    this->insert(*i);
  }
  __mutex->unlock();

  return *this;
}


} // end namespace fawkes

#endif
