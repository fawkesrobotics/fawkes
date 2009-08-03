
/***************************************************************************
 *  rwlock_map.h - Map with read/write lock
 *
 *  Created: Tue Jan 13 16:29:33 2009
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

#ifndef __CORE_UTILS_RWLOCK_MAP_H_
#define __CORE_UTILS_RWLOCK_MAP_H_

#include <core/threading/read_write_lock.h>
#include <core/utils/refptr.h>
#include <map>

namespace fawkes {


template <typename KeyType,
          typename ValueType,
          typename LessKey     = std::less<KeyType> >
class RWLockMap : public std::map<KeyType, ValueType, LessKey>
{
 public:
  RWLockMap();
  RWLockMap(const RWLockMap<KeyType, ValueType, LessKey> &lm);
  virtual ~RWLockMap();

  void                   lock_for_read();
  void                   lock_for_write();
  bool                   try_lock_for_read();
  bool                   try_lock_for_write();
  void                   unlock();
  RefPtr<ReadWriteLock>  rwlock() const;

  void     erase_locked(const KeyType &key);

 private:
  RefPtr<ReadWriteLock> __rwlock;

};


/** @class RWLockMap core/utils/rwlock_map.h
 * Hash map with a lock.
 * This class provides a map that has an intrinsic read/write lock. The lock can
 * be applied with the regular locking methods.
 *
 * @see ReadWriteLock
 * @ingroup FCL
 * @author Tim Niemueller
 */


/** Constructor. */
template <typename KeyType, typename ValueType, typename LessKey>
RWLockMap<KeyType, ValueType, LessKey>::RWLockMap()
  : __rwlock(new ReadWriteLock())
{}


/** Copy constructor.
 * @param lm RWLockMap to copy
 */
template <typename KeyType, typename ValueType, typename LessKey>
RWLockMap<KeyType, ValueType, LessKey>::RWLockMap(const RWLockMap<KeyType, ValueType, LessKey> &lm)
  : std::map<KeyType, ValueType, LessKey>::map(lm), __rwlock(new ReadWriteLock())
{}


/** Destructor. */
template <typename KeyType, typename ValueType, typename LessKey>
RWLockMap<KeyType, ValueType, LessKey>::~RWLockMap()
{}


/** Lock list for reading. */
template <typename KeyType, typename ValueType, typename LessKey>
void
RWLockMap<KeyType, ValueType, LessKey>::lock_for_read()
{
  __rwlock->lock_for_read();
}


/** Lock list for writing. */
template <typename KeyType, typename ValueType, typename LessKey>
void
RWLockMap<KeyType, ValueType, LessKey>::lock_for_write()
{
  __rwlock->lock_for_write();
}


/** Try to lock list for reading.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename KeyType, typename ValueType, typename LessKey>
bool
RWLockMap<KeyType, ValueType, LessKey>::try_lock_for_read()
{
  return __rwlock->try_lock_for_read();
}


/** Try to lock list for writing.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename KeyType, typename ValueType, typename LessKey>
bool
RWLockMap<KeyType, ValueType, LessKey>::try_lock_for_write()
{
  return __rwlock->try_lock_for_write();
}


/** Unlock list. */
template <typename KeyType, typename ValueType, typename LessKey>
void
RWLockMap<KeyType, ValueType, LessKey>::unlock()
{
  return __rwlock->unlock();
}


/** Remove item with lock.
 * The map is automatically locked and unlocked during the removal.
 * @param key key of the value to erase
 */
template <typename KeyType, typename ValueType, typename LessKey>
void
RWLockMap<KeyType, ValueType, LessKey>::erase_locked(const KeyType &key)
{
  __rwlock->lock_for_write();
  std::map<KeyType, ValueType, LessKey>::erase(key);
  __rwlock->unlock();
}


/** Get access to the internal rwlock.
 * Can be used with RwlockLocker.
 * @return internal rwlock
 */
template <typename KeyType, typename ValueType, typename LessKey>
RefPtr<ReadWriteLock>
RWLockMap<KeyType, ValueType, LessKey>::rwlock() const
{
  return __rwlock;
}


} // end namespace fawkes

#endif
