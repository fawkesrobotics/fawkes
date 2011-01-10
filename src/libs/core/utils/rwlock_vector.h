
/***************************************************************************
 *  rwlock_vector.h - Vector with read/write lock
 *
 *  Created: Mon Jan 10 11:35:24 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_UTILS_RWLOCK_VECTOR_H_
#define __CORE_UTILS_RWLOCK_VECTOR_H_

#include <core/threading/read_write_lock.h>
#include <core/utils/refptr.h>
#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

template <typename Type>
class RWLockVector : public std::vector<Type>
{
 public:
  RWLockVector();
  RWLockVector(const RWLockVector<Type> &lv);
  virtual ~RWLockVector();
  virtual void  lock_for_read() const;
  virtual void  lock_for_write() const;
  virtual bool  try_lock_for_read() const;
  virtual bool  try_lock_for_write() const;
  virtual void  unlock() const;
  RefPtr<ReadWriteLock> rwlock() const;

  void     push_back_locked(const Type& x);
  void     pop_back_locked();
  void     erase_locked(typename std::vector<Type>::iterator pos);
  void     erase_locked(typename std::vector<Type>::iterator first,
			typename std::vector<Type>::iterator last);

  RWLockVector<Type> &  operator=(const RWLockVector<Type> &lv);
  RWLockVector<Type> &  operator=(const std::vector<Type> &v);
 private:
  mutable RefPtr<ReadWriteLock> __rwlock;

};


/** @class RWLockVector <core/utils/rwlock_vector.h>
 * Vector with a lock.
 * This class provides a vector that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see ReadWriteLock
 * @ingroup FCL
 * @author Tim Niemueller
 */


/** Constructor. */
template <typename Type>
RWLockVector<Type>::RWLockVector()
  : __rwlock(new ReadWriteLock())
{}


/** Copy constructor.
 * @param lv RWLockVector to copy
 */
template <typename Type>
RWLockVector<Type>::RWLockVector(const RWLockVector<Type> &lv)
  : std::vector<Type>::vector(lv), __rwlock(new ReadWriteLock())
{}


/** Destructor. */
template <typename Type>
RWLockVector<Type>::~RWLockVector()
{}


/** Lock vector for reading. */
template <typename Type>
void
RWLockVector<Type>::lock_for_read() const
{
  __rwlock->lock_for_read();
}


/** Lock vector for writing. */
template <typename Type>
void
RWLockVector<Type>::lock_for_write() const
{
  __rwlock->lock_for_write();
}


/** Try to lock vector for reading.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename Type>
bool
RWLockVector<Type>::try_lock_for_read() const
{
  return __rwlock->try_lock_for_read();
}


/** Try to lock vector for writing.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename Type>
bool
RWLockVector<Type>::try_lock_for_write() const
{
  return __rwlock->try_lock_for_write();
}


/** Unlock vector. */
template <typename Type>
void
RWLockVector<Type>::unlock() const
{
  return __rwlock->unlock();
}


/** Push element to vector at back with lock protection.
 * @param x element to add
 */
template <typename Type>
void
RWLockVector<Type>::push_back_locked(const Type& x)
{
  __rwlock->lock_for_write();
  std::vector<Type>::push_back(x);
  __rwlock->unlock();
}


/** Remove last element with lock protection. */
template <typename Type>
void
RWLockVector<Type>::pop_back_locked()
{
  __rwlock->lock_for_write();
  std::vector<Type>::pop_back();
  __rwlock->unlock();
}


/** Erase given element with lock protection.
 * @param pos iterator for the object position to remove
 */
template <typename Type>
void
RWLockVector<Type>::erase_locked(typename std::vector<Type>::iterator pos)
{
  __rwlock->lock_for_write();
  std::vector<Type>::erase(pos);
  __rwlock->unlock();
}

/** Erase given element range with lock protection.
 * @param first iterator to first element to erase
 * @param last iterator to first element not to erase
 */
template <typename Type>
void
RWLockVector<Type>::erase_locked(typename std::vector<Type>::iterator first,
			       typename std::vector<Type>::iterator last)
{
  __rwlock->lock_for_write();
  std::vector<Type>::erase(first, last);
  __rwlock->unlock();
}


/** Get access to the internal read/write lock.
 * @return internal read/write lock
 */
template <typename Type>
RefPtr<ReadWriteLock>
RWLockVector<Type>::rwlock() const
{
  return __rwlock;
}


/** Copy values from another RWLockVector.
 * Copies the values one by one. Both instances are locked during the copying and
 * this instance is cleared before copying.
 * @param lv vector to copy
 * @return reference to this instance
 */
template <typename Type>
RWLockVector<Type> &
RWLockVector<Type>::operator=(const RWLockVector<Type> &lv)
{
  __rwlock->lock_for_write();
  lv.lock_for_read();
  this->clear();
  typename RWLockVector<Type>::const_iterator i;
  for (i = lv.begin(); i != lv.end(); ++i) {
    this->push_back(*i);
  }
  lv.unlock();
  __rwlock->unlock();

  return *this;
}


/** Copy values from a standard vector.
 * Copies the values one by one. This instance is locked during the copying and
 * cleared.
 * @param v vector to copy
 * @return reference to this instance
 */
template <typename Type>
RWLockVector<Type> &
RWLockVector<Type>::operator=(const std::vector<Type> &v)
{
  __rwlock->lock_for_write();
  this->clear();
  typename std::vector<Type>::const_iterator i;
  for (i = v.begin(); i != v.end(); ++i) {
    this->push_back(*i);
  }
  __rwlock->unlock();

  return *this;
}

} // end namespace fawkes

#endif
