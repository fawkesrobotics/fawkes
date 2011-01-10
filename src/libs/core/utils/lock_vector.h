
/***************************************************************************
 *  lock_vector.h - Lockable vector
 *
 *  Created: Mon Jan 10 11:11:59 2011
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

#ifndef __CORE_UTILS_LOCK_VECTOR_H_
#define __CORE_UTILS_LOCK_VECTOR_H_

#include <core/threading/mutex.h>
#include <core/utils/refptr.h>
#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

template <typename Type>
class LockVector : public std::vector<Type>
{
 public:
  LockVector();
  LockVector(const LockVector<Type> &lv);
  virtual ~LockVector();
  virtual void  lock() const;
  virtual bool  try_lock() const;
  virtual void  unlock() const;
  RefPtr<Mutex> mutex() const;

  void     push_back_locked(const Type& x);
  void     pop_back_locked();
  void     erase_locked(typename std::vector<Type>::iterator pos);
  void     erase_locked(typename std::vector<Type>::iterator first,
			typename std::vector<Type>::iterator last);

  LockVector<Type> &  operator=(const LockVector<Type> &lv);
  LockVector<Type> &  operator=(const std::vector<Type> &v);
 private:
  mutable RefPtr<Mutex> __mutex;

};


/** @class LockVector <core/utils/lock_vector.h>
 * Vector with a lock.
 * This class provides a vector that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see Mutex
 * @ingroup FCL
 * @author Tim Niemueller
 */


/** Constructor. */
template <typename Type>
LockVector<Type>::LockVector()
  : __mutex(new Mutex())
{}


/** Copy constructor.
 * @param lv LockVector to copy
 */
template <typename Type>
LockVector<Type>::LockVector(const LockVector<Type> &lv)
  : std::vector<Type>::vector(lv), __mutex(new Mutex())
{}


/** Destructor. */
template <typename Type>
LockVector<Type>::~LockVector()
{}


/** Lock vector. */
template <typename Type>
void
LockVector<Type>::lock() const
{
  __mutex->lock();
}


/** Try to lock vector.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename Type>
bool
LockVector<Type>::try_lock() const
{
  return __mutex->try_lock();
}


/** Unlock vector. */
template <typename Type>
void
LockVector<Type>::unlock() const
{
  return __mutex->unlock();
}


/** Push element to vector at back with lock protection.
 * @param x element to add
 */
template <typename Type>
void
LockVector<Type>::push_back_locked(const Type& x)
{
  __mutex->lock();
  std::vector<Type>::push_back(x);
  __mutex->unlock();
}


/** Remove last element with lock protection. */
template <typename Type>
void
LockVector<Type>::pop_back_locked()
{
  __mutex->lock();
  std::vector<Type>::pop_back();
  __mutex->unlock();
}


/** Erase given element with lock protection.
 * @param pos iterator for the object position to remove
 */
template <typename Type>
void
LockVector<Type>::erase_locked(typename std::vector<Type>::iterator pos)
{
  __mutex->lock();
  std::vector<Type>::erase(pos);
  __mutex->unlock();
}

/** Erase given element range with lock protection.
 * @param first iterator to first element to erase
 * @param last iterator to first element not to erase
 */
template <typename Type>
void
LockVector<Type>::erase_locked(typename std::vector<Type>::iterator first,
			       typename std::vector<Type>::iterator last)
{
  __mutex->lock();
  std::vector<Type>::erase(first, last);
  __mutex->unlock();
}


/** Get access to the internal mutex.
 * Can be used with MutexLocker.
 * @return internal mutex
 */
template <typename Type>
RefPtr<Mutex>
LockVector<Type>::mutex() const
{
  return __mutex;
}


/** Copy values from another LockVector.
 * Copies the values one by one. Both instances are locked during the copying and
 * this instance is cleared before copying.
 * @param lv vector to copy
 * @return reference to this instance
 */
template <typename Type>
LockVector<Type> &
LockVector<Type>::operator=(const LockVector<Type> &lv)
{
  __mutex->lock();
  lv.lock();
  this->clear();
  typename LockVector<Type>::const_iterator i;
  for (i = lv.begin(); i != lv.end(); ++i) {
    this->push_back(*i);
  }
  lv.unlock();
  __mutex->unlock();

  return *this;
}


/** Copy values from a standard vector.
 * Copies the values one by one. This instance is locked during the copying and
 * cleared.
 * @param v vector to copy
 * @return reference to this instance
 */
template <typename Type>
LockVector<Type> &
LockVector<Type>::operator=(const std::vector<Type> &v)
{
  __mutex->lock();
  this->clear();
  typename std::vector<Type>::const_iterator i;
  for (i = v.begin(); i != v.end(); ++i) {
    this->push_back(*i);
  }
  __mutex->unlock();

  return *this;
}

} // end namespace fawkes

#endif
