
/***************************************************************************
 *  lock_list.h - Lockable list
 *
 *  Created: Tue Oct 31 18:25:03 2006
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

#ifndef __CORE_UTILS_LOCK_LIST_H_
#define __CORE_UTILS_LOCK_LIST_H_

#include <core/threading/mutex.h>
#include <core/utils/refptr.h>
#include <list>

namespace fawkes {


/** @class LockList <core/utils/lock_list.h>
 * List with a lock.
 * This class provides a list that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see Mutex
 * @ingroup FCL
 * @author Tim Niemueller
 */
template <typename Type>
class LockList : public std::list<Type>
{
 public:
  /** Constructor. */
  LockList();

  /** Copy constructor.
   * @param ll LockList to copy
   */
  LockList(const LockList<Type> &ll);

  /** Destructor. */
  virtual ~LockList() {}


  /** Lock list. */
  virtual void  lock() const;

  /** Try to lock list.
   * @return true, if the lock has been aquired, false otherwise.
   */
  virtual bool  try_lock() const;

  /** Unlock list. */
  virtual void  unlock() const;

  /** Get access to the internal mutex.
   * Can be used with MutexLocker.
   * @return internal mutex
   */
  RefPtr<Mutex> mutex() const;

  /** Push element to list at back with lock protection.
   * @param x element to add
   */
  void     push_back_locked(const Type& x);

  /** Push element to list at front with lock protection.
   * @param x element to add
   */
  void     push_front_locked(const Type& x);

  /** Remove element from list with lock protection.
   * @param x element to remove
   */
  void     remove_locked(const Type& x);

  /** Copy values from another LockList.
   * Copies the values one by one. Both instances are locked during the copying and
   * this instance is cleared before copying.
   * @param ll list to copy
   * @return reference to this instance
   */
  LockList<Type> &  operator=(const LockList<Type> &ll);

  /** Copy values from a standard list.
   * Copies the values one by one. This instance is locked during the copying and
   * cleared.
   * @param l list to copy
   * @return reference to this instance
   */
  LockList<Type> &  operator=(const std::list<Type> &l);
 private:
  mutable RefPtr<Mutex> __mutex;

};




template <typename Type>
LockList<Type>::LockList()
  : __mutex(new Mutex())
{}


template <typename Type>
LockList<Type>::LockList(const LockList<Type> &ll)
  : std::list<Type>::list(ll), __mutex(new Mutex())
{}


template <typename Type>
void
LockList<Type>::lock() const
{
  __mutex->lock();
}


template <typename Type>
bool
LockList<Type>::try_lock() const
{
  return __mutex->try_lock();
}


template <typename Type>
void
LockList<Type>::unlock() const
{
  return __mutex->unlock();
}


template <typename Type>
void
LockList<Type>::push_back_locked(const Type& x)
{
  __mutex->lock();
  std::list<Type>::push_back(x);
  __mutex->unlock();
}


template <typename Type>
void
LockList<Type>::push_front_locked(const Type& x)
{
  __mutex->lock();
  std::list<Type>::push_front(x);
  __mutex->unlock();
}


template <typename Type>
void
LockList<Type>::remove_locked(const Type& x)
{
  __mutex->lock();
  std::list<Type>::remove(x);
  __mutex->unlock();
}


template <typename Type>
RefPtr<Mutex>
LockList<Type>::mutex() const
{
  return __mutex;
}


template <typename Type>
LockList<Type> &
LockList<Type>::operator=(const LockList<Type> &ll)
{
  __mutex->lock();
  ll.lock();
  this->clear();
  typename LockList<Type>::const_iterator i;
  for (i = ll.begin(); i != ll.end(); ++i) {
    this->push_back(*i);
  }
  ll.unlock();
  __mutex->unlock();

  return *this;
}


template <typename Type>
LockList<Type> &
LockList<Type>::operator=(const std::list<Type> &l)
{
  __mutex->lock();
  this->clear();
  typename std::list<Type>::const_iterator i;
  for (i = l.begin(); i != l.end(); ++i) {
    this->push_back(*i);
  }
  __mutex->unlock();

  return *this;
}

} // end namespace fawkes

#endif
