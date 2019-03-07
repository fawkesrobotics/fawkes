
/***************************************************************************
 *  rwlock_list.h - List with read/write lock
 *
 *  Created: Tue Jan 13 16:33:35 2009
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

#ifndef _CORE_UTILS_RWLOCK_LIST_H_
#define _CORE_UTILS_RWLOCK_LIST_H_

#include <core/threading/read_write_lock.h>
#include <core/utils/refptr.h>

#include <list>

namespace fawkes {

template <typename Type>
class RWLockList : public std::list<Type>
{
public:
	RWLockList();
	RWLockList(const RWLockList<Type> &ll);
	virtual ~RWLockList();
	virtual void          lock_for_read();
	virtual void          lock_for_write();
	virtual bool          try_lock_for_read();
	virtual bool          try_lock_for_write();
	virtual void          unlock();
	RefPtr<ReadWriteLock> rwlock() const;

	void push_back_locked(const Type &x);
	void push_front_locked(const Type &x);
	void remove_locked(const Type &x);

	RWLockList<Type> &operator=(const RWLockList<Type> &ll);
	RWLockList<Type> &operator=(const std::list<Type> &l);

private:
	RefPtr<ReadWriteLock> rwlock_;
};

/** @class RWLockList <core/utils/rwlock_list.h>
 * List with a read/write lock.
 * This class provides a list that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see ReadWriteLock
 * @ingroup FCL
 * @author Tim Niemueller
 */

/** Constructor. */
template <typename Type>
RWLockList<Type>::RWLockList() : rwlock_(new ReadWriteLock())
{
}

/** Copy constructor.
 * @param ll RWLockList to copy
 */
template <typename Type>
RWLockList<Type>::RWLockList(const RWLockList<Type> &ll)
: std::list<Type>::list(ll), rwlock_(new ReadWriteLock())
{
}

/** Destructor. */
template <typename Type>
RWLockList<Type>::~RWLockList()
{
}

/** Lock list for reading. */
template <typename Type>
void
RWLockList<Type>::lock_for_read()
{
	rwlock_->lock_for_read();
}

/** Lock list for writing. */
template <typename Type>
void
RWLockList<Type>::lock_for_write()
{
	rwlock_->lock_for_write();
}

/** Try to lock list for reading.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename Type>
bool
RWLockList<Type>::try_lock_for_read()
{
	return rwlock_->try_lock_for_read();
}

/** Try to lock list for writing.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename Type>
bool
RWLockList<Type>::try_lock_for_write()
{
	return rwlock_->try_lock_for_write();
}

/** Unlock list. */
template <typename Type>
void
RWLockList<Type>::unlock()
{
	return rwlock_->unlock();
}

/** Push element to list at back with lock protection.
 * @param x element to add
 */
template <typename Type>
void
RWLockList<Type>::push_back_locked(const Type &x)
{
	rwlock_->lock_for_write();
	std::list<Type>::push_back(x);
	rwlock_->unlock();
}

/** Push element to list at front with lock protection.
 * @param x element to add
 */
template <typename Type>
void
RWLockList<Type>::push_front_locked(const Type &x)
{
	rwlock_->lock_for_write();
	std::list<Type>::push_front(x);
	rwlock_->unlock();
}

/** Remove element from list with lock protection.
 * @param x element to remove
 */
template <typename Type>
void
RWLockList<Type>::remove_locked(const Type &x)
{
	rwlock_->lock_for_write();
	std::list<Type>::remove(x);
	rwlock_->unlock();
}

/** Get access to the internal read/write lock
 * @return internal rwlock
 */
template <typename Type>
RefPtr<ReadWriteLock>
RWLockList<Type>::rwlock() const
{
	return rwlock_;
}

/** Copy values from another RWLockList.
 * Copies the values one by one. Both instances are locked during the copying and
 * this instance is cleared before copying.
 * @param ll list to copy
 * @return reference to this instance
 */
template <typename Type>
RWLockList<Type> &
RWLockList<Type>::operator=(const RWLockList<Type> &ll)
{
	rwlock_->lock_for_write();
	ll.lock_for_read();
	this->clear();
	typename RWLockList<Type>::const_iterator i;
	for (i = ll.begin(); i != ll.end(); ++i) {
		this->push_back(*i);
	}
	ll.unlock();
	rwlock_->unlock();

	return *this;
}

/** Copy values from a standard list.
 * Copies the values one by one. This instance is locked during the copying and
 * cleared.
 * @param l list to copy
 * @return reference to this instance
 */
template <typename Type>
RWLockList<Type> &
RWLockList<Type>::operator=(const std::list<Type> &l)
{
	rwlock_->lock_for_write();
	this->clear();
	typename std::list<Type>::const_iterator i;
	for (i = l.begin(); i != l.end(); ++i) {
		this->push_back(*i);
	}
	rwlock_->unlock();

	return *this;
}

} // end namespace fawkes

#endif
