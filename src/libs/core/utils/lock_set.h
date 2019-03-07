
/***************************************************************************
 *  lock_set.h - Lockable set
 *
 *  Created: Fri Apr  9 15:24:51 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *             2010       Christoph Schwering
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

#ifndef _CORE_UTILS_LOCK_SET_H_
#define _CORE_UTILS_LOCK_SET_H_

#include <core/threading/mutex.h>
#include <core/utils/refptr.h>

#include <set>

namespace fawkes {

template <typename KeyType, typename LessKey = std::less<KeyType>>
class LockSet : public std::set<KeyType, LessKey>
{
public:
	LockSet();
	LockSet(const LockSet<KeyType, LessKey> &lm);
	virtual ~LockSet();

	void          lock() const;
	bool          try_lock() const;
	void          unlock() const;
	RefPtr<Mutex> mutex() const;

	/** Iterator. */
	typedef typename std::set<KeyType, LessKey>::iterator iterator;

	std::pair<iterator, bool> insert_locked(const KeyType &key);
	void                      erase_locked(const KeyType &key);

	LockSet<KeyType, LessKey> &operator=(const LockSet<KeyType, LessKey> &ll);
	LockSet<KeyType, LessKey> &operator=(const std::set<KeyType, LessKey> &l);

private:
	mutable RefPtr<Mutex> mutex_;
};

/** @class LockSet <core/utils/lock_set.h>
 * Set with a lock.
 * This class provides a set that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see Mutex
 * @ingroup FCL
 * @author Tim Niemueller
 */

/** Constructor. */
template <typename KeyType, typename LessKey>
LockSet<KeyType, LessKey>::LockSet() : mutex_(new Mutex())
{
}

/** Copy constructor.
 * @param lm LockSet to copy
 */
template <typename KeyType, typename LessKey>
LockSet<KeyType, LessKey>::LockSet(const LockSet<KeyType, LessKey> &lm)
: std::set<KeyType, LessKey>::set(lm), mutex_(new Mutex())
{
}

/** Destructor. */
template <typename KeyType, typename LessKey>
LockSet<KeyType, LessKey>::~LockSet()
{
}

/** Lock list. */
template <typename KeyType, typename LessKey>
void
LockSet<KeyType, LessKey>::lock() const
{
	mutex_->lock();
}

/** Try to lock list.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename KeyType, typename LessKey>
bool
LockSet<KeyType, LessKey>::try_lock() const
{
	return mutex_->try_lock();
}

/** Unlock list. */
template <typename KeyType, typename LessKey>
void
LockSet<KeyType, LessKey>::unlock() const
{
	return mutex_->unlock();
}

/** Insert item with lock.
 * The set is automatically locked and unlocked during the removal.
 * @param key key of the value to insert
 * @return iterator to inserted item
 */
template <typename KeyType, typename LessKey>
std::pair<typename LockSet<KeyType, LessKey>::iterator, bool>
LockSet<KeyType, LessKey>::insert_locked(const KeyType &key)
{
	mutex_->lock();
	std::pair<iterator, bool> ret = std::set<KeyType, LessKey>::insert(key);
	mutex_->unlock();
	return ret;
}

/** Remove item with lock.
 * The set is automatically locked and unlocked during the removal.
 * @param key key of the value to erase
 */
template <typename KeyType, typename LessKey>
void
LockSet<KeyType, LessKey>::erase_locked(const KeyType &key)
{
	mutex_->lock();
	std::set<KeyType, LessKey>::erase(key);
	mutex_->unlock();
}

/** Get access to the internal mutex.
 * Can be used with MutexLocker.
 * @return internal mutex
 */
template <typename KeyType, typename LessKey>
RefPtr<Mutex>
LockSet<KeyType, LessKey>::mutex() const
{
	return mutex_;
}

/** Copy values from another LockSet.
 * Copies the values one by one. Both instances are locked during the copying and
 * this instance is cleared before copying.
 * @param ll lock set to copy
 * @return reference to this instance
 */
template <typename KeyType, typename LessKey>
LockSet<KeyType, LessKey> &
LockSet<KeyType, LessKey>::operator=(const LockSet<KeyType, LessKey> &ll)
{
	mutex_->lock();
	ll.lock();
	this->clear();
	typename LockSet<KeyType, LessKey>::const_iterator i;
	for (i = ll.begin(); i != ll.end(); ++i) {
		this->insert(*i);
	}
	ll.unlock();
	mutex_->unlock();

	return *this;
}

/** Copy values from a standard set.
 * Copies the values one by one. This instance is locked during the copying and
 * cleared.
 * @param l set to copy
 * @return reference to this instance
 */
template <typename KeyType, typename LessKey>
LockSet<KeyType, LessKey> &
LockSet<KeyType, LessKey>::operator=(const std::set<KeyType, LessKey> &l)
{
	mutex_->lock();
	this->clear();
	typename std::set<KeyType, LessKey>::const_iterator i;
	for (i = l.begin(); i != l.end(); ++i) {
		this->insert(*i);
	}
	mutex_->unlock();

	return *this;
}

} // end namespace fawkes

#endif
