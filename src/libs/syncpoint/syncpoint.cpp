/***************************************************************************
 *  syncpoint.cpp - Fawkes SyncPoint
 *
 *  Created: Thu Jan 09 12:35:57 2014
 *  Copyright  2014  Till Hofmann
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <syncpoint/syncpoint.h>
#include <syncpoint/exceptions.h>

#include <string.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SyncPoint <syncpoint/syncpoint.h>
 * The SyncPoint class.
 * This class is used for dynamic synchronization of threads which depend
 * on each other, e.g. threads which are part of a processing chain.
 *
 * As an example, thread E generates data which is needed by thread W.
 * Therefore, both threads share a SyncPoint.
 * Thread W wait()s for the SyncPoint to be emitted.
 * Once thread E is done, it emit()s the SyncPoint, which wakes up thread W.
 *
 * @author Till Hofmann
 * @see SyncPointManager
 */

/** Constructor.
 * @param identifier The identifier of the SyncPoint. This must be in absolute
 * path style, e.g. '/some/syncpoint'.
 */
SyncPoint::SyncPoint(const char * identifier)
    : identifier_(identifier),
      mutex(new Mutex()),
      wait_condition(new WaitCondition(mutex))
{
  if (!strcmp(identifier, "")) {
    delete wait_condition;
    delete mutex;
    throw SyncPointInvalidIdentifierException(identifier);
  }
  if (strncmp(identifier, "/", 1)) {
    delete wait_condition;
    delete mutex;
    throw SyncPointInvalidIdentifierException(identifier);
  }
}

SyncPoint::~SyncPoint()
{
  delete wait_condition;
  delete mutex;
}

/**
 * @return the identifier of the SyncPoint
 */
const char *
SyncPoint::get_identifier() const {
  return identifier_;
}

/** EqualOperator.
 * Two SyncPoints are considered equal iff they have the same identifier
 * @param other The other SyncPoint
 * @return true if the identifiers of the SyncPoints are equal
 */
bool
SyncPoint::operator==(const SyncPoint &other) const
{
  return strcmp(identifier_, other.get_identifier()) == 0;
}

/** LessThan Operator.
 * Compare two SyncPoints using their identifiers.
 * @param other The other SyncPoint
 * @return true if strcmp returns a value < 0 for the identifiers
 */
bool
SyncPoint::operator<(const SyncPoint &other) const
{
  return strcmp(identifier_, other.get_identifier()) < 0;
}

/** Wake up all components which are waiting for this SyncPoint
 * @param component The identifier of the component emitting the SyncPoint
 */
void
SyncPoint::emit(const char * component)
{
  mutex->lock();
  if (!watchers.count(component)) {
    mutex->unlock();
    throw SyncPointNonWatcherCalledEmitException(component, get_identifier());
  }
  wait_condition->wake_all();
  mutex->unlock();
}

/** Wait until SyncPoint is emitted
 * @param component The identifier of the component waiting for the SyncPoint
 */
void
SyncPoint::wait(const char * component) {
  mutex->lock();
  // check if calling component is registered for this SyncPoint
  if (!watchers.count(component)) {
    mutex->unlock();
    throw SyncPointNonWatcherCalledWaitException(component, get_identifier());
  }
  wait_condition->wait();
  mutex->unlock();
}

} // namespace fawkes
