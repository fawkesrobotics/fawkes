/***************************************************************************
 *  syncpoint_manager.cpp - Fawkes SyncPointManager
 *
 *  Created: Thu Jan 09 15:22:19 2014
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

#include <syncpoint/syncpoint_manager.h>
#include <syncpoint/exceptions.h>

#include <string.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SyncPointManager <syncpoint/syncpoint_manager.h>
 * This class gives access to SyncPoints. SyncPoints should never be created
 * directly but always by using this class.
 *
 * All threads with the SyncPointManager Aspect share the same SyncPointManager.
 * SyncPointManager provides basic methods to get and release shared SyncPoints
 *
 * @author Till Hofmann
 * @see SyncPoint
 */

SyncPointManager::SyncPointManager()
{
}

SyncPointManager::~SyncPointManager()
{
}

/**
 * Get a SyncPoint. This allows accessing the SyncPoint's wait() and emit() methods
 * @param component The name of the component calling the method
 * @param identifier The identifier of the requested SyncPoint
 * @return A RefPtr to a SyncPoint which is shared by all threads with this
 * SyncPoint.
 * @throw SyncPointInvalidComponentException thrown if component name is invalid
 * @throw SyncPointAlreadyOpenedException thrown if SyncPoint is already opened
 * by the component
 */
RefPtr<SyncPoint>
SyncPointManager::get_syncpoint(const char * component, const char * identifier)
{
  if (!strcmp(component, ""))
    throw SyncPointInvalidComponentException(component, identifier);
  // insert a new SyncPoint if no SyncPoint with the same identifier exists,
  // otherwise, use that SyncPoint
  std::pair<std::set<RefPtr<SyncPoint> >::iterator,bool> ret = syncpoints_.insert(
      RefPtr<SyncPoint>(new SyncPoint(identifier)));
  std::set<RefPtr<SyncPoint> >::iterator it = ret.first;

  // add component to the set of watchers
  // check if component is already a watcher
  // insert returns a pair whose second element is false if element already exists
  if (!(*it)->watchers.insert(component).second) {
    throw SyncPointAlreadyOpenedException(component, identifier);
  }
  return *it;
}

/**
 * Release a SyncPoint. After releasing the SyncPoint, its wait() and emit()
 * methods cannot be called anymore by the releasing component
 * @param component The releasing component
 * @param sync_point A RefPtr to the released SyncPoint
 * @throw SyncPointReleasedDoesNotExistException thrown if the SyncPoint doesn't
 * exist, i.e. is not in the list of the manager's SyncPoints.
 * @throw SyncPointReleasedByNonWatcherException The releasing component is not
 * a watcher of the SyncPoint
 */
void
SyncPointManager::release_syncpoint(const char * component, RefPtr<SyncPoint> sync_point)
{
  std::set<RefPtr<SyncPoint> >::iterator sp_it = syncpoints_.find(
      sync_point);
  if (sp_it == syncpoints_.end()) {
    throw SyncPointReleasedDoesNotExistException(component, sync_point->get_identifier());
  }
  if (!(*sp_it)->watchers.erase(component)) {
    throw SyncPointReleasedByNonWatcherException(component, sync_point->get_identifier());
  }

}


/** @class SyncPointSetLessThan "syncpoint_manager.h"
 * Compare sets of syncpoints
 */

/**
 * LessThan Operator to use for the manager's SyncPoint set
 * Since we store RefPtrs to SyncPoints we have to override this operator
 * otherwise, each SyncPoint would be unique
 * @param sp1 A RefPtr to a SyncPoint
 * @param sp2 A RefPtr to a SyncPoint
 * @return true if strcmp returns a value < 0 for the SyncPoints' identifiers
 */
bool
SyncPointSetLessThan::operator()(const RefPtr<SyncPoint> sp1, const RefPtr<SyncPoint> sp2) const {
  return **sp1 < **sp2;
}

/**
 * Get the current list of all SyncPoints managed by this SyncPointManager
 * @return a set of SyncPoints
 */
std::set<RefPtr<SyncPoint>, SyncPointSetLessThan >
SyncPointManager::get_syncpoints() {
  return syncpoints_;
}

} // namespace fawkes
