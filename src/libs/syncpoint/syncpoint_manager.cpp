/***************************************************************************
 *  syncpoint_manager.cpp - Fawkes SyncPointManager
 *
 *  Created: Thu Jan 09 15:22:19 2014
 *  Copyright  2014-2018  Till Hofmann
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

#include <core/threading/mutex_locker.h>

#include <syncpoint/syncpoint_manager.h>
#include <syncpoint/exceptions.h>

#include "syncpoint_call_stats.h"

#include <string>

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

/** Constructor.
 *  @param logger the logger to use for logging messages
 */
SyncPointManager::SyncPointManager(MultiLogger *logger)
: mutex_(new Mutex()),
  logger_(logger)
{
}

SyncPointManager::~SyncPointManager()
{
  delete mutex_;
}

/**
 * Get a SyncPoint. This allows accessing the SyncPoint's wait() and emit() methods.
 * This function creates a SyncPoint with the given identifier if it does not
 * exist yet and constructs its predecessor.
 * @param component The name of the component calling the method
 * @param identifier The identifier of the requested SyncPoint
 * @return A RefPtr to a SyncPoint which is shared by all threads with this
 * SyncPoint.
 * @throw SyncPointInvalidComponentException thrown if component name is invalid
 * @throw SyncPointAlreadyOpenedException thrown if SyncPoint is already opened
 * by the component
 */
RefPtr<SyncPoint>
SyncPointManager::get_syncpoint(const std::string & component, const std::string & identifier)
{
  MutexLocker ml(mutex_);
  return get_syncpoint_no_lock(component, identifier);
}

/**
 * Release a SyncPoint. After releasing the SyncPoint, its wait() and emit()
 * methods cannot be called anymore by the releasing component.
 * This also releases the SyncPoint's predecessor if existent.
 * @param component The releasing component
 * @param sync_point A RefPtr to the released SyncPoint
 * @throw SyncPointReleasedDoesNotExistException thrown if the SyncPoint doesn't
 * exist, i.e. is not in the list of the manager's SyncPoints.
 * @throw SyncPointReleasedByNonWatcherException The releasing component is not
 * a watcher of the SyncPoint
 */
void
SyncPointManager::release_syncpoint(const std::string & component, RefPtr<SyncPoint> sync_point)
{
  MutexLocker ml(mutex_);
  release_syncpoint_no_lock(component, sync_point);
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
  MutexLocker ml(mutex_);
  return syncpoints_;
}


/** Find the prefix of the SyncPoint's identifier which is the identifier of
 *  the direct predecessor SyncPoint.
 *  The predecessor of a SyncPoint "/some/path" is "/some"
 *  @param identifier The identifier of the SyncPoint
 *  @return The identifier of the predecessor SyncPoint
 */
std::string
SyncPointManager::find_prefix(const std::string & identifier) const
{
  size_t last_pos = identifier.rfind("/");
  if (last_pos != 0) {
    return identifier.substr(0, last_pos);
  } else {
    return "/";
  }
}

RefPtr<SyncPoint>
SyncPointManager::get_syncpoint_no_lock(const std::string & component, const std::string & identifier)
{
  if (component == "") {
    throw SyncPointInvalidComponentException(component.c_str(), identifier.c_str());
  }
  // insert a new SyncPoint if no SyncPoint with the same identifier exists,
  // otherwise, use that SyncPoint
  std::pair<std::set<RefPtr<SyncPoint> >::iterator,bool> insert_ret;
  insert_ret = syncpoints_.insert(
      RefPtr<SyncPoint>(new SyncPoint(identifier, logger_)));
  std::set<RefPtr<SyncPoint> >::iterator sp_it = insert_ret.first;

  // add component to the set of watchers
  (*sp_it)->add_watcher(component);

  if (identifier != "/") {
    // create prefix SyncPoints.
    // If this is the root SyncPoint ("/"), there will be no prefix
    std::string prefix = find_prefix(identifier);
    RefPtr<SyncPoint> predecessor = get_syncpoint_no_lock(component, prefix);
    predecessor->successors_.insert(*sp_it);
    (*sp_it)->predecessor_ = predecessor;
  }

  return *sp_it;
}


void
SyncPointManager::release_syncpoint_no_lock(const std::string & component,
  RefPtr<SyncPoint> sync_point)
{
  std::set<RefPtr<SyncPoint> >::iterator sp_it = syncpoints_.find(sync_point);
  if (sp_it == syncpoints_.end()) {
    throw SyncPointReleasedDoesNotExistException(component.c_str(),
        sync_point->get_identifier().c_str());
  }
  if (component_watches_any_successor(sync_point, component)) {
    // successor is watched, do not release the syncpoint yet
    return;
  }
  (*sp_it)->unwait(component);
  if (!(*sp_it)->watchers_.erase(component)) {
    throw SyncPointReleasedByNonWatcherException(component.c_str(),
        sync_point->get_identifier().c_str());
  }
  if ((*sp_it)->is_emitter(component) && !(*sp_it)->is_watcher(component)) {
	  throw SyncPointCannotReleaseEmitter(component.c_str(),
	      (*sp_it)->get_identifier().c_str());
  }

  if (sync_point->predecessor_) {
    release_syncpoint_no_lock(component, sync_point->predecessor_);
  }
}

bool
SyncPointManager::component_watches_any_successor(
  const RefPtr<SyncPoint> syncpoint, const std::string component) const
{
  for (std::set<RefPtr<SyncPoint> >::const_iterator it = syncpoint->successors_.begin();
      it != syncpoint->successors_.end();
      it++) {
    if ((*it)->get_watchers().count(component)) {
      return true;
    }
  }
  return false;
}

} // namespace fawkes
