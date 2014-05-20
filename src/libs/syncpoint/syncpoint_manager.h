/***************************************************************************
 *  syncpoint_manager.h - Fawkes SyncPointManager
 *
 *  Created: Thu Jan 09 15:17:03 2014
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

#ifndef __SYNCPOINT_SYNCPOINT_MANAGER_H_
#define __SYNCPOINT_SYNCPOINT_MANAGER_H_

#include <set>

#include <syncpoint/syncpoint.h>
#include <core/utils/refptr.h>
#include <core/threading/mutex.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SyncPoint;

class SyncPointSetLessThan {
  public:
    bool operator()(const RefPtr<SyncPoint> sp1, const RefPtr<SyncPoint> sp2) const;
};

class SyncPointManager
{
  public:
    SyncPointManager();
    virtual ~SyncPointManager();

    RefPtr<SyncPoint> get_syncpoint(const char * component, const char * identifier);
    void release_syncpoint(const char * component, RefPtr<SyncPoint> syncpoint);

    std::set<RefPtr<SyncPoint>, SyncPointSetLessThan > get_syncpoints();

  protected:
    /**
     * Set of all existing SyncPoints
     */
    std::set<RefPtr<SyncPoint>, SyncPointSetLessThan > syncpoints_;
    /** Mutex used for all SyncPointManager calls */
    Mutex *mutex;


};

} // end namespace fawkes

#endif
