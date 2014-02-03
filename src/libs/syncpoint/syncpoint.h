/***************************************************************************
 *  syncpoint.h - Fawkes SyncPoint
 *
 *  Created: Thu Jan 09 12:22:03 2014
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

#ifndef __SYNCPOINT_SYNCPOINT_H_
#define __SYNCPOINT_SYNCPOINT_H_

#include <interface/interface.h>
#include <syncpoint/syncpoint_manager.h>
#include <core/threading/wait_condition.h>

#include <set>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SyncPointManager;

class SyncPoint
{
  public:
    SyncPoint(const char * identifier);
    virtual ~SyncPoint();

    /** send a signal to all waiting threads */
    virtual void emit(const char * component);

    /** wait for the sync point to be emitted */
    virtual void wait(const char * component);

    const char * get_identifier() const;
    bool operator==(const SyncPoint & other) const;
    bool operator<(const SyncPoint & other) const;

    /**
     * allow Syncpoint Manager to edit
     */
    friend SyncPointManager;

  private:
    const char * identifier_;
    std::set<const char *> watchers;

    WaitCondition wait_condition;
};

} // end namespace fawkes

#endif
