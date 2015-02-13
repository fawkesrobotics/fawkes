/***************************************************************************
 *  syncbarrier.h - Fawkes SyncBarrier
 *
 *  Created: Fri Jan 16 14:02:42 2015
 *  Copyright  2015  Till Hofmann
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

#ifndef __SYNCPOINT_SYNCBARRIER_H_
#define __SYNCPOINT_SYNCBARRIER_H_

#include <syncpoint/syncpoint.h>

#include <string>
#include <set>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


class SyncBarrier : public SyncPoint
{
  public:
    SyncBarrier(std::string identifier);

    /** register as emitter */
    virtual void register_emitter(const std::string & component);

    /** unregister as emitter */
    virtual void unregister_emitter(const std::string & component);

    /** send a signal to all waiting threads */
    virtual void emit(const std::string & component);

    /** wait for the barrier */
    virtual void wait(const std::string & component);

    /** check whether the barrier is already emitted */
    virtual bool is_emitted() const;

    /** set all registered emitter as pending */
    void reset_emitters();

    /**
     * allow Syncpoint Manager to edit
     */
    friend class SyncPointManager;

  private:
    void reset_emitters_();

  private:
    std::set<std::string> emitters_;
    std::set<std::string> pending_emitters_;

    RefPtr<SyncBarrier> predecessor_;
};

} // end namespace fawkes

#endif
