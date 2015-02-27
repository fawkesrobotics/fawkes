/***************************************************************************
 *  syncpoint.h - SyncPoint Aspect
 *
 *  Created: Thu Feb 19 14:31:42 2015
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

#ifndef __ASPECT_SYNCPOINT_H_
#define __ASPECT_SYNCPOINT_H_

#include <aspect/aspect.h>
#include <syncpoint/syncpoint.h>
#include <syncpoint/syncpoint_manager.h>
#include <core/threading/thread_loop_listener.h>

#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SyncPointAspect : public virtual Aspect, public ThreadLoopListener
{
  public:
    SyncPointAspect(SyncPoint::WakeupType type_in, std::string identifier_in, std::string identifier_out = "");
    SyncPointAspect(std::string out_identifier);
    virtual ~SyncPointAspect();

    void init_SyncPointAspect(Thread *thread, SyncPointManager *syncpoint_manager);
    void finalize_SyncPointAspect(Thread *thread, SyncPointManager *syncpoint_manager);

    void pre_loop(Thread *thread);
    void post_loop(Thread *thread);

  private:
    SyncPoint::WakeupType type_in_;
    std::string identifier_in_;
    std::string identifier_out_;
    bool has_input_syncpoint_;
    bool has_output_syncpoint_;
    RefPtr<SyncPoint> sp_in_;
    RefPtr<SyncPoint> sp_out_;
};

} // end namespace fawkes

#endif
