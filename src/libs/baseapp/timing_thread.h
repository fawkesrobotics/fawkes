/***************************************************************************
 *  timing_thread.h - Timing thread to achieve a desired main loop time
 *
 *  Created: Thu Jul 23 14:45:42 2015
 *  Copyright  2015-2017  Till Hofmann
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


#ifndef __LIBS_BASEAPP_TIMING_THREAD_H_
#define __LIBS_BASEAPP_TIMING_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/syncpoint_manager.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>

#include <utils/time/clock.h>


namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FawkesTimingThread
: public Thread,
  public SyncPointManagerAspect,
  public ConfigurableAspect,
//  public ConfigurationChangeHandler,
  public LoggingAspect
{
  public:
    FawkesTimingThread();

    virtual void init();
    virtual void loop();
    virtual void finalize();

  private:
    Clock *__clock;
    Time *__loop_start;
    Time *__loop_end;
    float __desired_loop_time_sec;
    uint __desired_loop_time_usec;
    float __min_loop_time_sec;
    uint __min_loop_time_usec;
    bool __enable_looptime_warnings;

    RefPtr<SyncPoint> __syncpoint_loop_start;
    RefPtr<SyncPoint> __syncpoint_loop_end;
};


} // namespace fawkes

#endif // __LIBS_BASEAPP_TIMING_THREAD_H_
