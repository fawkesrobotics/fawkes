/***************************************************************************
 *  test_thread.h - Thread to test SyncPoints
 *
 *  Created: Thu Mar 05 15:15:42 2015
 *  Copyright  2015  Till Hofmann
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

#ifndef __PLUGINS_SYNCPOINT_TEST_THREAD_H_
#define __PLUGINS_SYNCPOINT_TEST_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/blocked_timing.h>

class SyncPointTestThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::BlockedTimingAspect
{
 public:
    SyncPointTestThread(const char * name, BlockedTimingAspect::WakeupHook hook);

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  BlockedTimingAspect::WakeupHook hook_;
};

#endif
