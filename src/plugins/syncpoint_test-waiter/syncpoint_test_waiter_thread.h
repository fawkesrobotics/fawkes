/***************************************************************************
 *  syncpoint_test_waiter_thread.h - SyncPoint Test Waiter Plugin
 *
 *   Created on Wed Jan 08 17:12:13 2014
 *   Copyright  2014  Till Hofmann
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


#ifndef __PLUGINS_SYNCPOINT_TEST_WAITER_SYNCPOINT_TEST_WAITER_THREAD_H_
#define __PLUGINS_SYNCPOINT_TEST_WAITER_SYNCPOINT_TEST_WAITER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/syncpoint_manager.h>
#include <core/utils/refptr.h>

namespace fawkes {
  class Position3DInterface;
}

class SyncPointTestWaiterThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
 // public fawkes::BlockedTimingAspect,
  public fawkes::SyncPointManagerAspect
{
 public:
  SyncPointTestWaiterThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();


 private:
  fawkes::RefPtr<fawkes::SyncPoint> syncpoint_;
  uint loopcount_;
};

#endif
