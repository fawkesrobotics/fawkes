
/***************************************************************************
 *  postsync_thread.h - Thread to synchronize loops, PRE_LOOP hook
 *
 *  Created: Tue Sep 30 14:32:25 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PLAYER_POSTSYNC_THREAD_H_
#define __PLUGINS_PLAYER_POSTSYNC_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>

class PlayerPostSyncThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect
{
 public:
  PlayerPostSyncThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }
};


#endif
