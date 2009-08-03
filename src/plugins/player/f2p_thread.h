
/***************************************************************************
 *  f2p_thread.h - Thread that calls mappers to sync Fawkes to Player
 *
 *  Created: Tue Sep 30 13:11:46 2008
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

#ifndef __PLUGINS_PLAYER_F2P_THREAD_H_
#define __PLUGINS_PLAYER_F2P_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>

class PlayerClientThread;

class PlayerF2PThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect
{
 public:
  PlayerF2PThread(PlayerClientThread *client_thread);

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  PlayerClientThread *__client_thread;
};


#endif
