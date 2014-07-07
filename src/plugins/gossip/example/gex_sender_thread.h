
/***************************************************************************
 *  gex_sender_thread.h - Gossip Example Plugin - Sender
 *
 *  Created: Wed Mar 05 14:27:35 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_GOSSIP_EXAMPLE_GEX_SENDER_THREAD_H_
#define __PLUGINS_GOSSIP_EXAMPLE_GEX_SENDER_THREAD_H_

#include <plugins/gossip/aspect/gossip.h>

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/blocked_timing.h>

namespace fawkes {
  class Time;
}

class GossipExampleSenderThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GossipAspect
{
 public:
  GossipExampleSenderThread();
  virtual ~GossipExampleSenderThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::Time *last_sent_;
  unsigned int  counter_;
};

#endif
