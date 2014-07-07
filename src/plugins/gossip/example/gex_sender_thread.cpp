
/***************************************************************************
 *  gex_sender_thread.cpp - Gossip Example Plugin - Sender
 *
 *  Created: Wed Mar 05 14:29:19 2014
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

#include "gex_sender_thread.h"
#include <plugins/gossip/gossip/gossip_group.h>

#include "TestMessage.pb.h"

using namespace fawkes;

/** @class GossipExampleSenderThread "clips-protobuf-thread.h"
 * Gossip Example Plugin Thread - Sender.
 * @author Tim Niemueller
 */

/** Constructor. */
GossipExampleSenderThread::GossipExampleSenderThread()
  : Thread("GossipExampleSenderThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
    GossipAspect("example")
{
}


/** Destructor. */
GossipExampleSenderThread::~GossipExampleSenderThread()
{
}


void
GossipExampleSenderThread::init()
{
  last_sent_ = new Time(clock);
  counter_   = 0;
}


void
GossipExampleSenderThread::finalize()
{
  delete last_sent_;
}


void
GossipExampleSenderThread::loop()
{
  fawkes::Time now(clock);
  if (now - last_sent_ >= 2.0) {
    *last_sent_ = now;

    logger->log_debug(name(), "Sending");

    gossip_example::TestMessage m;
    m.set_counter(++counter_);
    m.set_sec(now.get_sec());
    m.set_nsec(now.get_nsec());
    gossip_group->broadcast(m);
  }
}
