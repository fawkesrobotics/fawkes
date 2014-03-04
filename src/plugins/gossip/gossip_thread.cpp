
/***************************************************************************
 *  gossip_thread.cpp -  Robot Group Communication Plugin
 *
 *  Created: Fri Feb 28 11:11:20 2014
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

#include "gossip_thread.h"

using namespace fawkes;

/** @class GossipThread "clips-protobuf-thread.h"
 * Robot Group Communication.
 * @author Tim Niemueller
 */

/** Constructor. */
GossipThread::GossipThread()
  : Thread("GossipThread", Thread::OPMODE_WAITFORWAKEUP),
    AspectProviderAspect(&gossip_aspect_inifin_)
{
}


/** Destructor. */
GossipThread::~GossipThread()
{
}


void
GossipThread::init()
{
}


void
GossipThread::finalize()
{
}


void
GossipThread::loop()
{
}
