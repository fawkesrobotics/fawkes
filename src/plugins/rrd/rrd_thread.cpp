
/***************************************************************************
 *  rrd_thread.cpp - RRD Thread
 *
 *  Created: Fri Dec 17 00:32:57 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include "rrd_thread.h"

using namespace fawkes;

/** @class RRDThread "rrd_thread.h"
 * RRD Thread.
 * This thread maintains an active connection to RRD and provides an
 * aspect to access RRD to make it convenient for other threads to use
 * RRD.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
RRDThread::RRDThread()
  : Thread("RRDThread", Thread::OPMODE_WAITFORWAKEUP),
    AspectProviderAspect("RRDAspect", &__rrd_aspect_inifin),
    __rrd_aspect_inifin(this)
{
}


/** Destructor. */
RRDThread::~RRDThread()
{
}


void
RRDThread::init()
{
}


void
RRDThread::finalize()
{
}


void
RRDThread::loop()
{
}
