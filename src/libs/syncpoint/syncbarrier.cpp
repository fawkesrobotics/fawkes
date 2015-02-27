/***************************************************************************
 *  syncbarrier.cpp - Fawkes SyncBarrier
 *
 *  Created: Fri Jan 16 14:02:42 2015
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

#include <syncpoint/syncpoint.h>
#include <syncpoint/syncbarrier.h>

#include <string.h>

using namespace std;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class SyncBarrier <syncpoint/syncbarrier.h>
 * The SyncBarrier class.
 * This class is used for dynamic synchronization of threads which depend
 * on each other, e.g. threads which are part of a processing chain.
 * Similar to a SyncPoint, components use a SyncBarrier either as an emitter or
 * as a waiter. Emitters first have to register with the barrier in order to be
 * allowed to emit it. With SyncBarriers, a waiter blocks until all registered
 * emitters have emitted the barrier. A SyncBarrier runs in iterations: Once an
 * emitter has emitted the barrier, it is no longer considered to be pending,
 * i.e. a waiter will not wait for the emitter until the SyncBarrier is reset.
 * A reset causes all registered emitters to be pending again.
 *
 * As an example, thread E and F generate data which is needed by thread W.
 * Therefore, all three threads share a SyncBarrier.
 * Thread W wait()s for the SyncBarrier to be emitted.
 * Once threads E and F are done, they emit() the SyncBarrier. If the barrier
 * has been emitted by both threads, thread W unblocks and continues to process
 * the data.
 *
 * @author Till Hofmann
 * @see SyncPointManager
 */

/** Constructor.
 * @param identifier The identifier of the SyncBarrier. This must be in absolute
 * path style, e.g. '/some/barrier'.
 */
SyncBarrier::SyncBarrier(string identifier) :
    SyncPoint(identifier)
{

}

/** Wait for the SyncBarrier.
 * This waits until all registered emitters of the SyncBarrier have emitted the barrier.
 */
void
SyncBarrier::wait(const std::string & component)
{
  SyncPoint::wait(component, SyncPoint::WAIT_FOR_ALL);
}

} // namespace fawkes
