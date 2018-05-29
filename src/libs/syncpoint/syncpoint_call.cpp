/***************************************************************************
 *  syncpoint_call.cpp - Utility class to represent a call to a SyncPoint
 *
 *  Created: Fri Aug 15 18:12:42 2014
 *  Copyright  2014  Till Hofmann
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

#include <syncpoint/syncpoint_call.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SyncPointCall <syncpoint/syncpoint.h>
 * A call (wait() or emit()) to a SyncPoint.
 * @author Till Hofmann
 * @see SyncPoint
 */

/** Constructor.
 * @param call_time Time at which the SyncPoint was called
 * @param caller The calling component
 * @param wait_time The time the caller had to wait for the SyncPoint (wait calls)
 */
SyncPointCall::SyncPointCall(const std::string & caller, Time call_time, Time wait_time)
  : caller_(caller),
    call_time_(call_time),
    wait_time_(wait_time)
{}

/** Get the time when the call was made
 * @return the call time
 */
Time
SyncPointCall::get_call_time() const
{
  return call_time_;
}

/** Get the wait time
 * @return the wait time
 */
Time
SyncPointCall::get_wait_time() const
{
  return wait_time_;
}

/** Get the name of the component which made the call
 * @return the component name
 */
std::string
SyncPointCall::get_caller() const
{
  return caller_;
}


} // namespace fawkes
