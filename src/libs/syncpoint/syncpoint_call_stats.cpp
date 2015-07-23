/***************************************************************************
 *  syncpoint_call_stats.cpp - Utility class to keep track of SP call stats
 *
 *  Created: Fri Aug 15 16:17:42 2014
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

#include <syncpoint/syncpoint_call_stats.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SyncPointCallStats <syncpoint/syncpoint_call_stats.h>
 * This class represents call stats of a single component to a single SyncPoint.
 * It keeps track of the first and last call and computes the call frequency.
 *
 * @see SyncPoint
 * @see SyncPointCall
 *
 */

/** Constructor. */
SyncPointCallStats::SyncPointCallStats()
: first_call_(TIME_MAX),
  last_call_(TIME_MIN),
  total_wait_time_(Time(0.f)),
  num_calls_(0)
{}

/** Add a call to the stats.
 * Update the first and last call and increment the call counter
 * @param new_call the time of the call
 * @param wait_time the time the caller had to wait, 0 for emit()
 */
void
SyncPointCallStats::update_calls(Time new_call, Time wait_time)
{
  num_calls_++;
  total_wait_time_ += wait_time;
  if (new_call < first_call_) {
    first_call_ = new_call;
  }
  if (new_call > last_call_) {
    last_call_ = new_call;
  }
}

/** Add a call to the stats.
 * @param call the new call
 */
void
SyncPointCallStats::update_calls(SyncPointCall call)
{
  update_calls(call.get_call_time(), call.get_wait_time());
}

/** Get the first call to the SyncPoint by the component
 * @return The time of the first call
 */
Time
SyncPointCallStats::get_first_call() const
{
  return first_call_;
}

/** Get the last call to the SyncPoint by the component
 * @return The time of the last call
 */
Time
SyncPointCallStats::get_last_call() const
{
  return last_call_;
}

/** Get the call frequency. This is calculated using the first and last call
 * and the number of calls
 * @return the call frequency
 */
float
SyncPointCallStats::get_call_frequency() const
{
  if (num_calls_ <= 1) {
    return 0.f;
  }
  return num_calls_ / (last_call_.in_sec() - first_call_.in_sec());
}

/** Get the average wait time. For emit calls, this is 0.
 * @return average wait time
 */
float
SyncPointCallStats::get_waittime_average() const
{
  return total_wait_time_.in_sec() / num_calls_;
}

/** Get total number of calls.
 * @return the total number of calls
 */
unsigned int
SyncPointCallStats::get_num_calls() const
{
  return num_calls_;
}

} // namespace fawkes
