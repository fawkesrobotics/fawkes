/***************************************************************************
 *  syncpoint_call_stats.h - Utility class to keep track of SP call stats
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

#ifndef _SYNCPOINT_SYNCPOINT_CALL_STATS_H_
#define _SYNCPOINT_SYNCPOINT_CALL_STATS_H_

#include <utils/time/time.h>

#include <syncpoint/syncpoint_call.h>

namespace fawkes {

class SyncPointCallStats {
  public:
    SyncPointCallStats();

    void update_calls(const Time& new_call, const Time& wait_time = Time(0.f));
    void update_calls(const SyncPointCall& call);
    Time get_first_call() const;
    Time get_last_call() const;
    float get_call_frequency() const;
    float get_waittime_average() const;
    unsigned int get_num_calls() const;

  private:
    Time first_call_;
    Time last_call_;
    Time total_wait_time_;
    unsigned int num_calls_;
};

} // namespace fawkes

#endif
