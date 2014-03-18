/***************************************************************************
 *  gazsim_timesource_source.h - Plugin sets the fawkes time
 *                                 to the simulation time
 *
 *  Created: Wed Sep 25 16:02:54 2013
 *  Copyright  2013  Frederik Zwilling
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

#include <boost/asio.hpp>
#include <utils/time/timesource.h>
#include <utils/time/clock.h>

#include "../msgs/SimTime.pb.h"

#ifndef __GAZEBO_TIMESOURCE_H_
#define __GAZEBO_TIMESOURCE_H_


typedef const boost::shared_ptr<gazsim_msgs::SimTime const> ConstSimTimePtr;

namespace fawkes
{
  /** @class GazsimTimesource
   * This time-source provides the simulation time from Gazebo in Fawkes
   * @author Frederik Zwilling
   */
  class GazsimTimesource : public TimeSource
  {
  public:
    //Constructor
    GazsimTimesource(Clock* clock);
    ///Destructor
   ~GazsimTimesource();

    virtual void get_time(timeval* tv) const;
    virtual timeval conv_to_realtime(const timeval* tv) const;
    virtual timeval conv_native_to_exttime(const timeval* tv) const;

    /// store data from gazebo time message
    void on_time_sync_msg(ConstSimTimePtr &msg);

  private:
    timeval get_system_time() const;
    timeval add(timeval a, timeval b) const;
    timeval subtract(timeval a, timeval b) const;

  private:
    Clock* clock_;

    //from last msg all in sec
    timeval last_sim_time_;
    timeval last_sys_recv_time_;
    double  last_real_time_factor_;
    timeval last_native_sim_time_;

  };

}

#endif
