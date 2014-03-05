/***************************************************************************
 *  gazsim_timesource_source.cpp - Plugin sets the fawkes time
 *                                 to the simulation time
 *
 *  Created: Wed Sep 25 16:11:35 2013
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

#include "gazsim_timesource_source.h"

using namespace fawkes;

/**
 * Constructor
 * @param clock pointer to fawkes clock
 */
GazsimTimesource::GazsimTimesource(Clock* clock)
{
  clock_ = clock;

  last_sim_time_ = get_system_time();
  last_real_time_factor_ = 1.0;
  clock_->get_systime(&last_sys_recv_time_);
  //registration will be done by plugin
}

GazsimTimesource::~GazsimTimesource()
{

}

/**
 * The Simulation time is estimated by looking at the timeinterval to the last received msg and the last real time factor.
 * With this method, I want to reduce the number of send messages from Gazebo
 * @param tv timeinterval
 */
void GazsimTimesource::get_time(timeval* tv) const
{
  //I do not use the Time - operator here because this would recursively call get_time
  timeval now = get_system_time();
  timeval interval = subtract(now, last_sys_recv_time_);  

  //doing this: timeval estimated_sim_interval = interval * last_real_time_factor_;
  timeval estimated_sim_interval;
  estimated_sim_interval.tv_usec = last_real_time_factor_ * (interval.tv_sec * 1000000  + interval.tv_usec);
  estimated_sim_interval.tv_sec = estimated_sim_interval.tv_usec / 1000000;
  estimated_sim_interval.tv_usec -= estimated_sim_interval.tv_sec * 1000000;

  timeval estimated_sim_now = add(last_sim_time_, estimated_sim_interval);

  //return
  *tv =  estimated_sim_now;
}

timeval GazsimTimesource::conv_to_realtime(const timeval* tv) const
{
  timeval interval = subtract(*tv, last_sim_time_);

  //doing this: timeval est_real_interval = interval / last_real_time_factor_;
  timeval est_real_interval;
  est_real_interval.tv_usec = (interval.tv_sec * 1000000  + interval.tv_usec) / last_real_time_factor_;
  est_real_interval.tv_sec = est_real_interval.tv_usec / 1000000;
  est_real_interval.tv_usec -= est_real_interval.tv_sec * 1000000;

  timeval result = add(last_sys_recv_time_, est_real_interval);
  return result;
}

/** store data from gazebo time message
 * @param msg message
 */
void GazsimTimesource::on_time_sync_msg(ConstSimTimePtr &msg)
{
  //we do not want to correct time back
  get_time(&last_sim_time_);
  last_real_time_factor_ = msg->real_time_factor();
  clock_->get_systime(&last_sys_recv_time_);
}

timeval GazsimTimesource::get_system_time() const
{
  timeval now_timeval;
  gettimeofday(&now_timeval,NULL);
  return now_timeval;
}

timeval GazsimTimesource::add(timeval a, timeval b) const
{
  timeval res;
  res.tv_sec = a.tv_sec + b.tv_sec;
  res.tv_usec = a.tv_usec + b.tv_usec;
  if(res.tv_usec > 1000000)
  {
    res.tv_usec -= 1000000;
    res.tv_sec++;
  }
  return res;
}

timeval GazsimTimesource::subtract(timeval a, timeval b) const
{
  timeval res;
  res.tv_sec = a.tv_sec - b.tv_sec;
  if(a.tv_usec >= b.tv_usec)
  {
    res.tv_usec = a.tv_usec - b.tv_usec;
  }
  else
  {
    res.tv_usec = 1000000 + a.tv_usec - b.tv_usec;
    res.tv_sec--;
  }
  return res;
}
