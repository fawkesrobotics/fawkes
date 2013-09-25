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


GazsimTimesource::GazsimTimesource()
{
  clock = Clock::instance();

  last_sim_time_ = get_system_time();
  last_real_time_factor_ = 1.0;
  clock->get_systime(last_sys_recv_time_);
  //registration will be done by plugin
}

GazsimTimesource::~GazsimTimesource()
{

}

/**
 * The Simulation time is estimated by looking at the timeinterval to the last received msg and the last real time factor.
 * With this method, I want to reduce the number of send messages from Gazebo
 */
void GazsimTimesource::get_time(timeval* tv) const
{
  double now = get_system_time();
  double interval = now - last_sys_recv_time_.in_sec();
  double estimated_sim_interval = interval * last_real_time_factor_;
  double estimated_sim_now = last_sim_time_ + estimated_sim_interval;
  Time result = estimated_sim_now;


  //return
  *tv =  *(result.get_timeval());
}

timeval GazsimTimesource::conv_to_realtime(const timeval* tv) const
{
  Time asked_sim(tv);
  double interval = asked_sim.in_sec() - last_sim_time_;
  double est_real_interval = interval / last_real_time_factor_;
  timeval result = *((last_sys_recv_time_ + est_real_interval).get_timeval());
  return result;
}

void GazsimTimesource::on_time_sync_msg(ConstTimeSyncPtr &msg)
{
  //we do not want to correct time back
  timeval sim_time;
  get_time(&sim_time);
  last_sim_time_ = ((double)sim_time.tv_sec) + 0.000001 * sim_time.tv_usec;
  last_real_time_factor_ = msg->real_time_factor();
  clock->get_systime(last_sys_recv_time_);
}

double GazsimTimesource::get_system_time() const
{
  timeval now_timeval;
  gettimeofday(&now_timeval,NULL);
  return ((double)now_timeval.tv_sec) + 0.000001 * now_timeval.tv_usec;
}
