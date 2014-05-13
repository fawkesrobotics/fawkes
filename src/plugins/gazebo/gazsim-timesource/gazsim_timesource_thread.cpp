/***************************************************************************
 *  gazsim_timesource_plugin.cpp - Plugin sets the fawkes time
 *                                 to the simulation time
 *
 *  Created: Sat Sep 21 20:56:29 2013
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

#include "gazsim_timesource_thread.h"

using namespace fawkes;

/** @class GazsimTimesourceThread "clips_thread.h"
 * Plugin provides the simulation time from gazebo
 * @author Frederik Zwilling
 */

GazsimTimesourceThread::GazsimTimesourceThread()
  : Thread("GazsimTimesourceThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

GazsimTimesourceThread::~GazsimTimesourceThread()
{
}


void GazsimTimesourceThread::init()
{
  logger->log_info(name(), "GazsimTimesource initializing");

  //Create Subscriber
  time_sync_sub_ = gazebo_world_node->Subscribe(config->get_string("/gazsim/topics/time"), &GazsimTimesourceThread::on_time_sync_msg, this);

  //Create Time Source
  time_source_ = new GazsimTimesource(clock);

  //register timesource and make it default
  clock->register_ext_timesource(time_source_, true);
}


void GazsimTimesourceThread::finalize()
{
  //remove time source
  clock->remove_ext_timesource(time_source_);
  delete time_source_;
}


void GazsimTimesourceThread::loop()
{
  //nothing interesting
}

void GazsimTimesourceThread::on_time_sync_msg(ConstSimTimePtr &msg)
{
  // logger->log_info(name(), "Got Simulation Time");
  
  //provide time source with newest message
  time_source_->on_time_sync_msg(msg);
}
