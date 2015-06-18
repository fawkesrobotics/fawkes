
/***************************************************************************
 *  sensor_thread.cpp - Laser thread that puses data into the interface
 *
 *  Created: Wed Oct 08 13:32:57 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "sensor_thread.h"
#include "acquisition_thread.h"

#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>
#include <interfaces/Laser1080Interface.h>

using namespace fawkes;

/** @class LaserSensorThread "sensor_thread.h"
 * Laser sensor thread.
 * This thread integrates into the Fawkes main loop at the sensor hook and
 * publishes new data when available from the LaserAcquisitionThread.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 * @param aqt LaserAcquisitionThread to get data from
 */
LaserSensorThread::LaserSensorThread(std::string &cfg_name,
				     std::string &cfg_prefix,
				     LaserAcquisitionThread *aqt)
  : Thread("LaserSensorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
  set_name("LaserSensorThread(%s)", cfg_name.c_str());
  __aqt        = aqt;
  __cfg_name   = cfg_name;
  __cfg_prefix = cfg_prefix;
}


void
LaserSensorThread::init()
{
  __laser360_if = NULL;
  __laser720_if = NULL;
  __laser1080_if = NULL;

  bool main_sensor  = false;

  __cfg_frame = config->get_string((__cfg_prefix + "frame").c_str());

  try {
    main_sensor = config->get_bool((__cfg_prefix + "main_sensor").c_str());
  } catch (Exception &e) {} // ignored, assume no

  __aqt->pre_init(config, logger);

  __num_values = __aqt->get_distance_data_size();

  std::string if_id = main_sensor ? "Laser" : ("Laser " + __cfg_name);

  if (__num_values == 360) {
    __laser360_if = blackboard->open_for_writing<Laser360Interface>(if_id.c_str());
    __laser360_if->set_auto_timestamping(false);
    __laser360_if->set_frame(__cfg_frame.c_str());
    __laser360_if->write();
  } else if (__num_values == 720){
    __laser720_if = blackboard->open_for_writing<Laser720Interface>(if_id.c_str());
    __laser720_if->set_auto_timestamping(false);
    __laser720_if->set_frame(__cfg_frame.c_str());
    __laser720_if->write();
  } else if (__num_values == 1080){
    __laser1080_if = blackboard->open_for_writing<Laser1080Interface>(if_id.c_str());
    __laser1080_if->set_auto_timestamping(false);
    __laser1080_if->set_frame(__cfg_frame.c_str());
    __laser1080_if->write();
  } else {
    throw Exception("Laser acquisition thread must produce either 360, 720, or 1080 "
		    "distance values, but it produces %u", __aqt->get_distance_data_size());
  }

}


void
LaserSensorThread::finalize()
{
  blackboard->close(__laser360_if);
  blackboard->close(__laser720_if);
  blackboard->close(__laser1080_if);
}

void
LaserSensorThread::loop()
{
  if ( __aqt->lock_if_new_data() ) {
    if (__num_values == 360) {
      __laser360_if->set_timestamp(__aqt->get_timestamp());
      __laser360_if->set_distances(__aqt->get_distance_data());
      __laser360_if->write();
    } else if (__num_values == 720) {
      __laser720_if->set_timestamp(__aqt->get_timestamp());
      __laser720_if->set_distances(__aqt->get_distance_data());
      __laser720_if->write();
    } else if (__num_values == 1080) {
      __laser1080_if->set_timestamp(__aqt->get_timestamp());
      __laser1080_if->set_distances(__aqt->get_distance_data());
      __laser1080_if->write();
    }
    __aqt->unlock();
  }
}
