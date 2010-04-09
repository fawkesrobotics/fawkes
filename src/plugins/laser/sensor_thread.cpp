
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
#include "filters/circle.h"
#include "filters/720to360.h"
#include "filters/deadspots.h"
#include "filters/cascade.h"
#include "filters/reverse_angle.h"

#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>

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
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR)
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

  bool spots_filter = false;
  bool main_sensor  = false;
  __clockwise_angle = false;

  try {
    spots_filter = config->get_bool((__cfg_prefix + "use_dead_spots_filter").c_str());
  } catch (Exception &e) {} // ignored, assume no
  try {
    main_sensor = config->get_bool((__cfg_prefix + "main_sensor").c_str());
  } catch (Exception &e) {} // ignored, assume no
  try {
    __clockwise_angle = config->get_bool((__cfg_prefix + "clockwise_angle").c_str());
  } catch (Exception &e) {} // ignored, assume no

  __aqt->pre_init(config, logger);

  __num_values = __aqt->get_distance_data_size();

  __filters360 = new LaserDataFilterCascade();
  __filters720 = new LaserDataFilterCascade();

  std::string if_id = main_sensor ? "Laser" : ("Laser " + __cfg_name);

  if (__num_values == 360) {
    __laser360_if = blackboard->open_for_writing<Laser360Interface>(if_id.c_str());
  } else if (__num_values == 720){
    __laser360_if = blackboard->open_for_writing<Laser360Interface>(if_id.c_str());
    __laser720_if = blackboard->open_for_writing<Laser720Interface>(if_id.c_str());
    __filters360->add_filter(new Laser720to360DataFilter());
  } else {
    throw Exception("Laser acquisition thread must produce either 360 or 720 "
		    "distance values, but it produces %u", __aqt->get_distance_data_size());
  }

  if (__clockwise_angle) {
    logger->log_debug(name(), "Setting up reverse angle filter for 360° interface");
    std::string rev_id = if_id + " CW";
    try {
      __reverse360_if = blackboard->open_for_writing<Laser360Interface>(rev_id.c_str());
      __reverse360_if->set_clockwise_angle(true);
      __reverse360_if->write();
      __reverse360 = new LaserReverseAngleDataFilter(360);
    } catch (Exception &e) {
      blackboard->close(__laser360_if);
      blackboard->close(__laser720_if);
      throw;
    }

    if (__num_values == 720) {
      logger->log_debug(name(), "Setting up dead spots filter for 720° interface");
      try {
	__reverse720_if = blackboard->open_for_writing<Laser720Interface>(rev_id.c_str());
	__reverse720_if->set_clockwise_angle(true);
	__reverse720_if->write();
	__reverse720 = new LaserReverseAngleDataFilter(720);
      } catch (Exception &e) {
	blackboard->close(__laser360_if);
	blackboard->close(__laser720_if);
	blackboard->close(__reverse360_if);
	delete __reverse360;
	throw;
      }
    }
  }

  if (spots_filter) {
    std::string spots_prefix = __cfg_prefix + "dead_spots/";
    logger->log_debug(name(), "Setting up dead spots filter for 360° interface");
    __filters360->add_filter(new LaserDeadSpotsDataFilter(config, logger, spots_prefix));
    logger->log_debug(name(), "Setting up dead spots filter for 720° interface");
    __filters720->add_filter(new LaserDeadSpotsDataFilter(config, logger, spots_prefix));
  }

}


void
LaserSensorThread::finalize()
{
  delete __filters360;
  delete __filters720;
  blackboard->close(__laser360_if);
  blackboard->close(__laser720_if);
}

void
LaserSensorThread::loop()
{
  if ( __aqt->lock_if_new_data() ) {
    if (__num_values == 360) {
      if (__filters360->has_filters()) {
	__filters360->filter(__aqt->get_distance_data(), __aqt->get_distance_data_size());
	__laser360_if->set_distances(__filters360->filtered_data());
      } else {
	__laser360_if->set_distances(__aqt->get_distance_data());
      }

      // We also provide the clockwise output
      if (__clockwise_angle) {
	__reverse360->filter(__laser360_if->distances(), 360);
	__reverse360_if->set_distances(__reverse360->filtered_data());
	__reverse360_if->write();
      }
    } else if (__num_values == 720) {
      if (__filters720->has_filters()) {
	__filters720->filter(__aqt->get_distance_data(), __aqt->get_distance_data_size());
	__laser720_if->set_distances(__filters720->filtered_data());
      } else {
	__laser720_if->set_distances(__aqt->get_distance_data());
      }
      __filters360->filter(__aqt->get_distance_data(), __aqt->get_distance_data_size());
      __laser360_if->set_distances(__filters360->filtered_data());

      // We also provide the clockwise output
      if (__clockwise_angle) {
	__reverse360->filter(__laser360_if->distances(), 360);
	__reverse360_if->set_distances(__reverse360->filtered_data());
	__reverse360_if->write();
	__reverse720->filter(__laser720_if->distances(), 720);
	__reverse720_if->set_distances(__reverse720->filtered_data());
	__reverse720_if->write();
      }
    }
    __laser360_if->write();
    if (__laser720_if)  __laser720_if->write();
    __aqt->unlock();
  }
}
