
/***************************************************************************
 *  sensor_thread.cpp - Laser thread that puses data into the interface
 *
 *  Created: Wed Oct 08 13:32:57 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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
 * @param aqt LaserAcquisitionThread to get data from
 */
LaserSensorThread::LaserSensorThread(LaserAcquisitionThread *aqt)
  : Thread("LaserSensorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR)
{
  __aqt    = aqt;
  __filter = NULL;
}


void
LaserSensorThread::init()
{
  __laser360_if = NULL;
  __laser720_if = NULL;
  __720to360filter = NULL;

  __aqt->pre_init(config, logger);

  __num_values = __aqt->get_distance_data_size();

  if (__num_values == 360) {
    __laser360_if = blackboard->open_for_writing<Laser360Interface>("Laser");
  } else if (__num_values == 720){
    __laser360_if = blackboard->open_for_writing<Laser360Interface>("Laser");
    __laser720_if = blackboard->open_for_writing<Laser720Interface>("Laser");
    __720to360filter = new Laser720to360DataFilter();
  } else {
    throw Exception("Laser acquisition thread must produce either 360 or 720 "
		    "distance values, but it produces %u", __aqt->get_distance_data_size());
  }
}


void
LaserSensorThread::finalize()
{
  blackboard->close(__laser360_if);
  blackboard->close(__laser720_if);
  delete __720to360filter;
  delete __filter;
}

void
LaserSensorThread::loop()
{
  if ( __aqt->lock_if_new_data() ) {
    if ( __filter ) {
      __filter->filter(__aqt->get_distance_data(), __aqt->get_distance_data_size());
      if (__num_values == 360) {
	__laser360_if->set_distances(__filter->filtered_data());
      } else if (__num_values == 720) {
	__laser720_if->set_distances(__filter->filtered_data());
      }
    } else {
      if (__num_values == 360) {
	__laser360_if->set_distances(__aqt->get_distance_data());
      } else if (__num_values == 720) {
	__laser720_if->set_distances(__aqt->get_distance_data());
	__720to360filter->filter(__aqt->get_distance_data(), __aqt->get_distance_data_size());
	__laser360_if->set_distances(__720to360filter->filtered_data());
      }
    }
    __laser360_if->write();
    if (__laser720_if)  __laser720_if->write();
    __aqt->unlock();
  }
}
