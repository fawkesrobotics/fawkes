
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
#include "circle_filter.h"

#include <interfaces/Laser360Interface.h>

using namespace fawkes;

/** @class LaserSensorThread "playerc_thread.h"
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
  __laser_if = blackboard->open_for_writing<Laser360Interface>("Laser");
}


void
LaserSensorThread::finalize()
{
  blackboard->close(__laser_if);
  delete __filter;
}

void
LaserSensorThread::loop()
{
  if ( __aqt->lock_if_new_data() ) {
    if ( __filter ) {
      __filter->filter(__aqt->get_distance_data(), __aqt->get_distance_data_size());
      __laser_if->set_distances(__filter->filtered_data());
    } else {
      __laser_if->set_distances(__aqt->get_distance_data());
    }
    __laser_if->write();
    __aqt->unlock();
  }
}
