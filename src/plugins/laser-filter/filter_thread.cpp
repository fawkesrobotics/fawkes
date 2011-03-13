
/***************************************************************************
 *  filter_thread.cpp - Thread that filters data in blackboard
 *
 *  Created: Sun Mar 13 01:12:53 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "filter_thread.h"
#include "filters/max_circle.h"
#include "filters/720to360.h"
#include "filters/deadspots.h"
#include "filters/cascade.h"
#include "filters/reverse_angle.h"

#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>

using namespace fawkes;

/** @class LaserFilterThread "filter_thread.h"
 * Laser filter thread.
 * This thread integrates into the Fawkes main loop at the sensor processing
 * hook, reads data from specified interfaces, filters it with a given
 * cascade, and then writes it back to an interface.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 */
LaserFilterThread::LaserFilterThread(std::string &cfg_name,
				     std::string &cfg_prefix)
  : Thread("LaserFilterThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
  set_name("LaserFilterThread(%s)", cfg_name.c_str());
  __cfg_name   = cfg_name;
  __cfg_prefix = cfg_prefix;
}


void
LaserFilterThread::init()
{
  __laser360_if = NULL;
  __laser720_if = NULL;

  __filters = new LaserDataFilterCascade();
}


void
LaserFilterThread::finalize()
{
  delete __filters;
  blackboard->close(__laser360_if);
  blackboard->close(__laser720_if);
}

void
LaserFilterThread::loop()
{
}
