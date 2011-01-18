
/***************************************************************************
 *  rrd_thread.cpp - RRD Thread
 *
 *  Created: Fri Dec 17 00:32:57 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include "rrd_example_thread.h"

#include <core/exceptions/system.h>
#include <utils/misc/string_conversions.h>
#include <utils/system/file.h>
#include <plugins/rrd/aspect/rrd_manager.h>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <rrd.h>

using namespace fawkes;

/** @class RRDExampleThread "rrd_example_thread.h"
 * RRD Example Thread.
 * This thread creates a simple RRD and stores random values.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
RRDExampleThread::RRDExampleThread()
  : Thread("RRDExampleThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}


/** Destructor. */
RRDExampleThread::~RRDExampleThread()
{
}


void
RRDExampleThread::init()
{
  std::vector<RRDDataSource> rrds;
  rrds.push_back(RRDDataSource("value", RRDDataSource::COUNTER));
  __test_rrd_def = new RRDDefinition("test", rrds);
  rrd_manager->add_rrd(__test_rrd_def);

  std::vector<RRDGraphDataDefinition> defs;
  std::vector<RRDGraphElement> els;

  defs.push_back(RRDGraphDataDefinition("value", RRDArchive::AVERAGE,
					_test_rrd_def));
  
  els.push_back(RRDGraphLine("value", 1, "FF0000", "Value", false));
  els.push_back(RRDGraphGPrint("value", RRDArchive::LAST,
			       "Current\\:%8.2lf %s"));
  els.push_back(RRDGraphGPrint("value", RRDArchive::AVERAGE,
			       "Average\\:%8.2lf %s"));
  els.push_back(RRDGraphGPrint("value", RRDArchive::MAX,
			       "Maximum\\:%8.2lf %s\\n"));

  __test_graph_def = new RRDGraphDefinition("testgraph", __test_rrd_def,
					    -600, -10, 10,
					    "Test Value", "Foo", 10,
					    false, defs, els);

  rrd_manager->add_graph(__test_graph_def);

  __loop_count = 0;
  __counter = 0;
}


void
RRDExampleThread::finalize()
{
  rrd_manager->remove_rrd(__test_rrd_def);
}


void
RRDExampleThread::loop()
{
  __loop_count++;
  if (rand() > RAND_MAX/2) __counter++;
  if (__loop_count == 10) {
    try {
      logger->log_debug(name(), "Adding data N:%u", __counter);
      rrd_manager->add_data(__test_rrd_def->get_name(), "N:%u", __counter);
    } catch (Exception &e) {
      logger->log_warn(name(), "Adding data to %s failed, exception follows",
		       __test_rrd_def->get_name());
      logger->log_warn(name(), e);
    }
    __loop_count = 0;
  }
}
