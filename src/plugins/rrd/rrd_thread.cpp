
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

#include "rrd_thread.h"

#include <core/exceptions/system.h>
#include <utils/misc/string_conversions.h>
#include <utils/system/file.h>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <rrd.h>

using namespace fawkes;

/** @class RRDThread "rrd_thread.h"
 * RRD Thread.
 * This thread maintains an active connection to RRD and provides an
 * aspect to access RRD to make it convenient for other threads to use
 * RRD.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
RRDThread::RRDThread()
  : Thread("RRDThread", Thread::OPMODE_CONTINUOUS),
    AspectProviderAspect("RRDAspect", &__rrd_aspect_inifin),
    __rrd_aspect_inifin(this)
{
}


/** Destructor. */
RRDThread::~RRDThread()
{
}


void
RRDThread::init()
{
  std::vector<RRDDataSource> rrds;
  rrds.push_back(RRDDataSource("value", RRDDataSource::COUNTER,
			       30, 0, RRDDataSource::UNKNOWN));
  test_rrd_def = new RRDDefinition("test", 10, false, rrds);
  add_rrd(test_rrd_def);

  std::vector<RRDGraphDataDefinition> defs;
  std::vector<RRDGraphElement *> els;

  defs.push_back(RRDGraphDataDefinition("value", RRDArchive::AVERAGE,
					test_rrd_def, "value"));
  
  els.push_back(new RRDGraphLine("value", 1, "FF0000", "Value", false));
  els.push_back(new RRDGraphGPrint("value", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("value", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("value", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  test_graph_def = new RRDGraphDefinition(test_rrd_def, -600, -10, 10,
					  "Test Value", "Foo", 10,
					  false, defs, els);

  add_graph(test_graph_def);

  __time_wait = new TimeWait(clock, 1000000);
  __loop_count = 0;
  __counter = 0;
}


void
RRDThread::finalize()
{
}


void
RRDThread::loop()
{
  __time_wait->mark_start();
  __loop_count++;
  if (rand() > RAND_MAX/2) __counter++;
  if (__loop_count == 10) {
    try {
      add_data(test_rrd_def->get_name(), "N:%u", __counter);
      generate_graphs();
    } catch (Exception &e) {
      logger->log_warn(name(), "Adding data to %s failed, exception follows",
		       test_rrd_def->get_name());
      logger->log_warn(name(), e);
    }
    __loop_count = 0;
  }
  __time_wait->wait_systime();
}


/** Generate all graphs. */
void
RRDThread::generate_graphs()
{
  std::vector<fawkes::RRDGraphDefinition *>::iterator g;
  for (g = __graphs.begin(); g != __graphs.end(); ++g) {
    size_t argc = 0;
    const char **argv = (*g)->get_argv(argc);

    //logger->log_debug(name(), "rrd_graph arguments:");
    //for (size_t j = 0; j < argc; ++j) {
    //  logger->log_debug(name(), "  %zu: %s", j, argv[j]);
    //}

    rrd_clear_error();
    rrd_info_t *i = rrd_graph_v(argc, (char **)argv);
    if (i == NULL) {
      throw Exception("Creating graph for %s failed: %s",
		      (*g)->get_rrd_def()->get_name(), rrd_get_error());
    }
    rrd_info_free(i);
  }
}

void
RRDThread::add_rrd(RRDDefinition *rrd_def)
{
  // generate filename
  char *filename;
  if (asprintf(&filename, "%s/%s.rrd", ".", rrd_def->get_name()) == -1) {
    throw OutOfMemoryException("Failed to creat filename for RRD %s",
			       rrd_def->get_name());
  }
  rrd_def->set_filename(filename);
  free(filename);

  if (! File::exists(rrd_def->get_filename()) || rrd_def->get_recreate()) {
    std::string size_s = StringConversions::to_string(rrd_def->get_step_sec());

    // build parameter array
    // "create" filename --step STEP --start START DS... RRA...
    size_t rrd_argc = 6 + rrd_def->get_ds().size() + rrd_def->get_rra().size();
    const char *rrd_argv[rrd_argc];
    size_t i = 0;
    rrd_argv[i++] = "create";
    rrd_argv[i++] = rrd_def->get_filename();
    rrd_argv[i++] = "--step";
    rrd_argv[i++] = size_s.c_str();
    rrd_argv[i++] = "--start";
    rrd_argv[i++] = "0";

    std::vector<RRDDataSource>::const_iterator d;
    for (d = rrd_def->get_ds().begin();
	 d != rrd_def->get_ds().end() && i < rrd_argc;
	 ++d)
    {
      rrd_argv[i++] = d->to_string();
    }

    std::vector<RRDArchive>::const_iterator a;
    for (a = rrd_def->get_rra().begin();
	 a != rrd_def->get_rra().end() && i < rrd_argc;
	 ++a)
    {
      rrd_argv[i++] = a->to_string();
    }

    //logger->log_debug(name(), "rrd_create arguments:");
    //for (size_t j = 0; j < i; ++j) {
    //  logger->log_debug(name(), "  %zu: %s", j, rrd_argv[j]);
    //}

    // Create RRD file
    rrd_clear_error();
    if (rrd_create(i, (char **)rrd_argv) == -1) {
      throw Exception("Creating RRD %s failed: %s",
		      rrd_def->get_name(), rrd_get_error());
    }
  }

  __rrds.push_back(rrd_def);
}

void
RRDThread::add_graph(RRDGraphDefinition *rrd_graph_def)
{
  // generate filename
  char *filename;
  if (asprintf(&filename, "%s/%s.png", ".",
	       rrd_graph_def->get_rrd_def()->get_name()) == -1) {
    throw OutOfMemoryException("Failed to create filename for PNG %s",
			       rrd_graph_def->get_rrd_def()->get_name());
  }
  rrd_graph_def->set_filename(filename);
  free(filename);

  __graphs.push_back(rrd_graph_def);
}

void
RRDThread::add_data(const char *rrd_name, const char *format, ...)
{
  std::vector<RRDDefinition *>::const_iterator d;
  for (d = __rrds.begin(); d != __rrds.end(); ++d) {
    RRDDefinition *rrd_def = *d;
    if (strcmp(rrd_name, rrd_def->get_name()) == 0) {
      char *data;
      va_list arg;
      va_start(arg, format);
      if (vasprintf(&data, format, arg) == -1) {
	va_end(arg);
	throw OutOfMemoryException("Failed to create data string for %s",
				   rrd_name);
      }
      va_end(arg);

      // filename data
      size_t rrd_argc = 3;
      const char *rrd_argv[rrd_argc];
      size_t i = 0;
      rrd_argv[i++] = "update";
      rrd_argv[i++] = rrd_def->get_filename();
      rrd_argv[i++] = data;

      //logger->log_debug(name(), "rrd_update arguments:");
      //for (size_t j = 0; j < i; ++j) {
      //  logger->log_debug(name(), "  %zu: %s", j, rrd_argv[j]);
      //}

      rrd_clear_error();
      if (rrd_update(i, (char **)rrd_argv) == -1) {
	free(data);
	throw Exception("Failed to update RRD %s: %s", rrd_name, rrd_get_error());
      }

      free(data);
      return;
    }
  }

  throw Exception("No RRD named %s registered", rrd_name);
}


const std::vector<RRDDefinition *> &
RRDThread::get_rrds() const
{
  return __rrds;
}


const std::vector<RRDGraphDefinition *> &
RRDThread::get_graphs() const
{
  return __graphs;
}
