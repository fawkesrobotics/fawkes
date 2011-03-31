
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
#include "filters/min_circle.h"
#include "filters/circle_sector.h"
#include "filters/min_merge.h"
#include "filters/projection.h"

#include <core/threading/barrier.h>
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>

#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>

#include <cstring>
#include <memory>

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
  __wait_barrier = NULL;
}


void
LaserFilterThread::init()
{
  try {
    open_interfaces(__cfg_prefix + "in/", __in, __in_bufs, false);
    open_interfaces(__cfg_prefix + "out/", __out, __out_bufs, true);

    if (__in.empty()) {
      throw Exception("No input interfaces defined for %s", __cfg_name.c_str());
    }
    if (__out.empty()) {
      throw Exception("No output interfaces defined for %s", __cfg_name.c_str());
    }


    std::map<std::string, std::string> filters;

    std::string fpfx = __cfg_prefix + "filters/";
    std::auto_ptr<Configuration::ValueIterator> i(config->search(fpfx.c_str()));
    while (i->next()) {
      std::string filter_name = std::string(i->path()).substr(fpfx.length());
      if (filter_name.find("/") != std::string::npos) {
	// If it contains a slash we assume it is a parameter for a filter
	continue;
      }

      if (! i->is_string()) {
	throw Exception("Filter value %s is not a string", i->path());
      }

      logger->log_debug(name(), "Adding filter %s (%s)",
			filter_name.c_str(), i->get_string().c_str());

      filters[filter_name] = i->get_string();
    }
    if (filters.empty()) {
      throw Exception("No filters defined for %s", __cfg_name.c_str());
    }

    if (filters.size() == 1) {
      std::string filter_name = filters.begin()->first;
      __filter = create_filter(filters[filter_name], fpfx + filter_name + "/",
			       __in[0].is_360 ? 360 : 720, __in_bufs);
    } else {
      LaserDataFilterCascade *cascade = new LaserDataFilterCascade(__in[0].is_360 ? 360 : 720,
								   __in_bufs);

      try {
	std::map<std::string, std::string>::iterator f;
	for (f = filters.begin(); f != filters.end(); ++f) {
	  cascade->add_filter(create_filter(f->second, fpfx + f->first + "/",
					    cascade->get_out_data_size(),
					    cascade->get_out_vector()));
	}
      } catch (Exception &e) {
	delete cascade;
	throw;
      }

      __filter = cascade;
    }

    if (__out[0].is_360 && (__filter->get_out_data_size() != 360)) {
      Exception e("Output interface and filter data size for %s do not match (%u != 360)",
		  __cfg_name.c_str(), __filter->get_out_data_size());
      delete __filter;
      throw e;
    } else if (!__out[0].is_360 && (__filter->get_out_data_size() != 720)) {
      Exception e("Output interface and filter data size for %s do not match (%u != 720)",
		  __cfg_name.c_str(), __filter->get_out_data_size());
      delete __filter;
      throw e;
    }

    __filter->set_out_vector(__out_bufs);

  } catch (Exception &e) {
    for (unsigned int i = 0; i < __in.size(); ++i) {
      blackboard->close(__in[i].interface);
    }
    for (unsigned int i = 0; i < __out.size(); ++i) {
      blackboard->close(__out[i].interface);
    }
    throw;
  }

  std::list<LaserFilterThread *>::iterator wt;
  for (wt = __wait_threads.begin(); wt != __wait_threads.end(); ++wt) {
    logger->log_debug(name(), "Depending on %s", (*wt)->name());
  }

  __wait_done  = true;
  __wait_mutex = new Mutex();
  __wait_cond  = new WaitCondition(__wait_mutex);
}


void
LaserFilterThread::finalize()
{
  delete __filter;
  delete __wait_cond;
  delete __wait_mutex;

  for (unsigned int i = 0; i < __in.size(); ++i) {
    blackboard->close(__in[i].interface);
  }
  __in.clear();
  for (unsigned int i = 0; i < __out.size(); ++i) {
    blackboard->close(__out[i].interface);
  }
  __out.clear();
}

void
LaserFilterThread::loop()
{
  // Wait for dependencies
  if (__wait_barrier) {
    std::list<LaserFilterThread *>::iterator wt;
    for (wt = __wait_threads.begin(); wt != __wait_threads.end(); ++wt) {
      (*wt)->wait_done();
    }
  }

  // Read input interfaces
  std::vector<LaserInterface>::iterator i;
  for (i = __in.begin(); i != __in.end(); ++i) {
    i->interface->read();
  }

  // Filter!
  __filter->filter();

  // Write output interfaces
  for (i = __out.begin(); i != __out.end(); ++i) {
    i->interface->write();
  }

  if (__wait_barrier) {
    __wait_mutex->lock();
    __wait_done = false;
    __wait_cond->wake_all();
    __wait_mutex->unlock();
    __wait_barrier->wait();
    __wait_mutex->lock();
    __wait_done = true;
    __wait_mutex->unlock();
  }
}


/** Wait until thread is done.
 * This method blocks the calling thread until this instance's thread has
 * finished filtering.
 */
void
LaserFilterThread::wait_done()
{
  __wait_mutex->lock();
  while (__wait_done) {
    //logger->log_debug(name(), "%s is waiting", Thread::current_thread()->name());
    __wait_cond->wait();
  }
  __wait_mutex->unlock();
}


void
LaserFilterThread::open_interfaces(std::string prefix,
				   std::vector<LaserInterface> &ifs,
				   std::vector<float *> &bufs, bool writing)
{
  std::auto_ptr<Configuration::ValueIterator> in(config->search(prefix.c_str()));
  while (in->next()) {
    if (! in->is_string()) {
      throw Exception("Config value %s is not of type string", in->path());
    } else {
      std::string uid = in->get_string();
      size_t sf;

      if ((sf = uid.find("::")) == std::string::npos) {
	throw Exception("Interface '%s' is not a UID", uid.c_str());
      }
      std::string type = uid.substr(0, sf);
      std::string id = uid.substr(sf + 2);

      LaserInterface lif;
      lif.interface = NULL;

      if (type == "Laser360Interface") {
	lif.is_360 = true;
      } else if (type == "Laser720Interface") {
	lif.is_360 = false;
      } else {
	throw Exception("Interfaces must be of type Laser360Interface or "
			"Laser720Interface, but it is '%s'", type.c_str());
      }

      lif.id = id;
      ifs.push_back(lif);
    }
  }

  if (ifs.empty()) {
    throw Exception("No interfaces defined at %s", prefix.c_str());
  }

  bufs.resize(ifs.size());

  bool must_360 = ifs[0].is_360;

  try {
    if (writing) {
      for (unsigned int i = 0; i < ifs.size(); ++i) {
	if (ifs[i].is_360) {
	  if (! must_360) {
	    throw Exception("Interfaces of mixed sizes for %s",
			    __cfg_name.c_str());
	  }
	  logger->log_debug(name(), "Opening writing Laser360Interface::%s", ifs[i].id.c_str());
	  Laser360Interface *laser360 = 
	    blackboard->open_for_writing<Laser360Interface>(ifs[i].id.c_str());

	  ifs[i].interface = laser360;
	  bufs[i] = laser360->distances();
	  
	} else {
	  if (must_360) {
	    throw Exception("Interfaces of mixed sizes for %s",
			    __cfg_name.c_str());
	  }

	  logger->log_debug(name(), "Opening writing Laser720Interface::%s", ifs[i].id.c_str());
	  Laser720Interface *laser720 = 
	    blackboard->open_for_writing<Laser720Interface>(ifs[i].id.c_str());

	  ifs[i].interface = laser720;
	  bufs[i] = laser720->distances();
	}
      }
    } else {
      for (unsigned int i = 0; i < ifs.size(); ++i) {
	if (ifs[i].is_360) {
	  logger->log_debug(name(), "Opening reading Laser360Interface::%s", ifs[i].id.c_str());
	  Laser360Interface *laser360 =
	    blackboard->open_for_reading<Laser360Interface>(ifs[i].id.c_str());

	  ifs[i].interface = laser360;
	  bufs[i] = laser360->distances();
	  
	} else {
	  logger->log_debug(name(), "Opening reading Laser720Interface::%s", ifs[i].id.c_str());
	  Laser720Interface *laser720 =
	    blackboard->open_for_reading<Laser720Interface>(ifs[i].id.c_str());

	  ifs[i].interface = laser720;
	  bufs[i] = laser720->distances();
	}
      }
    }
  } catch (Exception &e) {
    for (unsigned int i = 0; i < ifs.size(); ++i) {
      blackboard->close(ifs[i].interface);
    }
    ifs.clear();
    bufs.clear();
    throw;
  }
}


LaserDataFilter *
LaserFilterThread::create_filter(std::string filter_type, std::string prefix,
				 unsigned int in_data_size, std::vector<float *> &inbufs)
{
  if (filter_type == "720to360") {
    bool average = false;
    try {
      average = config->get_bool((prefix + "average").c_str());
    } catch (Exception &e) {} // ignore
    return new Laser720to360DataFilter(average, in_data_size, inbufs);
  } else if (filter_type == "reverse") {
    return new LaserReverseAngleDataFilter(in_data_size, inbufs);
  } else if (filter_type == "max_circle") {
    float radius = config->get_float((prefix + "radius").c_str());
    return new LaserMaxCircleDataFilter(radius, in_data_size, inbufs);
  } else if (filter_type == "min_circle") {
    float radius = config->get_float((prefix + "radius").c_str());
    return new LaserMinCircleDataFilter(radius, in_data_size, inbufs);
  } else if (filter_type == "circle_sector") {
    unsigned int from = config->get_uint((prefix + "from").c_str());
    unsigned int to   = config->get_uint((prefix + "to").c_str());
    return new LaserCircleSectorDataFilter(from, to, in_data_size, inbufs);
  } else if (filter_type == "deadspots") {
    return new LaserDeadSpotsDataFilter(config, logger, prefix, in_data_size, inbufs);
  } else if (filter_type == "min_merge") {
    return new LaserMinMergeDataFilter(in_data_size, inbufs);
  } else if (filter_type == "projection") {
    const bool left = config->get_bool((prefix + "left").c_str());
    const LaserProjectionDataFilter::Rotation rot(config->get_float((prefix + "x_rot").c_str()),
                                                  config->get_float((prefix + "y_rot").c_str()),
                                                  config->get_float((prefix + "z_rot").c_str()));
    const LaserProjectionDataFilter::Translation trans(config->get_float((prefix + "x_trans").c_str()),
                                                       config->get_float((prefix + "y_trans").c_str()),
                                                       config->get_float((prefix + "z_trans").c_str()));
    const LaserProjectionDataFilter::Rectangle robot_rect(config->get_float((prefix + "x_min").c_str()),
                                                          config->get_float((prefix + "x_max").c_str()),
                                                          config->get_float((prefix + "y_min").c_str()),
                                                          config->get_float((prefix + "y_max").c_str()));
    const float z_threshold = config->get_float((prefix + "z_threshold").c_str());
    return new LaserProjectionDataFilter(left, rot, trans, robot_rect, z_threshold, in_data_size, inbufs);
  } else {
    throw Exception("Unknown filter type %s", filter_type.c_str());
  }
}


/** Set threads to wait for in loop.
 * The threads produce data this thread depends on as input, therefore this
 * instance has to wait for these threads to get up to date data in each
 * loop.
 * @param threads threads this instance depends on
 */
void
LaserFilterThread::set_wait_threads(std::list<LaserFilterThread *> &threads)
{
  __wait_threads = threads;
}


/** Set wait barrier.
 * If there are any dependencies between laser filter threads a common
 * barrier is used to signal the end of filtering to reset internal
 * variables for the next loop.
 * @param barrier common "end of filtering" barrier
 */
void
LaserFilterThread::set_wait_barrier(fawkes::Barrier *barrier)
{
  __wait_barrier = barrier;
}
