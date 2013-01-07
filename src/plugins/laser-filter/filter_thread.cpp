
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
#ifdef HAVE_TF
#  include "filters/projection.h"
#endif

#include <core/threading/barrier.h>
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>

#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>

#include <cstring>
#include <memory>
#include <cstdio>

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
      
      std::string suffix = std::string(i->path()).substr(fpfx.length());
      std::string filter_name = std::string(suffix.substr(0,suffix.find("/")));
      std::string conf_key = std::string(suffix.substr(suffix.find("/")+1,suffix.length()));
      
      
      logger->log_debug(name(),"filter_name is %s", filter_name.c_str());
      logger->log_debug(name(),"conf_key is %s", conf_key.c_str());
      if (conf_key != "type") {
	continue;
      }

      if (! i->is_string()) {
	throw Exception("Filter value %s is not a string", i->path());
      }

	
      filters[filter_name] = i->get_string();
    }
    if (filters.empty()) {
      throw Exception("No filters defined for %s", __cfg_name.c_str());
    }

    if (filters.size() == 1) {
      std::string filter_name = filters.begin()->first;
      logger->log_debug(name(), "Adding filter %s (%s)",
			filter_name.c_str(), filters[filter_name].c_str());
      __filter = create_filter(filters[filter_name], fpfx + filter_name + "/",
			       __in[0].is_360 ? 360 : 720, __in_bufs);
    } else {
      LaserDataFilterCascade *cascade =
        new LaserDataFilterCascade(__in[0].is_360 ? 360 : 720, __in_bufs);

      try {
	std::map<std::string, std::string>::iterator f;
	for (f = filters.begin(); f != filters.end(); ++f) {
          logger->log_debug(name(), "Adding filter %s (%s) %zu %zu",
                            f->first.c_str(), f->second.c_str(), __in_bufs.size(),
                            cascade->get_out_vector().size());
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
  const size_t in_num = __in.size();
  for (size_t i = 0; i != in_num; ++i) {
    __in[i].interface->read();
    if (__in[i].is_360) {
      __in_bufs[i]->frame = __in[i].interface_typed.as360->frame();
    } else {
      __in_bufs[i]->frame = __in[i].interface_typed.as720->frame();
    }
  }

  // Filter!
  try {
    __filter->filter();
  } catch (Exception &e) {
    logger->log_warn(name(), "Filtering failed, exception follows");
    logger->log_warn(name(), e);
  }

  // Write output interfaces
  const size_t num = __out.size();
  for (size_t i = 0; i < num; ++i) {
    if (__out[i].is_360) {
      __out[i].interface_typed.as360->set_frame(__out_bufs[i]->frame.c_str());
    } else {
      __out[i].interface_typed.as720->set_frame(__out_bufs[i]->frame.c_str());
    }
    __out[i].interface->write();
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
				   std::vector<LaserDataFilter::Buffer *> &bufs, bool writing)
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

          ifs[i].interface_typed.as360 = laser360;
	  ifs[i].interface = laser360;
          bufs[i] = new LaserDataFilter::Buffer();
	  bufs[i]->values = laser360->distances();
	  
	} else {
	  if (must_360) {
	    throw Exception("Interfaces of mixed sizes for %s",
			    __cfg_name.c_str());
	  }

	  logger->log_debug(name(), "Opening writing Laser720Interface::%s", ifs[i].id.c_str());
	  Laser720Interface *laser720 = 
	    blackboard->open_for_writing<Laser720Interface>(ifs[i].id.c_str());

          ifs[i].interface_typed.as720 = laser720;
	  ifs[i].interface = laser720;
          bufs[i] = new LaserDataFilter::Buffer();
	  bufs[i]->values = laser720->distances();
	}
      }
    } else {
      for (unsigned int i = 0; i < ifs.size(); ++i) {
	if (ifs[i].is_360) {
	  logger->log_debug(name(), "Opening reading Laser360Interface::%s", ifs[i].id.c_str());
	  Laser360Interface *laser360 =
	    blackboard->open_for_reading<Laser360Interface>(ifs[i].id.c_str());

          ifs[i].interface_typed.as360 = laser360;
	  ifs[i].interface = laser360;
          bufs[i] = new LaserDataFilter::Buffer();
          bufs[i]->frame  = laser360->frame();
	  bufs[i]->values = laser360->distances();
	  
	} else {
	  logger->log_debug(name(), "Opening reading Laser720Interface::%s", ifs[i].id.c_str());
	  Laser720Interface *laser720 =
	    blackboard->open_for_reading<Laser720Interface>(ifs[i].id.c_str());

          ifs[i].interface_typed.as720 = laser720;
	  ifs[i].interface = laser720;
          bufs[i] = new LaserDataFilter::Buffer();
          bufs[i]->frame  = laser720->frame();
	  bufs[i]->values = laser720->distances();
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
				 unsigned int in_data_size,
                                 std::vector<LaserDataFilter::Buffer *> &inbufs)
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
#ifdef HAVE_TF
    const float not_from_x = config->get_float((prefix + "not_from_x").c_str());
    const float not_to_x = config->get_float((prefix + "not_to_x").c_str());
    const float not_from_y = config->get_float((prefix + "not_from_y").c_str());
    const float not_to_y = config->get_float((prefix + "not_to_y").c_str());
    const float only_from_z = config->get_float((prefix + "only_from_z").c_str());
    const float only_to_z = config->get_float((prefix + "only_to_z").c_str());
    const std::string frame =
      config->get_string((prefix + "target_frame").c_str());
    return new LaserProjectionDataFilter(tf_listener, frame,
                                         not_from_x, not_to_x,
                                         not_from_y, not_to_y,
                                         only_from_z, only_to_z,
                                         in_data_size, inbufs);
#else
    throw Exception("Projection filter unavailable, tf missing");
#endif
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
