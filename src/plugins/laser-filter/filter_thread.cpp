
/***************************************************************************
 *  filter_thread.cpp - Thread that filters data in blackboard
 *
 *  Created: Sun Mar 13 01:12:53 2011
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include "filters/1080to360.h"
#include "filters/720to360.h"
#include "filters/cascade.h"
#include "filters/circle_sector.h"
#include "filters/copy.h"
#include "filters/deadspots.h"
#include "filters/max_circle.h"
#include "filters/min_circle.h"
#include "filters/min_merge.h"
#include "filters/reverse_angle.h"
#ifdef HAVE_TF
#	include "filters/box_filter.h"
#	include "filters/map_filter.h"
#	include "filters/projection.h"
#endif

#include <core/threading/barrier.h>
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>
#include <interfaces/Laser1080Interface.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>
#include <utils/time/time.h>

#include <cstdio>
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
LaserFilterThread::LaserFilterThread(std::string &cfg_name, std::string &cfg_prefix)
: Thread("LaserFilterThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
	set_name("LaserFilterThread(%s)", cfg_name.c_str());
	cfg_name_     = cfg_name;
	cfg_prefix_   = cfg_prefix;
	wait_barrier_ = NULL;
}

void
LaserFilterThread::init()
{
	try {
		open_interfaces(cfg_prefix_ + "in/", in_, in_bufs_, false);
		open_interfaces(cfg_prefix_ + "out/", out_, out_bufs_, true);

		if (in_.empty()) {
			throw Exception("No input interfaces defined for %s", cfg_name_.c_str());
		}
		if (out_.empty()) {
			throw Exception("No output interfaces defined for %s", cfg_name_.c_str());
		}

		std::map<std::string, std::string> filters;

		std::string fpfx = cfg_prefix_ + "filters/";
#if __cplusplus >= 201103L
		std::unique_ptr<Configuration::ValueIterator> i(config->search(fpfx.c_str()));
#else
		std::auto_ptr<Configuration::ValueIterator> i(config->search(fpfx.c_str()));
#endif

		while (i->next()) {
			std::string suffix      = std::string(i->path()).substr(fpfx.length());
			std::string filter_name = std::string(suffix.substr(0, suffix.find("/")));
			std::string conf_key    = std::string(suffix.substr(suffix.find("/") + 1, suffix.length()));

			if (conf_key != "type")
				continue;

			if (!i->is_string()) {
				throw Exception("Filter value %s is not a string", i->path());
			}

			filters[filter_name] = i->get_string();
		}
		if (filters.empty()) {
			throw Exception("No filters defined for %s", cfg_name_.c_str());
		}

		if (filters.size() == 1) {
			std::string filter_name = filters.begin()->first;
			logger->log_debug(name(),
			                  "Adding filter %s (%s)",
			                  filter_name.c_str(),
			                  filters[filter_name].c_str());
			filter_ = create_filter(cfg_name_ + "/" + filter_name,
			                        filters[filter_name],
			                        fpfx + filter_name + "/",
			                        in_[0].size,
			                        in_bufs_);
		} else {
			LaserDataFilterCascade *cascade =
			  new LaserDataFilterCascade(cfg_name_, in_[0].size, in_bufs_);

			try {
				std::map<std::string, std::string>::iterator f;
				for (f = filters.begin(); f != filters.end(); ++f) {
					logger->log_debug(name(),
					                  "Adding filter %s (%s) %zu %zu",
					                  f->first.c_str(),
					                  f->second.c_str(),
					                  in_bufs_.size(),
					                  cascade->get_out_vector().size());
					cascade->add_filter(create_filter(cfg_name_ + "/" + f->first,
					                                  f->second,
					                                  fpfx + f->first + "/",
					                                  cascade->get_out_data_size(),
					                                  cascade->get_out_vector()));
				}
			} catch (Exception &e) {
				delete cascade;
				throw;
			}

			filter_ = cascade;
		}

		if (out_[0].size != filter_->get_out_data_size()) {
			Exception e("Output interface and filter data size for %s do not match (%u != %u)",
			            cfg_name_.c_str(),
			            out_[0].size,
			            filter_->get_out_data_size());
			delete filter_;
			throw e;
		}

		filter_->set_out_vector(out_bufs_);

	} catch (Exception &e) {
		for (unsigned int i = 0; i < in_.size(); ++i) {
			blackboard->close(in_[i].interface);
		}
		for (unsigned int i = 0; i < out_.size(); ++i) {
			blackboard->close(out_[i].interface);
		}
		throw;
	}

	std::list<LaserFilterThread *>::iterator wt;
	for (wt = wait_threads_.begin(); wt != wait_threads_.end(); ++wt) {
		logger->log_debug(name(), "Depending on %s", (*wt)->name());
	}

	wait_done_  = true;
	wait_mutex_ = new Mutex();
	wait_cond_  = new WaitCondition(wait_mutex_);
}

void
LaserFilterThread::finalize()
{
	delete filter_;
	delete wait_cond_;
	delete wait_mutex_;

	for (unsigned int i = 0; i < in_.size(); ++i) {
		blackboard->close(in_[i].interface);
	}
	in_.clear();
	for (unsigned int i = 0; i < out_.size(); ++i) {
		blackboard->close(out_[i].interface);
	}
	out_.clear();
}

void
LaserFilterThread::loop()
{
	// Wait for dependencies
	if (wait_barrier_) {
		std::list<LaserFilterThread *>::iterator wt;
		for (wt = wait_threads_.begin(); wt != wait_threads_.end(); ++wt) {
			(*wt)->wait_done();
		}
	}

	// Read input interfaces
	const size_t in_num = in_.size();
	for (size_t i = 0; i != in_num; ++i) {
		in_[i].interface->read();
		if (in_[i].size == 360) {
			in_bufs_[i]->frame      = in_[i].interface_typed.as360->frame();
			*in_bufs_[i]->timestamp = in_[i].interface_typed.as360->timestamp();
		} else if (in_[i].size == 720) {
			in_bufs_[i]->frame      = in_[i].interface_typed.as720->frame();
			*in_bufs_[i]->timestamp = in_[i].interface_typed.as720->timestamp();
		} else if (in_[i].size == 1080) {
			in_bufs_[i]->frame      = in_[i].interface_typed.as1080->frame();
			*in_bufs_[i]->timestamp = in_[i].interface_typed.as1080->timestamp();
		}
	}

	// Filter!
	try {
		filter_->filter();
	} catch (Exception &e) {
		logger->log_warn(name(), "Filtering failed, exception follows");
		logger->log_warn(name(), e);
	}

	// Write output interfaces
	const size_t num = out_.size();
	for (size_t i = 0; i < num; ++i) {
		if (out_[i].size == 360) {
			out_[i].interface_typed.as360->set_timestamp(out_bufs_[i]->timestamp);
			out_[i].interface_typed.as360->set_frame(out_bufs_[i]->frame.c_str());
		} else if (out_[i].size == 720) {
			out_[i].interface_typed.as720->set_timestamp(out_bufs_[i]->timestamp);
			out_[i].interface_typed.as720->set_frame(out_bufs_[i]->frame.c_str());
		} else if (out_[i].size == 1080) {
			out_[i].interface_typed.as1080->set_timestamp(out_bufs_[i]->timestamp);
			out_[i].interface_typed.as1080->set_frame(out_bufs_[i]->frame.c_str());
		}
		out_[i].interface->write();
	}

	if (wait_barrier_) {
		wait_mutex_->lock();
		wait_done_ = false;
		wait_cond_->wake_all();
		wait_mutex_->unlock();
		wait_barrier_->wait();
		wait_mutex_->lock();
		wait_done_ = true;
		wait_mutex_->unlock();
	}
}

/** Wait until thread is done.
 * This method blocks the calling thread until this instance's thread has
 * finished filtering.
 */
void
LaserFilterThread::wait_done()
{
	wait_mutex_->lock();
	while (wait_done_) {
		//logger->log_debug(name(), "%s is waiting", Thread::current_thread()->name());
		wait_cond_->wait();
	}
	wait_mutex_->unlock();
}

void
LaserFilterThread::open_interfaces(std::string                             prefix,
                                   std::vector<LaserInterface> &           ifs,
                                   std::vector<LaserDataFilter::Buffer *> &bufs,
                                   bool                                    writing)
{
#if __cplusplus >= 201103L
	std::unique_ptr<Configuration::ValueIterator> in(config->search(prefix.c_str()));
#else
	std::auto_ptr<Configuration::ValueIterator> in(config->search(prefix.c_str()));
#endif
	while (in->next()) {
		if (!in->is_string()) {
			throw Exception("Config value %s is not of type string", in->path());
		} else {
			std::string uid = in->get_string();
			size_t      sf;

			if ((sf = uid.find("::")) == std::string::npos) {
				throw Exception("Interface '%s' is not a UID", uid.c_str());
			}
			std::string type = uid.substr(0, sf);
			std::string id   = uid.substr(sf + 2);

			LaserInterface lif;
			lif.interface = NULL;

			if (type == "Laser360Interface") {
				lif.size = 360;
			} else if (type == "Laser720Interface") {
				lif.size = 720;
			} else if (type == "Laser1080Interface") {
				lif.size = 1080;
			} else {
				throw Exception("Interfaces must be of type Laser360Interface, "
				                "Laser720Interface, or Laser1080Interface, "
				                "but it is '%s'",
				                type.c_str());
			}

			lif.id = id;
			ifs.push_back(lif);
		}
	}

	if (ifs.empty()) {
		throw Exception("No interfaces defined at %s", prefix.c_str());
	}

	bufs.resize(ifs.size());

	unsigned int req_size = ifs[0].size;

	try {
		if (writing) {
			for (unsigned int i = 0; i < ifs.size(); ++i) {
				if (req_size != ifs[i].size) {
					throw Exception("Interfaces of mixed sizes for %s", cfg_name_.c_str());
				}

				if (ifs[i].size == 360) {
					logger->log_debug(name(), "Opening writing Laser360Interface::%s", ifs[i].id.c_str());
					Laser360Interface *laser360 =
					  blackboard->open_for_writing<Laser360Interface>(ifs[i].id.c_str());

					laser360->set_auto_timestamping(false);

					ifs[i].interface_typed.as360 = laser360;
					ifs[i].interface             = laser360;
					bufs[i]                      = new LaserDataFilter::Buffer();
					bufs[i]->name                = laser360->uid();
					bufs[i]->values              = laser360->distances();

				} else if (ifs[i].size == 720) {
					logger->log_debug(name(), "Opening writing Laser720Interface::%s", ifs[i].id.c_str());
					Laser720Interface *laser720 =
					  blackboard->open_for_writing<Laser720Interface>(ifs[i].id.c_str());

					laser720->set_auto_timestamping(false);

					ifs[i].interface_typed.as720 = laser720;
					ifs[i].interface             = laser720;
					bufs[i]                      = new LaserDataFilter::Buffer();
					bufs[i]->name                = laser720->uid();
					bufs[i]->values              = laser720->distances();

				} else if (ifs[i].size == 1080) {
					logger->log_debug(name(), "Opening writing Laser1080Interface::%s", ifs[i].id.c_str());
					Laser1080Interface *laser1080 =
					  blackboard->open_for_writing<Laser1080Interface>(ifs[i].id.c_str());

					laser1080->set_auto_timestamping(false);

					ifs[i].interface_typed.as1080 = laser1080;
					ifs[i].interface              = laser1080;
					bufs[i]                       = new LaserDataFilter::Buffer();
					bufs[i]->name                 = laser1080->uid();
					bufs[i]->values               = laser1080->distances();
				}
			}
		} else {
			for (unsigned int i = 0; i < ifs.size(); ++i) {
				if (ifs[i].size == 360) {
					logger->log_debug(name(), "Opening reading Laser360Interface::%s", ifs[i].id.c_str());
					Laser360Interface *laser360 =
					  blackboard->open_for_reading<Laser360Interface>(ifs[i].id.c_str());

					ifs[i].interface_typed.as360 = laser360;
					ifs[i].interface             = laser360;
					bufs[i]                      = new LaserDataFilter::Buffer();
					bufs[i]->name                = laser360->uid();
					bufs[i]->frame               = laser360->frame();
					bufs[i]->values              = laser360->distances();

				} else if (ifs[i].size == 720) {
					logger->log_debug(name(), "Opening reading Laser720Interface::%s", ifs[i].id.c_str());
					Laser720Interface *laser720 =
					  blackboard->open_for_reading<Laser720Interface>(ifs[i].id.c_str());

					ifs[i].interface_typed.as720 = laser720;
					ifs[i].interface             = laser720;
					bufs[i]                      = new LaserDataFilter::Buffer();
					bufs[i]->name                = laser720->uid();
					bufs[i]->frame               = laser720->frame();
					bufs[i]->values              = laser720->distances();

				} else if (ifs[i].size == 1080) {
					logger->log_debug(name(), "Opening reading Laser1080Interface::%s", ifs[i].id.c_str());
					Laser1080Interface *laser1080 =
					  blackboard->open_for_reading<Laser1080Interface>(ifs[i].id.c_str());

					ifs[i].interface_typed.as1080 = laser1080;
					ifs[i].interface              = laser1080;
					bufs[i]                       = new LaserDataFilter::Buffer();
					bufs[i]->name                 = laser1080->uid();
					bufs[i]->frame                = laser1080->frame();
					bufs[i]->values               = laser1080->distances();
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
LaserFilterThread::create_filter(std::string                             filter_name,
                                 std::string                             filter_type,
                                 std::string                             prefix,
                                 unsigned int                            in_data_size,
                                 std::vector<LaserDataFilter::Buffer *> &inbufs)
{
	if (filter_type == "copy") {
		return new LaserCopyDataFilter(filter_name, in_data_size, inbufs);
	} else if (filter_type == "720to360") {
		bool average = false;
		try {
			average = config->get_bool((prefix + "average").c_str());
		} catch (Exception &e) {
		} // ignore
		return new Laser720to360DataFilter(filter_name, average, in_data_size, inbufs);
	} else if (filter_type == "1080to360") {
		bool average = false;
		try {
			average = config->get_bool((prefix + "average").c_str());
		} catch (Exception &e) {
		} // ignore
		return new Laser1080to360DataFilter(filter_name, average, in_data_size, inbufs);
	} else if (filter_type == "reverse") {
		return new LaserReverseAngleDataFilter(filter_name, in_data_size, inbufs);
	} else if (filter_type == "max_circle") {
		float radius = config->get_float((prefix + "radius").c_str());
		return new LaserMaxCircleDataFilter(filter_name, radius, in_data_size, inbufs);
	} else if (filter_type == "min_circle") {
		float radius = config->get_float((prefix + "radius").c_str());
		return new LaserMinCircleDataFilter(filter_name, radius, in_data_size, inbufs);
	} else if (filter_type == "circle_sector") {
		unsigned int from = config->get_uint((prefix + "from").c_str());
		unsigned int to   = config->get_uint((prefix + "to").c_str());
		return new LaserCircleSectorDataFilter(filter_name, from, to, in_data_size, inbufs);
	} else if (filter_type == "deadspots") {
		return new LaserDeadSpotsDataFilter(filter_name, config, logger, prefix, in_data_size, inbufs);
	} else if (filter_type == "min_merge") {
		std::string timestamp_selection;
		try {
			timestamp_selection = config->get_string((prefix + "timestamp_selection").c_str());
		} catch (Exception &e) {
		} // ignored, use default

		if (timestamp_selection == "latest") {
			return new LaserMinMergeDataFilter(
			  filter_name, logger, in_data_size, inbufs, LaserMinMergeDataFilter::TIMESTAMP_LATEST);
		} else if (timestamp_selection == "first") {
			return new LaserMinMergeDataFilter(
			  filter_name, logger, in_data_size, inbufs, LaserMinMergeDataFilter::TIMESTAMP_FIRST);
		} else if (timestamp_selection == "index") {
			unsigned int timestamp_if_index = config->get_uint((prefix + "timestamp_index").c_str());
			return new LaserMinMergeDataFilter(filter_name,
			                                   logger,
			                                   in_data_size,
			                                   inbufs,
			                                   LaserMinMergeDataFilter::TIMESTAMP_INDEX,
			                                   timestamp_if_index);
		} else if (timestamp_selection != "") {
			throw Exception("Laser filter: unknown timestamp selection method '%s'",
			                timestamp_selection.c_str());
		} else {
			return new LaserMinMergeDataFilter(
			  filter_name, logger, in_data_size, inbufs, LaserMinMergeDataFilter::TIMESTAMP_LATEST);
		}
	} else if (filter_type == "projection") {
#ifdef HAVE_TF
		const float       not_from_x  = config->get_float((prefix + "not_from_x").c_str());
		const float       not_to_x    = config->get_float((prefix + "not_to_x").c_str());
		const float       not_from_y  = config->get_float((prefix + "not_from_y").c_str());
		const float       not_to_y    = config->get_float((prefix + "not_to_y").c_str());
		const float       only_from_z = config->get_float((prefix + "only_from_z").c_str());
		const float       only_to_z   = config->get_float((prefix + "only_to_z").c_str());
		const std::string frame       = config->get_string((prefix + "target_frame").c_str());
		return new LaserProjectionDataFilter(filter_name,
		                                     tf_listener,
		                                     frame,
		                                     not_from_x,
		                                     not_to_x,
		                                     not_from_y,
		                                     not_to_y,
		                                     only_from_z,
		                                     only_to_z,
		                                     in_data_size,
		                                     inbufs);
#else
		throw Exception("Projection filter unavailable, tf missing");
#endif
	} else if (filter_type == "map_filter") {
#ifdef HAVE_TF
		return new LaserMapFilterDataFilter(
		  filter_name, in_data_size, inbufs, tf_listener, config, prefix, logger);
#else
		throw Exception("Projection filter unavailable, tf missing");
#endif
	} else if (filter_type == "box_filter") {
#ifdef HAVE_TF
		return new LaserBoxFilterDataFilter(
		  filter_name, in_data_size, inbufs, tf_listener, config, logger, blackboard);
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
	wait_threads_ = threads;
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
	wait_barrier_ = barrier;
}
