
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

#include <interfaces/Laser1080Interface.h>
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
LaserSensorThread::LaserSensorThread(std::string &           cfg_name,
                                     std::string &           cfg_prefix,
                                     LaserAcquisitionThread *aqt)
: Thread("LaserSensorThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
	set_name("LaserSensorThread(%s)", cfg_name.c_str());
	aqt_        = aqt;
	cfg_name_   = cfg_name;
	cfg_prefix_ = cfg_prefix;
}

void
LaserSensorThread::init()
{
	laser360_if_  = NULL;
	laser720_if_  = NULL;
	laser1080_if_ = NULL;

	bool main_sensor = false;

	cfg_frame_ = config->get_string((cfg_prefix_ + "frame").c_str());

	try {
		main_sensor = config->get_bool((cfg_prefix_ + "main_sensor").c_str());
	} catch (Exception &e) {
	} // ignored, assume no

	aqt_->pre_init(config, logger);

	num_values_ = aqt_->get_distance_data_size();

	std::string if_id = main_sensor ? "Laser" : ("Laser " + cfg_name_);

	if (num_values_ == 360) {
		laser360_if_ = blackboard->open_for_writing<Laser360Interface>(if_id.c_str());
		laser360_if_->set_auto_timestamping(false);
		laser360_if_->set_frame(cfg_frame_.c_str());
		laser360_if_->write();
	} else if (num_values_ == 720) {
		laser720_if_ = blackboard->open_for_writing<Laser720Interface>(if_id.c_str());
		laser720_if_->set_auto_timestamping(false);
		laser720_if_->set_frame(cfg_frame_.c_str());
		laser720_if_->write();
	} else if (num_values_ == 1080) {
		laser1080_if_ = blackboard->open_for_writing<Laser1080Interface>(if_id.c_str());
		laser1080_if_->set_auto_timestamping(false);
		laser1080_if_->set_frame(cfg_frame_.c_str());
		laser1080_if_->write();
	} else {
		throw Exception("Laser acquisition thread must produce either 360, 720, or 1080 "
		                "distance values, but it produces %u",
		                aqt_->get_distance_data_size());
	}
}

void
LaserSensorThread::finalize()
{
	blackboard->close(laser360_if_);
	blackboard->close(laser720_if_);
	blackboard->close(laser1080_if_);
}

void
LaserSensorThread::loop()
{
	if (aqt_->lock_if_new_data()) {
		if (num_values_ == 360) {
			laser360_if_->set_timestamp(aqt_->get_timestamp());
			laser360_if_->set_distances(aqt_->get_distance_data());
			laser360_if_->write();
		} else if (num_values_ == 720) {
			laser720_if_->set_timestamp(aqt_->get_timestamp());
			laser720_if_->set_distances(aqt_->get_distance_data());
			laser720_if_->write();
		} else if (num_values_ == 1080) {
			laser1080_if_->set_timestamp(aqt_->get_timestamp());
			laser1080_if_->set_distances(aqt_->get_distance_data());
			laser1080_if_->write();
		}
		aqt_->unlock();
	}
}
