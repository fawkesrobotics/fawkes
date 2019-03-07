
/***************************************************************************
 *  urg_gbx_aqt.cpp - Thread for Hokuyo URG using the Gearbox library
 *
 *  Created: Fri Dec 04 20:47:50 2009 (at Frankfurt Airport)
 *  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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

#include "urg_gbx_aqt.h"

#include <core/threading/mutex.h>

#ifdef HAVE_URG_GBX_9_11
#	include <hokuyo_aist/hokuyo_aist.h>
#else
#	include <hokuyoaist/hokuyoaist.h>
#endif
#include <flexiport/flexiport.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#ifdef HAVE_URG_GBX_9_11
using namespace hokuyo_aist;
#else
using namespace hokuyoaist;
#endif
using namespace fawkes;

/** @class HokuyoUrgGbxAcquisitionThread "urg_gbx_aqt.h"
 * Laser acqusition thread for Hokuyo URG laser range finders.
 * This thread fetches the data from the laser. This implementation uses
 * the Gearbox library.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 */
HokuyoUrgGbxAcquisitionThread::HokuyoUrgGbxAcquisitionThread(std::string &cfg_name,
                                                             std::string &cfg_prefix)
: LaserAcquisitionThread("HokuyoUrgGbxAcquisitionThread")
{
	set_name("HokuyoURG_GBX(%s)", cfg_name.c_str());
	pre_init_done_ = false;
	cfg_name_      = cfg_name;
	cfg_prefix_    = cfg_prefix;
}

void
HokuyoUrgGbxAcquisitionThread::pre_init(fawkes::Configuration *config, fawkes::Logger *logger)
{
	if (pre_init_done_)
		return;

	number_of_values_ = _distances_size = 360;

	pre_init_done_ = true;
}

void
HokuyoUrgGbxAcquisitionThread::init()
{
	pre_init(config, logger);

	cfg_device_ = config->get_string((cfg_prefix_ + "device").c_str());

#ifdef HAVE_URG_GBX_9_11
	laser_ = new HokuyoLaser();
#	if __cplusplus >= 201103L
	std::unique_ptr<HokuyoLaser> laser(laser_);
#	else
	std::auto_ptr<HokuyoLaser> laser(laser_);
#	endif
#else
	laser_ = new Sensor();
#	if __cplusplus >= 201103L
	std::unique_ptr<Sensor> laser(laser_);
#	else
	std::auto_ptr<Sensor> laser(laser_);
#	endif
#endif
	std::string port_options = "type=serial,device=" + cfg_device_ + ",timeout=1";
	try {
#ifdef HAVE_URG_GBX_9_11
		laser_->Open(port_options);
#else
		laser_->open(port_options);
#endif
	} catch (flexiport::PortException &e) {
		throw Exception("Connecting to URG laser failed: %s", e.what());
	}

#ifdef HAVE_URG_GBX_9_11
	HokuyoSensorInfo info;
	laser_->GetSensorInfo(&info);

	data_      = new HokuyoData();
	first_ray_ = info.firstStep;
	last_ray_  = info.lastStep;
	front_ray_ = info.frontStep;

#else
	SensorInfo info;
	laser_->get_sensor_info(info);
	data_ = new ScanData();

	first_ray_ = info.first_step;
	last_ray_  = info.last_step;
	front_ray_ = info.front_step;
#endif

	slit_division_ = info.steps;
	num_rays_      = last_ray_ - first_ray_;
	front_idx_     = front_ray_ - first_ray_;

	step_per_angle_ = slit_division_ / 360.;
	angle_per_step_ = 360. / slit_division_;
	angular_range_  = (last_ray_ - first_ray_) * angle_per_step_;

	logger->log_info(name(), "VEND: %s", info.vendor.c_str());
	logger->log_info(name(), "PROD: %s", info.product.c_str());
	logger->log_info(name(), "FIRM: %s", info.firmware.c_str());
	logger->log_info(name(), "PROT: %s", info.protocol.c_str());
	logger->log_info(name(), "SERI: %s", info.serial.c_str());
	logger->log_info(name(),
	                 "Rays range:    %u..%u, front at %u (idx %u), "
	                 "%u rays total",
	                 first_ray_,
	                 last_ray_,
	                 front_ray_,
	                 front_idx_,
	                 num_rays_);
	logger->log_info(name(), "Slit Division: %u", slit_division_);
	logger->log_info(name(), "Step/Angle:    %f", step_per_angle_);
	logger->log_info(name(), "Angle/Step:    %f deg", angle_per_step_);
	logger->log_info(name(), "Angular Range: %f deg", angular_range_);

	alloc_distances(number_of_values_);
#ifdef HAVE_URG_GBX_9_11
	laser_->SetPower(true);
#else
	laser_->set_power(true);
#endif

	laser.release();
}

void
HokuyoUrgGbxAcquisitionThread::finalize()
{
	free(_distances);
	_distances = NULL;

	logger->log_debug(name(), "Stopping laser");
#ifdef HAVE_URG_GBX_9_11
	laser_->SetPower(false);
#else
	laser_->set_power(false);
#endif
	delete laser_;
	delete data_;
}

void
HokuyoUrgGbxAcquisitionThread::loop()
{
	// static Time ref(clock);
	// static Time now(clock);
	// static unsigned int scans = 0;

	// now.stamp();
	// if (now - &ref >= 1) {
	//   logger->log_debug(name(), "Current: %u scans/sec", scans);
	//   scans = 0;
	//   ref = now;
	// } else {
	//   ++scans;
	// }

	try {
		// GetNewRanges is causes scans/sec to be halfed
#ifdef HAVE_URG_GBX_9_11
		laser_->GetRanges(data_);
	} catch (HokuyoError &he) {
#else
		laser_->get_ranges(*data_);
	} catch (BaseError &he) {
#endif
		logger->log_warn(name(), "Failed to read data: %s", he.what());
		return;
	}

#ifdef HAVE_URG_GBX_9_11
	const uint32_t *ranges = data_->Ranges();
#else
	const uint32_t *ranges = data_->ranges();
#endif

	_data_mutex->lock();

	_new_data = true;
	_timestamp->stamp();
	for (unsigned int a = 0; a < 360; ++a) {
		unsigned int frontrel_idx = front_idx_ + roundf(a * step_per_angle_);
		unsigned int idx          = frontrel_idx % slit_division_;
		if (idx <= num_rays_) {
			// div by 1000.f: mm -> m
			_distances[a] = ranges[idx] / 1000.f;
		}
	}
	_data_mutex->unlock();
}
