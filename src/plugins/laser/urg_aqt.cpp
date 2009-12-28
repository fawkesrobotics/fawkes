
/***************************************************************************
 *  urg_aqt.cpp - Thread to retrieve laser data from Hokuyo URG
 *
 *  Created: Sat Nov 28 01:31:26 2009
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

#include "urg_aqt.h"

#include <core/threading/mutex.h>
#include <utils/time/wait.h>

#include <urg/UrgCtrl.h>
#include <urg/RangeSensorParameter.h>

#include <memory>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstdio>

using namespace qrk;
using namespace fawkes;



/** @class HokuyoUrgAcquisitionThread "urg_aqt.h"
 * Laser acqusition thread for Hokuyo URG laser range finders.
 * This thread fetches the data from the laser.
 * @author Tim Niemueller
 */


/** Constructor. */
HokuyoUrgAcquisitionThread::HokuyoUrgAcquisitionThread(std::string &cfg_name,
						       std::string &cfg_prefix)
  : LaserAcquisitionThread("HokuyoUrgAcquisitionThread")
{
  set_name("HokuyoURG(%s)", cfg_name.c_str());
  __pre_init_done = false;
  __cfg_name   = cfg_name;
  __cfg_prefix = cfg_prefix;
}


void
HokuyoUrgAcquisitionThread::pre_init(fawkes::Configuration *config,
				     fawkes::Logger        *logger)
{
  if (__pre_init_done)  return;

  __number_of_values = _distances_size = 360;

  __pre_init_done = true;
}

void
HokuyoUrgAcquisitionThread::init()
{
  pre_init(config, logger);

  __cfg_device = config->get_string((__cfg_prefix + "device").c_str());

  __ctrl = new UrgCtrl();
  std::auto_ptr<UrgCtrl> ctrl(__ctrl);
  if ( ! __ctrl->connect(__cfg_device.c_str()) ) {
    throw Exception("Connecting to URG laser failed: %s", __ctrl->what());
  }

  __ctrl->setCaptureMode(AutoCapture);

  std::vector<std::string> version_info;
  if (__ctrl->versionLines(version_info)) {
    for (unsigned int i = 0; i < version_info.size(); ++i) {
      std::string::size_type colon_idx      = version_info[i].find(":");
      std::string::size_type semi_colon_idx = version_info[i].find(";");
      if ((colon_idx == std::string::npos) ||
	  (semi_colon_idx == std::string::npos)) {
	logger->log_warn(name(), "Could not understand version info string '%s'",
			 version_info[i].c_str());
      } else {
	std::string::size_type val_len = semi_colon_idx - colon_idx - 1;
	std::string key   = version_info[i].substr(0, colon_idx);
	std::string value = version_info[i].substr(colon_idx+1, val_len);
	__device_info[key] = value;
	logger->log_info(name(), "%s: %s", key.c_str(), value.c_str());
      }
    }
  } else {
    throw Exception("Failed retrieving version info from device: %s", __ctrl->what());
  }

  if (__device_info.find("PROD") == __device_info.end()) {
    throw Exception("Failed to read product info for URG laser");
  }

  int scan_msec = __ctrl->scanMsec();

  try {
    __first_ray     = config->get_uint((__cfg_prefix + "first_ray").c_str());
    __last_ray      = config->get_uint((__cfg_prefix + "last_ray").c_str());
    __front_ray     = config->get_uint((__cfg_prefix + "front_ray").c_str());
    __slit_division = config->get_uint((__cfg_prefix + "slit_division").c_str());
  } catch (Exception &e) {
    logger->log_info(name(), "No or incomplete config data, reading from device");
    // Get data from device
    RangeSensorParameter p = __ctrl->parameter();
    __first_ray     = p.area_min;
    __last_ray      = p.area_max;
    __front_ray     = p.area_front;
    __slit_division = p.area_total;
  }

  __step_per_angle = __slit_division / 360.;
  __angle_per_step = 360. / __slit_division;
  __angular_range  = (__last_ray - __first_ray) * __angle_per_step;

  logger->log_info(name(), "Time per scan: %i msec", scan_msec);
  logger->log_info(name(), "Rays range:    %u..%u, front at %u",
		   __first_ray, __last_ray, __front_ray);
  logger->log_info(name(), "Slit Division: %u", __slit_division);
  logger->log_info(name(), "Step/Angle:    %f", __step_per_angle);
  logger->log_info(name(), "Angle/Step:    %f deg", __angle_per_step);
  logger->log_info(name(), "Angular Range: %f deg", __angular_range);

  // that should be 1000 really to convert msec -> usec. But empirically
  // the results are slightly better with 990 as factor.
  __timer = new TimeWait(clock, scan_msec * 990);

  alloc_distances(__number_of_values);

  ctrl.release();
}


void
HokuyoUrgAcquisitionThread::finalize()
{
  free(_distances);
  _distances = NULL;
  delete __timer;

  __ctrl->stop();
  delete __ctrl;

  logger->log_debug(name(), "Stopping laser");
}


void
HokuyoUrgAcquisitionThread::loop()
{
  __timer->mark_start();

  std::vector<long> values;
  int num_values = __ctrl->capture(values);
  if (num_values > 0) {
    //logger->log_debug(name(), "Captured %i values", num_values);
    _data_mutex->lock();

    _new_data = true;
    for (unsigned int a = 0; a < 360; ++a) {
      unsigned int front_idx = __front_ray + roundf(a * __step_per_angle);
      unsigned int idx = front_idx % __slit_division;
      if ( (idx >= __first_ray) && (idx <= __last_ray) ) {
	// 360-a: counter-clockwise -> clockwise
	// div by 1000.f: mm -> m
	_distances[360 - a] = values[idx] / 1000.f;
      }
    }
    _data_mutex->unlock();
  //} else {
    //logger->log_warn(name(), "No new scan available, ignoring");
  }

  __timer->wait();
}
