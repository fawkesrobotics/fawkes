
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
#  include <hokuyo_aist/hokuyo_aist.h>
#else
#  include <hokuyoaist/hokuyoaist.h>
#endif
#include <flexiport/flexiport.h>

#include <memory>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstdio>

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
  __pre_init_done = false;
  __cfg_name   = cfg_name;
  __cfg_prefix = cfg_prefix;
}


void
HokuyoUrgGbxAcquisitionThread::pre_init(fawkes::Configuration *config,
					fawkes::Logger        *logger)
{
  if (__pre_init_done)  return;

  __number_of_values = _distances_size = 360;

  __pre_init_done = true;
}

void
HokuyoUrgGbxAcquisitionThread::init()
{
  pre_init(config, logger);

  __cfg_device = config->get_string((__cfg_prefix + "device").c_str());

#ifdef HAVE_URG_GBX_9_11
  __laser = new HokuyoLaser();
#if __cplusplus >= 201103L
  std::unique_ptr<HokuyoLaser> laser(__laser);
#else
  std::auto_ptr<HokuyoLaser> laser(__laser);
#endif
#else
  __laser = new Sensor();
#if __cplusplus >= 201103L
  std::unique_ptr<Sensor> laser(__laser);
#else
  std::auto_ptr<Sensor> laser(__laser);
#endif
#endif
  std::string port_options = "type=serial,device=" + __cfg_device + ",timeout=1";
  try {
#ifdef HAVE_URG_GBX_9_11
    __laser->Open(port_options);
#else
    __laser->open(port_options);
#endif
  } catch (flexiport::PortException &e) {
    throw Exception("Connecting to URG laser failed: %s", e.what());
  }

#ifdef HAVE_URG_GBX_9_11
  HokuyoSensorInfo info;
  __laser->GetSensorInfo(&info);

  __data = new HokuyoData();
  __first_ray      = info.firstStep;
  __last_ray       = info.lastStep;
  __front_ray      = info.frontStep;

#else
  SensorInfo info;
  __laser->get_sensor_info(info);
  __data = new ScanData();

  __first_ray      = info.first_step;
  __last_ray       = info.last_step;
  __front_ray      = info.front_step;
#endif

  __slit_division  = info.steps;
  __num_rays       = __last_ray - __first_ray;
  __front_idx      = __front_ray - __first_ray;

  __step_per_angle = __slit_division / 360.;
  __angle_per_step = 360. / __slit_division;
  __angular_range  = (__last_ray - __first_ray) * __angle_per_step;

  logger->log_info(name(), "VEND: %s", info.vendor.c_str());
  logger->log_info(name(), "PROD: %s", info.product.c_str());
  logger->log_info(name(), "FIRM: %s", info.firmware.c_str());
  logger->log_info(name(), "PROT: %s", info.protocol.c_str());
  logger->log_info(name(), "SERI: %s", info.serial.c_str());
  logger->log_info(name(), "Rays range:    %u..%u, front at %u (idx %u), "
		   "%u rays total", __first_ray, __last_ray, __front_ray,
		   __front_idx, __num_rays);
  logger->log_info(name(), "Slit Division: %u", __slit_division);
  logger->log_info(name(), "Step/Angle:    %f", __step_per_angle);
  logger->log_info(name(), "Angle/Step:    %f deg", __angle_per_step);
  logger->log_info(name(), "Angular Range: %f deg", __angular_range);

  alloc_distances(__number_of_values);
#ifdef HAVE_URG_GBX_9_11
  __laser->SetPower(true);
#else
  __laser->set_power(true);
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
  __laser->SetPower(false);
#else
  __laser->set_power(false);
#endif
  delete __laser;
  delete __data;
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
    __laser->GetRanges(__data);
  } catch (HokuyoError &he) {
#else
    __laser->get_ranges(*__data);
  } catch (BaseError &he) {
#endif
    logger->log_warn(name(), "Failed to read data: %s", he.what());
    return;
  }

#ifdef HAVE_URG_GBX_9_11
  const uint32_t *ranges = __data->Ranges();
#else
  const uint32_t *ranges = __data->ranges();
#endif

  _data_mutex->lock();

  _new_data = true;
  _timestamp->stamp();
  for (unsigned int a = 0; a < 360; ++a) {
    unsigned int frontrel_idx = __front_idx + roundf(a * __step_per_angle);
    unsigned int idx = frontrel_idx % __slit_division;
    if ( idx <= __num_rays ) {
      // div by 1000.f: mm -> m
      _distances[a] = ranges[idx] / 1000.f;
    }
  }
  _data_mutex->unlock();
}
