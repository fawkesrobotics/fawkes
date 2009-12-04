
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
#include <utils/time/wait.h>

#include <hokuyo_aist/hokuyo_aist.h>
#include <flexiport/flexiport.h>

#include <memory>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstdio>

using namespace hokuyo_aist;
using namespace fawkes;


/** @class HokuyoUrgGbxAcquisitionThread "urg_gbx_aqt.h"
 * Laser acqusition thread for Hokuyo URG laser range finders.
 * This thread fetches the data from the laser. This implementation uses
 * the Gearbox library.
 * @author Tim Niemueller
 */


/** Constructor. */
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

  __cfg_device = config->get_bool((__cfg_prefix + "device").c_str());

  __laser = new HokuyoLaser();
  std::auto_ptr<HokuyoLaser> laser(__laser);
  std::string port_options = "type=serial:device=" + __cfg_device + ":timeout=1";
  try {
    __laser->Open(port_options);
  } catch (flexiport::PortException &e) {
    throw Exception("Connecting to URG laser failed: %s", e.what());
  }

  HokuyoSensorInfo info;
  __laser->GetSensorInfo(&info);

  __first_ray      = info.firstStep;
  __last_ray       = info.lastStep;
  __front_ray      = info.frontStep;
  __slit_division  = info.steps;

  __step_per_angle = __slit_division / 360.;
  __angle_per_step = 360. / __slit_division;
  __angular_range  = (__last_ray - __first_ray) * __angle_per_step;

  logger->log_info(name(), "VEND: %s", info.vendor.c_str());
  logger->log_info(name(), "PROD: %s", info.product.c_str());
  logger->log_info(name(), "FIRM: %s", info.firmware.c_str());
  logger->log_info(name(), "PROT: %s", info.protocol.c_str());
  logger->log_info(name(), "SERI: %s", info.serial.c_str());
  logger->log_info(name(), "Rays range:    %u..%u, front at %u",
		   __first_ray, __last_ray, __front_ray);
  logger->log_info(name(), "Slit Division: %u", __slit_division);
  logger->log_info(name(), "Step/Angle:    %f", __step_per_angle);
  logger->log_info(name(), "Angle/Step:    %f deg", __angle_per_step);
  logger->log_info(name(), "Angular Range: %f deg", __angular_range);

  //__timer = new TimeWait(clock, scan_msec * 1000);

  alloc_distances(__number_of_values);

  laser.release();
}


void
HokuyoUrgGbxAcquisitionThread::finalize()
{
  free(_distances);
  _distances = NULL;
  //delete __timer;

  logger->log_debug(name(), "Stopping laser");
  __laser->SetPower(false);
  delete __laser;

}


void
HokuyoUrgGbxAcquisitionThread::loop()
{
  //__timer->mark_start();

  __laser->GetNewRanges(__data);

  const uint32_t *ranges = __data->Ranges();

  //logger->log_debug(name(), "Captured %i values", num_values);
  _data_mutex->lock();

  _new_data = true;
  for (unsigned int a = 0; a < 360; ++a) {
    unsigned int front_idx = __front_ray + roundf(a * __step_per_angle);
    unsigned int idx = front_idx % __slit_division;
    if ( (idx >= __first_ray) && (idx <= __last_ray) ) {
      // 360-a: counter-clockwise -> clockwise
      // div by 1000.f: mm -> m
      _distances[360 - a] = ranges[idx] / 1000.f;
    }
  }
  _data_mutex->unlock();

//__timer->wait();
}
