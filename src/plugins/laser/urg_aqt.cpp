
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
HokuyoUrgAcquisitionThread::HokuyoUrgAcquisitionThread()
  : LaserAcquisitionThread("HokuyoUrgAcquisitionThread")
{
  __pre_init_done = false;
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

  __ctrl = new UrgCtrl();
  std::auto_ptr<UrgCtrl> ctrl(__ctrl);
  if ( ! __ctrl->connect("/dev/ttyACM0") ) {
    throw Exception("Connecting to URG laser failed: %s", __ctrl->what());
  }

  __ctrl->setCaptureMode(AutoCapture);

  int scan_msec = __ctrl->scanMsec();
  logger->log_info(name(), "Need %i msec per scan", scan_msec);
  __timer = new TimeWait(clock, scan_msec * 1000);

  _distances  = (float *)malloc(sizeof(float) * __number_of_values);

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
    logger->log_debug(name(), "Captured %i values", num_values);
    _data_mutex->lock();

    unsigned int start = 44;
    unsigned int end   = 725;
    unsigned int slit_division = 1024;
    float step_per_angle = 360. / slit_division;
    float angular_range  = (end - start) * step_per_angle;
    float angle_per_step = 1 / step_per_angle;

    logger->log_debug(name(), "start: %u  end: %u  slitdiv: %u  s/a: %f  a/s: %f  ar: %f", start, end, slit_division, step_per_angle, angle_per_step, angular_range);

    _new_data = true;
    for (unsigned int a = 0; a < 360; ++a) {
      unsigned int idx = roundf(a * angle_per_step);
      if ( (idx >= start) && (idx <= end) ) {
	_distances[a] = values[idx] / 1000.f;
      }
    }
    _data_mutex->unlock();
  }

  __timer->wait();
}
