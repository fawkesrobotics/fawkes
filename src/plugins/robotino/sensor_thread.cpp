
/***************************************************************************
 *  sensor_thread.cpp - Robotino sensor thread
 *
 *  Created: Sun Nov 13 15:35:24 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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
#include <rec/robotino/com/Com.h>
#include <rec/iocontrol/remotestate/SensorState.h>
#include <baseapp/run.h>

using namespace fawkes;

/** @class RobotinoSensorThread "sensor_thread.h"
 * Robotino sensor hook integration thread.
 * This thread integrates into the Fawkes main loop at the SENSOR hook and
 * writes new sensor data.
 * @author Tim Niemueller
 */

/** Constructor. */
RobotinoSensorThread::RobotinoSensorThread()
  : Thread("RobotinoSensorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR)
{
}


void
RobotinoSensorThread::init()
{
  cfg_hostname_ = config->get_string("/hardware/robotino/hostname");
  cfg_quit_on_disconnect_ = config->get_bool("/hardware/robotino/quit_on_disconnect");

  com_ = new rec::robotino::com::Com();
  com_->setAddress(cfg_hostname_.c_str());
  com_->connect(/* blocking */ false);
}


void
RobotinoSensorThread::finalize()
{
  delete com_;
}

void
RobotinoSensorThread::loop()
{
  if (com_->isConnected()) {
    rec::iocontrol::remotestate::SensorState sensor_state = com_->sensorState();
    if (sensor_state.sequenceNumber != last_seqnum_) {
      last_seqnum_ = sensor_state.sequenceNumber;
    }

  } else if (com_->connectionState() == rec::robotino::com::Com::NotConnected) {
    // retry connection
    if (cfg_quit_on_disconnect_) {
      fawkes::runtime::quit();
    } else {
      com_->connect(/* blocking */ false);
    }
  }
}
