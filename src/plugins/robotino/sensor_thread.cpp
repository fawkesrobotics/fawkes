
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

#include <interfaces/BatteryInterface.h>
#include <interfaces/RobotinoSensorInterface.h>

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

  last_seqnum_ = 0;

  batt_if_ = blackboard->open_for_writing<BatteryInterface>("Robotino");
  sens_if_ = blackboard->open_for_writing<RobotinoSensorInterface>("Robotino");
}


void
RobotinoSensorThread::finalize()
{
  blackboard->close(sens_if_);
  blackboard->close(batt_if_);
  delete com_;
}

void
RobotinoSensorThread::loop()
{
  if (com_->isConnected()) {
    rec::iocontrol::remotestate::SensorState sensor_state = com_->sensorState();
    if (sensor_state.sequenceNumber != last_seqnum_) {
      last_seqnum_ = sensor_state.sequenceNumber;

      // update sensor values in interface
      sens_if_->set_mot_velocity(sensor_state.actualVelocity);
      sens_if_->set_mot_position(sensor_state.actualPosition);
      sens_if_->set_mot_current(sensor_state.motorCurrent);
      sens_if_->set_distance(sensor_state.distanceSensor);
      sens_if_->set_bumper(sensor_state.bumper);
      sens_if_->set_digital_in(sensor_state.dIn);
      sens_if_->set_analog_in(sensor_state.aIn);
      sens_if_->write();

      batt_if_->set_voltage(roundf(sensor_state.voltage * 1000.));
      batt_if_->set_current(roundf(sensor_state.current));

      // 21.0V is empty, 26.0V is empty, from OpenRobotino lcdd
      float soc = (sensor_state.voltage - 21.0f) / 5.f;
      soc = std::min(1.f, std::max(0.f, soc));

      batt_if_->set_absolute_soc(soc);
      batt_if_->write();

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
