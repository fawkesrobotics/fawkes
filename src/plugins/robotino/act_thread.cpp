
/***************************************************************************
 *  act_thread.cpp - Robotino act thread
 *
 *  Created: Sun Nov 13 16:07:40 2011
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

#include "act_thread.h"
#include "sensor_thread.h"

#include <interfaces/MotorInterface.h>
#include <utils/math/angle.h>

#include <rec/robotino/com/Com.h>
#include <rec/robotino/com/OmniDrive.h>
#include <rec/iocontrol/remotestate/SensorState.h>


using namespace fawkes;

/** @class RobotinoActThread "act_thread.h"
 * Robotino act hook integration thread.
 * This thread integrates into the Fawkes main loop at the ACT hook and
 * executes motion commands.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param sensor_thread sensor thread from which to get the Com instance
 */
RobotinoActThread::RobotinoActThread(RobotinoSensorThread *sensor_thread)
  : Thread("RobotinoActThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
  sensor_thread_ = sensor_thread;
}


void
RobotinoActThread::init()
{
  com_ = sensor_thread_->com_;
  omni_drive_ = new rec::robotino::com::OmniDrive();

  last_seqnum_ = 0;

  // * 1000: OpenRobotino takes mm/sec
  cfg_max_vx_    = config->get_float("/hardware/robotino/max_vx") * 1000.;
  cfg_max_vy_    = config->get_float("/hardware/robotino/max_vy") * 1000.;
  cfg_max_omega_ = config->get_float("/hardware/robotino/max_omega");

  motor_if_ = blackboard->open_for_writing<MotorInterface>("Robotino");
}


void
RobotinoActThread::finalize()
{
  blackboard->close(motor_if_);
  if (com_->isConnected()) {
    rec::iocontrol::remotestate::SetState set_state;
    set_state.speedSetPoint[0] = 0.;
    set_state.speedSetPoint[1] = 0.;
    set_state.speedSetPoint[2] = 0.;

    com_->setSetState( set_state );
  }
  com_ = NULL;
}


void
RobotinoActThread::loop()
{
  if (com_->isConnected()) {
    rec::iocontrol::remotestate::SetState set_state;
    bool send_set_state = false;
    while (! motor_if_->msgq_empty()) {

      if (MotorInterface::TransRotMessage *msg =
          motor_if_->msgq_first_safe(msg))
      {
        float m1, m2, m3;
        omni_drive_->project(&m1, &m2, &m3,
                             msg->vx() * cfg_max_vx_,
			     msg->vy() * cfg_max_vy_,
			     msg->omega() * cfg_max_omega_);

        set_state.speedSetPoint[0] = m1;
        set_state.speedSetPoint[1] = m2;
        set_state.speedSetPoint[2] = m3;
        send_set_state = true;
      }
      else if (motor_if_->msgq_first_is<MotorInterface::ResetOdometryMessage>())
      {
        set_state.resetPosition[0] = set_state.resetPosition[1] =
          set_state.resetPosition[2] = true;
        send_set_state = true;
      }

      motor_if_->msgq_pop();
    }

    if (send_set_state)  com_->setSetState( set_state );        

    rec::iocontrol::remotestate::SensorState sensor_state = com_->sensorState();
    if (sensor_state.sequenceNumber != last_seqnum_) {
      motor_if_->set_odometry_position_x(sensor_state.odometryX / 1000.f);
      motor_if_->set_odometry_position_y(sensor_state.odometryY / 1000.f);
      motor_if_->set_odometry_orientation(deg2rad(sensor_state.odometryPhi));
      motor_if_->write();
      last_seqnum_ = sensor_state.sequenceNumber;
    }

  } else {
    if (! motor_if_->msgq_empty()) {
      logger->log_warn(name(), "Motor commands received while not connected");
      motor_if_->msgq_flush();
    }
  }
}
