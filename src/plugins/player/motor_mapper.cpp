
/***************************************************************************
 *  motor_mapper.cpp - Mapper Position2dProxy to MotorInterface
 *
 *  Created: Tue Sep 30 14:45:30 2008
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

#include "motor_mapper.h"

#include <interfaces/MotorInterface.h>
#include <libplayerc++/playerc++.h>
#include <cstdio>

using namespace PlayerCc;
using namespace fawkes;

/** @class PlayerMotorPositionMapper "motor_mapper.h"
 * Motor to position mapper for player integration.
 * This class is used to map a Player position2d proxy to a Fawkes
 * MotorInterface.
 *
 * It maps
 * - MotorInterface::GotoMessage to Position2dProxy::GoTo()
 * - MotorInterface::SetMotorStateMessage to Position2dProxy::SetMotorEnable()
 * - MotorInterface::ResetOdometryMessage to Position2dProxy::ResetOdometry()
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param varname variable name
 * @param interface Fawkes interface instance
 * @param proxy Player proxy instance
 */
PlayerMotorPositionMapper::PlayerMotorPositionMapper(std::string varname,
						     fawkes::MotorInterface *interface,
						     PlayerCc::Position2dProxy *proxy)
  : PlayerProxyFawkesInterfaceMapper(varname)
{
  __interface = interface;
  __proxy     = proxy;
}


void
PlayerMotorPositionMapper::sync_player_to_fawkes()
{
  if ( __proxy->IsFresh() ) {
    //printf("Setting %s to (%f, %f, %f)\n", varname().c_str(), __proxy->GetXPos(),
    //       __proxy->GetYPos(), __proxy->GetYaw());
    __interface->set_odometry_position_x(__proxy->GetXPos());
    __interface->set_odometry_position_y(__proxy->GetYPos());
    __interface->set_odometry_orientation(__proxy->GetYaw());
    __interface->write();
    __proxy->NotFresh();
  }
}

void
PlayerMotorPositionMapper::sync_fawkes_to_player()
{
  while ( ! __interface->msgq_empty() ) {
    if ( __interface->msgq_first_is<MotorInterface::SetMotorStateMessage>() ) {
      MotorInterface::SetMotorStateMessage *m = __interface->msgq_first<MotorInterface::SetMotorStateMessage>();
      __proxy->SetMotorEnable(m->motor_state() == MotorInterface::MOTOR_ENABLED);
    } else if ( __interface->msgq_first_is<MotorInterface::ResetOdometryMessage>() ) {
      __proxy->ResetOdometry();
    } else if ( __interface->msgq_first_is<MotorInterface::GotoMessage>() ) {
      MotorInterface::GotoMessage *m = __interface->msgq_first<MotorInterface::GotoMessage>();
      __proxy->GoTo(m->x(), m->y(), m->phi());
    }
    // all others are silently ignored

    __interface->msgq_pop();
  }
}
