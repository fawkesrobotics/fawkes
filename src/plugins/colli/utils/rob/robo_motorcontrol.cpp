/*
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
  ©                                                                            ©
  ©                                            ####   ####           .-""-.    ©
  ©       # #                             #   #    # #    #         /[] _ _\   ©
  ©       # #                                 #    # #             _|_o_LII|_  ©
  © ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ ©
  © #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| ©
  © #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  ©
  © #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  ©
  © '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  ©
  ©                                                               /__|    |__\ ©
  ©                                                                            ©
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
*/

/* Written by Stefan Jacobs
 * for module Colli-A*
 *
 * Containing Implementation for motor interface.
 *
 */

/***********************************************************************
 *
 * $Id$
 *
 * description:
 *
 * last modification: $Date$
 *         by author: $Author$
 *
 **********************************************************************/
#include "robo_motorcontrol.h"

#include <interfaces/MotorInterface.h>
#include <utils/math/angle.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


MotorControl::MotorControl( fawkes::MotorInterface* motor, fawkes::MotorInterface* motor_des )
{
  m_pMopo = motor;
  m_pMopo_des = motor_des;
  SetEmergencyStop();
}



MotorControl::~MotorControl()
{
}


float MotorControl::GetCurrentX()
{
  return m_pMopo->odometry_position_x();
}


float MotorControl::GetCurrentY()
{
  return m_pMopo->odometry_position_y();
}


float MotorControl::GetCurrentOri()
{
  return normalize_mirror_rad( m_pMopo->odometry_orientation() );
}


float MotorControl::GetMotorDesiredTranslation()
{
  float vx = m_pMopo_des->vx();
  float vy = m_pMopo_des->vy();
  float speed = sqrt(vx*vx + vy*vy);

  if ( vx > 0 )
    return speed;
  else
    return -speed;
}


float MotorControl::GetMotorDesiredRotation()
{
  return m_pMopo_des->omega();
}


float MotorControl::GetMotorCurrentTranslation()
{
  float vx = m_pMopo->vx();
  float vy = m_pMopo->vy();
  float speed = sqrt(vx*vx + vy*vy);

  if ( vx > 0 )
    return speed;
  else
    return -speed;
}


float MotorControl::GetMotorCurrentRotation()
{
  return m_pMopo->omega();
}



float MotorControl::GetUserDesiredTranslation()
{
  return m_MotorControlDesiredTranslation;
}


float MotorControl::GetUserDesiredRotation()
{
  return m_MotorControlDesiredRotation;
}


bool MotorControl::GetMovingAllowed()
{
  return m_pMopo->motor_state() == MotorInterface::MOTOR_ENABLED;
}


void MotorControl::SetDesiredTranslation( float speed )
{
  m_MotorControlDesiredTranslation = speed;
}


void MotorControl::SetDesiredRotation( float ori )
{
  m_MotorControlDesiredRotation = ori;
}


bool MotorControl::SendCommand()
{
  if ( m_MovingAllowed ) {
    m_pMopo->msgq_enqueue(new MotorInterface::TransRotMessage(m_MotorControlDesiredTranslation, 0,
                                                              m_MotorControlDesiredRotation));
    return true;
  } else {
    return false;
  }
}


void MotorControl::SetEmergencyStop()
{
  m_MovingAllowed = false;
  m_pMopo->msgq_enqueue(new MotorInterface::SetMotorStateMessage(MotorInterface::MOTOR_DISABLED));
}


void MotorControl::SetRecoverEmergencyStop()
{
  m_MovingAllowed = true;
  m_pMopo->msgq_enqueue(new MotorInterface::SetMotorStateMessage(MotorInterface::MOTOR_ENABLED));
}

} // end of namespace fawkes
