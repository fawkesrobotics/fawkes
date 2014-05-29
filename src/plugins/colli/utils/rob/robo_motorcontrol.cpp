
/***************************************************************************
 *  robo_motorcontrol.cpp - Motor control wrapper
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
 *             2014  Tobias Neumann
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

#include "robo_motorcontrol.h"

#include <interfaces/MotorInterface.h>
#include <utils/math/angle.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MotorControl <plugins/colli/utils/rob/robo_motorcontrol.cpp>
 * This class is an interface to the obligatory MotorControl in the BlackBoard.
 */

MotorControl::MotorControl( fawkes::MotorInterface* motor )
{
  m_MotorControlDesiredTranslationX = 0;
  m_MotorControlDesiredTranslationY = 0;
  m_MotorControlDesiredRotation     = 0;
  m_pMopo = motor;
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


float MotorControl::GetMotorDesiredTranslationX()
{
  return m_pMopo->des_vx();
}

float MotorControl::GetMotorDesiredTranslationY()
{
  return m_pMopo->des_vy();
}


float MotorControl::GetMotorDesiredRotation()
{
  return m_pMopo->des_omega();
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



float MotorControl::GetUserDesiredTranslationX()
{
  return m_MotorControlDesiredTranslationX;
}

float MotorControl::GetUserDesiredTranslationY()
{
  return m_MotorControlDesiredTranslationY;
}


float MotorControl::GetUserDesiredRotation()
{
  return m_MotorControlDesiredRotation;
}


bool MotorControl::GetMovingAllowed()
{
  return m_pMopo->motor_state() == MotorInterface::MOTOR_ENABLED;
}


void MotorControl::SetDesiredTranslationX( float speed )
{
  m_MotorControlDesiredTranslationX = speed;
}

void MotorControl::SetDesiredTranslationY( float speed )
{
  m_MotorControlDesiredTranslationY = speed;
}


void MotorControl::SetDesiredRotation( float ori )
{
  m_MotorControlDesiredRotation = ori;
}


bool MotorControl::SendCommand()
{
  if ( m_pMopo->has_writer() ) {

    m_pMopo->msgq_enqueue(new MotorInterface::TransRotMessage(m_MotorControlDesiredTranslationX,
                                                              m_MotorControlDesiredTranslationY,
                                                              m_MotorControlDesiredRotation));
    return true;
  } else {
    return false;
  }
}

} // end of namespace fawkes
