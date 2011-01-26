
/***************************************************************************
 *  globfromrel.cpp - Implementation of the global ball position model
 *
 *  Created: Fri Jun 03 22:56:22 2005
 *  Copyright  2005       Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
 *             2005-2006  Tim Niemueller [www.niemueller.de]
 *             2005       Martin Heracles <Martin.Heracles@rwth-aachen.de>
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <cmath>
#include <fvmodels/global_position/globfromrel.h>
#include <fvmodels/relative_position/relativepositionmodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class GlobalFromRelativePos <fvmodels/global_position/globfromrel.h>
 * Calculate global ball position based on a relative position model.
 * The relative position model must of course be tied to the ball.
 */

/** Constructor.
 * @param model relative position model for the ball.
 */
GlobalFromRelativePos::GlobalFromRelativePos(RelativePositionModel* model)
{
  m_pRelaModel	= model;
  m_fPosX       = 0.0f;
  m_fPosY       = 0.0f;
  m_fPhi        = 0.0f;
}


void
GlobalFromRelativePos::set_robot_position(float x, float y, float ori)
{
  m_fPosX = x;
  m_fPosY = y;
  m_fPhi  = ori;
}


void
GlobalFromRelativePos::set_position_in_image(unsigned int x, unsigned int y)
{
}


void
GlobalFromRelativePos::calc()
{
}


bool
GlobalFromRelativePos::is_pos_valid() const
{
  return m_pRelaModel->is_pos_valid();
}


float
GlobalFromRelativePos::get_x() const
{
  /*
  cout << " DETAILS of \"getX()\"" << endl
       << "   Formula: " << endl
       << "     ( relX * cos(phi) -" << endl
       << "       relY * sin(phi)   ) + robX" << endl
       << "     ( " << m_pRelaModel->getX() << " * " << "cos(" << m_fPhi << ") -" << endl
       << "       " << m_pRelaModel->getY() << " * " << "sin(" << m_fPhi << ")   ) + " << m_fPosX << endl
       << "     ( " << m_pRelaModel->getX() << " * " << cos(m_fPhi) << ") -" << endl
       << "       " << m_pRelaModel->getY() << " * " << sin(m_fPhi) << "    ) + " << m_fPosX << endl
       << "     ( " << m_pRelaModel->getX() * cos(m_fPhi) << ") -" << endl
       << "       " << m_pRelaModel->getY() * sin(m_fPhi) << ")   ) + " << m_fPosX << endl 
       << "  ---> " << (m_pRelaModel->getX() * cos(m_fPhi) - m_pRelaModel->getY() * sin(m_fPhi)) + m_fPosX << flush;
  */
  return (   m_pRelaModel->get_x() * cos(m_fPhi)
	   - m_pRelaModel->get_y() * sin(m_fPhi) )
         + m_fPosX;
}


float
GlobalFromRelativePos::get_y() const
{
  /*
  cout << " DETAILS of \"getY()\"" << endl
       << "   Formula: " << endl
       << "     ( relX * sin(phi) -" << endl
       << "       relY * cos(phi)   ) + robY" << endl
       << "     ( " << m_pRelaModel->getX() << " * " << "sin(" << m_fPhi << ") +" << endl
       << "       " << m_pRelaModel->getY() << " * " << "cos(" << m_fPhi << ")   ) + " << m_fPosY << endl
       << "     ( " << m_pRelaModel->getX() << " * " << sin(m_fPhi) << ") +" << endl
       << "       " << m_pRelaModel->getY() << " * " << cos(m_fPhi) << "    ) + " << m_fPosY << endl
       << "     ( " << m_pRelaModel->getX() * sin(m_fPhi) << ") +" << endl
       << "       " << m_pRelaModel->getY() * cos(m_fPhi) << ")   ) + " << m_fPosY << endl 
       << "  ---> " << (m_pRelaModel->getX() * sin(m_fPhi) + m_pRelaModel->getY() * cos(m_fPhi)) + m_fPosY << flush;
  */
  return (   m_pRelaModel->get_x() * sin(m_fPhi)
	   + m_pRelaModel->get_y() * cos(m_fPhi) )
         + m_fPosY;
}

} // end namespace firevision
