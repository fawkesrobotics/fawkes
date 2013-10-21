
/***************************************************************************
 *  robo_laserpoint.cpp - Class for one laser point
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
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

#include "robo_laserpoint.h"
#include "../geometry/trig_table.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

LaserPoint::LaserPoint( int numberOfReadings ) throw (int)
{
  m_NumberOfReadings = numberOfReadings;
  m_pTrigTable = new TrigTable( 2 );
  m_pLaserPoint.reserve(numberOfReadings);
}


LaserPoint::~LaserPoint()
{
    m_pLaserPoint.clear();
}


float
LaserPoint::GetLength( int number )
{
  return (m_pLaserPoint[RangeCheck(number)].length);
}

float
LaserPoint::GetPosX  ( int number )
{
  return (m_pLaserPoint[RangeCheck(number)].posX);
}

float
LaserPoint::GetPosY  ( int number )
{
  return (m_pLaserPoint[RangeCheck(number)].posY);
}

float
LaserPoint::GetRadians  ( int number )
{
  return (m_pLaserPoint[RangeCheck(number)].rad);
}


void
LaserPoint::SetLength  ( int number, float length )
{
  m_pLaserPoint[RangeCheck(number)].length = length;
}

void
LaserPoint::SetPosX  ( int number, float posX)
{
  m_pLaserPoint[RangeCheck(number)].posX = posX;
}

void
LaserPoint::SetPosX  ( int number)
{
  number = RangeCheck(number);

  m_pLaserPoint[number].posX =
    m_pLaserPoint[number].length * m_pTrigTable->GetCos(m_pLaserPoint[number].rad);
}

void
LaserPoint::SetPosY  ( int number, float posY )
{
  m_pLaserPoint[RangeCheck(number)].posY = posY;
}

void
LaserPoint::SetPosY  ( int number )
{
  number = RangeCheck(number);

  m_pLaserPoint[number].posY =
    m_pLaserPoint[number].length * m_pTrigTable->GetSin(m_pLaserPoint[number].rad);
}

void
LaserPoint::SetPos  ( int number )
{
  SetPosX( number );
  SetPosY( number );
}

void
LaserPoint::SetRadians( int number, float radians )
{
  m_pLaserPoint[RangeCheck(number)].rad = radians;
}

int
LaserPoint::RangeCheck  ( int number )
{
  while (number < 0)
    number += m_NumberOfReadings;
  return (number % m_NumberOfReadings);
}

} // namespace fawkes
