
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

/** @class LaserPoint <plugins/colli/utils/rob/robo_laserpoint.h>
 * This class contains methods for handling laser points data.
 */

/** Constructor.
 * @param numberOfReadings is the number of scan-readings the scan should hold.
 * @param * dbg is the an instance of the Debug-Class.
 *  \exception (int 1) the could not be instanced.
 */
LaserPoint::LaserPoint( int numberOfReadings ) throw (int)
{
  m_NumberOfReadings = numberOfReadings;
  m_pTrigTable = new TrigTable( 2 );
  m_pLaserPoint.reserve(numberOfReadings);
}

/** Destructor. */
LaserPoint::~LaserPoint()
{
    m_pLaserPoint.clear();
}

/** Returns the number-reading length.
 * @param number is the readings number.
 * @return float is the length.
 */
float
LaserPoint::GetLength( int number )
{
  return (m_pLaserPoint[RangeCheck(number)].length);
}

/** Returns the number-reading radians.
 * @param number is the readings number.
 * @return float is the angle in rad.
 */
float
LaserPoint::GetRadians( int number )
{
  return (m_pLaserPoint[RangeCheck(number)].rad);
}

/** Returns the number-reading x coordinate.
 * @param number is the readings number.
 * @return float is the x coordinate.
 */
float
LaserPoint::GetPosX( int number )
{
  return (m_pLaserPoint[RangeCheck(number)].posX);
}

/** Returns the number-reading y coordinate.
 * @param number is the readings number.
 * @return float is the y coordinate.
 */
float
LaserPoint::GetPosY( int number )
{
  return (m_pLaserPoint[RangeCheck(number)].posY);
}

/** Sets the number-readings length.
 * @param number is the readings number.
 * @param length is the readings length.
 */
void
LaserPoint::SetLength( int number, float length )
{
  m_pLaserPoint[RangeCheck(number)].length = length;
}

/** Sets the number-readings angle.
 * @param number is the readings number.
 * @param radians is the readings angle in rad.
 */
void
LaserPoint::SetRadians( int number, float radians )
{
  m_pLaserPoint[RangeCheck(number)].rad = radians;
}

/** Sets the number-readings x coordinate.
 * @param number is the readings number.
 * @param posX is the readings x coordinate.
 */
void
LaserPoint::SetPosX( int number, float posX)
{
  m_pLaserPoint[RangeCheck(number)].posX = posX;
}

/** Computes the number-readings x coordinate by the number-readings
 * radians and length and finally sets the computed value.
 * @param number is the readings number.
 */
void
LaserPoint::SetPosX( int number)
{
  number = RangeCheck(number);

  m_pLaserPoint[number].posX =
    m_pLaserPoint[number].length * m_pTrigTable->GetCos(m_pLaserPoint[number].rad);
}

/** Sets the number-readings x coordinate.
 * @param number is the readings number.
 * @param posY is the readings y coordinate.
 */
void
LaserPoint::SetPosY( int number, float posY )
{
  m_pLaserPoint[RangeCheck(number)].posY = posY;
}

/** Computes the number-readings y coordinate by the number-readings
 * radians and length and finally sets the computed value.
 * @param number is the readings number.
 */
void
LaserPoint::SetPosY( int number )
{
  number = RangeCheck(number);

  m_pLaserPoint[number].posY =
    m_pLaserPoint[number].length * m_pTrigTable->GetSin(m_pLaserPoint[number].rad);
}

/** Computes the number-readings coordinates by the number-readings
 * radians and length and finally sets the computed values.
 * @param number is the readings number.
 */
void
LaserPoint::SetPos( int number )
{
  SetPosX( number );
  SetPosY( number );
}



int
LaserPoint::RangeCheck( int number )
{
  while (number < 0)
    number += m_NumberOfReadings;
  return (number % m_NumberOfReadings);
}

} // namespace fawkes
