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
 * Containing Implementation for one laser point interface.
 *
 */

/***********************************************************************
 *
 * $Id$
 *
 * Description: Contains the implementation for handling laser scans.
 *
 *
 * last modified: $Date$
 *            by: $Author$
 *
 **********************************************************************/

#include "robo_laserpoint.h"
#include "../geometry/trig_table.h"

//~ #include <cmath>

//~ using namespace std;

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
