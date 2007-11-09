
/***************************************************************************
 *  npoint.cpp - Navigator Point
 *
 *  Generated: Tue Jun 05 13:57:09 2007
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */
 
#include <plugins/navigator/libnavi/npoint.h>
 
/** @class NPoint plugins/navigator/libnavi/npoint.h
 *   A point class for communicate such structures to the navigator GUI or others.
 *   
 *   @author Martin Liebenberg
 */
/** @var NPoint::x
 *      The x-coordinate of the point.
 */
/** @var NPoint::y
 *      The y-coordinate of the point.
 */
 
/** Constructor. */
NPoint::NPoint()
{
  x = 0.;
  y = 0.;
}
      
/** Constructor.
 * @param x the x-coordinate of the point
 * @param y the y-coordinate of the point
 */
NPoint::NPoint(double x, double y)
{
  this->x = x;
  this->y = y;
}


/** Destructor. */
NPoint::~NPoint()
{
}
        
/** Assign operator.
 * @param p the point to assign
 */ 
NPoint &NPoint::operator=(const NPoint &p)
{
  this->x = p.x;
  this->y = p.y; 
  return *this;
}
         
/** Comparison operator.
 * @param p the point to compare
 */
bool NPoint::operator==(const NPoint &p)
{
  return ((this->x == p.x) && (this->y == p.y));
}
