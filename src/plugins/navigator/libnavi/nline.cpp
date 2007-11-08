
/***************************************************************************
 *  nline.cpp - Navigator Line
 *
 *  Generated: Tue Jun 05 13:58:18 2007
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

#include <plugins/navigator/libnavi/nline.h>
#include <plugins/navigator/libnavi/obstacle.h>

/** @class NLine <plugins/navigator/libnavi/nline.h>
 * Class representing a line for the navigator.
 * A line class for communicate such structures to the navigator GUI or others.
 * This class defines a line by its two end-points.
 * @author Martin Liebenberg 
 */
/** @var NPoint NLine::p1
 * Point 1 of the line. 
 */
/** @var NPoint NLine::p2
 * Point 2 of the line. 
 */

/** Contructor. */
NLine::NLine()
{
  p1 = new NPoint();
  p2 = new NPoint();
}

/** Constructor.
 * @param p1 point 1 of the line
 * @param p2 point 2 of the line 
 */
NLine::NLine(NPoint *p1, NPoint *p2)
{
  Obstacle *op1 = dynamic_cast<Obstacle *>(p1);
  if ( op1 )
    {
      this->p1 = new Obstacle(op1->width, p1->x, p1->y, 0);
    }
  else
    {
      this->p1 = new NPoint(p1->x, p1->y);
    }
  Obstacle *op2 = dynamic_cast<Obstacle *>(p2);
  if ( op2 )
    {
      this->p2 = new Obstacle(op2->width, p2->x, p2->y, 0);
    }
  else
    {
      this->p2 = new NPoint(p2->x, p2->y);
    }
}

/** Constructor.
 * @param x1 the x-coordinate of point 1 of the line
 * @param y1 the y-coordinate of point 1 of the line
 * @param x2 the x-coordinate of point 2 of the line
 * @param y2 the y-coordinate of point 2 of the line 
 */
NLine::NLine(double x1, double y1, double x2, double y2)
{
  p1 = new NPoint(x1, y1);
  p2 = new NPoint(x2, y2);
}

