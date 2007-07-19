
/***************************************************************************
 *  obstacle.cpp - Obstacle
 *
 *  Generated: Tue Jun 05 13:50:17 2007
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
 
#include <plugins/navigator/libnavi/obstacle.h>

#include <cmath>
 
/** @class Obstacle plugins/navigator/libnavi/obstacle.h
 *   An obstacle class for communicate such structures to the navigator GUI or others.
 *   
 *   @author Martin Liebenberg
 */
/** @var Obstacle::width
 *              The width of the obstacle.
 */
/** @var Obstacle::x
 *      The x-coordinate of the obstacle.
 */
/** @var Obstacle::y
 *      The y-coordinate of the obstacle.
 */
/** @var Obstacle::distance
 *              The distance between the robot and the obstacle.
 */
/** @var Obstacle::orientation_degree
 *      The orientation towards the obstacle in degree.
 */
/** @var Obstacle::orientation_rad
 *      The orientation towards the obstacle in rad.
 */
 
/** Constructor.
 * @param width the width of the obstacle
 * @param distance the distance between the robot and the obstacle
 * @param orientation_rad the orientation towards the obstacle in rad
 */
Obstacle::Obstacle(double width, double distance, double orientation_rad)
{
  this->width = width;
  this->distance = distance;
  this->orientation_rad = orientation_rad;
  this->x = distance * cos(orientation_rad);
  this->y = distance * sin(orientation_rad);
  this->orientation_degree = (180 * orientation_rad) / 3.14159265;
}        
         
/** Constructor.
 * @param width the width of the obstacle
 * @param x the x-coordinate of the obstacle
 * @param y the y-coordinate of the obstacle
 * @param foo to distinguish between this constructor and the above
 */
Obstacle::Obstacle(double width, double x, double y, double foo)
{
  this->width = width;
  this->distance = sqrt(pow(x, 2.) - pow(x, 2.));
  this->orientation_rad = atan2(y, x);
  this->x = x;
  this->y = y;
  this->orientation_degree = (180 * orientation_rad) / 3.14159265;
}
