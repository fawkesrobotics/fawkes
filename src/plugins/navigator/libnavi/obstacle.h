
/***************************************************************************
 *  obstacle.h - Obstacle
 *
 *  Generated: Tue Jun 05 13:50:17 2007
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __NAVIGATOR_LIBNAVI_OBSTACLE_H_
#define __NAVIGATOR_LIBNAVI_OBSTACLE_H_

#include <plugins/navigator/libnavi/npoint.h>

class Obstacle : public NPoint
{
 public:
  Obstacle(double width, double distance, double orientation_rad);
         
  Obstacle(double width, double x, double y, double foo);
         
  double width;
  double distance;
  double orientation_degree;
  double orientation_rad;
};
 
#endif
