
/***************************************************************************
 *  map_range.c: Range routines
 *
 *  Created: Thu May 24 18:48:02 2012
 *  Copyright  2000  Brian Gerkey
 *             2000  Kasper Stoy
 *             2012  Tim Niemueller [www.niemueller.de]
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

/*  From:
 *  Player - One Hell of a Robot Server (LGPL)
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 */
/**************************************************************************
 * Desc: Range routines
 * Author: Andrew Howard
 * Date: 18 Jan 2003
**************************************************************************/

#include "map.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/// @cond EXTERNAL

// Extract a single range reading from the map.  Unknown cells and/or
// out-of-bound cells are treated as occupied, which makes it easy to
// use Stage bitmap files.
double
map_calc_range(map_t *map, double ox, double oy, double oa, double max_range)
{
	// Bresenham raytracing
	int  x0, x1, y0, y1;
	int  x, y;
	int  xstep, ystep;
	char steep;
	int  deltax, deltay, error, deltaerr;

	x0 = MAP_GXWX(map, ox);
	y0 = MAP_GYWY(map, oy);

	x1 = MAP_GXWX(map, ox + max_range * cos(oa));
	y1 = MAP_GYWY(map, oy + max_range * sin(oa));

	if (abs(y1 - y0) > abs(x1 - x0))
		steep = 1;
	else
		steep = 0;

	if (steep) {
		int tmp = x0;
		x0      = y0;
		y0      = tmp;

		tmp = x1;
		x1  = y1;
		y1  = tmp;
	}

	deltax   = abs(x1 - x0);
	deltay   = abs(y1 - y0);
	error    = 0;
	deltaerr = deltay;

	x = x0;
	y = y0;

	if (x0 < x1)
		xstep = 1;
	else
		xstep = -1;
	if (y0 < y1)
		ystep = 1;
	else
		ystep = -1;

	if (steep) {
		if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].occ_state > -1)
			return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
	} else {
		if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].occ_state > -1)
			return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
	}

	while (x != (x1 + xstep * 1)) {
		x += xstep;
		error += deltaerr;
		if (2 * error >= deltax) {
			y += ystep;
			error -= deltax;
		}

		if (steep) {
			if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].occ_state > -1)
				return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
		} else {
			if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].occ_state > -1)
				return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
		}
	}
	return max_range;
}

/// @endcond
