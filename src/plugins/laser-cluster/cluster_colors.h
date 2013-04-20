
/***************************************************************************
 *  cluster_colors.h - Visualization cluster colors
 *
 *  Created: Mon Nov 14 12:06:39 2011 (from tabletop-objects)
 *  Copyright  2011-2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_CLUSTER_CLUSTER_COLORS_H_
#define __PLUGINS_LASER_CLUSTER_CLUSTER_COLORS_H_

#include <stdint.h>

#define MAX_CENTROIDS 12

static const uint8_t table_color[3] = {0, 100, 0};

static const uint8_t cluster_colors[MAX_CENTROIDS][3] =
  { {176, 0, 30}, {255, 90, 0}, {137, 82, 39}, {56, 23, 90}, {99, 0, 30}, {255, 0, 0},
    {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {27, 117, 196}};

#endif
