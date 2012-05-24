
/***************************************************************************
 *  map_draw.c: Local map GUI functions
 *
 *  Created: Thu May 24 18:46:58 2012
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
 * Desc: Local map GUI functions
 * Author: Andrew Howard
 * Date: 18 Jan 2003
**************************************************************************/

#ifdef INCLUDE_RTKGUI

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <rtk.h>
#include "map.h"

/// @cond EXTERNAL

////////////////////////////////////////////////////////////////////////////
// Draw the occupancy map
void map_draw_occ(map_t *map, rtk_fig_t *fig)
{
  int i, j;
  int col;
  map_cell_t *cell;
  uint16_t *image;
  uint16_t *pixel;

  image = malloc(map->size_x * map->size_y * sizeof(image[0]));

  // Draw occupancy
  for (j = 0; j < map->size_y; j++)
  {
    for (i =  0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      pixel = image + (j * map->size_x + i);

      col = 127 - 127 * cell->occ_state;
      *pixel = RTK_RGB16(col, col, col);
    }
  }

  // Draw the entire occupancy map as an image
  rtk_fig_image(fig, map->origin_x, map->origin_y, 0,
                map->scale, map->size_x, map->size_y, 16, image, NULL);

  free(image);

  return;
}


////////////////////////////////////////////////////////////////////////////
// Draw the cspace map
void map_draw_cspace(map_t *map, rtk_fig_t *fig)
{
  int i, j;
  int col;
  map_cell_t *cell;
  uint16_t *image;
  uint16_t *pixel;

  image = malloc(map->size_x * map->size_y * sizeof(image[0]));

  // Draw occupancy
  for (j = 0; j < map->size_y; j++)
  {
    for (i =  0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      pixel = image + (j * map->size_x + i);

      col = 255 * cell->occ_dist / map->max_occ_dist;

      *pixel = RTK_RGB16(col, col, col);
    }
  }

  // Draw the entire occupancy map as an image
  rtk_fig_image(fig, map->origin_x, map->origin_y, 0,
                map->scale, map->size_x, map->size_y, 16, image, NULL);

  free(image);

  return;
}


////////////////////////////////////////////////////////////////////////////
// Draw a wifi map
void map_draw_wifi(map_t *map, rtk_fig_t *fig, int index)
{
  int i, j;
  int level, col;
  map_cell_t *cell;
  uint16_t *image, *mask;
  uint16_t *ipix, *mpix;

  image = malloc(map->size_x * map->size_y * sizeof(image[0]));
  mask = malloc(map->size_x * map->size_y * sizeof(mask[0]));

  // Draw wifi levels
  for (j = 0; j < map->size_y; j++)
  {
    for (i =  0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      ipix = image + (j * map->size_x + i);
      mpix = mask + (j * map->size_x + i);

      level = cell->wifi_levels[index];

      if (cell->occ_state == -1 && level != 0)
      {
        col = 255 * (100 + level) / 100;
        *ipix = RTK_RGB16(col, col, col);
        *mpix = 1;
      }
      else
      {
        *mpix = 0;
      }
    }
  }

  // Draw the entire occupancy map as an image
  rtk_fig_image(fig, map->origin_x, map->origin_y, 0,
                map->scale, map->size_x, map->size_y, 16, image, mask);

  free(mask);
  free(image);

  return;
}

/// @endcond

#endif
