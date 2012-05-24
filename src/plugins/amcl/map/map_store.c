
/***************************************************************************
 *  map_store.c: Global map storage functions
 *
 *  Created: Thu May 24 18:48:43 2012
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
 * Desc: Global map storage functions
 * Author: Andrew Howard
 * Date: 6 Feb 2003
**************************************************************************/

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "map.h"

/// @cond EXTERNAL

////////////////////////////////////////////////////////////////////////////
// Load an occupancy grid
int map_load_occ(map_t *map, const char *filename, double scale, int negate)
{
  FILE *file;
  char magic[3];
  int i, j;
  int ch, occ;
  int width, height, depth;
  map_cell_t *cell;

  // Open file
  file = fopen(filename, "r");
  if (file == NULL)
  {
    fprintf(stderr, "%s: %s\n", strerror(errno), filename);
    return -1;
  }

  // Read ppm header
  
  if ((fscanf(file, "%10s \n", magic) != 1) || (strcmp(magic, "P5") != 0))
  {
    fprintf(stderr, "incorrect image format; must be PGM/binary");
    return -1;
  }

  // Ignore comments
  while ((ch = fgetc(file)) == '#')
    while (fgetc(file) != '\n');
  ungetc(ch, file);

  // Read image dimensions
  if(fscanf(file, " %d %d \n %d \n", &width, &height, &depth) != 3)
  {
    fprintf(stderr, "Failed ot read image dimensions");
    return -1;
  }

  // Allocate space in the map
  if (map->cells == NULL)
  {
    map->scale = scale;
    map->size_x = width;
    map->size_y = height;
    map->cells = calloc(width * height, sizeof(map->cells[0]));
  }
  else
  {
    if (width != map->size_x || height != map->size_y)
    {
      //PLAYER_ERROR("map dimensions are inconsistent with prior map dimensions");
      return -1;
    }
  }

  // Read in the image
  for (j = height - 1; j >= 0; j--)
  {
    for (i = 0; i < width; i++)
    {
      ch = fgetc(file);

      // Black-on-white images
      if (!negate)
      {
        if (ch < depth / 4)
          occ = +1;
        else if (ch > 3 * depth / 4)
          occ = -1;
        else
          occ = 0;
      }

      // White-on-black images
      else
      {
        if (ch < depth / 4)
          occ = -1;
        else if (ch > 3 * depth / 4)
          occ = +1;
        else
          occ = 0;
      }

      if (!MAP_VALID(map, i, j))
        continue;
      cell = map->cells + MAP_INDEX(map, i, j);
      cell->occ_state = occ;
    }
  }
  
  fclose(file);
  
  return 0;
}


////////////////////////////////////////////////////////////////////////////
// Load a wifi signal strength map
/*
int map_load_wifi(map_t *map, const char *filename, int index)
{
  FILE *file;
  char magic[3];
  int i, j;
  int ch, level;
  int width, height, depth;
  map_cell_t *cell;

  // Open file
  file = fopen(filename, "r");
  if (file == NULL)
  {
    fprintf(stderr, "%s: %s\n", strerror(errno), filename);
    return -1;
  }

  // Read ppm header
  fscanf(file, "%10s \n", magic);
  if (strcmp(magic, "P5") != 0)
  {
    fprintf(stderr, "incorrect image format; must be PGM/binary");
    return -1;
  }

  // Ignore comments
  while ((ch = fgetc(file)) == '#')
    while (fgetc(file) != '\n');
  ungetc(ch, file);

  // Read image dimensions
  fscanf(file, " %d %d \n %d \n", &width, &height, &depth);

  // Allocate space in the map
  if (map->cells == NULL)
  {
    map->size_x = width;
    map->size_y = height;
    map->cells = calloc(width * height, sizeof(map->cells[0]));
  }
  else
  {
    if (width != map->size_x || height != map->size_y)
    {
      //PLAYER_ERROR("map dimensions are inconsistent with prior map dimensions");
      return -1;
    }
  }

  // Read in the image
  for (j = height - 1; j >= 0; j--)
  {
    for (i = 0; i < width; i++)
    {
      ch = fgetc(file);

      if (!MAP_VALID(map, i, j))
        continue;

      if (ch == 0)
        level = 0;
      else
        level = ch * 100 / 255 - 100;

      cell = map->cells + MAP_INDEX(map, i, j);
      cell->wifi_levels[index] = level;
    }
  }
  
  fclose(file);

  return 0;
}
*/

/// @endcond
