
/***************************************************************************
 *  cspace.cpp: Cached distance map
 *
 *  Created: Thu May 24 18:46:12 2012
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

#include <queue>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "map.h"

/// @cond EXTERNAL

class CellData
{
  public:
    map_t* map_;
    unsigned int i_, j_;
    unsigned int src_i_, src_j_;
};

class CachedDistanceMap
{
  public:
    CachedDistanceMap(double scale, double max_dist) : 
      distances_(NULL), scale_(scale), max_dist_(max_dist) 
    {
      cell_radius_ = max_dist / scale;
      distances_ = new double *[cell_radius_+2];
      for(int i=0; i<=cell_radius_+1; i++)
      {
	distances_[i] = new double[cell_radius_+2];
        for(int j=0; j<=cell_radius_+1; j++)
	{
	  distances_[i][j] = sqrt(i*i + j*j);
	}
      }
    }
    ~CachedDistanceMap()
    {
      if(distances_)
      {
	for(int i=0; i<=cell_radius_+1; i++)
	  delete[] distances_[i];
	delete[] distances_;
      }
    }
    double** distances_;
    double scale_;
    double max_dist_;
    int cell_radius_;
};


bool operator<(const CellData& a, const CellData& b)
{
  return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;
}

CachedDistanceMap*
get_distance_map(double scale, double max_dist)
{
  static CachedDistanceMap* cdm = NULL;

  if(!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist))
  {
    if(cdm)
      delete cdm;
    cdm = new CachedDistanceMap(scale, max_dist);
  }

  return cdm;
}

static unsigned int delta(const unsigned int x, const unsigned int y)
{
  if(x < y)
  {
	return y - x;
  }
  return x - y;
}

void enqueue(map_t* map, unsigned int i, unsigned int j, 
	     unsigned int src_i, unsigned int src_j,
	     std::priority_queue<CellData>& Q,
	     CachedDistanceMap* cdm,
	     unsigned char* marked)
{
  if(marked[MAP_INDEX(map, i, j)])
    return;

  unsigned int di = delta(i, src_i);
  unsigned int dj = delta(j, src_j);
  double distance = cdm->distances_[di][dj];

  if(distance > cdm->cell_radius_)
    return;

  map->cells[MAP_INDEX(map, i, j)].occ_dist = distance * map->scale;

  CellData cell;
  cell.map_ = map;
  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;

  Q.push(cell);

  marked[MAP_INDEX(map, i, j)] = 1;
}

// Update the cspace distance values
void map_update_cspace(map_t *map, double max_occ_dist)
{
  unsigned char* marked;
  std::priority_queue<CellData> Q;

  marked = new unsigned char[map->size_x*map->size_y];
  memset(marked, 0, sizeof(unsigned char) * map->size_x*map->size_y);

  map->max_occ_dist = max_occ_dist;

  CachedDistanceMap* cdm = get_distance_map(map->scale, map->max_occ_dist);

  // Enqueue all the obstacle cells
  CellData cell;
  cell.map_ = map;
  for(int i=0; i<map->size_x; i++)
  {
    cell.src_i_ = cell.i_ = i;
    for(int j=0; j<map->size_y; j++)
    {
      if(map->cells[MAP_INDEX(map, i, j)].occ_state == +1)
      {
	map->cells[MAP_INDEX(map, i, j)].occ_dist = 0.0;
	cell.src_j_ = cell.j_ = j;
	marked[MAP_INDEX(map, i, j)] = 1;
	Q.push(cell);
      }
      else
	map->cells[MAP_INDEX(map, i, j)].occ_dist = max_occ_dist;
    }
  }

  while(!Q.empty())
  {
    CellData current_cell = Q.top();
    if(current_cell.i_ > 0)
      enqueue(map, current_cell.i_-1, current_cell.j_, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);
    if(current_cell.j_ > 0)
      enqueue(map, current_cell.i_, current_cell.j_-1, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);
    if((int)current_cell.i_ < map->size_x - 1)
      enqueue(map, current_cell.i_+1, current_cell.j_, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);
    if((int)current_cell.j_ < map->size_y - 1)
      enqueue(map, current_cell.i_, current_cell.j_+1, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    Q.pop();
  }

  delete[] marked;
}

#if 0
// TODO: replace this with a more efficient implementation.  Not crucial,
// because we only do it once, at startup.
void map_update_cspace(map_t *map, double max_occ_dist)
{
  int i, j;
  int ni, nj;
  int s;
  double d;
  map_cell_t *cell, *ncell;

  map->max_occ_dist = max_occ_dist;
  s = (int) ceil(map->max_occ_dist / map->scale);

  // Reset the distance values
  for (j = 0; j < map->size_y; j++)
  {
    for (i = 0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      cell->occ_dist = map->max_occ_dist;
    }
  }

  // Find all the occupied cells and update their neighbours
  for (j = 0; j < map->size_y; j++)
  {
    for (i = 0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      if (cell->occ_state != +1)
        continue;
          
      cell->occ_dist = 0;

      // Update adjacent cells
      for (nj = -s; nj <= +s; nj++)
      {
        for (ni = -s; ni <= +s; ni++)
        {
          if (!MAP_VALID(map, i + ni, j + nj))
            continue;

          ncell = map->cells + MAP_INDEX(map, i + ni, j + nj);
          d = map->scale * sqrt(ni * ni + nj * nj);

          if (d < ncell->occ_dist)
            ncell->occ_dist = d;
        }
      }
    }
  }
  
  return;
}
#endif

/// @endcond
