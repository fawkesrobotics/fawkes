
/***************************************************************************
 *  occupancygrid.cpp - An occupancy-grid
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  AllemaniACs
 *             2013  Bahram Maleki-Fard
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

#include "occupancygrid.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif
OccupancyGrid::OccupancyGrid(int width, int height,
                               int cell_width, int cell_height)
{
  m_Width = width;
  m_Height = height;
  m_CellWidth = cell_width;
  m_CellHeight = cell_height;

  initGrid();
}


OccupancyGrid::~OccupancyGrid()
{
  m_OccupancyProb.clear();
}

void
OccupancyGrid::setCellWidth(int width)
{
  m_CellWidth = width;
}

int
OccupancyGrid::getCellWidth()
{
  return m_CellWidth;
}

void
OccupancyGrid::setCellHeight(int height)
{
  m_CellHeight = height;
}

int
OccupancyGrid::getCellHeight()
{
  return m_CellHeight;
}

void
OccupancyGrid::setWidth(int width)
{
  m_Width = width;
  initGrid();
}

int
OccupancyGrid::getWidth()
{
  return m_Width;
}

void
OccupancyGrid::setHeight(int height)
{
  m_Height = height;
  initGrid();
}

int
OccupancyGrid::getHeight()
{
  return m_Height;
}


void
OccupancyGrid::setProb(int x, int y, Probability prob)
{
  if( (x < m_Width) && (y < m_Height) && ((isProb(prob)) || (prob == 2.0)) )
    m_OccupancyProb[x][y] = prob;
}

void
OccupancyGrid::fill(Probability prob)
{
  if((isProb(prob)) || (prob == -1)) {
    for(int x = 0; x < m_Width; x++) {
      for(int y = 0; y < m_Height; y++) {
        m_OccupancyProb[x][y] = prob;
      }
    }
  }
}

//  Probability OccupancyGrid::getProb(int x, int y)
//  {
//    return m_OccupancyProb[x][y];
//  }


void
OccupancyGrid::initGrid()
{
  m_OccupancyProb.clear();
  std::vector<Probability> row;
  row.resize(m_Height, 0.0);
  m_OccupancyProb.resize(m_Width, row);
  fill( 0.0 );
}

} // namespace fawkes
