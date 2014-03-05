
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

/** @class OccupancyGrid <plugins/colli/utils/occupancygrid/occupancygrid.h>
 * Occupancy Grid class for general use. Many derivated classes
 * exist, which are usually used instead of this general class.
 * Note: the coord system is assumed to map x onto width an y onto
 * height, with x being the first coordinate !
 */

/** Constructs an empty occupancy grid
 *
 * @param width the width of the grid in # of cells
 * @param height the height of the cells in # of cells
 * @param cell_width the cell width in cm
 * @param cell_height the cell height in cm
 */
OccupancyGrid::OccupancyGrid(int width, int height, int cell_width, int cell_height)
{
  m_Width = width;
  m_Height = height;
  m_CellWidth = cell_width;
  m_CellHeight = cell_height;

  initGrid();
}

/** Destructor */
OccupancyGrid::~OccupancyGrid()
{
  m_OccupancyProb.clear();
}

/** Get the cell width
 * @return the cell width in cm
 */
int
OccupancyGrid::getCellWidth()
{
  return m_CellWidth;
}

/** Get the cell height
 * @return the height of the cells in cm
 */
int
OccupancyGrid::getCellHeight()
{
  return m_CellHeight;
}

/** Get the width of the grid
 * @return the width of the grid in # of cells
 */
int
OccupancyGrid::getWidth()
{
  return m_Width;
}

/** Get the height of the grid
 * @return the height of the grid in # cells
 */
int
OccupancyGrid::getHeight()
{
  return m_Height;
}

/** Resets the cell width
 * @param width the width of the cells in cm
 */
void
OccupancyGrid::setCellWidth(int width)
{
  m_CellWidth = width;
}

/** Resets the cell height
 * @param height the height of the cells in cm
 */
void
OccupancyGrid::setCellHeight(int height)
{
  m_CellHeight = height;
}

/** Resets the width of the grid and constructs a new empty grid
 * @param width the cell width in cm
 */
void
OccupancyGrid::setWidth(int width)
{
  m_Width = width;
  initGrid();
}

/** Resets the height of the grid and constructs a new empty grid
 * @param height the height of the grid in # of cells
 */
void
OccupancyGrid::setHeight(int height)
{
  m_Height = height;
  initGrid();
}


/** Reset the occupancy probability of a cell
 * @param x the x-position of the cell
 * @param y the y-position of the cell
 * @param prob the occupancy probability of cell (x,y)
 */
void
OccupancyGrid::setProb(int x, int y, Probability prob)
{
  if( (x < m_Width) && (y < m_Height) && ((isProb(prob)) || (prob == 2.0)) )
    m_OccupancyProb[x][y] = prob;
}

/** Resets all occupancy probabilities
 * @param prob the occupancy probability the grid will become filled with
 */
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

/** Get the occupancy probability of a cell
 * @param x the x-position of the cell
 * @param y the y-position of the cell
 * @return the occupancy probability of cell (x,y)
 */
Probability
OccupancyGrid::getProb(int x, int y)
{
  if( (x >= 0) && (x < m_Width) && (y >= 0) && (y < m_Height) ) {
    return m_OccupancyProb[x][y];
  } else {
    return 1;
  }
}

/** Operator (), get occupancy probability of a cell
 * @param x the x-position of the cell
 * @param y the y-position of the cell
 * @return the occupancy probability of cell (x,y)
 */
Probability&
OccupancyGrid::operator () (const int x, const int y)
{
  return m_OccupancyProb[x][y];
}

/** Init a new empty grid with the predefined parameters */
void
OccupancyGrid::initGrid()
{
  m_OccupancyProb.clear();
  std::vector<Probability> column;
  column.resize(m_Height, 0.0);
  m_OccupancyProb.resize(m_Width, column);
  fill( 0.0 );
}

} // namespace fawkes
