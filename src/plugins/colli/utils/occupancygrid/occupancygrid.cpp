
/***************************************************************************
 *  occupancygrid.cpp - An occupancy-grid
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  AllemaniACs
 *             2013-2014  Bahram Maleki-Fard
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
  width_ = width;
  height_ = height;
  cell_width_ = cell_width;
  cell_height_ = cell_height;

  init_grid();
}

/** Destructor */
OccupancyGrid::~OccupancyGrid()
{
  occupancy_probs_.clear();
}

/** Get the cell width
 * @return the cell width in cm
 */
int
OccupancyGrid::get_cell_width()
{
  return cell_width_;
}

/** Get the cell height
 * @return the height of the cells in cm
 */
int
OccupancyGrid::get_cell_height()
{
  return cell_height_;
}

/** Get the width of the grid
 * @return the width of the grid in # of cells
 */
int
OccupancyGrid::get_width()
{
  return width_;
}

/** Get the height of the grid
 * @return the height of the grid in # cells
 */
int
OccupancyGrid::get_height()
{
  return height_;
}

/** Resets the cell width
 * @param width the width of the cells in cm
 */
void
OccupancyGrid::set_cell_width(int width)
{
  cell_width_ = width;
}

/** Resets the cell height
 * @param height the height of the cells in cm
 */
void
OccupancyGrid::set_cell_height(int height)
{
  cell_height_ = height;
}

/** Resets the width of the grid and constructs a new empty grid
 * @param width the cell width in cm
 */
void
OccupancyGrid::set_width(int width)
{
  width_ = width;
  init_grid();
}

/** Resets the height of the grid and constructs a new empty grid
 * @param height the height of the grid in # of cells
 */
void
OccupancyGrid::set_height(int height)
{
  height_ = height;
  init_grid();
}


/** Reset the occupancy probability of a cell
 * @param x the x-position of the cell
 * @param y the y-position of the cell
 * @param prob the occupancy probability of cell (x,y)
 */
void
OccupancyGrid::set_prob(int x, int y, Probability prob)
{
  if( (x < width_) && (y < height_) && ((isProb(prob)) || (prob == 2.f)) )
    occupancy_probs_[x][y] = prob;
}

/** Resets all occupancy probabilities
 * @param prob the occupancy probability the grid will become filled with
 */
void
OccupancyGrid::fill(Probability prob)
{
  if((isProb(prob)) || (prob == -1.f)) {
    for(int x = 0; x < width_; x++) {
      for(int y = 0; y < height_; y++) {
        occupancy_probs_[x][y] = prob;
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
OccupancyGrid::get_prob(int x, int y)
{
  if( (x >= 0) && (x < width_) && (y >= 0) && (y < height_) ) {
    return occupancy_probs_[x][y];
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
  return occupancy_probs_[x][y];
}

/** Init a new empty grid with the predefined parameters */
void
OccupancyGrid::init_grid()
{
  occupancy_probs_.clear();
  std::vector<Probability> column;
  column.resize(height_, 0.f);
  occupancy_probs_.resize(width_, column);
  fill( 0.f );
}

} // namespace fawkes
