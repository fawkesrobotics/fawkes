
/***************************************************************************
 *  occupancygrid.h - An occupancy-grid
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

#ifndef __PLUGINS_COLLI_UTILS_OCCUPANCYGRID_OCCUPANCYGRID_H_
#define __PLUGINS_COLLI_UTILS_OCCUPANCYGRID_OCCUPANCYGRID_H_

#include "probability.h"

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Occupancy threshold. */
const float OCCUPANCY_THRESHOLD = 0.45;

class OccupancyGrid
{
 public:

  OccupancyGrid(int width, int height, int cell_width=5, int cell_height=5);
  virtual ~OccupancyGrid();

  ///\brief Get the cell width (in cm)
  int getCellWidth();

   ///\brief Get the cell height (in cm)
  int getCellHeight();

  ///\brief Get the width of the grid
  int getWidth();

  ///\brief Get the height of the grid
  int getHeight();

  ///\brief Resets the cell width (in cm)
  void setCellWidth(int cell_width);

  ///\brief Resets the cell height (in cm)
  void setCellHeight(int cell_height);

  ///\brief Resets the width of the grid and constructs a new empty grid
  void setWidth(int width);

  ///\brief Resets the height of the grid and constructs a new empty grid
  void setHeight(int height);

  ///\brief Reset the occupancy probability of a cell
  virtual void setProb(int x, int y, Probability prob);

  ///\brief Resets all occupancy probabilities
  void fill(Probability prob);

  ///\brief Get the occupancy probability of a cell
  Probability getProb(int x, int y);

  ///\brief Get the occupancy probability of a cell
  Probability& operator () (const int x, const int y);

  ///\brief Init a new empty grid with the predefined parameters */
  void initGrid();

  /// The occupancy probability of the cells in a 2D array
  std::vector<std::vector<Probability> > m_OccupancyProb;


 protected:
  int m_CellWidth;   /**< Cell width in cm */
  int m_CellHeight;  /**< Cell height in cm */
  int m_Width;       /**< Width of the grid in # cells */
  int m_Height;      /**< Height of the grid in # cells */

};

} // namespace fawkes

#endif
