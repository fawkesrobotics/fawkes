
/***************************************************************************
 *  occupancygrid.h - An occupancy-grid
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
const float OCCUPANCY_THRESHOLD = 0.45f;

class OccupancyGrid
{
 public:
  OccupancyGrid(int width, int height, int cell_width=5, int cell_height=5);
  virtual ~OccupancyGrid();

  ///\brief Get the cell width (in cm)
  int get_cell_width();

   ///\brief Get the cell height (in cm)
  int get_cell_height();

  ///\brief Get the width of the grid
  int get_width();

  ///\brief Get the height of the grid
  int get_height();

  ///\brief Resets the cell width (in cm)
  void set_cell_width(int cell_width);

  ///\brief Resets the cell height (in cm)
  void set_cell_height(int cell_height);

  ///\brief Resets the width of the grid and constructs a new empty grid
  void set_width(int width);

  ///\brief Resets the height of the grid and constructs a new empty grid
  void set_height(int height);

  ///\brief Reset the occupancy probability of a cell
  virtual void set_prob(int x, int y, Probability prob);

  ///\brief Resets all occupancy probabilities
  void fill(Probability prob);

  ///\brief Get the occupancy probability of a cell
  Probability get_prob(int x, int y);

  ///\brief Get the occupancy probability of a cell
  Probability& operator () (const int x, const int y);

  ///\brief Init a new empty grid with the predefined parameters */
  void init_grid();

  /// The occupancy probability of the cells in a 2D array
  std::vector<std::vector<Probability> > occupancy_probs_;

 protected:
  int cell_width_;   /**< Cell width in cm */
  int cell_height_;  /**< Cell height in cm */
  int width_;       /**< Width of the grid in # cells */
  int height_;      /**< Height of the grid in # cells */

};

} // namespace fawkes

#endif
