#ifndef _COLLI_UTILS_OCCUPANCYGRID_OCCUPANCYGRID_H_
#define _COLLI_UTILS_OCCUPANCYGRID_OCCUPANCYGRID_H_

#include "probability.h"

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

const float OCCUPANCY_THRESHOLD = 0.45;

/** Occupancy Grid class for general use. Many derivated classes
exist, which are usually used instead of this general class.
Note: the coord system is assumed to map x onto width an y onto
height, with x being the first coordinate !
*/
class OccupancyGrid
{
 public:

  /**
   * Constructs an empty occupancy grid
   *
   * @param width the width of the grid in # of cells
   * @param height the height of the cells in # of cells
   * @param cell_width the cell width in cm
   * @param cell_height the cell height in cm
   * @param drawing_resolution the number of pixels / cm
   */
  OccupancyGrid(int width, int height,
                int cell_width=5, int cell_height=5);

  /**
   * Destructor
   *
   */
  virtual ~OccupancyGrid();

  /**
   * Resets the cell width
   *
   * @param width the width of the cells in cm
   */
  void setCellWidth(int cell_width);

  /**
   * Get the cell width
   *
   * @return the cell width in cm
   */
  int getCellWidth();

  /**
   * Resets the cell height
   *
   * @param width the height of the cells in cm
   */
  void setCellHeight(int cell_height);

  /**
   * Get the cell height
   *
   *
   * @return the height of the cells in cm
   */
  int getCellHeight();

  /**
   * Resets the width of the grid and constructs
   * a new empty grid
   *
   * @param width the cell width in cm
   */
  void setWidth(int width);

  /**
   * Get the width of the grid
   *
   *
   * @return the width of the grid in # of cells
   */
  int getWidth();

  /**
   * Resets the height of the grid and constructs
   * a new empty grid
   *
   * @param height the height of the grid in # of cells
   */
  void setHeight(int height);

  /**
   * Get the height of the grid
   *
   *
   * @return the height of the grid in # cells
   */
  int getHeight();

  /**
   * Reset the occupancy probability of a cell
   *
   * @param x the x-position of the cell
   * @param y the y-position of the cell
   * @param prob the occupancy probability of cell (x,y)
   */
  virtual void setProb(int x, int y, Probability prob);

  /**
   * Resets all occupancy probabilities
   *
   * @param prob the occupancy probability the
   * grid will become filled with
   */
  void fill(Probability prob);

  /**
   * Get the occupancy probability of a cell
   *
   * @param x the x-position of the cell
   * @param y the y-position of the cell
   *
   * @return the occupancy probability of cell (x,y)
   */
  Probability getProb(int x, int y) {
    if( (x >= 0) && (x < m_Width) && (y >= 0) && (y < m_Height) ) {
      return m_OccupancyProb[x][y];
    } else {
      return 1;
    }
  }

  Probability& operator () (const int x, const int y) {
    return m_OccupancyProb[x][y];
  };


  /**
   * Init a new empty grid with the predefined
   * parameters
   *
   */
  void initGrid();


  /// The occupancy probability of the cells in a 2D array
  std::vector<std::vector<Probability> > m_OccupancyProb;


 protected:
  /// Cell width in cm
  int m_CellWidth;
  /// Cell height in cm
  int m_CellHeight;
  /// Width of the grid in # cells
  int m_Width;
  /// Height of the grid in # cells
  int m_Height;


};

} // namespace fawkes

#endif
