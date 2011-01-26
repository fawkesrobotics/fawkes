
/***************************************************************************
 *  globalpositionmodel.cpp - Abstract class defining a position model for
 *                            calculation of global position
 *
 *  Created: Wed Mar 21 15:44:10 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <fvmodels/global_position/globalpositionmodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class GlobalPositionModel <fvmodels/global_position/globalpositionmodel.h>
 * Global Position Model Interface.
 * This interface defines the API for global position models.
 *
 * @fn void GlobalPositionModel::set_robot_position(float x, float y, float ori)
 * Set the global position of the object.
 * @param x x coordinate of position
 * @param y y coordinate of position
 * @param ori orientation of robot
 *
 * @fn void GlobalPositionModel::set_position_in_image(unsigned int x, unsigned int y)
 * Set the position of the object as recognized in the image.
 * @param x x coordinate in pixels
 * @param y y coordinate in pixels
 *
 * @fn float GlobalPositionModel::get_x() const
 * Get global x coordinate of object.
 * @return x coordinate of object
 *
 * @fn float GlobalPositionModel::get_y() const
 * Get global y coordinate of object.
 * @return y coordinate of object
 *
 * @fn void GlobalPositionModel::calc()
 * Calculate position.
 * From the data set via setRobotPosition() or setPositionInImage() calculate the
 * objects global position.
 *
 * @fn bool GlobalPositionModel::is_pos_valid() const
 * Check if the position is valid.
 * @return true, if the calculated position is valid, false otherwise
 *
 * @author Tim Niemueller
 */

/** Empty virtual destructor. */
GlobalPositionModel::~GlobalPositionModel()
{
}

} // end namespace firevision
