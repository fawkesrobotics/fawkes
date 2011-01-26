
/***************************************************************************
 *  globalpositionmodel.h - Abstract class defining a position model for
 *                          calculation of global position
 *
 *  Created: Tue May 31 13:43:22 2005
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

#ifndef __FIREVISION_GLOBALPOSITIONMODEL_H_
#define __FIREVISION_GLOBALPOSITIONMODEL_H_

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class GlobalPositionModel
{

 public:
  virtual ~GlobalPositionModel();

  virtual void		set_robot_position(float x, float y, float ori)	      = 0;
  virtual void          set_position_in_image(unsigned int x, unsigned int y) = 0;

  virtual float		get_x() const					      = 0;
  virtual float		get_y() const					      = 0;

  virtual void          calc()                                                = 0;

  virtual bool          is_pos_valid() const                                  = 0;

};

} // end namespace firevision

#endif

