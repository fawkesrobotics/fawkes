
/***************************************************************************
 *  colormodel.h - Abstract class defining a color model
 *
 *  Created: Wed May 18 13:51:06 2005
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

#ifndef __FIREVISION_MODELS_COLOR_COLORMODEL_H_
#define __FIREVISION_MODELS_COLOR_COLORMODEL_H_

#include <fvutils/base/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ColorModel
{
 public:
  virtual ~ColorModel();

  virtual color_t       determine(unsigned int y,
				  unsigned int u,
				  unsigned int v) const  = 0;

  virtual const char *  get_name()                        = 0;

  virtual void uv_to_image(unsigned char *yuv422_planar_buffer, unsigned int y);

};

} // end namespace firevision

#endif
