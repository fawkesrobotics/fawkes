
/**************************************************************************
 *  colormap.h - colormap interface
 *
 *  Created: Sat Mar 29 12:45:29 2008
 *  Copyright  2005-2008  Tim Niemueller  [www.niemueller.de]
 *
 ***************************************************************************/

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

#ifndef __FIREVISION_FVUTILS_COLORMAP_COLORMAP_H_
#define __FIREVISION_FVUTILS_COLORMAP_COLORMAP_H_

#include <fvutils/base/types.h>
#include <sys/types.h>
#include <list>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ColormapFileBlock;

class Colormap
{
 public:
  virtual ~Colormap();

  virtual color_t         determine(unsigned int y, unsigned int u, unsigned int v) const = 0;
  virtual void            set(unsigned int y, unsigned int u, unsigned int v, color_t c)  = 0;
  
  virtual void            reset()                                                         = 0;
  virtual void            set(unsigned char *buffer)                                      = 0;

  virtual size_t          size()                                                          = 0;
  virtual void            to_image(unsigned char *yuv422_planar_buffer,
				   unsigned int level = 0);
  virtual unsigned int    image_height() const;
  virtual unsigned int    image_width() const;

  virtual unsigned char * get_buffer() const                                              = 0;

  virtual Colormap &      operator+=(const Colormap & cmlt)                               = 0;
  virtual Colormap &      operator+=(const char *filename)                                = 0;

  virtual unsigned int    width() const                                                   = 0;
  virtual unsigned int    height() const                                                  = 0;
  virtual unsigned int    depth() const                                                   = 0;
  virtual unsigned int    deepness() const                                                = 0;

  virtual std::list<ColormapFileBlock *>  get_blocks()                                    = 0;
};

} // end namespace firevision

#endif
