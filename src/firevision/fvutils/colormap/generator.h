
/**************************************************************************
 *  generator.h - interface for generating arbitrary colormaps
 *
 *  Created: Wed Mar 01 13:51:39 2006
 *  Copyright  2005-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ***************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_FVUTILS_COLORMAP_GENERATOR_H__
#define __FIREVISION_FVUTILS_COLORMAP_GENERATOR_H__

#include <fvutils/base/roi.h>
#include <string>
#include <map>

class YuvColormap;
class Histogram;

class ColormapGenerator
{

 public:

  virtual ~ColormapGenerator();

  virtual void           set_buffer(unsigned char *buffer,
				    unsigned int width, unsigned int height) = 0;
  virtual YuvColormap *  get_current()                                       = 0;

  virtual void           consider()                                          = 0;
  virtual void           calc()                                              = 0;
  virtual void           undo()                                              = 0;
  virtual void           reset()                                             = 0;
  virtual void           reset_undo()                                        = 0;

  virtual bool           has_histograms()                                    = 0;
  virtual std::map< hint_t, Histogram *> *  get_histograms()                 = 0;
};


#endif
