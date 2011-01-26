
/***************************************************************************
 *  scaler.h - Scaler interface
 *
 *  Generated: Tue May 16 14:52:58 2006 (Automatica 2006)
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

#ifndef __FIREVISION_FVUTILS_SCALERS_SCALER_H_
#define __FIREVISION_FVUTILS_SCALERS_SCALER_H_

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Scaler {

 public:
  virtual ~Scaler();

  virtual void             set_scale_factor(float factor)                         = 0;
  virtual void             set_original_dimensions(unsigned int width,
						   unsigned int height)           = 0;
  virtual void             set_scaled_dimensions(unsigned int width,
						 unsigned int height)             = 0;
  virtual void             set_original_buffer(unsigned char *buffer)             = 0;
  virtual void             set_scaled_buffer(unsigned char *buffer)               = 0;
  virtual void             scale()                                                = 0;
  virtual unsigned int     needed_scaled_width()                                  = 0;
  virtual unsigned int     needed_scaled_height()                                 = 0;
  virtual float            get_scale_factor()                                     = 0;

};

} // end namespace firevision

#endif
