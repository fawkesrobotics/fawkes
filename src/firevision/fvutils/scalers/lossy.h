
/***************************************************************************
 *  lossy.h - lossy scaler, will just through away information, can only
 *            downscale images!
 *
 *  Generated: Tue May 16 14:57:16 2006 (Automatica 2006)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_UTILS_SCALERS_LOSSY_H_
#define __FIREVISION_UTILS_SCALERS_LOSSY_H_

#include <fvutils/scalers/scaler.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class LossyScaler : public Scaler {

 public:
  LossyScaler();
  virtual ~LossyScaler();

  virtual void             set_scale_factor(float factor);
  virtual void             set_original_dimensions(unsigned int width,
						   unsigned int height);
  virtual void             set_scaled_dimensions(unsigned int width,
						 unsigned int height);
  virtual void             set_original_buffer(unsigned char *buffer);
  virtual void             set_scaled_buffer(unsigned char *buffer);
  virtual void             scale();
  virtual unsigned int     needed_scaled_width();
  virtual unsigned int     needed_scaled_height();
  virtual float            get_scale_factor();

 private:
  unsigned int    orig_width;
  unsigned int    orig_height;
  unsigned char  *orig_buffer;

  unsigned int    scal_width;
  unsigned int    scal_height;
  unsigned char  *scal_buffer;

  float           scale_factor;

};

} // end namespace firevision

#endif
