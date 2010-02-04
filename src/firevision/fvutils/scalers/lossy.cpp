
/***************************************************************************
 *  lossy.cpp - lossy scaler
 *
 *  Generated: Tue May 16 14:59:30 2006 (Automatica 2006)
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


#include <fvutils/scalers/lossy.h>
#include <fvutils/color/yuv.h>

#include <cmath>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LossyScaler <fvutils/scalers/lossy.h>
 * Lossy image scaler.
 * This scaler just takes the required pixels from the image and throws away
 * the rest. No enhancement of the image is done.
 * This is only suitable for downscaling. The scale factor must be between
 * 0 and 1.
 */

/** Constructor. */
LossyScaler::LossyScaler()
{
  orig_width = orig_height = 0;
  scal_width = scal_height = 0;
  orig_buffer = NULL;
  scal_buffer = NULL;

  scale_factor = 1.f;
}


/** Destructor. */
LossyScaler::~LossyScaler()
{
}


void
LossyScaler::set_scale_factor(float factor)
{
  if ( (factor <= 0) || (factor > 1) ) {
    scale_factor = 1.f;
  } else {
    scale_factor = factor;
  }

  if (orig_width != 0) {
    scal_width = (unsigned int) ceil(orig_width * scale_factor);
    scal_width += (scal_width % 2);
  }
  if (orig_height != 0) {
    scal_height = (unsigned int) ceil(orig_height * scale_factor);
    scal_height += (scal_width % 2);
  }
}


void
LossyScaler::set_original_dimensions(unsigned int width,
				     unsigned int height)
{
  orig_width  = width;
  orig_height = height;
}


void
LossyScaler::set_scaled_dimensions(unsigned int width,
				   unsigned int height)
{
  scal_width  = width;
  scal_height = height;
  
  float scale_factor_width  = 1.0;
  float scale_factor_height = 1.0;

  if (orig_width != 0) {
    scale_factor_width = scal_width / float(orig_width);
  }
  if (orig_height != 0) {
    scale_factor_height = scal_height / float(orig_height);
  }

  scale_factor = (scale_factor_width < scale_factor_height) ? scale_factor_width : scale_factor_height;

  scal_width  = (unsigned int) floor(orig_width * scale_factor);
  scal_height = (unsigned int) floor(orig_height * scale_factor);

  scal_width  += (scal_width % 2);
  scal_height += (scal_height % 2);
}


void
LossyScaler::set_original_buffer(unsigned char *buffer)
{
  orig_buffer = buffer;
}


void
LossyScaler::set_scaled_buffer(unsigned char *buffer)
{
  scal_buffer = buffer;
}


unsigned int
LossyScaler::needed_scaled_width()
{
  return scal_width;
}


unsigned int
LossyScaler::needed_scaled_height()
{
  return scal_height;
}


float
LossyScaler::get_scale_factor()
{
  return scale_factor;
}

void
LossyScaler::scale()
{
  if ( orig_width  == 0 ) return;
  if ( orig_height == 0 ) return;
  if ( scal_width  == 0 ) return;
  if ( scal_height == 0 ) return;
  if ( orig_buffer == NULL ) return;
  if ( scal_buffer == NULL ) return;
  if ( scal_width < needed_scaled_width() ) return;
  if ( scal_height < needed_scaled_height() ) return;

  float skip = 1 / scale_factor;
  unsigned char *oyp = orig_buffer;
  unsigned char *oup = YUV422_PLANAR_U_PLANE( orig_buffer, orig_width, orig_height );
  unsigned char *ovp = YUV422_PLANAR_V_PLANE( orig_buffer, orig_width, orig_height );

  unsigned char *syp = scal_buffer;
  unsigned char *sup = YUV422_PLANAR_U_PLANE( scal_buffer, scal_width, scal_height );
  unsigned char *svp = YUV422_PLANAR_V_PLANE( scal_buffer, scal_width, scal_height );

  memset( syp,   0, scal_width * scal_height );
  memset( sup, 128, scal_width * scal_height );

  float oh_float = 0.0;
  float ow_float = 0.0;

  unsigned int oh_pixel;
  unsigned int ow_pixel;
  unsigned int ow_pixel_next;

  for (unsigned int h = 0; h < scal_height; ++h) {
    oh_pixel = (unsigned int) rint(oh_float);
    ow_float = 0.0;

    if (oh_pixel >= orig_height) {
      oh_pixel = orig_height - 1;
    }
    for (unsigned int w = 0; w < scal_width; w += 2) {
      ow_pixel = (unsigned int) rint(ow_float);
      ow_pixel_next = (unsigned int) rint( ow_float + skip);
      
      if (ow_pixel >= orig_width) {
	ow_pixel = orig_width - 1;
      }

      if (ow_pixel_next >= orig_width) {
	ow_pixel_next = orig_width - 1;
      }

      syp[ h * scal_width + w ] = oyp[ oh_pixel * orig_width + ow_pixel ];
      syp[ h * scal_width + w + 1 ] = oyp[ oh_pixel * orig_width + ow_pixel_next ];
      sup[ (h * scal_width + w) / 2 ] = (oup[ (oh_pixel * orig_width + ow_pixel) / 2 ] + oup[ (oh_pixel * orig_width + ow_pixel_next) / 2 ]) / 2;
      svp[ (h * scal_width + w) / 2 ] = (ovp[ (oh_pixel * orig_width + ow_pixel) / 2 ] + ovp[ (oh_pixel * orig_width + ow_pixel_next) / 2 ]) / 2;

      ow_float += 2 * skip;
    }
    oh_float += skip;
  }
}

} // end namespace firevision
