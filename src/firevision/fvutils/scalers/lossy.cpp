
/***************************************************************************
 *  lossy.cpp - lossy scaler
 *
 *  Generated: Tue May 16 14:59:30 2006 (Automatica 2006)
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

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


#include <fvutils/scalers/lossy.h>
#include <fvutils/color/yuv.h>

#include <cmath>
#include <cstring>

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
  // to be dividable by 2, needed for YUV for simplicity
  unsigned int w = (unsigned int)ceil( orig_width * scale_factor );
  return ( w + (w % 2) );
}


unsigned int
LossyScaler::needed_scaled_height()
{
  // to be dividable by 2, needed for YUV for simplicity
  unsigned int h = (unsigned int)ceil( orig_height * scale_factor );
  return ( h + (h % 2) );
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

  const unsigned int skip = (unsigned int)ceil( 1 / scale_factor );
  unsigned char *oyp = orig_buffer;
  unsigned char *oup = YUV422_PLANAR_U_PLANE( orig_buffer, orig_width, orig_height );
  unsigned char *ovp = YUV422_PLANAR_V_PLANE( orig_buffer, orig_width, orig_height );

  unsigned char *syp = scal_buffer;
  unsigned char *sup = YUV422_PLANAR_U_PLANE( scal_buffer, scal_width, scal_height );
  unsigned char *svp = YUV422_PLANAR_V_PLANE( scal_buffer, scal_width, scal_height );

  memset( syp,   0, scal_width * scal_height );
  memset( sup, 128, scal_width * scal_height );

  for ( unsigned int h = 0; h < orig_height; h += skip ) {
    for ( unsigned int w = 0; w < orig_width; w += 2 * skip ) {
      syp[ (h / skip) * scal_width + (w / skip) ] = oyp[ h * orig_width + w ];
      syp[ (h / skip) * scal_width + (w / skip) + 1 ] = oyp[ h * orig_width + w + skip ];
      sup[ ((h / skip) * scal_width + (w / skip)) / 2 ] = (oup[ (h * orig_width + w) / 2 ] + oup[ (h * orig_width + w + skip) / 2 ]) / 2;
      svp[ ((h / skip) * scal_width + (w / skip)) / 2 ] = (ovp[ (h * orig_width + w) / 2 ] + ovp[ (h * orig_width + w + skip) / 2 ]) / 2;
    }
  }

}
