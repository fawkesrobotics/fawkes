
/***************************************************************************
 *  scaler.cpp - Scaler interface
 *
 *  Generated: Thu Mar 29 11:04:03 2007
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

#include <fvutils/scalers/scaler.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Scaler <fvutils/scalers/scaler.h>
 * Image scaler interface.
 * Image scalers allow for scaling images by a given factor.
 * @author Tim Niemueller
 *
 * @fn void Scaler::set_scale_factor(float factor)
 * Set scale factor.
 * @param factor scale factor
 *
 * @fn void Scaler::set_original_dimensions(unsigned int width, unsigned int height)
 * Set original image dimensions.
 * @param width image width
 * @param height height
 *
 * @fn void Scaler::set_scaled_dimensions(unsigned int width, unsigned int height)
 * Set dimenins of scaled image buffer.
 * @param width image width
 * @param height height
 * 
 * @fn void Scaler::set_original_buffer(unsigned char *buffer)
 * Set original image buffer.
 * @param buffer YUV 422 planar buffer
 *
 * @fn void Scaler::set_scaled_buffer(unsigned char *buffer)
 * Set scaled image buffer.
 * @param buffer YUV 422 planar buffer
 *
 * @fn void Scaler::scale()
 * Scale image.
 *
 * @fn unsigned int Scaler::needed_scaled_width()
 * Minimum needed width of scaled image depending on factor and original image width.
 * @return minimum needed width
 *
 * @fn unsigned int Scaler::needed_scaled_height()
 * Minimum needed height of scaled image depending on factor and original image height.
 * @return minimum needed height
 *
 * @fn float Scaler::get_scale_factor()
 * Returns the scale factor.
 * @return the scale factor
 */

/** Virtual empty destructor. */
Scaler::~Scaler()
{
}

} // end namespace firevision
