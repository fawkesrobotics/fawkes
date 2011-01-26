
/***************************************************************************
 *  colormodel.cpp - Abstract class defining a color model
 *
 *  Created: Wed Mar 21 18:38:17 2007
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

#include <fvmodels/color/colormodel.h>
#include <fvutils/color/color_object_map.h>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColorModel <fvmodels/color/colormodel.h>
 * Color model interface.
 * This interface defines the API for color models.
 *
 * @fn color_t ColorModel::determine(unsigned int y, unsigned int u, unsigned int v) const
 * Determine classification of YUV pixel.
 * Given a pixel in the YUV colorspace the colormodel determines the color
 * classification based on some a-priori knowledge.
 * @param y Y value
 * @param u U value
 * @param v V value
 * @return color classification
 *
 * @fn const char * ColorModel::get_name()
 * Get name of color model.
 * @return name of color model.
 *
 * @author Tim Niemueller
 */


/** Virtual empty destructor. */
ColorModel::~ColorModel()
{
}


/** Create image from color model.
 * Create image from color model, useful for debugging and analysing.
 * This method produces a representation of the color model for the full U/V plane
 * at the given value of Y for visual inspection of the colormap.
 * The dimensions of the resulting image are 512x512 pixels. It uses
 * strong colors as defined by ColorObjectMap.
 * Color models may override this method, but they must produce a 512x512
 * YUV422_PLANAR image.
 * @param yuv422_planar_buffer contains the image upon return, must be initialized
 * with the appropriate memory size before calling, dimensions are 512x512 pixels.
 * @param y Brightness value of the color
 */
void
ColorModel::uv_to_image(unsigned char *yuv422_planar_buffer, unsigned int y)
{
  unsigned char *yp = yuv422_planar_buffer;
  unsigned char *up = YUV422_PLANAR_U_PLANE(yuv422_planar_buffer, 512, 512);
  unsigned char *vp = YUV422_PLANAR_V_PLANE(yuv422_planar_buffer, 512, 512);

  YUV_t c;
  for (unsigned int v = 256; v > 0 ; --v) {
    for (unsigned int u = 0; u < 256; ++u) {
      c = ColorObjectMap::get_color(determine(y, u, v));

      *yp++ = c.Y;
      *yp++ = c.Y;
      *up++ = c.U;
      *vp++ = c.V;
    }
    // Double line
    memcpy(yp, (yp - 512), 512);
    yp += 512;
    memcpy(up, (up - 256), 256);
    memcpy(vp, (vp - 256), 256);
    up += 256;
    vp += 256;
  }
}

} // end namespace firevision
