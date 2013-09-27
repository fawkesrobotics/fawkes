
/***************************************************************************
 *  qualifiers.cpp - Pixel qualifier
 *
 *  Created: Mon Jun 09 22:54:00 2008
 *  Copyright  2008  Christof Rath <c.rath@student.tugraz.at>
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <fvclassifiers/qualifiers.h>
#include <core/exceptions/software.h>
#include <fvutils/color/yuv.h>

#include <cstdlib>

using fawkes::upoint_t;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Qualifier qualifiers.h <apps/nao_loc/qualifiers.h>
 * Abstract Qualifier for a single pixel
 *
 * @author Christof Rath
 */

/** Default constructor
 */
Qualifier::Qualifier()
{
  buffer_     = 0;
  width_      = 0;
  height_     = 0;
  size_       = 0;
  colorspace_ = CS_UNKNOWN;
}

/** Constructor.
 * @param buffer containing the image
 * @param width of the image
 * @param height of the image
 * @param colorspace the colorspace in action
 */
Qualifier::Qualifier(unsigned char* buffer, unsigned int width,
                     unsigned int height, colorspace_t colorspace)
{
  if (!buffer)
    throw fawkes::NullPointerException("Qualifier: the buffer may not be null!");
  if (!width || !height)
    throw fawkes::IllegalArgumentException("Qualifier: width and height may not be 0!");

  set_buffer(buffer, width, height);
  colorspace_ = colorspace;
}


/** Destructor.
 */
Qualifier::~Qualifier()
{
}

/** Get buffer.
 * @return pointer to buffer
 */
unsigned char*
Qualifier::get_buffer()
{
  return buffer_;
}

/** buffer setter
 * @param buffer containing the image
 * @param width of the image (if 0 the param will be ignored)
 * @param height of the image (if 0 the param will be ignored)
 */
void
Qualifier::set_buffer(unsigned char* buffer, unsigned int width,
                      unsigned int height)
{
  buffer_ = buffer;

  if (width)
    width_  = width;

  if (height)
    height_ = height;

  if (width || height)
    size_ = width_ * height_;
}




/** Get colorspace.
 * @return colorspace
 */
colorspace_t
Qualifier::get_colorspace()
{
  return colorspace_;
}


/** colorspace setter
 * @param colorspace the colorspace in action
 */
void
Qualifier::set_colorspace(colorspace_t colorspace)
{
  colorspace_ = colorspace;
}






/** @class LumaQualifier qualifiers.h <apps/nao_loc/qualifiers.h>
 * LumaQualifier for a single pixel.
 * Uses the value of the Y-channel
 *
 * @author Christof Rath
 */


/** Constructor.
 * @param buffer containing the image
 * @param width of the image
 * @param height of the image
 * @param colorspace the colorspace in action
 */
LumaQualifier::LumaQualifier(unsigned char* buffer, unsigned int width,
                             unsigned int height, colorspace_t colorspace)
 :Qualifier(buffer, width, height, colorspace)
{
}


/** Getter.
 * @param pixel the pixel of interest
 * @return a corresponding int value
 */
int
LumaQualifier::get(upoint_t pixel)
{
  if (pixel.x >= width_)
    throw fawkes::OutOfBoundsException("LumaQualifier: requested Pixel is out of bounds!", pixel.x, 0, width_);
  if (pixel.y >= height_)
    throw fawkes::OutOfBoundsException("LumaQualifier: requested Pixel is out of bounds!", pixel.y, 0, height_);

  return buffer_[pixel.y * width_ + pixel.x];
}






/** @class SkyblueQualifier qualifiers.h <apps/nao_loc/qualifiers.h>
 * SkyblueQualifier for a single pixel.
 * Uses the value of the U/V-channels
 *
 * @author Christof Rath
 */

/** Constructor.
 * @param buffer containing the image
 * @param width of the image
 * @param height of the image
 * @param colorspace the colorspace in action
 */
SkyblueQualifier::SkyblueQualifier(unsigned char* buffer, unsigned int width,
           unsigned int height, colorspace_t colorspace)
 :Qualifier(buffer, width, height, colorspace)
{
}


/** Getter.
 * @param pixel the pixel of interest
 * @return a corresponding int value
 */
int
SkyblueQualifier::get(upoint_t pixel)
{
  if (pixel.x >= width_)
    throw fawkes::OutOfBoundsException("SkyblueQualifier: requested Pixel is out of bounds!", pixel.x, 0, width_);
  if (pixel.y >= height_)
    throw fawkes::OutOfBoundsException("SkyblueQualifier: requested Pixel is out of bounds!", pixel.y, 0, height_);

  unsigned int u_addr = size_ + (pixel.y * width_ + pixel.x) / 2;
  unsigned char u = buffer_[u_addr];
  unsigned char v = 255 - buffer_[u_addr + size_ / 2];

  if ((u < threshold_) || (v < threshold_))
    return 0;

  return u + v;
}






/** @class YellowQualifier qualifiers.h <apps/nao_loc/qualifiers.h>
 * YellowQualifier for a single pixel.
 * Uses the value of the U/V-channels
 *
 * @author Christof Rath
 */

/** Constructor.
 * @param buffer containing the image
 * @param width of the image
 * @param height of the image
 * @param colorspace the colorspace in action
 */
YellowQualifier::YellowQualifier(unsigned char* buffer, unsigned int width,
         unsigned int height, colorspace_t colorspace)
 :Qualifier(buffer, width, height, colorspace)
{
}


/** Getter.
 * @param pixel the pixel of interest
 * @return a corresponding int value
 */
int
YellowQualifier::get(upoint_t pixel)
{
  if (pixel.x >= width_)
    throw fawkes::OutOfBoundsException("YellowQualifier: requested Pixel is out of bounds!", pixel.x, 0, width_);
  if (pixel.y >= height_)
    throw fawkes::OutOfBoundsException("YellowQualifier: requested Pixel is out of bounds!", pixel.y, 0, height_);

  unsigned int y_addr = (pixel.y * width_ + pixel.x);
  unsigned int u_addr = size_ + y_addr / 2;
  unsigned char y = buffer_[y_addr];
  unsigned int u = (255 - buffer_[u_addr]) * y;
  unsigned int v = (255 - abs(127 - buffer_[u_addr + size_ / 2]) * 2) * y;

  if ((u <= threshold_) || (v <= threshold_))
    return 0;

  return (u + v);
}

} // end namespace firevision
