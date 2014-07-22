
/***************************************************************************
 *  roi.cpp - Implementation for Region Of Interest (ROI) representation
 *
 *  Generated: Thu Jul 14 12:01:58 2005
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

#include <fvutils/base/roi.h>

#include <cstdlib>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ROI <fvutils/base/roi.h>
 * Region of interest.
 * The ROI class is central to FireVision. All image processing is concentrated
 * on the pre-classified interesting parts of the image, denoted by the
 * regions of interest (ROIs).
 *
 * A ROI is a rectangular area of the image. Its start is denoted by the upper
 * left corner of this rectangle and the size is given by its width and height.
 *
 * @author Tim Niemueller
 */

ROI* ROI::roi_full_image = NULL;


/** Constructor. */
ROI::ROI()
{
  num_hint_points = 1;
  start.x = start.y = width = height = image_width = image_height = line_step = pixel_step = 0;
  color = C_BACKGROUND;
}


/** Constructor.
 * @param start_x Upper left corner of ROI X coordinate
 * @param start_y Upper left corner of ROI y coordinate
 * @param width Width of extent of ROI
 * @param height height of extent of ROI
 * @param image_width width of full image this ROI belongs to
 * @param image_height height of full image this ROI belongs to
 */
ROI::ROI(unsigned int start_x, unsigned int start_y,
   unsigned int width, unsigned int height,
   unsigned int image_width, unsigned int image_height)
{
  num_hint_points = 1;
  start.x = start_x;
  start.y = start_y;
  this->width = width;
  this->height = height;
  this->image_width = image_width;
  this->image_height = image_height;
  line_step = image_width;
  pixel_step = 1;
  color = C_BACKGROUND;
}


/** Copy constructor.
 * @param roi reference to ROI to copy
 */
ROI::ROI(const ROI &roi)
{
  start           = roi.start;
  width           = roi.width;
  height          = roi.height;
  image_width     = roi.image_width;
  image_height    = roi.image_height;
  line_step       = roi.line_step;
  pixel_step      = roi.pixel_step;
  hint            = roi.hint;
  color           = roi.color;
  num_hint_points = roi.num_hint_points;
}


/** Copy constructor.
 * @param roi pointer to ROI to copy
 */
ROI::ROI(const ROI *roi)
{
  start           = roi->start;
  width           = roi->width;
  height          = roi->height;
  image_width     = roi->image_width;
  image_height    = roi->image_height;
  line_step       = roi->line_step;
  pixel_step      = roi->pixel_step;
  hint            = roi->hint;
  color           = roi->color;
  num_hint_points = roi->num_hint_points;
}


/** Set upper left corner of ROI.
 * @param p point
 */
void
ROI::set_start(upoint_t p)
{
  start.x = p.x;
  start.y = p.y;
}


/** Set upper left corner.
 * @param x x coordinate in image
 * @param y y coordinate in image
 */
void
ROI::set_start(unsigned int x, unsigned int y)
{
  start.x = x;
  start.y = y;
}


/** Set width of ROI.
 * @param width new width
 */
void
ROI::set_width(unsigned int width)
{
  this->width = width;
}


/** Get width of ROI.
 * @return width
 */
unsigned int
ROI::get_width() const
{
  return width;
}


/** Set height of ROI.
 * @param height new height
 */
void
ROI::set_height(unsigned int height)
{
  this->height = height;
}


/** Get height of ROI.
 * @return height
 */
unsigned int
ROI::get_height() const
{
  return height;
}


/** Set full image width.
 * Set the width of the image that contains this ROI.
 * @param image_width full width of image.
 */
void
ROI::set_image_width(unsigned int image_width)
{
  this->image_width = image_width;
}


/** Get full image width.
 * Get the width of the image that contains this ROI.
 * @return full width of image.
 */
unsigned int
ROI::get_image_width() const
{
  return image_width;
}


/** Set full image height
 * Set the height of the image that contains this ROI.
 * @param image_height full height of image.
 */
void
ROI::set_image_height(unsigned int image_height)
{
  this->image_height = image_height;
}


/** Get full image height.
 * Get the height of the image that contains this ROI.
 * @return full height of image.
 */
unsigned int
ROI::get_image_height() const
{
  return image_height;
}


/** Set linestep.
 * The linestep is the offset in bytes from the beginning of one line
 * in the buffer to the beginning of the next line.
 * @param step new line step
 */
void
ROI::set_line_step(unsigned int step)
{
  line_step = step;
}


/** Get linestep.
 * @return line step
 * @see setLineStep()
 */
unsigned int
ROI::get_line_step() const
{
  return line_step;
}


/** Set pixel step.
 * The pixel step is the offset in bytes to get from one pixel to the next
 * in the buffer.
 * @param step new pixel step.
 */
void
ROI::set_pixel_step(unsigned int step)
{
  pixel_step = step;
}


/** Get pixel step.
 * @return pixel step.
 * @see setPixelStep()
 */
unsigned int
ROI::get_pixel_step() const
{
  return pixel_step;
}


/** Get hint.
 * The hint gives an intuition what is in the ROI. In most cases this will
 * depend on the color that the classifier used.
 * @return hint
 */
unsigned int
ROI::get_hint() const
{
  return hint;
}


/** Set hint.
 * @param hint new hint
 * @see getHint()
 */
void
ROI::set_hint(unsigned int hint)
{
  this->hint = hint;
}


/** Check if this ROI contains the given coordinates.
 * @param x x coordinate in image
 * @param y y coordinate in image
 * @return true if this ROI contains the given point, false otherwise
 */
bool
ROI::contains(unsigned int x, unsigned int y)
{
  if ( (x >= start.x) &&
       (x < start.x+width) &&
       (y >= start.y) &&
       (y < start.y+height) ) {

    num_hint_points += 1;
    return true;
  } else {
    return false;
  }
}


/** Intersect this ROI with another.
 * @param roi The other roi
 * @return A new ROI that covers the intersection of this and the other ROI.
 * If there's no intersection, width = height = 0.
 */
ROI
ROI::intersect(ROI const &roi) const
{
  ROI rv;
  if (start.x + width <= roi.start.x
      || roi.start.x + roi.width <= start.x
      || start.y + height <= roi.start.y
      || roi.start.y + roi.height <= start.y) {

    return rv;

  } else {

    rv = new ROI(this);
    rv.start.x = start.x <= roi.start.x ? roi.start.x : start.x;
    rv.start.y = start.y <= roi.start.y ? roi.start.y : start.y;
    if (start.x + width < roi.start.x + roi.width) {
      rv.width = start.x + width - rv.start.x;
    } else {
      rv.width = roi.start.x + roi.width - rv.start.x;
    }
    if (start.y + height < roi.start.y + roi.height) {
      rv.height = start.y + height - rv.start.y;
    } else {
      rv.height = roi.start.y + roi.height - rv.start.y;
    }

  }
  return rv;
}


/** Check if this ROI neighbours a pixel.
 * This checks if the given pixel is close to this ROI considered with
 * the given margin.
 * @param x x coordinate in image
 * @param y y coordinate in image
 * @param margin margin
 * @return true if this ROI is a neigbour of the given pixel, false otherwise
 */
bool
ROI::neighbours(unsigned int x, unsigned int y, unsigned int margin) const
{
  return ( (static_cast<int>(x) >= static_cast<int>(start.x) - static_cast<int>(margin)) &&
     (x <= start.x + width + margin) &&
     (static_cast<int>(y) >= static_cast<int>(start.y) - static_cast<int>(margin)) &&
     (y <= start.y + height + margin) );
}


/** Check if this ROI neighbours another ROI.
 * This checks if the given ROI is close to this ROI considered with
 * the given margin.
 * @param roi ROI
 * @param margin margin
 * @return true if this ROI is a neigbour of the given ROI, false otherwise
 */
bool
ROI::neighbours(ROI *roi, unsigned int margin) const
{
  //Testing only x -> y test returns always true
  bool overlapping_x = neighbours(roi->start.x, start.y, margin)
    || neighbours(roi->start.x + roi->width, start.y, margin)
    || roi->neighbours(start.x, roi->start.y, margin)
    || roi->neighbours(start.x + width, roi->start.y, margin);

  //Testing only y -> x test returns always true
  bool overlapping_y = roi->neighbours(roi->start.x, start.y, margin)
    || roi->neighbours(roi->start.x, start.y + height, margin)
    || neighbours(start.x, roi->start.y, margin)
    || neighbours(start.x, roi->start.y + roi->height, margin);

  return overlapping_x && overlapping_y;
}


/** Extend ROI to include given pixel.
 * @param x x coordinate of pixel to include
 * @param y y coordinate of pixel to include
 */
void
ROI::extend(unsigned int x, unsigned int y)
{

  if (x < start.x) { width  += start.x - x; start.x = x; }
  if (y < start.y) { height += start.y - y; start.y = y; }
  if (x >= start.x + width)  { width  += (x - (start.x + width) + 1); }
  if (y >= start.y + height) { height += (y - (start.y + height) + 1); }

  num_hint_points += 1;
}


/** Grow this ROI by a given margin.
 * @param margin margin to grow by
 */
void
ROI::grow(unsigned int margin)
{
  if (start.x < margin) {
    start.x = 0;
  } else {
    start.x -= margin;
  }

  if (start.y < margin) {
    start.y = 0;
  } else {
    start.y -= margin;
  }

  if ((start.x + width + margin) > image_width) {
    width += (image_width - (start.x + width));
  } else {
    width += margin;
  }

  if ((start.y + height + margin) > image_height) {
    height += (image_height - (start.y + height));
  } else {
    height += margin;
  }

}


/** Merge two ROIs.
 * This ROI will be extended in any direction necessary to fully include the given
 * ROI.
 * @param roi ROI to include
 * @return this instance
 */
ROI&
ROI::operator+=(ROI &roi)
{

  if (roi.start.x < start.x) { width  += start.x - roi.start.x; start.x = roi.start.x; }
  if (roi.start.y < start.y) { height += start.y - roi.start.y; start.y = roi.start.y; }
  if (roi.start.x + roi.width  > start.x + width)  { width  += roi.start.x + roi.width  - (start.x + width); }
  if (roi.start.y + roi.height > start.y + height) { height += roi.start.y + roi.height - (start.y + height); }

  num_hint_points += roi.num_hint_points;

  return *this;
}


/** Check if this ROI contains less hint points than the given ROI.
 * @param roi ROI to compare to.
 * @return true, if the this ROI is smaller, false otherwise
 */
bool
ROI::operator<(const ROI &roi) const
{
  return (color < roi.color) || (num_hint_points < roi.num_hint_points);
}


/** Check if this ROI contains more hint points than the given ROI.
 * @param roi ROI to compare to.
 * @return true, if the this ROI is greater, false otherwise
 */
bool
ROI::operator>(const ROI &roi) const
{
  return (color > roi.color) || (num_hint_points > roi.num_hint_points);
}


/** Check if this ROI marks the same region for the same object
 * and an image of the same base size and step parameters like the
 * given ROI.
 * @param roi ROI to compare to
 * @return true, if ROIs are similar, false otherwise
 */
bool
ROI::operator==(const ROI &roi) const
{
  return  (start.x == roi.start.x) &&
          (start.y == roi.start.y) &&
          (width == roi.width) &&
          (height == roi.height) &&
          (image_width == roi.image_width) &&
          (image_height == roi.image_height) &&
          (line_step == roi.line_step) &&
          (pixel_step == roi.pixel_step) &&
          (hint == roi.hint) &&
          (color == roi.color) &&
          (num_hint_points == roi.num_hint_points);
}


/** Check if this ROI does not mark the same region for the same object
 * and an image of the same base size and step parameters like the
 * given ROI.
 * @param roi ROI to compare to
 * @return true, if ROIs are not similar, false otherwise
 */
bool
ROI::operator!=(const ROI &roi) const
{
  return (num_hint_points != roi.num_hint_points);
}


/** Assign the given ROI data to this ROI.
 * @param roi ROI to copy
 * @return this instance
 */
ROI&
ROI::operator=(const ROI &roi)
{
  this->start.x         = roi.start.x;
  this->start.y         = roi.start.y;
  this->width           = roi.width;
  this->height          = roi.height;
  this->image_width     = roi.image_width;
  this->image_height    = roi.image_height;
  this->line_step       = roi.line_step;
  this->pixel_step      = roi.pixel_step;
  this->hint            = roi.hint;
  this->color           = roi.color;
  this->num_hint_points = roi.num_hint_points;

  return *this;
}

/** Get ROI buffer start.
 * This uses the ROI's step and start data to calculate where
 * the ROI starts in the given buffer.
 * @param buffer buffer
 * @return pointer into buffer where the ROI starts
 */
unsigned char*
ROI::get_roi_buffer_start(unsigned char *buffer) const
{
  return (buffer + (start.y * line_step) + (start.x * pixel_step));
}


/** Gives an estimate of the number of points in this ROI that
 * are classified to the given hint
 * It is: num_hint_points <= total_num_of_scanline_points
 * If you call contains and the point is actually included in
 * this ROI this number is incremented. So you need to make
 * sure to only call contains() for a point of the given hint
 * class. This should always be the case anyway.
 * If you extend the region by one very point the number will
 * be incremented by one although the region may grow by more
 * than just one point of the hint class.
 * If you merge to ROIs by using the += operator this region
 * adds the number of hint points of the region being merged
 * to its own number. The region may grow by more than this
 * number of points though.
 * @return an estimate of the number of points of the hint class
 *
 */
unsigned int
ROI::get_num_hint_points() const
{
  return num_hint_points;
}


/** Get full image ROI for given size.
 * Shortcut to get a full size ROI. This ROI is a static member so this
 * method is not thread-safe or reentrant. It is also only valid until the
 * next call to full_image() with different parameters. Line step is assumed
 * to be the image width, the pixel step is assumed to be one. So this is
 * only useful for b/w or planar images.
 * @param width image width
 * @param height image height
 * @return full image ROI
 */
ROI *
ROI::full_image(unsigned int width, unsigned int height)
{
  if (roi_full_image == NULL) {
    roi_full_image = new ROI();
    roi_full_image->start.x      = 0;
    roi_full_image->start.y      = 0;
    roi_full_image->pixel_step   = 1;
  }
  roi_full_image->width        = width;
  roi_full_image->height       = height;
  roi_full_image->image_width  = roi_full_image->width;
  roi_full_image->image_height = roi_full_image->height;
  roi_full_image->line_step    = roi_full_image->width;

  return roi_full_image;
}

} // end namespace firevision
