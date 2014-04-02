/***************************************************************************
 * similarity.h - A colormodel that detects colors which are similar to a
 * given reference color. Tolerance is expressed in maximum saturation and
 * chroma deviation.
 *
 * Uses the algorithm ported from the VLC colorthreshold filter written by
 * Sigmund Augdal and Antoine Cellerier. Cf.
 * modules/video_filter/colorthres.c in the VLC source tree.
 *
 * (C) 2014 Victor Mataré.
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

#include "similarity.h"
#include <stddef.h>
#include <fvutils/color/yuv.h>
#include <fvutils/color/threshold.h>
#include <math.h>


namespace firevision
{

/** @class ColorModelSimilarity <fvmodels/color/similarity.cpp>
 * Matches colors that are similar to given reference colors.
 * @author Victor Mataré
 */

ColorModelSimilarity::ColorModelSimilarity() {}

const char * ColorModelSimilarity::get_name() {
  return "ColorModelSimilarity";
}

/** Determine the color class of a given YUV value.
 * Color classes have to be defined beforehand with ColorModelSimilarity::add_color.
 * If multiple color classes have been defined, they are tried in reverse order, i.e. the class
 * that has been added last is tried first. We return on the first match, so think of the color
 * classes as a priority list.
 * @param y Luminance (ignored)
 * @param u Chroma U
 * @param v Chroma V
 * @return The color_t value from the matching color class, or C_OTHER if no match was found.
 */
color_t ColorModelSimilarity::determine(unsigned int y, unsigned int u, unsigned int v) const {
  for(std::vector<color_class_t *>::const_iterator it = color_classes_.begin();
      it != color_classes_.end(); it++) {
    if((*it)->luma_threshold >= 0) {
      if (is_similar_y(y, u - 0x80, v - 0x80,
        (*it)->ref_y, (*it)->ref_u, (*it)->ref_v, (*it)->ref_length,
        (*it)->chroma_threshold, (*it)->saturation_threshold, (*it)->luma_threshold)) {
        return (*it)->result;
      }
    }
    else {
      if(is_similar(u - 0x80, v - 0x80,
        (*it)->ref_u, (*it)->ref_v, (*it)->ref_length,
        (*it)->chroma_threshold, (*it)->saturation_threshold)) {
        return (*it)->result;
      }
    }
  }
  return C_OTHER;
}

/** Add a color to be recognized by this colormodel.
 * @param color_class The ColorModelSimilarity::color_class_t that will be returned by
 * ColorModelSimilarity::determine on a match ColorModelSimilarity::color_class_t
 */
void ColorModelSimilarity::add_color(color_class_t *color_class) {
  color_classes_.push_back(color_class);
}

/** Add multiple colors to this colormodel.
 * @param color_classes A list of
 */
void ColorModelSimilarity::add_colors(std::vector<color_class_t *> color_classes) {
  color_classes_.insert(color_classes_.end(), color_classes.begin(), color_classes.end());
}

/** Remove all colors from this colormodel.
 */
void ColorModelSimilarity::delete_colors() {
  color_classes_.clear();
}

} /* namespace firevision */


