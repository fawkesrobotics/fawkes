/***************************************************************************
 * similarity.h - A colormodel that detects colors which are similar to a
 * given reference color. Tolerance is expressed in maximum saturation and
 * chroma deviation.
 *
 * The algorithm is ported from the VLC colorthreshold filter written by
 * Sigmund Augdal and Antoine Cellerier. Cf.
 * modules/video_filter/colorthres.c in the VLC source tree.
 *
 * Initially ported in 2014 by Victor Mataré.
 *
 * The original code is licensed under GPL 2.1, so we do the same.
 ****************************************************************************/

/* This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 */

#include "similarity.h"
#include <stddef.h>
#include <fvutils/color/rgbyuv.h>
#include <fvutils/color/yuv.h>
#include <fvutils/color/threshold.h>
#include <math.h>


namespace firevision
{

/** @class ColorModelSimilarity <fvmodels/color/similarity.cpp>
 * Matches colors that are similar to given reference colors.
 * @author Victor Mataré
 */

ColorModelSimilarity::ColorModelSimilarity()
{
  color_classes_ = NULL;
  num_classes_ = 0;
}

ColorModelSimilarity::~ColorModelSimilarity()
{
  free(color_classes_);
}

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
  for(int i=num_classes_ - 1; i >= 0; i--) {
    if(is_similar(u - 0x80, v - 0x80,
        color_classes_[i].ref_u, color_classes_[i].ref_v, color_classes_[i].ref_length,
        color_classes_[i].chroma_threshold, color_classes_[i].saturation_threshold)) {
      return color_classes_[i].result;
    }
  }
  return C_OTHER;
}

/** Add a color to be recognized by this colormodel.
 * @param color_class The ::color_t that will be returned by ColorModelSimilarity::determine on a match
 * @param reference The base color for the similarity matcher
 * @param chroma_threshold Maximum difference between reference an input color
 * @param saturation_threshold Minimum saturation of input color
 */
void ColorModelSimilarity::add_color(color_t color_class, RGB_t reference, int chroma_threshold, int saturation_threshold) {
  color_classes_ = (color_class_t_ *) realloc((void *)color_classes_, (num_classes_ + 1) * sizeof(color_class_t_));

  color_classes_[num_classes_].result = color_class;
  color_classes_[num_classes_].chroma_threshold = chroma_threshold;
  color_classes_[num_classes_].saturation_threshold = saturation_threshold;

  int ignore, ref_u, ref_v;

  RGB2YUV(reference.R, reference.G, reference.B,
    ignore, ref_u, ref_v);
  ref_u -= 0x80;
  ref_v -= 0x80;
  color_classes_[num_classes_].ref_u = ref_u;
  color_classes_[num_classes_].ref_v = ref_v;
  color_classes_[num_classes_].ref_length = sqrt(ref_u * ref_u + ref_v * ref_v);

  num_classes_++;
}

} /* namespace firevision */


