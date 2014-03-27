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


#ifndef __FIREVISION_MODELS_COLOR_SIMILARITY_H_
#define __FIREVISION_MODELS_COLOR_SIMILARITY_H_

#include "colormodel.h"
#include <fvutils/color/rgb.h>
#include <fvutils/color/rgbyuv.h>
#include <cmath>
#include <vector>

namespace firevision
{

class ColorModelSimilarity : public firevision::ColorModel
{
  public:
    ColorModelSimilarity();

    virtual color_t determine(unsigned int y, unsigned int u,
      unsigned int v) const;

    virtual const char * get_name();

    /**
     * Parameters that define a certain color
     */
    typedef struct color_class_t {
      private:
        color_t result;
        int ref_u;
        int ref_v;
        int ref_length;
        int chroma_threshold;
        int saturation_threshold;

        /**
         * Define the RGB values for the reference color
         * @param ref A 3-element list [R, G, B]
         */
        void set_reference(std::vector<unsigned int> &ref) {
          if (ref.at(0) > 0xff || ref.at(1) > 0xff || ref.at(2) > 0xff)
            throw "invalid reference color";
          int r = ref.at(0), g = ref.at(1), b = ref.at(2);
          int y, u, v;
          RGB2YUV(r, g, b, y, u, v);
          ref_u = u - 0x80;
          ref_v = v - 0x80;
          ref_length = sqrt(ref_u * ref_u + ref_v * ref_v);
        }
      public:
        /**
         * Initialize a color class
         * @param expect Discrete color_t represented by this class
         * @param v A 3-element list [R, G, B]
         * @param chroma_threshold Required color similarity (higher = more similar), 0..255
         * @param saturation_threshold Required saturation (higher = more saturation), 0..255
         */
        color_class_t(color_t expect, std::vector<unsigned int> &v, int chroma_threshold, int saturation_threshold) {
          this->result = expect;
          this->chroma_threshold = chroma_threshold;
          this->saturation_threshold = saturation_threshold;
          set_reference(v);
        }
    } color_class_t;

    void add_color(color_class_t *color_class);

  private:
    std::vector<color_class_t *> color_classes_;
};

} /* namespace firevision */

#endif /* __FIREVISION_MODELS_COLOR_SIMILARITY_H_ */
