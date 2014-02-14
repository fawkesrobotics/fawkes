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

    typedef struct color_class_t {
      color_t result;
      int ref_u;
      int ref_v;
      int ref_length;
      int chroma_threshold;
      int saturation_threshold;

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
