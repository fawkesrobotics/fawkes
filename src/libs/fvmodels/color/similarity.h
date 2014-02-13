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

    typedef struct _color_class_t {
      color_t result;
      int ref_u;
      int ref_v;
      int ref_length;
      int chroma_threshold;
      int saturation_threshold;

      _color_class_t(color_t expect, RGB_t reference, int chroma_threshold, int saturation_threshold) {
        this->result = expect;
        int ignore;
        RGB2YUV(reference.R, reference.G, reference.B,
          ignore, ref_u, ref_v);
        ref_u -= 0x80;
        ref_v -= 0x80;
        this->ref_length = sqrt(ref_u * ref_u + ref_v * ref_v);
        this->chroma_threshold = chroma_threshold;
        this->saturation_threshold = saturation_threshold;
      }
    } color_class_t;

    void add_color(color_class_t *color_class);

  private:
    std::vector<color_class_t *> color_classes_;
};

} /* namespace firevision */

#endif /* __FIREVISION_MODELS_COLOR_SIMILARITY_H_ */
