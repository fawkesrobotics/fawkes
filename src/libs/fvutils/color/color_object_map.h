
/***************************************************************************
 *  color_object_map.h - Mapping between color and roi
 *
 *  Created: Mon May 16th 2008
 *  Copyright  2008 Christof Rath <c.rath@student.tugraz.at>
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

#ifndef __FIREVISION_MODELS_COLOR_COLOR_MAPPING_H__
#define __FIREVISION_MODELS_COLOR_COLOR_MAPPING_H__

#include <fvutils/base/types.h>
#include <fvutils/base/roi.h>
#include <fvutils/color/yuv.h>

#include <map>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ColorObjectMap
{
 public:
  ~ColorObjectMap();
  static const ColorObjectMap& get_instance() { return *__singleton; }
  static YUV_t get_color(color_t color);

  const color_t& get(hint_t hint) const
  { return __color_for_hint.find(hint) != __color_for_hint.end() ? __color_for_hint.find(hint)->second : __c_other; }
  const hint_t get(color_t color) const
  { return __hint_for_color.find(color) != __hint_for_color.end() ? __hint_for_color.find(color)->second : __h_unknown; }

 private:
  ColorObjectMap();
  void set_mapping(hint_t roi, color_t color);

  static ColorObjectMap    *__singleton;
  std::map<hint_t, color_t> __color_for_hint;
  std::map<color_t, hint_t> __hint_for_color;
  color_t                   __c_other;
  hint_t                    __h_unknown;
};

} // end namespace firevision

#endif // __FIREVISION_MODELS_COLOR_COLOR_MAPPING_H__
