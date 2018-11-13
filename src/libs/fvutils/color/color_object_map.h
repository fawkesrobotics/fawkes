
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

#ifndef _FIREVISION_MODELS_COLOR_COLOR_MAPPING_H__
#define _FIREVISION_MODELS_COLOR_COLOR_MAPPING_H__

#include <fvutils/base/types.h>
#include <fvutils/base/roi.h>
#include <fvutils/color/yuv.h>

#include <map>

namespace firevision {

class ColorObjectMap
{
 public:
  ~ColorObjectMap();
  static const ColorObjectMap& get_instance() { return *singleton_; }
  static YUV_t get_color(color_t color);

  color_t get(hint_t hint) const
  {
	  return color_for_hint_.find(hint) != color_for_hint_.end()
	    ? color_for_hint_.find(hint)->second
	    : c_other_;
  }
  hint_t get(color_t color) const
  {
	  return hint_for_color_.find(color) != hint_for_color_.end()
	    ? hint_for_color_.find(color)->second
	    : h_unknown_;
  }

 private:
  ColorObjectMap();
  void set_mapping(hint_t roi, color_t color);

  static ColorObjectMap    *singleton_;
  std::map<hint_t, color_t> color_for_hint_;
  std::map<color_t, hint_t> hint_for_color_;
  color_t                   c_other_;
  hint_t                    h_unknown_;
};

} // end namespace firevision

#endif // FIREVISION_MODELS_COLOR_COLOR_MAPPING_H___
