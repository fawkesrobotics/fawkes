
/***************************************************************************
 *  color_object_map.cpp - Mapping between color and roi
 *
 *  Created: Mon May 16 00:00:00 2008
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

#include <fvutils/color/color_object_map.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColorObjectMap <fvutils/color/color_object_map.h>
 * Color mapping class.
 * This class defines a mapping between regions of interest and @see color_t
 * values. It also provides corresponding @see YUVColor values for a color_t.
 *
 * @author Christof Rath
 */

/** @var static ColorObjectMap* ColorObjectMap::__singleton
 * A singelton instance of ColorObjectMap
 */
/** @var std::map<hint_t, color_t> ColorObjectMap::__color_for_hint
 * A list of color_t with hint_t (ROI's) as index
 */
/** @var std::map<color_t, hint_t> ColorObjectMap::__hint_for_color
 * A list of hint_t (ROI's) with color_t as index
 */
/** @var color_t ColorObjectMap::__c_other
 * The default color
 */
/** @var hint_t ColorObjectMap::__h_unknown
 * The default hint
 */

/** @fn static const ColorObjectMap& ColorObjectMap::get_instance()
 * ColorObjectMap getter.
 * @return the one and only instance of ColorObjectMap
 */

/** @fn const color_t& ColorObjectMap::get(hint_t hint) const
 * Inline color_t reference getter.
 * @param hint the ROI of interest
 * @return the matching color_t value
 */

/** @fn const hint_t ColorObjectMap::get(color_t color) const
 * Inline hint_t(ROI) reference getter
 * @param color value of interest
 * @return corresponding ROI
 */

/** Static initialzer */
ColorObjectMap* ColorObjectMap::__singleton = new ColorObjectMap();

/** Default Contructor.
 * The constructor is private to implement a singelton pattern
 */
ColorObjectMap::ColorObjectMap()
{
  __c_other   = C_OTHER;
  __h_unknown = H_UNKNOWN;

  //Standard mapping:
  set_mapping(H_BALL, C_ORANGE);
  set_mapping(H_ROBOT, C_BLACK);
  set_mapping(H_ROBOT_OPP, C_RED);
  set_mapping(H_FIELD, C_GREEN);
  set_mapping(H_GOAL_YELLOW, C_YELLOW);
  set_mapping(H_GOAL_BLUE, C_CYAN);
  set_mapping(H_LINE, C_WHITE);
  set_mapping(H_BACKGROUND, C_BACKGROUND);
}

/** Destructor */
ColorObjectMap::~ColorObjectMap()
{
}

/** YUV_t getter.
 * @param color a color_t value (@see color_t enumeration)
 * @return a corresponding YUV color
 */
YUV_t ColorObjectMap::get_color(color_t color)
{
  switch (color) {
    case C_ORANGE:
      return YUV_t::orange();

    case C_MAGENTA:
      return YUV_t::magenta();

    case C_CYAN:
      return YUV_t::cyan();

    case C_BLUE:
      return YUV_t::blue();

    case C_YELLOW:
      return YUV_t::yellow();

    case C_GREEN:
      return YUV_t::green();

    case C_WHITE:
      return YUV_t::white();

    case C_RED:
      return YUV_t::red();

    case C_BLACK:
      return YUV_t::black();

    default: //also C_BACKGROUND
      return YUV_t::gray();
  }
}

/** Mapping setter.
 * Sets the mapping between ROI and color_t values
 * @param roi region of interest (@see hint_t enumeration)
 * @param color matching color_t value (@see color_t enumeration)
 */
void ColorObjectMap::set_mapping(hint_t roi, color_t color)
{
  hint_t cur_roi = get(color);
  if (cur_roi != H_UNKNOWN) //There is a previous mapping -> unlink it
  {
    color_t cur_col = get(roi);
    __color_for_hint[cur_roi] = C_OTHER;
    __hint_for_color[cur_col] = H_UNKNOWN;
  }

  __color_for_hint[roi] = color;
  __hint_for_color[color] = roi;
}

} // end namespace firevision
