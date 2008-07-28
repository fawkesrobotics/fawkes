/***************************************************************************
 *  color_object_map.cpp - Mapping between color and roi
 *
 *  Created: Mon May 16th 2008
 *  Copyright  2008 Christof Rath <c.rath@student.tugraz.at>
 *
 *  $Id$
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


#include "color_object_map.h"

/** @class ColorObjectMap color_object_map.h <fvutils/color/color_object_map.h>
 * Color mapping class.
 * This class defines a mapping between regions of interest and @see color_t 
 * values. It also provides corresponding @see YUVColor values for a color_t.
 *
 * @param singelton_ A singelton instance of ColorObjectMap
 * @param cols_ A list of color_t with hint_t (ROI's) as index
 * @param rois_ A list of ROI's with color_t as index
 *
 * @author Christof Rath
 */

/** Static initialzer */
ColorObjectMap* ColorObjectMap::singleton_ = NULL;

/** Default Contructor. 
 * The constructor is private to implement a singelton pattern
 */
ColorObjectMap::ColorObjectMap()
{
	col_ = new std::map<hint_t, color_t>();
	roi_ = new std::map<color_t, hint_t>();

	//Standard mapping:
	set_mapping(H_BALL,        C_ORANGE);
	set_mapping(H_ROBOT,       C_BLACK);
	set_mapping(H_ROBOT_OPP,   C_RED);
	set_mapping(H_FIELD,       C_GREEN);
	set_mapping(H_GOAL_YELLOW, C_YELLOW);
	set_mapping(H_GOAL_BLUE,   C_CYAN);
	set_mapping(H_LINE,        C_WHITE);
	set_mapping(H_BACKGROUND,  C_BACKGROUND);
}

/** Destructor */
ColorObjectMap::~ColorObjectMap()
{
	delete col_;
	delete roi_;
}

/**
 * ColorObjectMap getter.
 * @return the one and only instance of ColorObjectMap
 */
ColorObjectMap* 
ColorObjectMap::get_instance()
{
	if (!singleton_)
		singleton_ = new ColorObjectMap();

	return singleton_;
}

/** YUV_t getter.
 * @param color a color_t value (@see color_t enumeration)
 * @return a corresponding YUV color
 */
YUV_t
ColorObjectMap::get_color(color_t color)
{
	switch (color)
	{
		case C_ORANGE:
			return YUV_t(128, 30, 230);

		case C_MAGENTA:
			return YUV_t(105, 212, 234);

		case C_CYAN:
			return YUV_t(178, 170, 0);

		case C_BLUE:
			return YUV_t(29, 255, 107);

		case C_YELLOW:
			return YUV_t(225, 0, 148);

		case C_GREEN:
			return YUV_t(64, 95, 85);

		case C_WHITE:
			return YUV_t(255, 128, 128);

		case C_RED:
			return YUV_t(76, 86, 252);

		case C_BLACK:
			return YUV_t(0, 128, 128);

		default: //also C_BACKGROUND
			return YUV_t(128, 128, 128);
	}
}

/** Mapping setter.
 * Sets the mapping between ROI and color_t values
 * @param roi region of interest (@see hint_t enumeration)
 * @param color matching color_t value (@see color_t enumeration)
 */
void
ColorObjectMap::set_mapping(hint_t roi, color_t color)
{
	hint_t cur_roi = get(color);
	if (cur_roi != H_UNKNOWN)
	{
		color_t cur_col = get(roi);
		(*col_)[cur_roi] = C_OTHER;
		(*roi_)[cur_col] = H_UNKNOWN;
	}

	(*col_)[roi] = color;
	(*roi_)[color] = roi;
}

/** color_t getter.
 * @param roi the ROI of interest
 * @return the matching color_t value
 */
color_t
ColorObjectMap::get(hint_t roi)
{
	if (col_->count(roi))
		return (*col_)[roi];

	return C_OTHER;
}

/** ROI getter
 * @param color value of interest
 * @return corresponding ROI
 */
hint_t
ColorObjectMap::get(color_t color)
{
	if (roi_->count(color))
		return (*roi_)[color];

	return H_UNKNOWN;
}

