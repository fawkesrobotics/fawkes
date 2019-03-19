/***************************************************************************
 *  field_drawer.h - Encapsulates a soccer field
 *
 *  Created:  Tue Sep 23 00:00:00 2009
 *  Copyright 2008 Christof Rath <christof.rath@gmail.com>
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef _FVUTILS_DRAWER_FIELD_DRAWER_H__
#define _FVUTILS_DRAWER_FIELD_DRAWER_H__

#include <fvutils/color/yuv.h>
#include <fvutils/draw/field.h>
#include <utils/math/types.h>

namespace firevision {

class SharedMemoryImageBuffer;

class FieldDrawer
{
public:
	FieldDrawer(const FieldLines &lines);
	virtual ~FieldDrawer();

	void set_head_yaw(float head_yaw);
	void set_own_pos(fawkes::field_pos_t own_position);
	void set_own_pos_est(fawkes::field_pos_t own_position_estimate);

	void set_line_points(const fld_line_points_t *points);
	void set_line_points_est(const fld_line_points_t *points_est);

	void set_color_background(YUV_t color);
	void set_color_field(YUV_t color);
	void set_color_lines(YUV_t color);
	void set_color_line_points(YUV_t color);
	void set_color_line_points_est(YUV_t color);
	void set_color_own_pos(YUV_t color);
	void set_color_own_pos_est(YUV_t color);

	virtual void draw_field(unsigned char *yuv422_planar,
	                        unsigned int   img_width,
	                        unsigned int   img_height,
	                        bool           draw_background = true,
	                        bool           draw_landscape  = true);

protected:
	inline void clear_own_pos();
	inline float
	             get_scale(unsigned int img_width, unsigned int img_height, bool draw_landscape = true) const;
	virtual void draw_line_points(bool draw_landscape = true, float scale = 0) const;
	virtual void draw_lines(YUV_t color, bool draw_landscape = true, float scale = 0) const;

	unsigned char *_img_buffer;
	unsigned int   _img_width;
	unsigned int   _img_height;

private: //Members
	const FieldLines &  lines_;
	fawkes::field_pos_t own_position_, own_pos_est_;
	float               head_yaw_;

	const fld_line_points_t *points_;
	const fld_line_points_t *points_est_;

	YUV_t c_background_;
	YUV_t c_field_;
	YUV_t c_lines_;
	YUV_t c_line_points_;
	YUV_t c_line_points_est_;
	YUV_t c_own_pos_;
	YUV_t c_own_pos_est_;
};

} // end namespace firevision

#endif
