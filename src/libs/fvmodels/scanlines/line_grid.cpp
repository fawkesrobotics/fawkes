
/***************************************************************************
 *  line_grid.cpp - Implementation of the line grid scanline model
 *
 *  Created: Wed Mar 25 17:31:00 2009
 *  Copyright  2009 Christof Rath <c.rath@student.tugraz.at>
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

#include <core/exceptions/software.h>
#include <fvmodels/scanlines/line_grid.h>
#include <fvutils/base/roi.h>
#include <fvutils/draw/drawer.h>

#include <cstring>

using fawkes::upoint_t;

namespace firevision {

/** @class ScanlineLineGrid <fvmodels/scanlines/line_grid.h>
 * Grid of scan lines.
 * A grid of scan lines (i.e. horizontal and/or vertical lines) instead of only
 * points on the grid crossings.
 * The behavior of the ScanlineGrid (grid.h) class can be modeled if offset_hor
 * is set to the same value as offset_x in the Grid class, offset_ver = 0 and
 * gap is set to offset_y - 1. The advantage of doing this is a performance gain
 * as the LineGrid is pre-calculated and getting the next point is only an
 * iterator increment.
 */

/** Constructor.
 * @param width Width of grid (most likely equal to image_width)
 * @param height Height of grid (most likely equal to image_height)
 * @param offset_hor Offset between horizontal lines (set to 0 to get only vertical lines)
 * @param offset_ver Offset between vertical lines (set to 0 to get only horizontal lines)
 * @param roi The grid will only be calculated within the roi (if NULL the grid gets
 *            calculated over the complete width/height).
 *            The provided object will be deleted by ScanlineLineGrid!
 * @param gap Gap between two points on the line
 */
ScanlineLineGrid::ScanlineLineGrid(unsigned int width,
                                   unsigned int height,
                                   unsigned int offset_hor,
                                   unsigned int offset_ver,
                                   ROI *        roi,
                                   unsigned int gap)
{
	roi_        = NULL;
	next_pixel_ = gap + 1;
	set_grid_params(width, height, offset_hor, offset_ver, roi);
	//reset is done in set_grid_params ()
}

/** Destructor
 */
ScanlineLineGrid::~ScanlineLineGrid()
{
	delete roi_;
}

upoint_t ScanlineLineGrid::operator*()
{
	return *cur_;
}

upoint_t *ScanlineLineGrid::operator->()
{
	return &*cur_;
}

void
ScanlineLineGrid::calc_coords()
{
	point_list_.clear();
	bool         more_to_come = true;
	upoint_t     coord;
	unsigned int next_px;

	if (offset_hor_ > 0) //horizontal lines
	{
		more_to_come = true;
		next_px      = std::min(next_pixel_, offset_ver_ ? offset_ver_ : width_);
		coord.x      = roi_->start.x;
		coord.y      = roi_->start.y
		          + ((roi_->height - 1) % offset_hor_) / 2; //Center the horizontal lines in the image
		point_list_.push_back(coord);

		while (more_to_come) {
			if (coord.x < (roi_->image_width - next_px)) {
				coord.x += next_px;
			} else {
				if (coord.y < (roi_->image_height - offset_hor_)) {
					coord.x = roi_->start.x;
					coord.y += offset_hor_;
				} else {
					more_to_come = false;
				}
			}

			if (more_to_come)
				point_list_.push_back(coord);
		}
	}

	if (offset_ver_ > 0) //vertical lines
	{
		more_to_come = true;
		next_px      = std::min(next_pixel_, offset_hor_ ? offset_hor_ : height_);
		coord.x      = roi_->start.x
		          + ((roi_->width - 1) % offset_ver_) / 2; //Center the vertical lines in the image
		coord.y = roi_->start.y;
		point_list_.push_back(coord);

		while (more_to_come) {
			if (coord.y < (roi_->image_height - next_px)) {
				coord.y += next_px;
			} else {
				if (coord.x < (roi_->image_width - offset_ver_)) {
					coord.x += offset_ver_;
					coord.y = roi_->start.y;
				} else {
					more_to_come = false;
				}
			}

			if (more_to_come)
				point_list_.push_back(coord);
		}
	}

	reset();
}

upoint_t *
ScanlineLineGrid::operator++()
{
	if (cur_ != point_list_.end())
		++cur_;
	return cur_ != point_list_.end() ? &*cur_ : &point_list_.back();
}

upoint_t *
ScanlineLineGrid::operator++(int)
{
	if (cur_ != point_list_.end()) {
		upoint_t *res = &*cur_++;
		return res;
	} else
		return &point_list_.back();
}

bool
ScanlineLineGrid::finished()
{
	return cur_ == point_list_.end();
}

void
ScanlineLineGrid::reset()
{
	cur_ = point_list_.begin();
}

const char *
ScanlineLineGrid::get_name()
{
	return "ScanlineModel::LineGrid";
}

unsigned int
ScanlineLineGrid::get_margin()
{
	return std::max(offset_ver_, offset_hor_);
}

void
ScanlineLineGrid::set_robot_pose(float x, float y, float ori)
{
	// ignored
}

void
ScanlineLineGrid::set_pan_tilt(float pan, float tilt)
{
	// ignored
}

/** Sets the dimensions of the grid.
 * Set width and height of scanline grid. Implicitly resets the grid.
 *
 * @param width Width of grid (most likely equal to image_width)
 * @param height Height of grid (most likely equal to image_height)
 * @param roi The grid will only be calculated within the roi (if NULL the grid gets
 *            calculated over the complete width/height).
 *            The provided object will be deleted by ScanlineLineGrid!
 */
void
ScanlineLineGrid::set_dimensions(unsigned int width, unsigned int height, ROI *roi)
{
	width_  = width;
	height_ = height;

	set_roi(roi);
}

/** Sets the region-of-interest.
 * @param roi The grid will only be calculated within the roi (if NULL the grid gets
 *            calculated over the complete width/height).
 *            The provided object will be deleted by ScanlineLineGrid!
 */
void
ScanlineLineGrid::set_roi(ROI *roi)
{
	delete roi_;

	if (!roi)
		roi_ = new ROI(0, 0, width_, height_, width_, height_);
	else {
		roi_ = roi;
		//Use roi image width/height as grid boundary
		roi_->set_image_width(roi_->start.x + roi_->width);
		roi_->set_image_height(roi_->start.y + roi_->height);

		if (roi_->image_width > width_)
			throw fawkes::OutOfBoundsException("ScanlineLineGrid: ROI is out of grid bounds!",
			                                   roi_->image_width,
			                                   0,
			                                   width_);
		if (roi_->image_height > height_)
			throw fawkes::OutOfBoundsException("ScanlineLineGrid: ROI is out of grid bounds!",
			                                   roi_->image_height,
			                                   0,
			                                   height_);
	}

	calc_coords();
}

/** Sets offset.
 * Set horizontal and vertical offset by which the pointer in the grid is advanced.
 * This function implicitly resets the grid.
 *
 * @param offset_hor Offset between horizontal lines (set to 0 to get only vertical lines)
 * @param offset_ver Offset between vertical lines (set to 0 to get only horizontal lines)
 */
void
ScanlineLineGrid::set_offset(unsigned int offset_hor, unsigned int offset_ver)
{
	offset_hor_ = offset_hor;
	offset_ver_ = offset_ver;

	calc_coords();
}

/** Set all grid parameters.
 * Set width, height, horizontal and vertical offset by which the pointer in the
 * grid is advanced.
 * Implicitly resets the grid.
 *
 * @param width Width of grid (most likely equal to image_width)
 * @param height Height of grid (most likely equal to image_height)
 * @param offset_hor Offset between horizontal lines (set to 0 to get only vertical lines)
 * @param offset_ver Offset between vertical lines (set to 0 to get only horizontal lines)
 * @param roi The grid will only be calculated within the roi (if NULL the grid gets
 *            calculated over the complete width/height).
 *            The provided object will be deleted by ScanlineLineGrid!
 */
void
ScanlineLineGrid::set_grid_params(unsigned int width,
                                  unsigned int height,
                                  unsigned int offset_hor,
                                  unsigned int offset_ver,
                                  ROI *        roi)
{
	offset_hor_ = offset_hor;
	offset_ver_ = offset_ver;

	set_dimensions(width, height, roi);
}

} // end namespace firevision
