
/***************************************************************************
 *  depth_drawer.cpp - Skeleton Visualization GUI: depth drawer
 *
 *  Created: Tue Mar 29 17:17:47 2011 (on the way to Magdeburg for GO2011)
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "depth_drawer.h"

#include <GL/glut.h>
#include <core/exception.h>
#include <fvcams/camera.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>
#include <plugins/openni/utils/colors.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace fawkes;
using namespace fawkes::openni;
using namespace firevision;

/** @class SkelGuiDepthDrawer "image_drawer.h"
 * Draw images from camera in texture.
 * Uses texture mapping to show an image acquired from a camera in the
 * background.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param depth_cam camera to capture depth image
 * @param label_cam label to capture label frame
 * @param max_depth maximum depth value to expect
 */
SkelGuiDepthDrawer::SkelGuiDepthDrawer(firevision::Camera *depth_cam,
                                       firevision::Camera *label_cam,
                                       unsigned int        max_depth)
: SkelGuiTextureDrawer(depth_cam->pixel_width(), depth_cam->pixel_height()),
  depth_cam_(depth_cam),
  label_cam_(label_cam),
  rgb_buf_raii_(malloc_buffer(RGB, width_, height_)),
  rgb_buf_((unsigned char *)*rgb_buf_raii_),
  max_depth_(max_depth),
  histogram_raii_(malloc(max_depth_ * sizeof(float))),
  histogram_((float *)*histogram_raii_),
  show_labels_(true)
{
}

/** Destructor. */
SkelGuiDepthDrawer::~SkelGuiDepthDrawer()
{
}

/** Toggle label state.
 * Turns on or off the label coloring of the depth map.
 */
void
SkelGuiDepthDrawer::toggle_show_labels()
{
	show_labels_ = !show_labels_;
}

/** Fill texture. */
void
SkelGuiDepthDrawer::fill_texture()
{
	try {
		depth_cam_->capture();
	} catch (Exception &e) {
		printf("Capturing depth image failed, exception follows\n");
		e.print_trace();
		throw;
	}

	uint16_t *   depth      = (uint16_t *)depth_cam_->buffer();
	unsigned int num_points = 0;
	memset(histogram_, 0, max_depth_ * sizeof(float));

	// base histogram
	for (unsigned int i = 0; i < width_ * height_; ++i) {
		if (depth[i] != 0) {
			++histogram_[depth[i]];
			++num_points;
		}
	}

	// accumulative histogram
	for (unsigned int i = 1; i < max_depth_; ++i) {
		histogram_[i] += histogram_[i - 1];
	}

	// set gray value in histogram
	if (num_points > 0) {
		for (unsigned int i = 1; i < max_depth_; ++i) {
			histogram_[i] = truncf(256. * (1.f - (histogram_[i] / num_points)));
		}
	}

	if (label_cam_) {
		try {
			label_cam_->capture();
		} catch (Exception &e) {
			printf("Capturing label image failed, exception follows\n");
			e.print_trace();
			throw;
		}
		uint16_t *     l = (uint16_t *)label_cam_->buffer();
		uint16_t *     d = depth;
		unsigned char *r = rgb_buf_;
		for (unsigned int i = 0; i < width_ * height_; ++i, ++l, ++d, r += 3) {
			r[0]               = 0;
			r[1]               = 0;
			r[2]               = 0;
			unsigned int color = *l % NUM_USER_COLORS;
			if (!show_labels_ || (*l == 0))
				color = NUM_USER_COLORS;

			if (*d != 0) {
				float hv = histogram_[*d];
				r[0]     = hv * USER_COLORS[color][0];
				r[1]     = hv * USER_COLORS[color][1];
				r[2]     = hv * USER_COLORS[color][2];
			}
		}
		label_cam_->dispose_buffer();
	} else {
		uint16_t *     d = depth;
		unsigned char *r = rgb_buf_;
		for (unsigned int i = 0; i < width_ * height_; ++i, ++d, r += 3) {
			r[0] = 0;
			r[1] = 0;
			r[2] = 0;
			if (*d != 0) {
				float hv = histogram_[*d];
				r[0]     = hv;
				r[1]     = hv;
				r[2]     = hv;
			}
		}
	}

	copy_rgb_to_texture(rgb_buf_);

	depth_cam_->dispose_buffer();
}
