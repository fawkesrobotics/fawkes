
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
#include <plugins/openni/utils/colors.h>

#include <fvcams/camera.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>

#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <GL/glut.h>

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
				       unsigned int max_depth)
  : SkelGuiTextureDrawer(depth_cam->pixel_width(), depth_cam->pixel_height()),
    __max_depth(max_depth)
{
  __depth_cam      = depth_cam;
  __label_cam      = label_cam;
  __rgb_buf        = malloc_buffer(RGB, __width, __height);
  __histogram      = (float *)malloc(__max_depth * sizeof(float));
  __show_labels    = true;
}

/** Destructor. */
SkelGuiDepthDrawer::~SkelGuiDepthDrawer()
{
  free(__rgb_buf);
  free(__histogram);
}

/** Toggle label state.
 * Turns on or off the label coloring of the depth map.
 */
void
SkelGuiDepthDrawer::toggle_show_labels()
{
  __show_labels = ! __show_labels;
}

/** Fill texture. */
void
SkelGuiDepthDrawer::fill_texture()
{
  try {
    __depth_cam->capture();
  } catch (Exception &e) {
    printf("Capturing depth image failed, exception follows\n");
    e.print_trace();
    throw;
  }

  uint16_t *depth = (uint16_t *)__depth_cam->buffer();
  unsigned int num_points = 0;
  memset(__histogram, 0, __max_depth * sizeof(float));

  // base histogram
  for (unsigned int i = 0; i < __width * __height; ++i) {
    if (depth[i] != 0) {
      ++__histogram[depth[i]];
      ++num_points;
    }
  }

  // accumulative histogram
  for (unsigned int i = 1; i < __max_depth; ++i) {
    __histogram[i] += __histogram[i-1];
  }

  // set gray value in histogram
  if (num_points > 0) {
    for (unsigned int i = 1; i < __max_depth; ++i) {
      __histogram[i] = truncf(256. * (1.f - (__histogram[i] / num_points)));
    }
  }

  if (__label_cam) {
    try {
      __label_cam->capture();
    } catch (Exception &e) {
      printf("Capturing label image failed, exception follows\n");
      e.print_trace();
      throw;
    }
    uint16_t *l = (uint16_t *)__label_cam->buffer();
    uint16_t *d = depth;
    unsigned char *r = __rgb_buf;
    for (unsigned int i = 0; i < __width * __height; ++i, ++l, ++d, r += 3) {
      r[0] = 0; r[1] = 0; r[2] = 0;
      unsigned int color = *l % NUM_USER_COLORS;
      if (!__show_labels || (*l == 0)) color = NUM_USER_COLORS;

      if (*d != 0) {
	float hv = __histogram[*d];
	r[0] = hv * USER_COLORS[color][0];
	r[1] = hv * USER_COLORS[color][1];
	r[2] = hv * USER_COLORS[color][2];
      }
    }
    __label_cam->dispose_buffer();
  } else {
    uint16_t *d = depth;
    unsigned char *r = __rgb_buf;
    for (unsigned int i = 0; i < __width * __height; ++i, ++d, r += 3) {
      r[0] = 0; r[1] = 0; r[2] = 0;
      if (*d != 0) {
	float hv = __histogram[*d];
	r[0] = hv;
	r[1] = hv;
	r[2] = hv;
      }
    }
  }

  copy_rgb_to_texture(__rgb_buf);

  __depth_cam->dispose_buffer();
}

