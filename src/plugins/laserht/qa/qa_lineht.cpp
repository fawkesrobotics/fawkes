
/***************************************************************************
 *  qa_lineht.cpp - QA for Line Hough Transform
 *
 *  Created: Wed Dec 30 12:00:00 2009
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
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

/// @cond QA

#include "../hough_transform.h"
#include <utils/time/tracker.h>
#include <utils/math/angle.h>
#include <utils/math/coord.h>

#include <cstdio>
#include <unistd.h>

using namespace fawkes;

int
main(int argc, char **argv)
{
  HoughTransform *ht = new HoughTransform(2);

  unsigned int num_vals = 24;
  int angle_step = 360 / num_vals;
  float r_scale = 100.;

  int **values = new int*[num_vals];
  for (unsigned int i = 0; i < num_vals; ++i) {
    values[i] = new int[2];
  }

  float samples[][2] =
    { { 0,  1}, { 1,  0},
      { 0,  1}, {-1,  0},
      {-1,  0}, { 0, -1},
      { 1,  0}, { 0, -1},
      { 0,  1}, { 1,  1},
      { 1,  0}, { 1,  1},
      { 0, -1}, { 1, -1},
      {-1,  0}, {-1,  1}
    };

  printf("Num samples: %zu\n", (sizeof(samples)/sizeof(float *))/2);

  for (size_t S = 0; S < (sizeof(samples)/sizeof(float *))/2; ++S) {
    float x[2], y[2];
    x[0] =  samples[2 * S    ][0]; y[0] = samples[2 * S    ][1];
    x[1] =  samples[2 * S + 1][0]; y[1] = samples[2 * S + 1][1];

    ht->reset();

    for (unsigned int i = 0; i < 2; ++i) {
      for (unsigned int j = 0; j < num_vals; ++j) {
	float theta = deg2rad(j * angle_step);
	float r   = x[i] * cos(theta) + y[i] * sin(theta);
	r *= r_scale;
	values[j][0] = (int)roundf(r);
	values[j][1] = j * angle_step;
	//printf("i=%u  j=%u  theta=%f  r=%f v[0]=%i  v[1]=%i\n",
	//		i, j, theta, r, values[j][0], values[j][1]);
      }
      ht->process(values, num_vals);
    }

    int max_values[2];
    unsigned int max_count = ht->max(max_values);
    printf("Max count: %u  (%i, %i)\n", max_count, max_values[0],
	   max_values[1]);

    float phi = deg2rad(max_values[1]);
    float r   = max_values[0] / r_scale;
    float x1, y1, x2, y2;
    polar2cart2d(phi, r, &x1, &y1);

    float y_factor = 1;
    float alpha; // = deg2rad((max_values[1] % 90));
    if ( ((max_values[1] >= 0) && (max_values[1] < 90)) ||
	 (max_values[1] >= 270) ) {
      y_factor = -1;
      alpha = deg2rad(90 - (max_values[1] % 90));
    } else {
    alpha = deg2rad((max_values[1] % 90));
    }
    float dx   = 1 * cos(alpha);
    float dy   = 1 * y_factor * sin(alpha);
    x2 = x1 + dx;
    y2 = y1 + dy;
    
    printf("p1=(%f,%f)  p2=(%f, %f)\n", x[0], y[0], x[1], y[1]);
    printf("r=%f  phi=%f  alpha=%f  dx=%f  dy=%f  p1=(%f,%f)  p2=(%f,%f)\n\n",
	   r, phi, alpha, dx, dy, x1, y1, x2, y2);

  }

  delete ht;
  for (unsigned int i = 0; i < num_vals; ++i) {
    delete[] values[i];
  }
  delete[] values;

  return 0;
}

/// @endcond
