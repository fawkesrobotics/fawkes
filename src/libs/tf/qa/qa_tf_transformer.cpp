
/***************************************************************************
 *  qa_tf_transformer.cpp - QA for tf transformer
 *
 *  Created: Thu Oct 20 18:18:39 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

// Do not include in api reference
///@cond QA

#include <tf/transformer.h>
#include <cstdio>

using namespace fawkes::tf;

int
main(int argc, char **argv)
{

  printf("Populating data\n");
  Quaternion q(0, 0, 0, 1);
  Vector3 v(1, 0, 0);
  Transform t(q, v);

  fawkes::Time time;

  StampedTransform st(t, time, "/world", "/robot");

  printf("Setting transform\n");
  Transformer transformer;
  transformer.set_transform(st);

  printf("Looking up transform\n");
  StampedTransform res;
  transformer.lookupTransform("/robot", "/world", time, res);

  Quaternion res_q(res.getRotation());
  Vector3 res_v(res.getOrigin());
  printf("Read transform Q (%f,%f,%f,%f) V (%f,%f,%f)\n",
         res_q.x(), res_q.y(), res_q.z(), res_q.w(),
         res_v.x(), res_v.y(), res_v.z());
  printf("Done\n");

  return 0;
}

/// @endcond
