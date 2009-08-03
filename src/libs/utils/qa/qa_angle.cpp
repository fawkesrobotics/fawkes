
/***************************************************************************
 *  qa_angle.cpp - angle QA app
 *
 *  Created: Mon Jun 18 15:54:55
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <utils/math/angle.h>
#include <cstdio>

using namespace fawkes;

int
main(int argc, char **argv)
{
  float f = -2 * M_PI;
  float fnm = normalize_mirror_rad(f);
  float expd = 0;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);

  f = 2 * M_PI;
  fnm = normalize_mirror_rad(f);
  expd = 0;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);


  f = 2 * M_PI + 1;
  fnm = normalize_mirror_rad(f);
  expd = 1;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);


  f = - 2 * M_PI - 1.4;
  fnm = normalize_mirror_rad(f);
  expd = -1.4;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);


  f = - 2 * M_PI - 2.9;
  fnm = normalize_mirror_rad(f);
  expd = -2.9;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);


  f = - 3 * M_PI - 1;
  fnm = normalize_mirror_rad(f);
  expd = f + 4 * M_PI;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);


  f = -M_PI;
  float fnr = normalize_rad(f);
  expd = M_PI;
  printf("f=%f   normalize_rad(f)=%f   expected=%f\n", f, fnr, expd);

  f = 3 * M_PI;
  fnr = normalize_rad(f);
  expd = M_PI;
  printf("f=%f   normalize_rad(f)=%f   expected=%f\n", f, fnr, expd);

  f = - 3 * M_PI;
  fnr = normalize_rad(f);
  expd = M_PI;
  printf("f=%f   normalize_rad(f)=%f   expected=%f\n", f, fnr, expd);

  f = - 2 * M_PI - 1;
  fnr = normalize_rad(f);
  expd = 2 * M_PI - 1;
  printf("f=%f   normalize_rad(f)=%f   expected=%f\n", f, fnr, expd);

  f = 10 * M_PI;
  fnr = normalize_rad(f);
  expd = 0;
  printf("f=%f   normalize_rad(f)=%f   expected=%f\n", f, fnr, expd);

  return 0;
}
