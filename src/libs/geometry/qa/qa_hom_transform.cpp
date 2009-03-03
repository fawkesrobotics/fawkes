
/***************************************************************************
 *  qa_matrix.cpp - DESC
 *
 *  Created:  Fri Feb 13 14:31:48 2009
 *  Copyright 2009 Christof Rath <christof.rath@gmail.com>
 *
 *  $Id$
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
/// @cond EXAMPLES

#include <geometry/hom_transform.h>
#include <geometry/hom_point.h>
#include <utils/time/tracker.h>

#include <iostream>
#include <cmath>

using namespace fawkes;
using namespace std;

int
main(int argc, char **argv)
{
  TimeTracker *tt = new TimeTracker();
  unsigned int loop_count = 0;
  unsigned int ttc_trans = tt->add_class("Tra");
  unsigned int ttc_rot = tt->add_class("Rot");
  unsigned int ttc_inv = tt->add_class("Inv");

  HomTransform ht;
  for (loop_count = 0; loop_count < 10; ++loop_count) {
    tt->ping_start(ttc_trans);
    ht.trans(1, 2, 3);
    tt->ping_end(ttc_trans);

    tt->ping_start(ttc_rot);
    ht.rotate_x(M_PI_2);
    tt->ping_end(ttc_rot);

    tt->ping_start(ttc_trans);
    ht.trans(1, 2, 3);
    tt->ping_end(ttc_trans);

    tt->ping_start(ttc_rot);
    ht.rotate_y(23);
    tt->ping_end(ttc_rot);

    tt->ping_start(ttc_trans);
    ht.trans(1, 2, 3);
    tt->ping_end(ttc_trans);

    tt->ping_start(ttc_rot);
    ht.rotate_z(M_PI_2);
    tt->ping_end(ttc_rot);

    tt->ping_start(ttc_inv);
    ht.invert();
    tt->ping_end(ttc_inv);

    tt->ping_start(ttc_inv);
    ht.invert();
    tt->ping_end(ttc_inv);
  }

  ht.print_info("HomTransform");
  HomPoint p0 = HomPoint(0.1f, 0.2f, 0.3f);
  cout << "0:  " << p0 << endl << endl << endl;

  HomPoint p = ht * p0;
  cout << "p:  " << p << endl << endl << endl;

  ht.invert().print_info("HomTransform inverted");
  p0 = ht * p;
  cout << "0': " << p0 << endl << endl << endl;

  ht.invert().print_info("HomTransform");
  tt->print_to_stdout();
}


/// @endcond
