
/***************************************************************************
 *  qa_matrix.cpp - DESC
 *
 *  Created:  Fri Feb 17 14:31:48 2009
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

#include <geometry/vector.h>
#include <utils/time/tracker.h>
#include <core/exceptions/software.h>

#include <iostream>
#include <cmath>

using namespace fawkes;
using namespace std;

int
main(int argc, char **argv)
{
//  TimeTracker *tt = new TimeTracker();
//  unsigned int loop_count = 0;
//  unsigned int ttc_trans = tt->add_class("Tra");
//  unsigned int ttc_rot = tt->add_class("Rot");
//  unsigned int ttc_inv = tt->add_class("Inv");

  Vector v1;
  v1.x(1);
  v1.y(2);
  v1.z(3);

  cout << "v1: " << v1 << endl;
  Vector v2 = v1 / 10;
  cout << "v2 = v1 / 10: " << v2 << endl;
  v1 /= 10;
  cout << "v1 /= 10: " << v1 << endl << endl << endl;

  Vector v4;
  v4.x(1);
  v4.y(2);
  v4.z(3);

  Vector v5;
  v5.x(4);
  v5.y(5);
  v5.z(6);

  Vector v6(4);
  v6.x(7);
  v6.y(8);
  v6.z(9);

  cout << "v4: " << v4 << " v5: " << v5 << endl;
  Vector v7 = v4 + v5;
  cout << "v7 = v4 + v5: " << v7 << endl;
  v4 += v5;
  cout << "v4 += v5: " << v4 << endl << endl;

  try {
    Vector v8 = v4 + v6;
  }
  catch (fawkes::TypeMismatchException &e) {
    cout << e.what() << endl << endl << endl;
  }

  try {
    v4 += v6;
  }
  catch (fawkes::TypeMismatchException &e) {
    cout << e.what() << endl << endl << endl;
  }
}


/// @endcond
