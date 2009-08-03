/***************************************************************************
 *  calibration.cpp - Abstract class defining a camera calibration matrix K
 *                    for a finite camera
 *
 *  Generated: Thu May 8 13:24 2008
 *  Copyright  2008  Christof Rath <c.rath@student.tugraz.at>
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

#include "calibration.h"
#include <iostream>
#include <core/exceptions/software.h>

using namespace fawkes;

/** @class Calibration <models/camera/calibration.h>
 * A Calibration matrix for a finite camera.
 *
 * @author Christof Rath
 */

/** Hidden default constructor.
 */
Calibration::Calibration(): Matrix(3, 3)
{
  id();
}

/** Constructor.
 * @param k 3x3 Calibration matrix of the camera
 */
Calibration::Calibration(const fawkes::Matrix& k): Matrix(3, 3)
{
  K(k);
}

/** Copy Constructor.
 * @param cal the Calibration to copy
 */
Calibration::Calibration(const Calibration& cal): Matrix(3, 3)
{
  K(cal.K());
}

/** Destructor.
 */
Calibration::~Calibration()
{
}

/** Calibration getter.
 * @return The calibration matrix
 */
Matrix
Calibration::K() const
{
  return get_submatrix(0, 0, 3, 3);
}

/** Sets the calibration matrix.
 * The matrix k has a size 3x3. The elements (row by row):
 * scale factor in x-direction, skew, x-coordinate of the principal point
 * 0, scale factor in y-direction, y-coordinate of the principal point
 * 0, 0, 1
 * @param k the calibration matrix
 */
Calibration&
Calibration::K(const fawkes::Matrix& k)
{
  unsigned int i,j;
  k.size(i, j);

  if (i != j || i != 3)
    throw IllegalArgumentException("The calibration matrix has to be 3 by 3");

  id();
  overlay (0, 0, k);
  return *this;
}

