/***************************************************************************
 *  projective_cam.cpp - Abstract class defining a projective camera model
 *
 *  Created: Thu May 08 15:08:00 2008
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

#include "projective_cam.h"

#include <geometry/hom_point.h>
#include <geometry/vector.h>
#include <core/exceptions/software.h>

#include <cmath>
#include <iostream>

using namespace fawkes;
using std::cout;
using std::endl;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class AboveHorizonException <fvmodels/camera/projective_cam.h>
 * The point that should be calculated lies above the horizon
 * @ingroup Exceptions
 */
/** Constructor
 * @param msg message, appended to exception, base message "PostureException"
 * @param img_pt the point in the image
 */
AboveHorizonException::AboveHorizonException(const char *msg, const center_in_roi_t img_pt) throw()
  : fawkes::Exception("AboveHorizonException: %s (%0.1f, %0.1f)", msg, img_pt.x, img_pt.y)
{
  __img_pt = img_pt;
}

/**
 * @return the point in the image that caused the exception
 */
const center_in_roi_t &
AboveHorizonException::get_img_pt() const
{
  return __img_pt;
}





/** @class ProjectiveCam projective_cam.h <fvmodels/camera/projective_cam.h>
 * Abstract class for projective cameras
 * @author Christof Rath
 */

/** Constructor.
 * @param cal Calibration matrix of the camera
 * @param loc Location of the camera (= translation + rotation)
 */
ProjectiveCam::ProjectiveCam(const Calibration &cal, const HomTransform *loc) :
  __cal(cal)
{
  __p       = NULL;
  __gpa_inv = NULL;
  __gpa_inv_data = new float[9];

  if (loc) set_location(*loc);
}

/** Constructor.
 * @param cal Calibration matrix of the camera
 * @param roll of the camera
 * @param pitch of the camera
 * @param yaw of the camera
 * @param height of the camera
 * @param x of the camera (in front if yaw is zero)
 * @param y of the camera (left if yaw is zero)
 */
ProjectiveCam::ProjectiveCam(const Calibration &cal,
                             float roll, float pitch, float yaw,
                             float height, float x, float y):
  __cal(cal)
{
  __p            = NULL;
  __gpa_inv      = NULL;
  __gpa_inv_data = new float[9];

  set_location(roll, pitch, yaw, height, x, y);
}

/** Copy Constructor
 * @param pc the ProjectiveCam to copy
 */
ProjectiveCam::ProjectiveCam(const ProjectiveCam &pc):
  __cal(pc.__cal)
{
  __p            = (pc.__p != NULL ? new Matrix(*pc.__p) : NULL);
  __gpa_inv_data = new float[9];

  if (pc.__gpa_inv) {
    for (unsigned int i = 0; i < 9; ++i) {
      __gpa_inv_data[i] = pc.__gpa_inv_data[i];
    }

    __gpa_inv = new Matrix(3, 3, __gpa_inv_data, false);
  }
  else __gpa_inv = NULL;
}

/** Destructor.
 */
ProjectiveCam::~ProjectiveCam()
{
  delete   __p;
  delete   __gpa_inv;
  delete[] __gpa_inv_data;
}


/** Sets a new location for the camera
 * @param roll of the camera
 * @param pitch of the camera
 * @param height of the camera
 * @param yaw of the camera
 * @param x of the camera (in front if yaw is zero)
 * @param y of the camera (left if yaw is zero)
 * @return a reference to the camera
 */
ProjectiveCam&
ProjectiveCam::set_location(float roll, float pitch, float yaw, float height, float x, float y)
{
  HomTransform t;

//  cout << "roll: " << roll << " pitch: " << pitch << " height: " << height << " yaw: " << yaw << endl;

  //Transformation of world frame into cam frame [rot_x(-pi/2)+rot_z(-pi/2)]:
  t.rotate_x(-M_PI_2);
  t.rotate_z(-M_PI_2);

  t.rotate_y(pitch);
  t.rotate_x(-roll);

  t.trans(-x, y, height);
  t.rotate_z(yaw);

  return set_location(t);
}

/** Sets a new location for the camera
 * @param loc the new location (remember the transformation from world frame
 *            into cam frame [rot_x(-pi/2)+rot_z(-pi/2)] before the rest of the
 *            transformation)
 * @return a reference to the camera
 */
ProjectiveCam&
ProjectiveCam::set_location(const HomTransform& loc)
{
  if (__p) {
    delete __gpa_inv;
    delete __p;
    __p = NULL;
  }

  __p = new Matrix (__cal * loc.get_matrix().get_submatrix(0, 0, 3, 4));

  __gpa_inv = new Matrix(3, 3, __gpa_inv_data, false);
  __gpa_inv->overlay(0, 0, __p->get_submatrix(0, 0, 3, 2));
  __gpa_inv->overlay(0, 2, __p->get_submatrix(0, 3, 3, 1));
  __gpa_inv->invert();

  return *this;
}

/** Returns a point in the world under the ground plane assumption.
 * @param img_p a point in the image (x-px, y-px)
 * @return a point in the world (x-meters, y-meters)
 */
fawkes::cart_coord_2d_t
ProjectiveCam::get_GPA_world_coord(const center_in_roi_t &img_p) const
{
  Vector img_v(3);
  img_v.x(img_p.x);
  img_v.y(img_p.y);
  img_v.z(1);

  Vector wld_v = *__gpa_inv * img_v;

  wld_v /= wld_v.z();

  if (wld_v.x() < 0) {
    throw AboveHorizonException("The given point is above the horizon!\n", img_p);
  }

  return (fawkes::cart_coord_2d_t){ wld_v.x(), -wld_v.y() };
}

/** Returns an image point of a world point under the ground plane assumption.
 * @param wld_p a point on the ground (x-meters, y-meters)
 * @return a point in the image (x-px, y-px)
 */
center_in_roi_t
ProjectiveCam::get_GPA_image_coord(const fawkes::cart_coord_2d_t &wld_p) const
{
  Vector wld_v(4);
  wld_v.x(wld_p.x);
  wld_v.y(wld_p.y);
  wld_v.z(0); //GPA
  wld_v.set(3, 1);

  Vector img_v = *__p * wld_v;
  img_v /= img_v.z();

  center_in_roi_t res;
  res.x = img_v.x();
  res.y = img_v.y();

  return res;
}

/**
 * Calibration getter.
 * @return the calibration matrix
 */
Calibration
ProjectiveCam::get_cal() const
{
  return Calibration(__cal);
}

/**
 * P matrix getter.
 * @return the P matrix
 */
Matrix
ProjectiveCam::get_p() const
{
  return Matrix(*__p);
}

/** Returns the modified P matrix.
 * With the ground plane assumption the third column can be ignored.
 * @return modified P matrix
 */
Matrix
ProjectiveCam::get_GPA_p() const
{
  Matrix res(3, 3);
  res.overlay(0, 0, __p->get_submatrix(0, 0, 3, 2)); //first two columns
  res.overlay(0, 2, __p->get_submatrix(0, 3, 3, 1)); //fourth column

  return res;
}

/** Prints the matrix P.
 * @param name Heading of the output
 * @param col_sep a string used to separate columns (defaults to '\\t')
 * @param row_sep a string used to separate rows (defaults to '\\n')
 */
void
ProjectiveCam::print_info (const char *name, const char *col_sep, const char *row_sep) const
{
  __p->print_info(name ? name : "Projective Camera", col_sep, row_sep);
  __cal.print_info("Calibration Matrix", col_sep, row_sep);
}

} // end namespace firevision
