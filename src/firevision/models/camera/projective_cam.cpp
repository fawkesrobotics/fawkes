/***************************************************************************
 *  projective_cam.cpp - Abstract class defining a projective camera model
 *
 *  Generated: Thu May 8 15:08 2008
 *  Copyright  2008  Christof Rath <c.rath@student.tugraz.at>
 *
 *  $Id$
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
#include <cmath>
#include <iostream>

using namespace fawkes;
using std::cout; 
using std::endl;



/** @class AboveHorizonException projective_cam.h <models/camera/projective_cam.h>
 * The point that should be calculated lies above the horizon
 * @ingroup Exceptions
 */
/** Constructor
 * @param msg message, appended to exception, base message "PostureException"
 * @param img_pt the point in the image
 */
AboveHorizonException::AboveHorizonException(const char *msg, fawkes::point_t img_pt) throw()
  : fawkes::Exception("AboveHorizonException: %s", msg)
{
  __img_pt = img_pt;
}

/**
 * @return the point in the image that caused the exception
 */
fawkes::point_t
AboveHorizonException::get_img_pt() const
{
  return __img_pt;
}





/** @class ProjectiveCam projective_cam.h <models/camera/projective_cam.h>
 * Abstract class for projective cameras
 * @author Christof Rath
 */

/** Constructor.
 * @param cal Calibration matrix of the camera
 * @param loc Location of the camera (= translation + rotation)
 */
ProjectiveCam::ProjectiveCam(const Calibration& cal, const HomTransform& loc)
{
  cal_ = new Calibration(cal);
  p_ = 0;
  set_location(loc);
}

/** Constructor.
 * @param cal Calibration matrix of the camera
 * @param roll of the camera
 * @param pitch of the camera
 * @param height of the camera
 * @param yaw of the camera
 * @param x of the camera (in front if yaw is zero)
 * @param y of the camera (left if yaw is zero)
 */
ProjectiveCam::ProjectiveCam(const Calibration& cal, float roll, float pitch, float height, float yaw, float x, float y)
{
  cal_ = new Calibration(cal);
  p_ = 0;
  set_location(roll, pitch, height, yaw, x, y);
}

/** Copy Constructor 
 * @param pc the ProjectiveCam to copy
 */
ProjectiveCam::ProjectiveCam(const ProjectiveCam& pc)
{
  //	cal_ = new Calibration(pc.cal_);
  //	p_ = new Matrix (pc.p_);
}

/** Destructor.
 */
ProjectiveCam::~ProjectiveCam()
{
  delete cal_;
  delete p_;
}


/** Obtain inverse matrix.
 * @return the invers matrix
 HomTransform
 ProjectiveCam::get_inverse() const
 {
   HomTransform t(m_matrix->get_inverse());
   
   return t;
   }
   */ 

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
ProjectiveCam::set_location(float roll, float pitch, float height, float yaw, float x, float y)
{
  HomTransform t;

//  cout << "roll: " << roll << " pitch: " << pitch << " height: " << height << " yaw: " << yaw << endl;

  //Transformation of world frame into cam frame [rot_x(-pi/2)+rot_z(-pi/2)]:
  t.rotate_x(-M_PI_2);
  t.rotate_z(-M_PI_2 - yaw);

  t.rotate_x(roll);
  t.rotate_y(pitch);
  t.trans(-x, y, height);

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
  if (p_) delete p_;

  p_ = new Matrix (*cal_ * loc.get_matrix().get_submatrix(0, 0, 3, 4));

  return *this;
}

/** Returns a point in the world under the ground plane assumption.
 * @param img_p a point in the image (x-px, y-px)
 * @return a point in the world (x-meters, y-meters)
 */
fawkes::cart_coord_2d_t 
ProjectiveCam::get_GPA_world_coord(fawkes::point_t img_p) const
{
  Matrix p = get_GPA_p().invert();
  
  Vector img_v(3);
  img_v.x(img_p.x);
  img_v.y(img_p.y);
  img_v.z(1);
  
  Vector wld_v = p * img_v;
  
  wld_v /= wld_v.z();

  if (wld_v.x() < 0) throw AboveHorizonException("The given point is above the horizon!\n", img_p);

  fawkes::cart_coord_2d_t res;
  res.x = wld_v.x();
  res.y = -wld_v.y();
  
  return res;
}

/** Returns an image point of a world point under the ground plane assumption.
 * @param wld_p a point on the ground (x-meters, y-meters)
 * @return a point in the image (x-px, y-px)
 */
fawkes::point_t
ProjectiveCam::get_GPA_image_coord(const fawkes::cart_coord_2d_t wld_p) const
{
  Vector wld_v(4);
  wld_v.x(wld_p.x);
  wld_v.y(wld_p.y);
  wld_v.z(0); //GPA
  wld_v.set(3, 1);
  
  Vector img_v = *p_ * wld_v;
  img_v /= img_v.z();
  
  point_t res;
  res.x = static_cast<unsigned int>(roundf(img_v.x()));
  res.y = static_cast<unsigned int>(roundf(img_v.y()));
  
  return res;
}

/**
 * Calibration getter.
 * @return the calibration matrix
 */
Calibration
ProjectiveCam::get_cal() const
{
  return Calibration(*cal_);
}

/**
 * P matrix getter.
 * @return the P matrix
 */
Matrix
ProjectiveCam::get_p() const
{
  return Matrix(*p_);
}

/** Returns the modified P matrix.
 * With the ground plane assumption the third column can be ignored.
 */
Matrix
ProjectiveCam::get_GPA_p() const
{
  Matrix res(3, 3);
  res.overlay(0, 0, p_->get_submatrix(0, 0, 3, 2)); //first two columns
  res.overlay(0, 2, p_->get_submatrix(0, 3, 3, 1)); //fourth column
  
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
  p_->print_info(name ? name : "Projective Camera", col_sep, row_sep);
  cal_->print_info("Calibration Matrix", col_sep, row_sep);
}

