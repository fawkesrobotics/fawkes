
/***************************************************************************
 *  projective_cam.h - Abstract class defining a projective camera model
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

#ifndef __FIREVISION_PROJECTIVE_CAM_H__
#define __FIREVISION_PROJECTIVE_CAM_H__

#include <fvmodels/camera/calibration.h>
#include <geometry/hom_transform.h>
#include <fvutils/base/types.h>

#include <core/exception.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class AboveHorizonException : public fawkes::Exception
{
 public:
  AboveHorizonException(const char *msg, const center_in_roi_t img_pt) throw();

  const center_in_roi_t& get_img_pt() const;

 private:
  center_in_roi_t __img_pt;
};

class ProjectiveCam
{
 public:
  ProjectiveCam(const ProjectiveCam& pc);
  virtual ~ProjectiveCam();

  //virtual HomTransform get_inverse() const;

  virtual ProjectiveCam& set_location(const fawkes::HomTransform& loc);
  virtual ProjectiveCam& set_location(float roll, float pitch, float yaw, float height, float x = 0, float y = 0);

  virtual fawkes::cart_coord_2d_t get_GPA_world_coord(const center_in_roi_t &img_p) const;
  virtual center_in_roi_t         get_GPA_image_coord(const fawkes::cart_coord_2d_t &wld_p) const;

  virtual void print_info (const char* name = 0, const char *col_sep = 0, const char *row_sep = 0) const;

 protected:
  ProjectiveCam(const Calibration &cal, const fawkes::HomTransform *loc = 0);
  ProjectiveCam(const Calibration &cal, float roll, float pitch, float yaw, float height, float x = 0, float y = 0);
  Calibration get_cal() const;

  fawkes::Matrix get_p() const;
  fawkes::Matrix get_GPA_p() const;

 private:
  Calibration    __cal;
  fawkes::Matrix *__p;
  fawkes::Matrix *__gpa_inv;
  float          *__gpa_inv_data;
};

} // end namespace firevision

#endif // __FIREVISION_PROJECTIVE_CAM_H__
