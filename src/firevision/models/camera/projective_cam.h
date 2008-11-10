
/***************************************************************************
 *  projective_cam.h - Abstract class defining a projective camera model
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


#ifndef __FIREVISION_PROJECTIVE_CAM_H__
#define __FIREVISION_PROJECTIVE_CAM_H__

#include <geometry/hom_transform.h>
#include <fvutils/base/types.h>
#include "calibration.h"

#include <core/exception.h>

class AboveHorizonException : public fawkes::Exception {
  public:
    AboveHorizonException(const char *msg) throw();
};

class ProjectiveCam
{
  public:
    ProjectiveCam(const ProjectiveCam& pc);
    virtual ~ProjectiveCam();

    //virtual HomTransform get_inverse() const;

    virtual ProjectiveCam& set_location(const fawkes::HomTransform& loc);
    virtual ProjectiveCam& set_location(float roll, float pitch, float height, float yaw = 0, float x = 0, float y = 0);
    
    virtual fawkes::cart_coord_2d_t get_GPA_world_coord(const fawkes::point_t img_p) const;
    virtual fawkes::point_t         get_GPA_image_coord(const fawkes::cart_coord_2d_t wld_p) const;

    virtual void print_info (const char* name = 0, const char *col_sep = 0, const char *row_sep = 0) const;

  protected:
    ProjectiveCam(const Calibration& cal, const fawkes::HomTransform& loc);
    ProjectiveCam(const Calibration& cal, float roll, float pitch, float height, float yaw = 0, float x = 0, float y = 0);
    Calibration get_cal() const;

    fawkes::Matrix get_p() const;
    fawkes::Matrix get_GPA_p() const;

  private:
    Calibration* cal_;
    fawkes::Matrix* p_;
};

#endif // __FIREVISION_PROJECTIVE_CAM_H__
