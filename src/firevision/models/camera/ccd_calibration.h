
/***************************************************************************
 *  ccd_calibration.h - Class defining a ccd camera calibration matrix K
 *
 *  Generated: Thu May 8 13:53 2008
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

#ifndef __FIREVISION_CCD_CALIBRATION_H__
#define __FIREVISION_CCD_CALIBRATION_H__

#include "calibration.h"

class CCDCalibration: public Calibration
{
  public:
    CCDCalibration(float ax, float ay, float x0, float y0);
    CCDCalibration(float hor_fov, unsigned int img_width, unsigned int img_height);
    CCDCalibration(const CCDCalibration& cp);
    virtual ~CCDCalibration();
};

#endif // __FIREVISION_CCD_CALIBRATION_H__
