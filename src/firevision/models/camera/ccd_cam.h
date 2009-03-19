/***************************************************************************
 *  ccd_cam.h - Class defining a ccd camera model
 *
 *  Generated: Thu May 8 16:08 2008
 *  Copyright  2008  Christof Rath <c.rath@student.tugraz.at>
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

#ifndef __FIREVISION_CCD_CAM_H__
#define __FIREVISION_CCD_CAM_H__

#include "projective_cam.h"
#include "ccd_calibration.h"


class CCDCam: public ProjectiveCam
{
  public:
    CCDCam(const CCDCalibration &cal, const fawkes::HomTransform *loc = 0);
    CCDCam(const float ax, const float ay, const float x0, const float y0, const fawkes::HomTransform *loc = 0);
    CCDCam(const CCDCam& cp);

    virtual ~CCDCam();
};

#endif // __FIREVISION_CCD_CAM_H__
