/***************************************************************************
 *  ccd_cam.h - Class defining a ccd camera model
 *
 *  Created: Thu May 08 16:08:00 2008
 *  Copyright  2008  Christof Rath <c.rath@student.tugraz.at>
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

#include <models/camera/projective_cam.h>
#include <models/camera/ccd_calibration.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CCDCam: public ProjectiveCam
{
  public:
    CCDCam(const CCDCalibration &cal, const fawkes::HomTransform *loc = 0);
    CCDCam(const float ax, const float ay, const float x0, const float y0, const fawkes::HomTransform *loc = 0);
    CCDCam(const CCDCam& cp);

    virtual ~CCDCam();
};

} // end namespace firevision

#endif // __FIREVISION_CCD_CAM_H__
