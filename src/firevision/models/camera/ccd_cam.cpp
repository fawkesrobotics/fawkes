/***************************************************************************
 *  ccd_cam.cpp - Class defining a ccd camera model
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

#include "ccd_cam.h"

using namespace fawkes;

/** @class CCDCam ccd_cam.h <models/camera/ccd_cam.h>
 * A class for a ccd camera model
 * @author Christof Rath
 */

/** Constructor.
 * @param cal Calibration matrix of the camera
 * @param loc Location of the camera (= translation + rotation)
 */
CCDCam::CCDCam(const CCDCalibration& cal, const HomTransform& loc)
: ProjectiveCam (cal, loc)
{
}

/** Constructor.
 * @param ax is the scale factor in the x-coordinate direction
 * @param ay is the scale factor in the y-coordinate direction
 * @param x0 is the x-coordinate of the principal point
 * @param y0 is the y-coordinate of the principal point
 * @param loc Location of the camera (= translation + rotation)
 */
CCDCam::CCDCam(const float ax, const float ay, const float x0, const float y0, const HomTransform& loc)
: ProjectiveCam (CCDCalibration(ax, ay, x0, y0), loc)
{
}

/** Copy constructor
 * @param cp the CCDCam to copy
 */
CCDCam::CCDCam(const CCDCam& cp) : ProjectiveCam(cp)
{
}


/** Destructor.
 */
CCDCam::~CCDCam()
{
}

