
/***************************************************************************
 *  stereo_processor.cpp - Stereo processor interface
 *
 *  Created: Fri May 18 16:02:08 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <fvstereo/stereo_processor.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class StereoProcessor <fvstereo/stereo_processor.h>
 * Stereo processor interface.
 * This interface provides access to different stereo processing
 * implementations.
 *
 * @author Tim Niemueller
 *
 * @fn virtual void StereoProcessor::get_xyz(unsigned int px, unsigned int py, float *x, float *y, float *z) = 0
 * Get coordinates for pixel in camera coordinate system.
 * This retrieves coordinates in the coordinate system of the stereo camera.
 * Retrieving new positions may fail if no valid disparity information could
 * be calculated for the given point. This is indicated with the return value.
 * @param px x position of pixel in image
 * @param py y position of pixel in image
 * @param x upon successful return contains the x coordinate of the point in the camera coordinate sytem
 * @param y upon successful return contains the y coordinate of the point in the camera coordinate sytem
 * @param z upon successful return contains the z coordinate of the point in the camera coordinate sytem
 * @return true, if valid information could be retrieved and was written to (x,y,z), false otherwise
 *
 * @fn virtual void StereoProcessor::get_world_xyz(unsigned int px, unsigned int py, float *x, float *y, float *z) = 0
 * Get coordinates for pixel in robot coordinate system.
 * This retrieves coordinates in the coordinate system of the robot holding
 * the stereo camera. The robot coordinate system is a right-handed cardanic
 * coordinate system with the X axis pointing forward, the Y axis pointing
 * right and the Z axis pointing downwards.
 * Retrieving new positions may fail if no valid disparity information could
 * be calculated for the given point. This is indicated with the return value.
 * @param px x position of pixel in image
 * @param py y position of pixel in image
 * @param x upon successful return contains the x coordinate of the point in the robot coordinate sytem
 * @param y upon successful return contains the y coordinate of the point in the robot coordinate sytem
 * @param z upon successful return contains the z coordinate of the point in the robot coordinate sytem
 * @return true, if valid information could be retrieved and was written to (x,y,z), false otherwise
 *
 * @fn virtual void StereoProcessor::preprocess_stereo() = 0
 * Do any pre-processing needed.
 * Do all the preprocessing needed to calculate the disparity or the YUV images.
 * This has been split out to be able to do only one thing.
 *
 * @fn virtual void StereoProcessor::calculate_disparity(ROI *roi = 0) = 0
 * Caculate disparity images.
 * Depending on the data the specific stereo processor needs the disparity image
 * is calculated.
 * If a region of interest (ROI) is supplied then only this region is processed.
 * @param roi region of interest to process
 *
 * @fn virtual void StereoProcessor::calculate_yuv(bool both = false) = 0
 * Caculate yuv images.
 * This will calculate YUV images of the cameras. By default only the reference
 * image is converted to YUV, as this is sufficient in most cases. If you need both
 * images specify so as argument. A call to this method is valid only after calling
 * calculate_disparity() as this may rely on data produced there.
 * @param both if true, both YUV images are calculated for the left and right camera,
 * if false only the YUV image of the reference camera is calculated (dependant on
 * camera)
 *
 * @fn virtual unsigned char *  StereoProcessor::disparity_buffer() = 0
 * Get the disparity image buffer.
 * This returns a buffer containing the disparity image. The buffer is not copied
 * so do not change anything in the buffer or subsequent calls to get_xyz() and
 * get_world_xyz() will return invalid results.
 * @return disparity buffer
 *
 * @fn virtual size_t           StereoProcessor::disparity_buffer_size() const = 0
 * Get disparity buffer size.
 * @return size in bytes of the disparity image buffer
 *
 * @fn virtual unsigned char *  StereoProcessor::yuv_buffer_right() = 0
 * Get YUV-formatted buffer of reference camera.
 * This will return the YUV buffer of the reference image. This is only available
 * after calling calculate_yuv().
 * @return YUV buffer of the reference image
 * 
 * @fn virtual unsigned char *  StereoProcessor::yuv_buffer_left() = 0
 * Get YUV-formatted buffer of left camera.
 * This will return the YUV buffer of the auxiliary image. This is only available
 * after calling calculate_yuv().
 * @return YUV buffer of the auxiliary image
 *
 */

/** Virtual empty destructor. */
StereoProcessor::~StereoProcessor()
{
}

} // end namespace firevision
