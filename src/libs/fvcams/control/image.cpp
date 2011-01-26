
/***************************************************************************
 *  image.cpp - Abstract class defining a camera image controller
 *
 *  Created: Wed Apr 22 11:32:56 CEST 2009
 *  Copyright  2009      Tobias Kellner
 *             2005-2009 Tim Niemueller [www.niemueller.de]
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

#include <fvcams/control/image.h>
#include <core/exceptions/software.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CameraControlImage <fvcams/control/image.h>
 * Camera image control interface.
 * Some cameras feature adjustable image controls
 * like size, format or mirroring.
 *
 * This interface shall be implemented by such cameras.
 *
 * @author Tobias Kellner
 * @author Tim Niemueller
 *
 * @fn unsigned int CameraControlImage::width() = 0
 * Get the current width of the image.
 * @return width in pixels
 *
 * @fn unsigned int CameraControlImage::height() = 0
 * Get the current height of the image.
 * @return height in pixels
 *
 * @fn void CameraControlImage::set_size(unsigned int width, unsigned int height) = 0
 * Set the image size the camera should use.
 * @param width new width of the image
 * @param height new height of the image
 * @exception Exception thrown for instance if size setting at run-time is not supported
 */

using fawkes::NotImplementedException;

/** Empty virtual destructor. */
CameraControlImage::~CameraControlImage()
{
}


/** Get the image format the camera currently uses.
 * Check implementation documentation for details on the format.
 * @return a string describing the image format
 * @throws NotImplementedException Not implemented by this control
 */
const char *
CameraControlImage::format()
{
  throw NotImplementedException("Not implemented");
}


/** Set the image format the camera should use.
 * Check implementation documentation for details on the format.
 * @param format the new image format
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlImage::set_format(const char *format)
{
  throw NotImplementedException("Not implemented");
}


/** Get the current image size.
 * @param[out] width upon return contains the width of the image
 * @param[out] height upon return contains the height of the image
 */
void
CameraControlImage::size(unsigned int &width, unsigned int &height)
{
  width = this->width();
  height = this->height();
}

/** Return whether the camera image is horizontally mirrored.
 * @return true if the image is horizontally mirrored
 * @throws NotImplementedException Not implemented by this control
 */
bool
CameraControlImage::horiz_mirror()
{
  throw NotImplementedException("Not implemented");
}


/** Return whether the camera image is vertically mirrored.
 * @return true if the image is vertically mirrored
 * @throws NotImplementedException Not implemented by this control
 */
bool
CameraControlImage::vert_mirror()
{
  throw NotImplementedException("Not implemented");
}


/** Get information about current camera image mirroring.
 * @param[out] horiz upon return contains flag if horizontal mirroring is enabled
 * @param[out] vert upon return contains flag if vertical mirroring is enabled
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlImage::mirror(bool &horiz, bool &vert)
{
  horiz = horiz_mirror();
  vert = vert_mirror();
}


/** Set whether the camera should mirror images horizontally.
 * @param enabled if true, images should be mirrored horizontally
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlImage::set_horiz_mirror(bool enabled)
{
  throw NotImplementedException("Not implemented");
}


/** Set whether the camera should mirror images vertically.
 * @param enabled if true, images should be mirrored vertically
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlImage::set_vert_mirror(bool enabled)
{
  throw NotImplementedException("Not implemented");
}


/** Set whether the camera should mirror images.
 * @param horiz true to mirror images horizontally, false to disable mirroring
 * @param vert true to mirror images vertically, false to disable mirroring
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlImage::set_mirror(bool horiz, bool vert)
{
  set_horiz_mirror(horiz);
  set_vert_mirror(vert);
}


/** Get the number of frames per second the camera tries to deliver.
 * @return the current fps
 * @throws NotImplementedException Not implemented by this control
 */
unsigned int
CameraControlImage::fps()
{
  throw NotImplementedException("Not implemented");
}


/** Set the number of frames per second the camera tries to deliver.
 * @param fps the new fps
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlImage::set_fps(unsigned int fps)
{
  throw NotImplementedException("Not implemented");
}


/** Get current lens x correction
 * @return current lens x correction
 * @throws NotImplementedException Not implemented by this control
 */
unsigned int
CameraControlImage::lens_x_corr()
{
  throw NotImplementedException("Not implemented");
}


/** Get current lens y correction
 * @return current lens y correction
 * @throws NotImplementedException Not implemented by this control
 */
unsigned int
CameraControlImage::lens_y_corr()
{
  throw NotImplementedException("Not implemented");
}


/** Get current lens correction
 * @param[out] x_corr where the current lens x correction will be stored
 * @param[out] y_corr where the current lens y correction will be stored
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlImage::lens_corr(unsigned int &x_corr, unsigned int &y_corr)
{
  x_corr = this->lens_x_corr();
  y_corr = this->lens_y_corr();
}


/** Set lens x correction
 * @param x_corr new lens x correction
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlImage::set_lens_x_corr(unsigned int x_corr)
{
  throw NotImplementedException("Not implemented");
}


/** Set lens y correction
 * @param y_corr new lens y correction
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlImage::set_lens_y_corr(unsigned int y_corr)
{
  throw NotImplementedException("Not implemented");
}


/** Set lens correction
 * @param x_corr new lens x correction
 * @param y_corr new lens y correction
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlImage::set_lens_corr(unsigned int x_corr, unsigned int y_corr)
{
  set_lens_x_corr(x_corr);
  set_lens_y_corr(y_corr);
}

} // end namespace firevision
