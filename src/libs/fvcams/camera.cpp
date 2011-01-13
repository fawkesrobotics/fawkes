
/***************************************************************************
 *  camera.cpp - Abstract class defining a camera
 *
 *  Created: Wed Jan 17 14:48:17 2007
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
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

#include <fvcams/camera.h>

#include <core/exception.h>
#include <core/exceptions/software.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Camera <fvcams/camera.h>
 * Camera interface for image aquiring devices in FireVision.
 *
 * In general cameras shall initiate a continuous flow of images and shall
 * be optimized for real-time image processing (25 Hz).
 *
 * @fn void Camera::open() = 0
 * Open the camera.
 * The camera is opened, but image transfer not yet started. This can be used
 * to detect general problems with the camera while delaying the real transfer
 * startup until it is needed.
 *
 * @fn void Camera::start()
 * Start image transfer from the camera.
 * For many cameras opening the camera and starting transmission of images are
 * two tasks. This method will simply initiate the transfer after the camera
 * as been opened. And exception shall be thrown if the camera has not been
 * opened.
 *
 * @fn void Camera::stop()
 * Stop image transfer from the camera.
 * This will stop the image transfer initiated with start(). This can be used
 * to start and stop the image transfer at will without opening and closing
 * operations inbetween.
 *
 * @fn void Camera::close()
 * Close camera.
 * This closes the camera device. The camera must have been stopped before
 * calling close().
 *
 * @fn void Camera::capture()
 * Capture an image.
 * Although cameras shall operate with a continuous image flow where possible
 * sometimes capturing an image means copying a buffer or advancing a buffer
 * list pointer. This shall be done in this method. For a camera-using
 * application it is mandatory to call capture() just before accessing the
 * image buffer.
 *
 * @fn void Camera::flush()
 * Flush image queue.
 * Some cameras may have an image buffer queue. With this it can happen that
 * if the processing of an image took longer than desired it is needed to flush
 * this buffer queue.
 *
 * @fn bool Camera::ready()
 * Camera is ready for taking pictures.
 * The camera has been opened and started correctly and may now provide images.
 * @return true, if the camera is ready, false otherwise
 *
 * @fn void Camera::print_info()
 * Print out camera information.
 * Shall print out camera information and current setup information on stdout.
 *
 * @fn unsigned char * Camera::buffer()
 * Get access to current image buffer.
 * This will return a pointer to the current buffer. The buffer contains an
 * image of the given colorspace, width and height.
 * @return pointer to image buffer
 *
 * @fn void Camera::dispose_buffer()
 * Dispose current buffer.
 * Some cameras need disposal of the current buffer (for example to free space
 * in a queue to retrieve the next image). This is done with this method. It
 * has to be called after all work has been done on the image as desired. After
 * dispose_buffer() has been called no further access may happen to the image
 * buffer or undesired behavior may happen.
 *
 * @fn unsigned int Camera::buffer_size()
 * Size of buffer.
 * Gets the size in bytes of the buffer returned by buffer().
 * @return size of buffer in bytes
 *
 * @fn colorspace_t Camera::colorspace()
 * Colorspace of returned image.
 * @return colorspace of image returned by buffer()
 *
 * @fn unsigned int Camera::pixel_width()
 * Width of image in pixels.
 * @return width of image in pixels
 *
 * @fn unsigned int Camera::pixel_height()
 * Height of image in pixels.
 * @return height of image in pixels
 *
 * @fn void Camera::set_image_number(unsigned int n)
 * Set image number to retrieve.
 * If a camera is able to retrieve several images this method can be used to
 * select the image to be retrieved with the next call to capture().
 * @param n image number to set
 *
 */

/** Virtual empty destructor. */
Camera::~Camera()
{
}

/** Get the Time of the last successfully captured image.
 * Returns a Time representing the time when the last image was captured
 * successfully. Note that calling this function is only valid after capture()
 * and before dispose_buffer() has been called -- it is only valid when an image
 * is currently available.
 * @return Time of the currently processed image. The pointer shall be valid at
 * least until the next call to dispose_buffer().
 * @throw NotImplementedException thrown if Camera does not support time stamping
 */
fawkes::Time *
Camera::capture_time()
{
  throw fawkes::NotImplementedException("Timestamping not supported by this camera");
}

} // end namespace firevision
