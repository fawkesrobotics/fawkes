
/***************************************************************************
 *  mjpeg_reply.cpp - Web request MJPEG stream reply
 *
 *  Created: Wed Feb 05 17:55:41 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include "mjpeg_reply.h"

#include <core/exceptions/system.h>
#include <fvutils/compression/jpeg_compressor.h>
#include <fvcams/shmem.h>

#include <cerrno>
#include <unistd.h>
#include <cstring>

using namespace firevision;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class DynamicMJPEGStreamWebReply <webview/file_reply.h>
 * Dynamic raw file transfer reply.
 * This dynamic file transfer reply transmits the given file with a mime type
 * determined with libmagic.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cam camera to get the image buffer from
 */
DynamicMJPEGStreamWebReply::DynamicMJPEGStreamWebReply(firevision::SharedMemoryCamera *cam)
  : DynamicWebReply(WebReply::HTTP_OK), next_frame_(true)
{
  add_header("Content-type", "multipart/x-mixed-replace;boundary=MJPEG-next-frame");
  cam_ = cam;
  jpeg_ = new JpegImageCompressor(10);
  jpeg_->set_image_dimensions(cam_->pixel_width(), cam_->pixel_height());
  jpeg_->set_compression_destination(ImageCompressor::COMP_DEST_MEM);
  size_t size = jpeg_->recommended_compressed_buffer_size();
  buffer_ = (unsigned char *)malloc(size);
  jpeg_->set_destination_buffer(buffer_, size);
}

/** Destructor. */
DynamicMJPEGStreamWebReply::~DynamicMJPEGStreamWebReply()
{
  delete jpeg_;
  free(buffer_);
}

size_t
DynamicMJPEGStreamWebReply::size()
{
  return -1;
}

size_t
DynamicMJPEGStreamWebReply::next_chunk(size_t pos, char *buffer, size_t buf_max_size)
{
  if (buf_max_size == 0)  return 0;

  size_t written = 0;

  if (next_frame_) {
    usleep(1000000);
    cam_->capture();
    jpeg_->set_image_buffer(cam_->colorspace(), cam_->buffer());
    jpeg_->compress();
    cam_->dispose_buffer();
    buffer_length_ = jpeg_->compressed_size();
    buffer_ongoing_ = buffer_;

    char *header;
    if (asprintf(&header,
		 "--MJPEG-next-frame\r\n"
		 "Content-type: image/jpeg\r\n"
		 "Content-length: %zu\r\n"
		 "\r\n", buffer_length_) == -1) {
      return -2;
    }
    size_t header_len = strlen(header);
    memcpy(buffer, header, header_len);
    buffer += header_len;
    buf_max_size -= header_len;
    written += header_len;

    next_frame_ = false;
  }

  size_t remaining = buffer_ + buffer_length_ - buffer_ongoing_;
  if (remaining <= buf_max_size) {
    memcpy(buffer, buffer_ongoing_, remaining);
    next_frame_ = true;
    written += remaining;
  } else {
    memcpy(buffer, buffer_ongoing_, buf_max_size);
    buffer_ongoing_ += buf_max_size;
    written += buf_max_size;
  }

  return written;
}

} // end namespace fawkes
