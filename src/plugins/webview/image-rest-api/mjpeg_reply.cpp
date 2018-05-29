
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
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>
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
 * @param stream_producer stream producer to query for JPEG buffers
 */
DynamicMJPEGStreamWebReply::DynamicMJPEGStreamWebReply(std::shared_ptr<WebviewJpegStreamProducer> stream_producer)
  : DynamicWebReply(WebReply::HTTP_OK)
{
  next_buffer_mutex_ = new fawkes::Mutex();
  next_buffer_waitcond_ = new fawkes::WaitCondition(next_buffer_mutex_);
  next_frame_ = true;

  add_header("Content-type", "multipart/x-mixed-replace;boundary=MJPEG-next-frame");
  stream_producer_ = stream_producer;
  stream_producer_->add_subscriber(this);
}

/** Destructor. */
DynamicMJPEGStreamWebReply::~DynamicMJPEGStreamWebReply()
{
  stream_producer_->remove_subscriber(this);
  delete next_buffer_mutex_;
  delete next_buffer_waitcond_;
}

size_t
DynamicMJPEGStreamWebReply::size()
{
  return -1;
}

void
DynamicMJPEGStreamWebReply::handle_buffer(std::shared_ptr<WebviewJpegStreamProducer::Buffer> buffer)
{
  next_buffer_mutex_->lock();
  next_buffer_ = buffer;
  next_buffer_waitcond_->wake_all();
  next_buffer_mutex_->unlock();
}

size_t
DynamicMJPEGStreamWebReply::next_chunk(size_t pos, char *buffer, size_t buf_max_size)
{
  if (buf_max_size == 0)  return  0;

  size_t written = 0;

  if (next_frame_) {
    next_buffer_mutex_->lock();
    while (! next_buffer_) {
      next_buffer_waitcond_->wait();
    }
    buffer_ = next_buffer_;
    next_buffer_.reset();
    next_buffer_mutex_->unlock();

    char *header;
    if (asprintf(&header,
		 "--MJPEG-next-frame\r\n"
		 "Content-type: image/jpeg\r\n"
		 "Content-length: %zu\r\n"
		 "\r\n", buffer_->size()) == -1) {
      return -2;
    }
    size_t header_len = strlen(header);
    memcpy(buffer, header, header_len);
    buffer += header_len;
    buf_max_size -= header_len;
    written += header_len;

    buffer_bytes_written_ = 0;
    next_frame_ = false;
  }

  size_t remaining = buffer_->size() - buffer_bytes_written_;
  if (remaining <= buf_max_size) {
    memcpy(buffer, buffer_->data() + buffer_bytes_written_, remaining);
    next_frame_ = true;
    written += remaining;
  } else {
    memcpy(buffer, buffer_->data() + buffer_bytes_written_, buf_max_size);
    buffer_bytes_written_ += buf_max_size;
    written += buf_max_size;
  }

  return written;
}

} // end namespace fawkes
