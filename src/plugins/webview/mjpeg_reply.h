
/***************************************************************************
 *  mjpeg_reply.h - Web request MJPEG stream reply
 *
 *  Created: Wed Feb 05 17:54:06 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WEBVIEW_MJPEG_REPLY_H_
#define __PLUGINS_WEBVIEW_MJPEG_REPLY_H_

#include <webview/reply.h>
#include <cstdio>

namespace firevision {
  class SharedMemoryCamera;
  class JpegImageCompressor;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class DynamicMJPEGStreamWebReply : public DynamicWebReply
{
 public:
  DynamicMJPEGStreamWebReply(firevision::SharedMemoryCamera *cam);
  virtual ~DynamicMJPEGStreamWebReply();

  virtual size_t size();
  virtual size_t next_chunk(size_t pos, char *buffer, size_t buf_max_size);

 private:
  firevision::SharedMemoryCamera  *cam_;
  firevision::JpegImageCompressor *jpeg_;
  bool next_frame_;
  unsigned char *buffer_;
  size_t         buffer_length_;
  unsigned char *buffer_ongoing_;
};

} // end namespace fawkes

#endif
