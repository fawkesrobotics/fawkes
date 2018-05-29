
/***************************************************************************
 *  jpeg_stream_producer.h - Camera image stream producer
 *
 *  Created: Thu Feb 06 12:48:34 2014
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

#ifndef __PLUGINS_WEBVIEW_JPEG_STREAM_PRODUCER_H_
#define __PLUGINS_WEBVIEW_JPEG_STREAM_PRODUCER_H_

#include <core/threading/thread.h>
#include <core/utils/lock_list.h>
#include <aspect/clock.h>

#include <string>
#include <memory>

namespace firevision {
  class SharedMemoryCamera;
  class JpegImageCompressor;
}


namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class TimeWait;
class Mutex;
class WaitCondition;

class WebviewJpegStreamProducer
: public fawkes::Thread,
  public fawkes::ClockAspect
{
 public:
  class Buffer {
   public:
    Buffer(unsigned char *data, size_t size);
    ~Buffer();

    /** Get data buffer.
     * @return data buffer */
    const unsigned char *  data() const
    { return data_; }

    /** Get buffer size.
     * @return b uffer size. */
    size_t  size() const
    { return size_; }

  private:
    unsigned char *data_;
    size_t         size_;
  };

  class Subscriber {
   public:
    virtual ~Subscriber();
    virtual void handle_buffer(std::shared_ptr<Buffer> buffer) = 0;
  };

 public:
  WebviewJpegStreamProducer(const std::string & image_id,
			    unsigned int quality, float fps, bool vflip);
  virtual ~WebviewJpegStreamProducer();

  void add_subscriber(Subscriber *subscriber);
  void remove_subscriber(Subscriber *subscriber);
  std::shared_ptr<Buffer> wait_for_next_frame();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  std::string    image_id_;
  unsigned int   quality_;
  float          fps_;
  bool           vflip_;
  unsigned char *in_buffer_;

  TimeWait *timewait_;

  firevision::SharedMemoryCamera  *cam_;
  fawkes::LockList<Subscriber *>   subs_;
  firevision::JpegImageCompressor *jpeg_;

  std::shared_ptr<Buffer>         last_buf_;
  fawkes::Mutex         *last_buf_mutex_;
  fawkes::WaitCondition *last_buf_waitcond_;
};

} // end namespace fawkes

#endif
