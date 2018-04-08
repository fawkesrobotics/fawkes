
/***************************************************************************
 *  jpeg_stream_producer.cpp - Image JPEG stream producer
 *
 *  Created: Thu Feb 06 13:04:53 2014
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

#include "jpeg_stream_producer.h"

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/wait_condition.h>

#include <fvcams/shmem.h>
#include <fvutils/compression/jpeg_compressor.h>
#include <fvutils/color/conversions.h>
#include <utils/time/wait.h>

#include <cstdlib>

using namespace firevision;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebviewJpegStreamProducer::Buffer "jpeg_stream_producer.h"
 * Image buffer passed to stream subscribers.
 */

/** Constructor.
 * @param data data buffer
 * @param size size in bytes of @p data
 */
WebviewJpegStreamProducer::Buffer::Buffer(unsigned char *data, size_t size)
  : data_(data), size_(size)
{
}

/** Destructor. */
WebviewJpegStreamProducer::Buffer::~Buffer()
{
  free(data_);
}


/** @class WebviewJpegStreamProducer::Subscriber "jpeg_stream_producer.h"
 * JPEG stream subscriber.
 *
 * @fn void WebviewJpegStreamProducer::Subscriber::handle_buffer(std::shared_ptr<Buffer> buffer) throw() = 0
 * Notification if a new buffer is available.
 * @param buffer new buffer
 */

/** Destructor. */
WebviewJpegStreamProducer::Subscriber::~Subscriber()
{
}

/** @class WebviewJpegStreamProducer "jpeg_stream_producer.h"
 * JPEG stream producer.
 * This class takes an image ID and some parameters and then creates a stream
 * of JPEG buffers that is either passed to subscribers or can be queried
 * using the wait_for_next_frame() method.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param image_id ID of the shared memory image buffer to get the input image from
 * @param quality JPEG quality value, depends on used compressor (system default)
 * @param fps frames per second to achieve
 * @param vflip true to enable vertical flipping, false to disable
 */
WebviewJpegStreamProducer::WebviewJpegStreamProducer(const std::string & image_id,
						     unsigned int quality, float fps, bool vflip)
  : Thread("WebviewJpegStreamProducer", Thread::OPMODE_WAITFORWAKEUP)
{
  set_coalesce_wakeups(true);
  set_prepfin_conc_loop(true);
  set_name("WebviewJpegStreamProducer[%s]", image_id.c_str());

  last_buf_mutex_ = new Mutex();
  last_buf_waitcond_ = new WaitCondition(last_buf_mutex_);

  quality_  = quality;
  image_id_ = image_id;
  fps_      = fps;
  vflip_    = vflip;
}

/** Destructor. */
WebviewJpegStreamProducer::~WebviewJpegStreamProducer()
{
  delete last_buf_mutex_;
  delete last_buf_waitcond_;
}

/** Add a subscriber.
 * @param subscriber subscriber to add, must be valid until removed or as long
 * as this instance is valid.
 */
void
WebviewJpegStreamProducer::add_subscriber(Subscriber *subscriber)
{
  subs_.lock();
  subs_.push_back(subscriber);
  subs_.sort();
  subs_.unique();
  subs_.unlock();
  wakeup();
}

/** Remove a subscriber.
 * @param subscriber subscriber to remove
 */
void
WebviewJpegStreamProducer::remove_subscriber(Subscriber *subscriber)
{
  subs_.lock();
  subs_.remove(subscriber);
  subs_.unlock();
}


/** Blocks caller until new thread is available.
 * @return newest available buffer once it becomes available
 */
std::shared_ptr<WebviewJpegStreamProducer::Buffer>
WebviewJpegStreamProducer::wait_for_next_frame()
{
  MutexLocker lock(last_buf_mutex_);
  wakeup();
  while (! last_buf_) {
    last_buf_waitcond_->wait();
  }
  return last_buf_;
}

void
WebviewJpegStreamProducer::init()
{
  cam_  = new SharedMemoryCamera(image_id_.c_str(), /* deep copy */ false);
  jpeg_ = new JpegImageCompressor(quality_);
  jpeg_->set_image_dimensions(cam_->pixel_width(), cam_->pixel_height());
  jpeg_->set_compression_destination(ImageCompressor::COMP_DEST_MEM);
  if (jpeg_->supports_vflip())  jpeg_->set_vflip(vflip_);

  in_buffer_ = malloc_buffer(YUV422_PLANAR,
			     cam_->pixel_width(), cam_->pixel_height());
  jpeg_->set_image_buffer(YUV422_PLANAR, in_buffer_);

  long int loop_time = (long int)roundf((1. / fps_) * 1000000.);
  timewait_ = new TimeWait(clock, loop_time);
}

void
WebviewJpegStreamProducer::loop()
{
  last_buf_mutex_->lock();
  last_buf_.reset();
  last_buf_mutex_->unlock();

  timewait_->mark_start();

  size_t size = jpeg_->recommended_compressed_buffer_size();
  unsigned char *buffer = (unsigned char *)malloc(size);
  jpeg_->set_destination_buffer(buffer, size);

  cam_->lock_for_read();
  cam_->capture();
  firevision::convert(cam_->colorspace(), YUV422_PLANAR,
		      cam_->buffer(), in_buffer_,
		      cam_->pixel_width(), cam_->pixel_height());
  jpeg_->compress();
  cam_->dispose_buffer();
  cam_->unlock();

  std::shared_ptr<Buffer> shared_buf =
	  std::make_shared<Buffer>(buffer, jpeg_->compressed_size());
  subs_.lock();
#if (__GNUC__ * 10000 + __GNUC_MINOR__ * 100) > 40600
  for (auto &s : subs_) {
#else
    fawkes::LockList<Subscriber *>::iterator si;
  for (si = subs_.begin(); si != subs_.end(); ++si) {
    Subscriber *s = *si;
#endif
    s->handle_buffer(shared_buf);
  }
  bool go_on = ! subs_.empty();
  subs_.unlock();

  last_buf_mutex_->lock();
  last_buf_ = shared_buf;
  last_buf_waitcond_->wake_all();
  last_buf_mutex_->unlock();

  if (go_on) {
    timewait_->wait_systime();
    wakeup();
  }
}

void
WebviewJpegStreamProducer::finalize()
{
  delete jpeg_;
  delete cam_;
  delete timewait_;
  free(in_buffer_);
}

} // end namespace fawkes

