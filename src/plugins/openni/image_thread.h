
/***************************************************************************
 *  image_thread.h - OpenNI image provider thread
 *
 *  Created: Thu Mar 17 13:58:25 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_OPENNI_IMAGE_THREAD_H_
#define _PLUGINS_OPENNI_IMAGE_THREAD_H_

#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <plugins/openni/aspect/openni.h>

// OpenNI relies on GNU extension to detect Linux
#if defined(__linux__) && not defined(linux)
#	define linux true
#endif
#if defined(__i386__) && not defined(i386)
#	define i386 true
#endif
#include <XnCppWrapper.h>

namespace firevision {
class SharedMemoryImageBuffer;
}

class OpenNiImageThread : public fawkes::Thread,
                          public fawkes::BlockedTimingAspect,
                          public fawkes::LoggingAspect,
                          public fawkes::ConfigurableAspect,
                          public fawkes::ClockAspect,
                          public fawkes::OpenNiAspect
{
public:
	OpenNiImageThread();
	virtual ~OpenNiImageThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	xn::ImageGenerator *image_gen_;
	xn::ImageMetaData * image_md_;

	firevision::SharedMemoryImageBuffer *image_buf_yuv_;
	firevision::SharedMemoryImageBuffer *image_buf_rgb_;

	typedef enum { DEBAYER_BILINEAR, DEBAYER_NEAREST_NEIGHBOR, CONVERT_YUV, CONVERT_RGB } CopyMode;
	CopyMode cfg_copy_mode_;

	unsigned short int usb_vendor_;
	unsigned short int usb_product_;
	unsigned int       image_width_;
	unsigned int       image_height_;

	fawkes::Time *capture_start_;
};

#endif
