
/***************************************************************************
 *  retriever_thread.cpp - FireVision Retriever Thread
 *
 *  Created: Tue Jun 26 17:39:11 2007
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <apps/retriever/retriever_thread.h>

#include <cams/camera.h>
#include <fvutils/ipc/shm_image.h>
#include <utils/time/tracker.h>
#include <fvutils/writers/seq_writer.h>
#include <fvutils/writers/jpeg.h>

#include <cstring>

/** @class FvRetrieverThread <apps/retriever/retriever_thread.h>
 * FireVision retriever thread.
 * This implements the functionality of the FvRetrieverPlugin.
 * @author Tim Niemueller
 */

/** Constructor. */
FvRetrieverThread::FvRetrieverThread()
  : Thread("FvRetrieverThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC)
{
  seq_writer = NULL;
}


/** Destructor. */
FvRetrieverThread::~FvRetrieverThread()
{
}


void
FvRetrieverThread::init()
{
  try {
    logger->log_debug(name(), "Registering for camera '%s'",
		      config->get_string("/firevision/retriever/camera").c_str());
    cam = vision_master->register_for_camera(config->get_string("/firevision/retriever/camera").c_str(), this);
  } catch (Exception &e) {
    e.append("FvRetrieverThread::init() failed");
    throw;
  }
  try {
    shm = new SharedMemoryImageBuffer("retriever", cam->colorspace(),
				      cam->pixel_width(), cam->pixel_height());
    if ( ! shm->is_valid() ) {
      throw Exception("Shared memory segment not valid");
    }
  } catch (Exception &e) {
    delete cam;
    cam = NULL;
    throw;
  }

  seq_writer = NULL;
  try {
    if ( config->get_bool("/firevision/retriever/save_images") ) {
      logger->log_info(name(), "Writing images to disk");
      Writer* writer = new JpegWriter();
      seq_writer = new SeqWriter(writer);
      std::string save_path;
       try {
 	save_path = config->get_string("/firevision/retriever/save_path");
       } catch (Exception &e) {
	 save_path = ("recorded_images");
	 logger->log_info(name(), "No save path specified. Using './%s'", save_path.c_str());
       }
      seq_writer->set_path( save_path.c_str() );
      seq_writer->set_dimensions( cam->pixel_width(), cam->pixel_height() );
      seq_writer->set_colorspace( cam->colorspace() );
    }
  } catch (Exception &e) {
    // ignored, not critical
  }

  __tt = NULL;
  try {
    if ( config->get_bool("/firevision/retriever/use_time_tracker") ) {
      __tt = new TimeTracker();
      __ttc_capture = __tt->add_class("Capture");
      __ttc_memcpy  = __tt->add_class("Memcpy");
      __ttc_dispose = __tt->add_class("Dispose");
      __loop_count  = 0;
    }
  } catch (Exception &e) {
    // ignored, not critical
  }
}


void
FvRetrieverThread::finalize()
{
  logger->log_debug(name(), "Unregistering from vision master");
  vision_master->unregister_thread(this);
  delete cam;
  delete shm;
  delete seq_writer;
  delete __tt;
}


/** Thread loop. */
void
FvRetrieverThread::loop()
{
  if (__tt) {
    // use time tracker
    __tt->ping_start(__ttc_capture);
    cam->capture();
    __tt->ping_end(__ttc_capture);
    __tt->ping_start(__ttc_memcpy);
    memcpy(shm->buffer(), cam->buffer(), cam->buffer_size()-1);
    __tt->ping_end(__ttc_memcpy);
    __tt->ping_start(__ttc_dispose);
    cam->dispose_buffer();
    __tt->ping_end(__ttc_dispose);
    if ( (++__loop_count % 200) == 0 ) {
      // output results every 200 loops
      __tt->print_to_stdout();
    }
  } else {
    // no time tracker
    cam->capture();
    memcpy(shm->buffer(), cam->buffer(), cam->buffer_size()-1);
    cam->dispose_buffer();
  }

  if (seq_writer) {
    seq_writer->write( shm->buffer() );
  }
}
