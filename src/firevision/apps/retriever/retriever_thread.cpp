
/***************************************************************************
 *  retriever_thread.cpp - FireVision Retriever Thread
 *
 *  Created: Tue Jun 26 17:39:11 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <apps/retriever/retriever_thread.h>

#include <cams/camera.h>
#include <fvutils/ipc/shm_image.h>
#include <utils/time/tracker.h>

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
    vision_master->unregister_thread(this);
    delete cam;
    cam = NULL;
    throw;
  }

  __tt = new TimeTracker();
  __ttc_capture = __tt->add_class("Capture");
  __ttc_memcpy  = __tt->add_class("Memcpy");
  __ttc_dispose = __tt->add_class("Dispose");
  __loop_count  = 0;
}


void
FvRetrieverThread::finalize()
{
  logger->log_debug(name(), "Unregistering from vision master");
  vision_master->unregister_thread(this);
  delete cam;
  delete shm;
}


/** Thread loop. */
void
FvRetrieverThread::loop()
{
  /*
  logger->log_debug(name(), "Capturing frame from camera into shared memory buffer"
		    " (cam buffer size: %lu, shm buffer size: %lu)",
		    cam->buffer_size(), shm->data_size());
  */
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
    __tt->print_to_stdout();
  }
  //logger->log_debug(name(), "DONE");
}
