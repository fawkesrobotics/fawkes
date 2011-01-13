
/***************************************************************************
 *  pipeline_thread.cpp - SwissRanger Save Pipeline Thread
 *
 *  Created: Fri Jan 22 10:50:13 2010
 *  Copyright  2005-2010  Tim Niemueller [www.niemueller.de]
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

#include "pipeline_thread.h"

#include <fvcams/camera.h>

#include <sys/time.h>
#include <stdlib.h>
#include <cstdio>

using namespace fawkes;

/** @class FvSrSavePipelineThread "pipeline_thread.h"
 * SrSave vision image processing pipeline.
 * This thread implements an image processing pipeline that uses a colormodel and
 * classifier to determine regions of interest (ROI) which contain a significant
 * amount with "pixels of ball color". The best ROI is then filtered for edge detection.
 * On the edges a circle shape detection is carried out to confirm the result and to
 * get the required data to calculate the relative and global position of the ball.
 *
 * @author Tim Niemueller
 */


/** Constructor. */
FvSrSavePipelineThread::FvSrSavePipelineThread()
  : Thread("FvSrSavePipelineThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC)
{
}


/** Destructor. */
FvSrSavePipelineThread::~FvSrSavePipelineThread()
{
}


/** Initialize the pipeline thread.
 * Camera is requested, config parameters are obtained from the config db, and
 * other miscellaneous init stuff is done here.
 */
void
FvSrSavePipelineThread::init()
{
  try {
    __cam = vision_master->register_for_raw_camera("swissranger:any:mode=CARTESIAN_FLOAT", this );
  } catch (Exception& e) {
    e.append("FvSrSavePipelineThread::init() failed since no camera is specified");
    throw;
  }
}


/** Thread finalization. */
void
FvSrSavePipelineThread::finalize()
{
  vision_master->unregister_thread(this);
}

/** A new image is retrieved from the camera and the classifier looks for a ball
 * in the image */
void
FvSrSavePipelineThread::loop()
{
  __cam->capture();

  const unsigned int width  = __cam->pixel_width();
  const unsigned int height = __cam->pixel_height();

  float *fbuf = (float *)__cam->buffer();
  float *x = fbuf;
  float *y = x + width * height;
  float *z = y + width * height;

  char *filename;
  if (asprintf(&filename, "swissranger-%05u.pts", __frame_i++) != -1) {
    FILE *f = fopen(filename, "w");

    for (unsigned int h = 0; h < height; ++h) {
      for (unsigned int w = 0; w < width; ++w) {
	fprintf(f, "%f %f %f 128 128 128\n",
		*x++ * 2000., *y++ * 2000., *z++ * 2000.);
      }
    }

    fclose(f);
    free(filename);
  } else {
    logger->log_warn(name(), "Failed to allocate filename");
  }


  __cam->dispose_buffer();
}

