
/***************************************************************************
 *  pipeline.cpp - Implementation of the image processing pipeline for
 *                 geegaw
 *
 *  Generated: Tue Apr 10 13:44:45 2007 (based on suricate)
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

/// @cond RCSoftX

#include "pipeline.h"
#include "config.h"

#include <utils/system/console_colors.h>
#include <utils/system/argparser.h>
#include <utils/math/angle.h>

#include <fvutils/ipc/shm_image.h>
#include <fvutils/ipc/shm_registry.h>
#include <fvutils/color/conversions.h>

#include <fvutils/draw/drawer.h>

#include <cams/leutron.h>
#include <cams/sony_evid100p_control.h>

#include <models/scanlines/beams.h>
#include <models/color/thresholds.h>
#include <models/color/lookuptable.h>
#include <models/relative_position/box_relative.h>

#include <classifiers/simple.h>

#include <unistd.h>
#include <iostream>

using namespace std;

GeegawPipeline::GeegawPipeline(ArgumentParser *argp, GeegawConfig *config)
{
  param_width = param_height = 0;
  msg_prefix = cblue + "GeegawPipeline: " + cnormal;
  // box_visible = false;
  quit = false;

  // This doesn't change because we do the conversion to YUV422_PLANAR
  // and then work on this image
  cspace_to   = YUV422_PLANAR;

  this->argp   = argp;
  this->config = config;

  scanlines     = NULL;
  cm            = NULL;
  /*
  box_rel      = NULL;
  box_glob     = NULL;
  box_relvelo  = NULL;
  box_globvelo = NULL;
  */
  classifier    = NULL;

  generate_output = argp->hasArgument("o");

  cam = NULL;
  camctrl = NULL;
}


GeegawPipeline::~GeegawPipeline()
{
  cout << msg_prefix << "destructor called" << endl;

  finalize();

  delete cam;

  cam = NULL;
  camctrl = NULL;

  cout << msg_prefix << "Deleting shared memory buffer for final image" << endl;
  delete shm_buffer;
  cout << msg_prefix << "Deleting shared memory buffer for source image" << endl;
  delete shm_buffer_src;
  cout << msg_prefix << "Freeing temporary buffers" << endl;
  free(buffer1);
  free(buffer2);
  free(buffer3);

  delete scanlines;
  delete cm;
  /*
  delete box_rel;
  delete box_glob;
  delete box_relvelo;
  delete box_globvelo;
  */
  delete classifier;

}


void
GeegawPipeline::init()
{

  cam = new LeutronCamera();
  camctrl = new SonyEviD100PControl( "/dev/ttyS0" );

  cam->open();
  cam->start();

  width  = cam->pixel_width();
  height = cam->pixel_height();
  cspace_from = cam->colorspace();

  /* NOTE:
   * buffer_src is the place where the converted image is stored. 
   * After the processing of a given region is done the resulting image
   * has to be placed in buffer. Further steps are done on this buffer!
   * At the beginning of the loop buffer_src will contain the source image in
   * YUV422_PLANAR format that just arrived from the camera. In later runs
   * there are already some filtered ROIs. buffer_src is considered to be
   * read-only. Do not write to this buffer!
   * buffer2 and buffer3 are available for any operations you would
   * like to do for temporary storage. Avoid copying too much data around
   * and think about in-place operations. Two buffers should be sufficient
   * for most operations.
   */

  buffer_size = colorspace_buffer_size(YUV422_PLANAR, width, height);

  cout << msg_prefix << "Creating shared memory segment for final image" << endl;
  shm_buffer     = new SharedMemoryImageBuffer(YUV422_PLANAR, width, height, FIREVISION_SHM_IMAGE_OMNI_PROCESSED);
  cout << msg_prefix << "Creating shared memory segment for source image" << endl;
  shm_buffer_src = new SharedMemoryImageBuffer(YUV422_PLANAR, width, height, FIREVISION_SHM_IMAGE_OMNI_RAW);

  buffer     = shm_buffer->getBuffer();
  buffer_src = shm_buffer_src->getBuffer();
  buffer1    = (unsigned char *)malloc( buffer_size );
  buffer2    = (unsigned char *)malloc( buffer_size );
  buffer3    = (unsigned char *)malloc( buffer_size );

  // models
  scanlines = new ScanlineBeams(width, height, 
				/* start x  */    width / 2,
				/* start y  */    height,
				/* stop_y   */    200,
				/* offset_y */     10,
				/* angle_from */  deg2rad(-30),
				/* angle_range */ deg2rad(60),
				/* num beams */   10);

  cm  = new ColorModelLookupTable( "../etc/firevision/colormaps/geegaw.colormap",
				   config->LookupTableWidth,
				   config->LookupTableHeight,
				   FIREVISION_SHM_LUT_OMNI_COLOR,
				   true /* destroy on free */);
  cm->reset();


  /*
  // Position models for box
  box_rel      = new BoxRelative(width, height,
				 config->CameraHeight,
				 config->CameraOffsetX,
				 config->CameraOffsetY,
				 config->CameraOrientation,
				 config->HorizontalViewingAngle,
				 config->VerticalViewingAngle
				 );
  box_rel->setPanTilt(config->CameraBearing, config->CameraSlope);
  
  box_glob     = new BallGlobal( box_rel );
  */

  // Classifier
  // classifier   = new ReallySimpleClassifier(width, height, scanlines, cm, 20 /* min pixels to consider */, 30 /* initial box extent */);

}


void
GeegawPipeline::finalize()
{
  if (cam != NULL)  cam->close();
}


void
GeegawPipeline::handle_signal(int signum)
{
  if ( (signum == SIGINT) ||
       (signum == SIGTERM) ) {
    quit = true;
  }
}

void
GeegawPipeline::run(unsigned int delay)
{
  SignalManager::instance()->register_handler(SIGINT, this);

  delay *= 1000;

  while ( !quit ) {
    loop();
    usleep(delay);
  }

  SignalManager::instance()->finalize();
}


void
GeegawPipeline::run(unsigned int delay, unsigned int times)
{
  SignalManager::instance()->register_handler(SIGINT, this);

  delay *= 1000;

  for (unsigned int i = 0; !quit && (i < times); ++i) {
    loop();
    usleep(delay);
  }

  SignalManager::instance()->finalize();
}



/*
RelativePositionModel *
GeegawPipeline::getRelativeBoxPosModel()
{
  return box_rel;
}


GlobalPositionModel *
GeegawPipeline::getGlobalBoxPosModel()
{
  return box_glob;
}


VelocityModel *
GeegawPipeline::getBoxRelativeVelocityModel()
{
  return box_relvelo;
}


VelocityModel *
GeegawPipeline::getBoxGlobalVelocityModel()
{
  return box_globvelo;
}
*/

CameraControl *
GeegawPipeline::getCameraControl()
{
  return camctrl;
}


ScanlineModel *
GeegawPipeline::getScanlineModel()
{
  return scanlines;
}


void
GeegawPipeline::getDataTakenTime(long int *sec, long int *usec)
{
  *sec  = data_taken_time.tv_sec;
  *usec = data_taken_time.tv_usec;
}


bool
GeegawPipeline::obstacles_found()
{
  return (! obstacles.empty());
}


void
GeegawPipeline::loop()
{

  cam->capture();

  gettimeofday(&data_taken_time, NULL);

  // Convert buffer (re-order bytes) and set classifier buffer
  convert(cspace_from, cspace_to, cam->buffer(), buffer_src, width, height);
  memcpy(buffer, buffer_src, buffer_size);

  Drawer *d = new Drawer();
  d->setBuffer( buffer, width, height );
  while ( ! scanlines->finished() ) {
    d->drawPoint((*scanlines)->x, (*scanlines)->y);
    ++(*scanlines);
  }

  // Classify image, find ROIs by color
  cam->dispose_buffer();
}

/// @endcond
