
/***************************************************************************
 *  pipeline.cpp - Implementation of the image processing pipeline for
 *                 cannikin
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

// leutron.h MUST be included before pipeline.h otherwise you will
// get BadBugs(TM): The dsylib.h conflicts with X.h. There are some variables
// in dsylib.h named like some definitions in X.h
#include <cams/leutron.h>

#include "pipeline.h"
#include "config.h"

#include <utils/system/console_colors.h>
#include <utils/system/argparser.h>

#include <fvutils/ipc/shm_image.h>
#include <fvutils/ipc/shm_registry.h>
#include <fvutils/color/conversions.h>

#include <cams/factory.h>

#include <models/scanlines/grid.h>
#include <models/scanlines/cornerhorizon.h>
#include <models/color/thresholds.h>
#include <models/color/lookuptable.h>
#include <models/relative_position/box_relative.h>
#include <models/global_position/ballglobal.h>

#include <classifiers/simple.h>

#include <unistd.h>
#include <iostream>

using namespace std;

CannikinPipeline::CannikinPipeline(ArgumentParser *argp, CannikinConfig *config)
{
  param_width = param_height = 0;
  msg_prefix = cblue + "CannikinPipeline: " + cnormal;
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

  if ( (use_fileloader = argp->hasArgument("L")) ) {
    cout <<  msg_prefix << "Fileloader requested. Looking for needed parameters" << endl;
    if ( (file = argp->getArgument("f")) == NULL) {
      file = "../src/modules/robocup/firevision/images/office-ball-kabel-wirrwarr_748x572_leutron.yuv422packed.raw";
      cout << msg_prefix << cred << "No file given, using default (" << file << ")" << cnormal << endl;
    }
  }

  generate_output = argp->hasArgument("o");

  state = CANNIKIN_STATE_UNINITIALIZED;
  mode = DETECT_CUP;
}


CannikinPipeline::~CannikinPipeline()
{
  cout << msg_prefix << "destructor called" << endl;

  finalize();

  delete cam;
  delete camctrl;

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
CannikinPipeline::init()
{

  const char *as = config->CameraString.c_str();
  cam = CameraFactory::instance( as );

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
  shm_buffer     = new SharedMemoryImageBuffer(YUV422_PLANAR, width, height, FIREVISION_SHM_IMAGE_FRONT_PROCESSED);
  cout << msg_prefix << "Creating shared memory segment for source image" << endl;
  shm_buffer_src = new SharedMemoryImageBuffer(YUV422_PLANAR, width, height, FIREVISION_SHM_IMAGE_FRONT_RAW);

  buffer     = shm_buffer->getBuffer();
  buffer_src = shm_buffer_src->getBuffer();
  buffer1    = (unsigned char *)malloc( buffer_size );
  buffer2    = (unsigned char *)malloc( buffer_size );
  buffer3    = (unsigned char *)malloc( buffer_size );

  // models
  scanlines = new ScanlineGrid(width, height,
			       config->ScanlineGridXOffset, config->ScanlineGridYOffset);

  string colormap_filestem = ColorModelLookupTable::composeFilename(config->ColormapDirectory
								    + "/" +
								    config->ColormapFilestem );
  
  colormap_filestem_cindex = colormap_filestem.find("%s");
  if ( colormap_filestem_cindex != string::npos ) {
    string tmp;
    tmp = colormap_filestem;
    tmp.insert(colormap_filestem_cindex, "orange");
    colormaps[CC_ORANGE] = strdup(tmp.c_str());

    tmp = colormap_filestem;
    tmp.insert(colormap_filestem_cindex, "yellow");
    colormaps[CC_YELLOW] = strdup(tmp.c_str());

    tmp = colormap_filestem;
    tmp.insert(colormap_filestem_cindex, "red");
    colormaps[CC_RED] = strdup(tmp.c_str());

    tmp = colormap_filestem;
    tmp.insert(colormap_filestem_cindex, "blue");
    colormaps[CC_BLUE] = strdup(tmp.c_str());

    tmp = colormap_filestem;
    tmp.insert(colormap_filestem_cindex, "green");
    colormaps[CC_GREEN] = strdup(tmp.c_str());
  }

  cm  = new ColorModelLookupTable( config->LookupTableWidth,
				   config->LookupTableHeight,
				   FIREVISION_SHM_LUT_FRONT_COLOR,
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
  if ( config->ClassifierType != "simple") {
    cout << msg_prefix << cyellow << "Only the really simple classifier is supporter at this time" << cnormal << endl;
  }
  classifier   = new ReallySimpleClassifier(width, height, scanlines, cm, 20 /* min pixels to consider */, 30 /* initial box extent */);

}


void
CannikinPipeline::set_colormap(cup_color_t c, const char *file)
{
  if ( colormaps.find(c) != colormaps.end() ) {
    free(colormaps[c]);
  }
  if ( file == NULL ) {
    // remove
    colormaps.erase(c);
  } else {
    // set
    colormaps[c] = strdup(file);
  }
}


void
CannikinPipeline::finalize()
{
  if (cam != NULL)  cam->close();
}


void
CannikinPipeline::handle_signal(int signum)
{
  if ( (signum == SIGINT) ||
       (signum == SIGTERM) ) {
    quit = true;
  }
}

void
CannikinPipeline::run(unsigned int delay)
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
CannikinPipeline::run(unsigned int delay, unsigned int times)
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
CannikinPipeline::getRelativeBoxPosModel()
{
  return box_rel;
}


GlobalPositionModel *
CannikinPipeline::getGlobalBoxPosModel()
{
  return box_glob;
}


VelocityModel *
CannikinPipeline::getBoxRelativeVelocityModel()
{
  return box_relvelo;
}


VelocityModel *
CannikinPipeline::getBoxGlobalVelocityModel()
{
  return box_globvelo;
}
*/

CameraControl *
CannikinPipeline::getCameraControl()
{
  return camctrl;
}


ScanlineModel *
CannikinPipeline::getScanlineModel()
{
  return scanlines;
}


void
CannikinPipeline::getDataTakenTime(long int *sec, long int *usec)
{
  *sec  = data_taken_time.tv_sec;
  *usec = data_taken_time.tv_usec;
}


void
CannikinPipeline::set_mode(cannikin_mode_t mode)
{
  this->mode = mode;
  switch (mode) {
  case DETECT_CUP:
    state = CANNIKIN_STATE_REINITIALIZE_COLORMAP;
    break;
  case DETERMINE_CUP_COLOR:
    state = CANNIKIN_STATE_DETERMINE_CUP_COLOR;
    break;
  default: break;
  }
}


void
CannikinPipeline::set_cup_color(cup_color_t c) {
  cup_color = c;
  if ( state == CANNIKIN_STATE_DETECTION ) {
    state = CANNIKIN_STATE_REINITIALIZE_COLORMAP;
  }
}


bool
CannikinPipeline::is_cup_visible()
{
  return cup_visible;
}


bool
CannikinPipeline::done_determining_cup_color()
{
  return cup_color_determination_done;
}


cup_color_t
CannikinPipeline::determined_cup_color()
{
  return _determined_cup_color;
}


void
CannikinPipeline::loop()
{

  if ( state == CANNIKIN_STATE_UNINITIALIZED ) return;

  switch (state) {

  case CANNIKIN_STATE_REINITIALIZE_COLORMAP:
    reinitialize_colormap();
    state = CANNIKIN_STATE_DETECTION;
    break;

  case CANNIKIN_STATE_DETECTION:
    detect_cup();
    break;

  case CANNIKIN_STATE_DETERMINE_CUP_COLOR:
    determine_cup_color();
    break;

  default: return;
  }

}


void
CannikinPipeline::reinitialize_colormap()
{
  if ( colormaps.find(cup_color) != colormaps.end() ) {
    cm->load(colormaps[cup_color]);
  } else {
    cm->reset();
  }
}


void
CannikinPipeline::detect_cup()
{
  cam->capture();

  gettimeofday(&data_taken_time, NULL);

  // Convert buffer (re-order bytes) and set classifier buffer
  convert(cspace_from, cspace_to, cam->buffer(), buffer_src, width, height);
  memcpy(buffer, buffer_src, buffer_size);


  // Classify image, find ROIs by color
  classifier->setSrcBuffer( buffer_src );
  rois = classifier->classify();

  if (rois->empty()) {
    if ( generate_output ) {
      cout << msg_prefix << cred << "No ROIs!" << cnormal << endl;
    }
    // No box
    shm_buffer->setCircleFound(false);
    shm_buffer->setROI(0, 0, 0, 0);
    //box_rel->reset();
  }

  // Go through all ROIs, filter and recognize shapes
  for (r = rois->begin(); r != rois->end(); ++r) {

    if ( generate_output ) {
      cout << msg_prefix << cgreen << "ROI:     " << cnormal
  	   << "start: (" << (*r).start.x << "," << (*r).start.y << ")"
    	   << "   width: " << (*r).width
    	   << "   height: " << (*r).height
           << endl;
    }

    
    // Try to detect box shape
    if ((*r).hint == H_BALL) {
      // box_visible = true;

      // classifier->getMassPointOfBall( &(*r), &mass_point );
      // update ball position
      // box_rel->setCenter( mass_point.x, mass_point.y );
      // box_rel->calc();

      cout << msg_prefix << cgreen << "Mass point found at (" << mass_point.x
	   << "," << mass_point.y << ")" << endl;

      shm_buffer->setCircle( mass_point.x, mass_point.y, 8 );
      shm_buffer->setCircleFound( true );

      shm_buffer->setROI(r->start.x, r->start.y, r->width, r->height);

      /*
      if ( generate_output ) {
	cout << msg_prefix << cgreen << "RelPosU: " << cnormal
	     << "X: " << box_rel->getX()
	     << "  Y: " << box_rel->getY()
	     << "  Dist: " << box_rel->getDistance()
	     << "  Bear: " << box_rel->getBearing()
	     << endl;

	cout << msg_prefix << cgreen << "GlobPos: " << cnormal
	     << "X: " << box_glob->getX()
	     << "  Y: " << box_glob->getY()
	     << endl;
      }
      */

      break;
    } // end is box
  } // end for rois

  rois->clear();
  delete rois;

  cam->dispose_buffer();

}


void
CannikinPipeline::determine_cup_color()
{
  cam->capture();
  cam->dispose_buffer();
}
