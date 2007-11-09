
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

#include <cams/factory.h>
#include <cams/controlfactory.h>

#include <models/scanlines/beams.h>
#include <models/scanlines/grid.h>
#include <models/color/thresholds.h>
#include <models/color/lookuptable.h>
#include <models/relative_position/box_relative.h>

#include <classifiers/simple.h>
#include <filters/roidraw.h>

#include <unistd.h>
#include <iostream>

using namespace std;

GeegawPipeline::GeegawPipeline(ArgumentParser *argp, GeegawConfig *config, bool object_mode)
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

  generate_output = argp->has_arg("o");

  cam = NULL;
  camctrl = NULL;

  mode = object_mode ? MODE_LOSTNFOUND : MODE_OBSTACLES;

  deter_colormaps.push_back("../etc/firevision/colormaps/red_box.colormap");
  deter_colormaps.push_back("../etc/firevision/colormaps/blue_box.colormap");
  deter_colormaps.push_back("../etc/firevision/colormaps/pineapple.colormap");
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
  delete deter_cm;
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

  try {
    cam     = CameraFactory::instance( config->CameraType.c_str() );
  } catch (UnknownCameraTypeException &e) {
    cout << msg_prefix << "Failed to instantiate camera." << endl;
    e.print_trace();
    throw;
  }
  try {
    camctrl = CameraControlFactory::instance( config->CameraControlType.c_str() );
  } catch (UnknownCameraControlTypeException &e) {
    cout << msg_prefix << "Failed to instantiate camera control." << endl;
    e.print_trace();
    throw;
  }

  cout << msg_prefix << "Opening camera, this may take a while..." << endl;

  try {
    cam->open();
    cam->start();
  } catch (Exception &e) {
    e.print_trace();
    throw;
  }

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
  shm_buffer     = new SharedMemoryImageBuffer("geegaw-processed", YUV422_PLANAR, width, height);
  cout << msg_prefix << "Creating shared memory segment for source image" << endl;
  shm_buffer_src = new SharedMemoryImageBuffer("geegaw-raw", YUV422_PLANAR, width, height);

  buffer     = shm_buffer->buffer();
  buffer_src = shm_buffer_src->buffer();
  buffer1    = (unsigned char *)malloc( buffer_size );
  buffer2    = (unsigned char *)malloc( buffer_size );
  buffer3    = (unsigned char *)malloc( buffer_size );

  // models
  if ( mode == MODE_LOSTNFOUND ) {
    scanlines = new ScanlineGrid(width, height, 5, 5);
  } else {
    scanlines = new ScanlineBeams(width, height, 
				  /* start x  */    width / 2,
				  /* start y  */    height,
				  /* stop_y   */    150,
				  /* offset_y */     5,
				  /* distribute_start_x */ true,
				  /* angle_from */  deg2rad(-30),
				  /* angle_range */ deg2rad(60),
				  /* num beams */   20);
  }

  cm  = new ColorModelLookupTable( "../etc/firevision/colormaps/geegaw.colormap",
				   config->LookupTableWidth,
				   config->LookupTableHeight,
				   FIREVISION_SHM_LUT_OMNI_COLOR,
				   true /* destroy on free */);
  deter_cm  = new ColorModelLookupTable( "../etc/firevision/colormaps/geegaw.colormap",
					 config->LookupTableWidth,
					 config->LookupTableHeight,
					 FIREVISION_SHM_LUT_FRONT_COLOR,
					 true /* destroy on free */);

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

  rel_pos = new BoxRelative(width, height,
			    config->CameraHeight,
			    config->CameraOffsetX,
			    config->CameraOffsetY,
			    config->CameraOrientation,
			    config->HorizontalViewingAngle,
			    config->VerticalViewingAngle
			    );
  rel_pos->setRadius(3);

  object_relposmod = new BoxRelative(width, height,
				     config->CameraHeight,
				     config->CameraOffsetX,
				     config->CameraOffsetY,
				     config->CameraOrientation,
				     config->HorizontalViewingAngle,
				     config->VerticalViewingAngle
				     );
  object_relposmod->setRadius(3);


  // Classifier
  classifier   = new ReallySimpleClassifier(width, height,
                                            scanlines, cm,
                                            10 /* min pixels to consider */,
                                            30 /* initial box extent */,
                                            /* upward */ (mode == MODE_OBSTACLES),
                                            /* neighbourhood min match */ 5);

  deter_classifier   = new ReallySimpleClassifier(width, height,
						  scanlines, deter_cm,
						  10 /* min pixels to consider */,
						  30 /* initial box extent */,
						  /* upward */ true,
						  /* neighbourhood min match */ 5);

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


float
GeegawPipeline::object_bearing()
{
  return _object_bearing;
}


float
GeegawPipeline::object_distance()
{
  return _object_distance;
}


std::list<polar_coord_t> &
GeegawPipeline::getObstacles()
{
  return obstacles;
}


void
GeegawPipeline::pan_tilt(float *pan, float *tilt)
{
  *pan  = this->pan;
  *tilt = this->tilt;
}


RelativePositionModel *
GeegawPipeline::object_relpos()
{
  return object_relposmod;
}


void
GeegawPipeline::detect_obstacles()
{
  classifier->setSrcBuffer( buffer_src );
  rois = classifier->classify();
  obstacles.clear();

  FilterROIDraw *rdf = new FilterROIDraw();

  // Go through all ROIs, count them as obstacles
  unsigned int max_obstacles = 10;
  if ( rois->empty() ) {
    // cout << "Doh, no ROIs!" << endl;
  }
  bool first = true;
  for (r = rois->begin(); r != rois->end(); ++r) {
    rdf->set_dst_buffer(buffer, &(*r));
    rdf->apply();
    polar_coord_t o;
    camctrl->pan_tilt_rad(&pan, &tilt);
    rel_pos->setPanTilt(pan, tilt);
    rel_pos->setCenter( (*r).start.x + (*r).width / 2,
		        (*r).start.y + (*r).height );
    rel_pos->calc_unfiltered();
    o.phi = rel_pos->getBearing();
    o.r   = rel_pos->getDistance();
    obstacles.push_back(o);
    if (obstacles.size() == max_obstacles) break;

    if ( first ) {
      // First is the biggest ROI, set as object
      object_relposmod->setPanTilt(pan, tilt);
      object_relposmod->setCenter( (*r).start.x + (*r).width / 2,
				   (*r).start.y + (*r).height / 2 );
      object_relposmod->calc_unfiltered();
      _object_bearing = object_relposmod->getBearing();
      _object_distance = object_relposmod->getDistance();
      first = false;
    }
  }
  rois->clear();
  delete rois;
  delete rdf;
}


void
GeegawPipeline::detect_object()
{
  classifier->setSrcBuffer( buffer_src );
  rois = classifier->classify();
  obstacles.clear();

  if ( ! rois->empty() ) {
    FilterROIDraw *rdf = new FilterROIDraw();
  
    r = rois->begin();
    rdf->set_dst_buffer(buffer, &(*r));
    rdf->apply();
    camctrl->pan_tilt_rad(&pan, &tilt);
    rel_pos->setPanTilt(pan, tilt);
    rel_pos->setCenter( (*r).start.x + (*r).width / 2,
		        (*r).start.y + (*r).height );
    rel_pos->calc_unfiltered();

    // First is the biggest ROI, set as object
    object_relposmod->setPanTilt(pan, tilt);
    object_relposmod->setCenter( (*r).start.x + (*r).width / 2,
				   (*r).start.y + (*r).height / 2 );
    object_relposmod->calc_unfiltered();
    _object_bearing = object_relposmod->getBearing();
    _object_distance = object_relposmod->getDistance();

    delete rdf;
  }
  rois->clear();
  delete rois;
}


void
GeegawPipeline::add_object()
{
  if ( (add_status == ADDSTATUS_SUCCESS) || (add_status == ADDSTATUS_FAILURE) ) {
    return;
  }
  obstacles.clear();

  add_status = ADDSTATUS_INPROGRESS;

  deter_classifier->setSrcBuffer( buffer_src );
  rois = deter_classifier->classify();

  if ( rois->empty() ) {
    cout << msg_prefix << "No ROIs found while trying to add" << endl;
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
	
      if ( /* (*r).contains(width / 2, height / 2) && */
	  ((*r).width > 100) && ((*r).height > 100) ) {
	// we have a possible ROI
	if ( determine_cycle_num > 0 ) {
	  --determine_cycle_num;
	}
	++determined_valid_frames;
	if ( determined_valid_frames > 4 ) {
	  // we have the object, add it!
          cout << msg_prefix << "Found object and adding colormap" << endl;
	  *cm += *deter_cm;
	  add_status = ADDSTATUS_SUCCESS;
	}
      } else {
        cout << msg_prefix << "Object seen but ROI too small (only " << (*r).width << "x" << (*r).height << ")" << endl;
      }
    } else {
      cout << "ROI does not contain object " << endl;
    }
  } // end for rois

  ++determine_cycle_num;
  cout << msg_prefix << determine_cycle_num << " cycles done" << endl;
  if ( determine_cycle_num > 10 ) {
    // we tried for 10 frames, but did not get the needed valid frames, switch
    // to next color
    determine_cycle_num = 0;
    determined_valid_frames = 0;

    if ( deter_nextcm < deter_colormaps.size() ) {
      cout << msg_prefix << "Loading next colormap " << deter_colormaps[deter_nextcm] << endl;
      deter_cm->load(deter_colormaps[deter_nextcm++]);
    } else {
      // no more colormaps to load!
      cout << msg_prefix << "No more colormaps to try, adding failed" << endl;
      add_status = ADDSTATUS_FAILURE;
    }
  }
}


void
GeegawPipeline::loop()
{
  camctrl->process_control();

  camctrl->start_get_pan_tilt();
  cam->capture();

  gettimeofday(&data_taken_time, NULL);

  // Convert buffer (re-order bytes) and set classifier buffer
  convert(cspace_from, cspace_to, cam->buffer(), buffer_src, width, height);
  memcpy(buffer, buffer_src, buffer_size);

  if ( mode == MODE_ADD_OBJECT ) {
    if ( (add_status == ADDSTATUS_NOTRUNNING) || (add_status == ADDSTATUS_INPROGRESS) ) {
      add_object();
    }
  } else if ( mode == MODE_OBSTACLES ) {
    detect_obstacles();
  } else if ( mode == MODE_LOSTNFOUND ) {
    detect_object();
  }


  // Classify image, find ROIs by color
  cam->dispose_buffer();
  camctrl->process_control();
}


void
GeegawPipeline::setMode(GeegawPipeline::GeegawOperationMode mode)
{
  this->mode = mode;
  if ( mode == MODE_ADD_OBJECT ) {
    cout << msg_prefix << "Switching to ADD_OBJECT mode" << endl;
    determined_valid_frames = 0;
    determine_cycle_num = 0;
    if ( deter_colormaps.size() > 0 ) {
      deter_cm->load(deter_colormaps[0]);
      deter_nextcm = 1;
      add_status = ADDSTATUS_NOTRUNNING;
      last_add_status = ADDSTATUS_NOTRUNNING;
    } else {
      cout << msg_prefix << "No colormaps set, cannot add" << endl;
      add_status = ADDSTATUS_FAILURE;
      last_add_status = ADDSTATUS_NOTRUNNING;
    }
  } else if ( mode == MODE_OBSTACLES ) {
    cout << msg_prefix << "Switching to OBSTACLES mode" << endl;
    delete classifier;
    classifier   = new ReallySimpleClassifier(width, height,
					      scanlines, cm,
					      10 /* min pixels to consider */,
					      30 /* initial box extent */,
					      /* upward */ true,
					      /* neighbourhood min match */ 5);
  } else if ( mode == MODE_LOSTNFOUND ) {
    cout << msg_prefix << "Switching to LOSTNFOUND mode" << endl;
    delete classifier;
    classifier   = new ReallySimpleClassifier(width, height,
					      scanlines, cm,
					      10 /* min pixels to consider */,
					      30 /* initial box extent */,
					      /* upward */ false,
					      /* neighbourhood min match */ 5);
  } else if ( mode == MODE_RESET_COLORMAP ) {
    cout << msg_prefix << "RESETting colormap" << endl;
    cm->reset();
  }
}


GeegawPipeline::GeegawOperationMode
GeegawPipeline::getMode()
{
  return mode;
}

bool
GeegawPipeline::addStatusChanged()
{
  return (add_status != last_add_status);
}


GeegawPipeline::GeegawAddStatus
GeegawPipeline::addStatus()
{
  last_add_status = add_status;
  return add_status;
}


void
GeegawPipeline::setColormap(std::string colormap_filename_without_path)
{
  cout << msg_prefix << "Loading colormap " << (config->ColormapDirectory + "/" + colormap_filename_without_path) << endl;
  cm->load((config->ColormapDirectory + "/" + colormap_filename_without_path).c_str());
}

/// @endcond

