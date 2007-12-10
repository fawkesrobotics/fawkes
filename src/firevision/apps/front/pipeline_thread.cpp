
/***************************************************************************
 *  pipeline_thread.cpp - Front Pipeline Thread
 *
 *  Generated: Wed Jun 15 16:30:22 2005
 *  Ported to Fawkes FireVision: Sun Dec 09 23:40:04 2007
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

#include <apps/front/pipeline_thread.h>

#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>

#include <utils/math/angle.h>
#include <interfaces/object.h>

#include <cams/camera.h>
#include <cams/cameracontrol.h>
#include <classifiers/simple.h>
#include <classifiers/shrinker.h>
#include <classifiers/border_shrinker.h>
#include <models/scanlines/grid.h>
#include <models/scanlines/cornerhorizon.h>
#include <models/shape/rcd_circle.h>
#include <models/relative_position/ballrelative.h>
#include <models/global_position/ballglobal.h>
#include <models/color/lookuptable.h>
#include <filters/hv_search.h>

#include <sys/time.h>
#include <stdlib.h>
#include <cstdio>


/** @class FvFrontPipelineThread <apps/front/pipeline_thread.h>
 * Front vision image processing pipeline.
 * This thread implements an image processing pipeline that uses a colormodel and
 * classifier to determine regions of interest (ROI) which contain a significant
 * amount with "pixels of ball color". The best ROI is then filtered for edge detection.
 * On the edges a circle shape detection is carried out to confirm the result and to
 * get the required data to calculate the relative and global position of the ball.
 *
 * @author Tim Niemueller
 */


/** Constructor. */
FvFrontPipelineThread::FvFrontPipelineThread()
  : Thread("FvFrontPipelineThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC)
{
  __circle_model   = NULL;
  __ball_rel       = NULL;
  __ball_glob      = NULL;
  __classifier     = NULL;
  __shrinker       = NULL;
  __hv_search      = NULL;
  __scanlines      = NULL;
  __shm_buffer     = NULL;
  __cspace_to      = YUV422_PLANAR;
}


/** Destructor. */
FvFrontPipelineThread::~FvFrontPipelineThread()
{
}


/** Initialize the pipeline thread.
 * Camera is requested, config parameters are obtained from the config db, and
 * other miscellaneous init stuff is done here.
 */
void
FvFrontPipelineThread::init()
{
  try {
    __cam = vision_master->register_for_camera( config->get_string("/firevision/front/camera").c_str(), this );
  } catch (Exception& e) {
    e.append("FvFrontPipelineThread::init() failed since no camera is specified");
    throw;
  }

  __camctrl = NULL;

  // get _all_ config values
  try {
    __cfg_ball_circumfer  = config->get_float("/general/ball_circumference");
    __cfg_scanline_model  = config->get_string("/firevision/front/scanline_model");
    __cfg_shrinker_type   = config->get_string("/firevision/front/shrinker_type");
    __cfg_colormap        = config->get_string("/firevision/front/colormap_file");
    __cfg_colormap_width  = config->get_uint("/firevision/front/colormap_width");
    __cfg_colormap_height = config->get_uint("/firevision/front/colormap_height");
    __cfg_cam_height      = config->get_float("/firevision/front/camera_height");
    __cfg_cam_pan         = deg2rad(config->get_float("/firevision/front/camera_base_pan"));
    __cfg_cam_tilt        = deg2rad(config->get_float("/firevision/front/camera_base_tilt"));
    __cfg_cam_offset_x    = config->get_float("/firevision/front/camera_offset_x");
    __cfg_cam_offset_y    = config->get_float("/firevision/front/camera_offset_y");
    __cfg_cam_hor_va      = deg2rad(config->get_float("/firevision/front/camera_horizontal_viewing_angle"));
    __cfg_cam_ver_va      = deg2rad(config->get_float("/firevision/front/camera_vertical_viewing_angle"));
    __cfg_field_length    = config->get_float("/general/field_length");
    __cfg_field_width     = config->get_float("/general/field_width");
    __cfg_field_border    = config->get_float("/general/field_border");
    __cfg_grid_offset_x   = config->get_uint("/firevision/front/grid_offset_x");
    __cfg_grid_offset_y   = config->get_uint("/firevision/front/grid_offset_y");

    __cfg_rcd_max_failures      = config->get_uint("/firevision/front/rcd_max_failures");
    __cfg_rcd_min_pixels        = config->get_uint("/firevision/front/rcd_min_pixels");
    __cfg_rcd_min_interpix_dist = config->get_uint("/firevision/front/rcd_min_inter_pixel_distance");
    __cfg_rcd_max_dist_p4       = config->get_uint("/firevision/front/rcd_max_dist_p4");
    __cfg_rcd_max_dist_all      = config->get_uint("/firevision/front/rcd_max_dist_all");
    __cfg_rcd_hwratio           = config->get_float("/firevision/front/rcd_hwratio");
    __cfg_rcd_hollowrate        = config->get_float("/firevision/front/rcd_hollowrate");
    __cfg_rcd_max_runtime       = config->get_uint("/firevision/front/rcd_max_runtime");
  } catch (Exception &e) {
    e.append("Could not fetch all configuration values required for front vision");
    throw;
  }

  // interface
  try {
    __ball_interface = interface_manager->open_for_writing<ObjectPositionInterface>("Front");
    __ball_interface->set_object_type( ObjectPositionInterface::BALL );
  } catch (Exception &e) {
    e.append("Opening ball interface for writing failed");
    throw;
  }

  __img_width   = __cam->pixel_width();
  __img_height  = __cam->pixel_height();
  __cspace_from = __cam->colorspace();
  
  // shm buffer
  // TODO: image dimensions are temporally scaled down by a factor of two
  // until we have appropriate viewing tools
  __buffer_size = colorspace_buffer_size(__cspace_to, __img_width, __img_height);
  __shm_buffer = new SharedMemoryImageBuffer("front-processed",
					     __cspace_to, __img_width, __img_height);
  

  __buffer = __shm_buffer->buffer();


  // models
  if ( __cfg_scanline_model == "cornerhorizon_grid" ) {
    ScanlineGrid *grid = new ScanlineGrid(__img_width, __img_height,
					  __cfg_grid_offset_x, __cfg_grid_offset_y);
    __scanlines = new CornerHorizon( grid,
				     __cfg_field_length, __cfg_field_width, __cfg_field_border,
				     __img_width, __img_height,
				     __cfg_cam_height, __cfg_cam_pan,
				     __cfg_cam_hor_va, __cfg_cam_ver_va);
  } else {
    __scanlines = new ScanlineGrid(__img_width, __img_height,
				   __cfg_grid_offset_x, __cfg_grid_offset_y);
				   
  }

  __circle_model = new RcdCircleModel( __cfg_rcd_max_failures,
				       __cfg_rcd_min_pixels,
				       __cfg_rcd_min_interpix_dist,
				       __cfg_rcd_max_dist_p4,
				       __cfg_rcd_max_dist_all,
				       __cfg_rcd_hwratio,
				       __cfg_rcd_hollowrate,
				       __cfg_rcd_max_runtime / 1000.f);


  // Position models for ball
  __ball_rel = new BallRelative(__img_width, __img_height,
				__cfg_cam_height, __cfg_cam_offset_x, __cfg_cam_offset_y,
				__cfg_cam_pan, __cfg_cam_hor_va, __cfg_cam_ver_va,
				__cfg_ball_circumfer);
  
  __ball_glob = new BallGlobal( __ball_rel );


  __cm = new ColorModelLookupTable(__cfg_colormap.c_str(),
				   __cfg_colormap_width, __cfg_colormap_height,
				   "front-colormap", /* destroy on delete */ true);

  __classifier = new SimpleColorClassifier(__scanlines, __cm);


  if ( __cfg_shrinker_type == "shrinker" ) {
    __shrinker = new Shrinker();
  } else if ( __cfg_shrinker_type == "border" ) {
    __shrinker = new BorderShrinker(0, 0, 0, 50);
  } else {
    __shrinker = NULL;
  }


  __hv_search = new FilterHVSearch(__cm, C_ORANGE);

}


/** Thread finalization. */
void
FvFrontPipelineThread::finalize()
{
  delete __scanlines;
  delete __classifier;
  delete __cm;
  delete __circle_model;
  delete __ball_rel;
  delete __ball_glob;
  delete __shrinker;
  delete __hv_search;

  vision_master->unregister_thread(this);
  delete __cam;
  delete __camctrl;

  try {
    interface_manager->close(__ball_interface);
  } catch (Exception &e) {
    e.append("Closing ball interface failed");
    throw;
  }
}


/** Detect the ball in the given ROI and update models appropriately.
 * Pre-condition: buffer contains the image with the ROI filtered for edges
 * Post-condition: If a ball was found the position models are updated so that
 * it contains the current ball.
 * @param roi the ROI where to search the ball
 * @return true, if a ball was found in the given ROI, false otherwise.
 */
bool
FvFrontPipelineThread::detect_ball_and_update_models( ROI *roi )
{
  __circle_model->parseImage(__buffer, roi);
  Circle *circle = (Circle *)__circle_model->getMostLikelyShape();
  if (circle != NULL) {
    // A circle was found
    if ( __generate_output ) {
      logger->log_debug(name(), "Circle: center: (%i, %i), radius: %f  count: %i",
			circle->center.x, circle->center.y, circle->radius, circle->count);
    }

    __ball_rel->setRadius( circle->radius );
    // note that BallRelative requires the circle-center wrt image, not wrt roi
    __ball_rel->setCenter(int(circle->center.x + (roi)->start.x),
			  int(circle->center.y + (roi)->start.y) );

    __shm_buffer->set_circle_found(true);
    __shm_buffer->set_roi(roi->start.x, roi->start.y, roi->width, roi->height);
    __shm_buffer->set_circle((int)(roi->start.x + circle->center.x),
			     (int)(roi->start.y + circle->center.y),
			     (int)circle->radius);

    return true;

  } else {
    if ( __generate_output ) {
      logger->log_debug(name(), "No circle was found in the ROI");
    }

    __shm_buffer->set_circle_found(false);
    __ball_rel->reset();

    return false;
  }
}



void
FvFrontPipelineThread::fetch_pantilt_and_update_models()
{
  float pan  = 0.f;
  float tilt = 0.f;

  // camctrl->getPanTiltRad(&pan, &tilt);
  __ball_rel->setPanTilt(pan, tilt);
  __scanlines->setPanTilt(pan, tilt);
  /*
  // Maybe needed for more sophisticated models
  // ball_relvelo->setPanTilt(pan, tilt);
  // ball_globvelo->setPanTilt(pan, tilt);
  */
}


/** A new image is retrieved from the camera and the classifier looks for a ball
 * in the image */
void
FvFrontPipelineThread::loop()
{
  if ( __camctrl != NULL ) {
    __camctrl->process_control();
    __camctrl->start_get_pan_tilt();
  }

  __ball_visible = false;
  __already_fetched_pantilt = false;

  __cam->capture();
  gettimeofday(&__data_taken_time, NULL);

  // no convert needed, buffer is guaranteed to be in YUV422_PLANAR format

  // Classify image, find ROIs by color
  __classifier->set_src_buffer( __cam->buffer(), __img_width, __img_height );
  __rois = __classifier->classify();

  if (__rois->empty()) {
    if ( __generate_output ) {
      logger->log_debug(name(), "No ROIs!");
    }
    // No ball
    __shm_buffer->set_circle_found(false);
    __shm_buffer->set_roi(0, 0, 0, 0);
    __ball_rel->reset();
  }


  // Go through all ROIs, filter and recognize shapes
  for (__r = __rois->begin(); __r != __rois->end(); ++__r) {

    if ( __generate_output ) {
      logger->log_debug(name(), "ROI: start: (%u, %u) extent: %u x %u",
			(*__r).start.x, (*__r).start.y,
			(*__r).width, (*__r).height);
    }

    // Edge detection
    __hv_search->set_src_buffer(__cam->buffer(), &(*__r));
    __hv_search->set_dst_buffer(__buffer, &(*__r));
    __hv_search->apply();
    
    // Try to detect ball shape
    if ((*__r).hint == H_BALL) {

      if ( __shrinker != NULL ) {
	// If you have to deal with reflections on the floor, use the shrinker.
	// If not (e.g. if there is a carpet), it is better to switch it off
	// (more accurate balls).
	__shrinker->setFilteredBuffer( __buffer );
	__shrinker->shrink( &(*__r) );
      }

      __ball_visible = detect_ball_and_update_models( &(*__r) );

      if ( ! __already_fetched_pantilt ) {
	// This will block until data is available! Since we called startGetPanTilt before this
	// should not block for too long. The pan/tilt unit takes about 25ms to supply data
	fetch_pantilt_and_update_models();
	__already_fetched_pantilt = true;
      }

      if ( __ball_visible ) {
	// a ball/round shape was detected

	// relative position update
	if ( __generate_output ) {
	  __ball_rel->calc_unfiltered();
	  logger->log_debug(name(), "RelPosU: X: %f  Y: %f  Dist: %f  Bearing: %f",
			    __ball_rel->getX(), __ball_rel->getY(),
			    __ball_rel->getDistance(), __ball_rel->getBearing());
	}

	__ball_rel->calc();
	
	if ( __generate_output ) {
	  logger->log_debug(name(), "RelPos: X: %f  Y: %f  Dist: %f  Bearing: %f",
			    __ball_rel->getX(), __ball_rel->getY(),
			    __ball_rel->getDistance(), __ball_rel->getBearing());
	  logger->log_debug(name(), "GlobPos: X: %f  Y: %f",
			    __ball_glob->getX(), __ball_glob->getY());
	}

	// we only process a single ROI for time constraint reasons
	break;
      }
    } // end is ball
  } // end for rois


  // write data to interface
  if (__ball_visible) {
    __ball_interface->set_visible(true);
    __ball_interface->set_relative_x( __ball_rel->getX() );
    __ball_interface->set_relative_y( __ball_rel->getY() );
    __ball_interface->set_distance( __ball_rel->getDistance() );
    __ball_interface->set_yaw( __ball_rel->getBearing() );
  } else {
    __ball_interface->set_visible(false);
  }
}

