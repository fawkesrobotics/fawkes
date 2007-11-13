
/***************************************************************************
 *  pipeline_thread.cpp - Omni Ball Pipeline Thread
 *
 *  Created: Fri July 27 12:01:59 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *             2007       Daniel Beck
 *             2005       Martin Heracles
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

#include <apps/omni_ball/pipeline_thread.h>

#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/scalers/lossy.h>

#include <models/mirror/mirrormodel.h>
#include <models/relative_position/omni_relative.h>
#include <models/global_position/omni_global.h>
#include <models/scanlines/grid.h>
#include <models/color/lookuptable.h>
#include <models/mirror/bulb.h>

#include <classifiers/simple.h>
#include <cams/camera.h>
#include <interfaces/object.h>

#include <stdlib.h>
#include <cstdio>


/** @class FvOmniBallPipelineThread <apps/omni_ball/pipeline_thread.h>
 * Ball detector thread.
 *
 * @author Tim Niemueller (Base)
 * @author Daniel Beck
 */


/** Constructor. */
FvOmniBallPipelineThread::FvOmniBallPipelineThread()
  : Thread("FvOmniBallThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC)
{
  scanline = NULL;
  cm = NULL;
  mirror = NULL;
  rel_pos = NULL;
  glob_pos = NULL;
  classifier = NULL;
  shm_buffer = NULL;
  ball_interface = NULL;

  cspace_to = YUV422_PLANAR;
}


/** Destructor. */
FvOmniBallPipelineThread::~FvOmniBallPipelineThread()
{
  delete scanline;
  delete cm;
  delete mirror;
  delete rel_pos;
  delete glob_pos;
  delete classifier;
  delete shm_buffer;
}


/** Initialize the pipeline thread.
 * Camera is requested, config parameters are obtained from the config db, and
 * other miscellaneous init stuff is done here.
 */
void
FvOmniBallPipelineThread::init()
{
  try 
    {
      cam = vision_master->register_for_camera( config->get_string("/omnivision/camera").c_str(), this );
    } 
  catch (Exception& e) 
    {
      e.append("FvOmniBallPipelineThread::init() failed since no camera is specified");
      throw;
    }

  img_width = cam->pixel_width();
  img_height = cam->pixel_height();
  cspace_from = cam->colorspace();
  
  // shm buffer
  // TODO: image dimensions are temporally scaled down by a factor of two
  // until we have appropriate viewing tools
  buffer_size = colorspace_buffer_size(cspace_to, img_width, img_height);
  shm_buffer = new SharedMemoryImageBuffer("omni-processed",
					   cspace_to, img_width/2, img_height/2);
  // TODO: working on the camera buffer...
  buffer = /*shm_buffer*/cam->buffer();
  
  // mirror
  char* bulb_matrix_file;

  try
    {
      bulb_matrix_file = strdup( config->get_string("/omnivision/mirrormodel").c_str() );
    }
  catch (Exception &e)
    {
      e.append("OmniBallPipeline::init() failed since config parameters are missing");
      throw;
    }

  mirror = new Bulb( bulb_matrix_file, "omni-ball-bulb",
		     true /* destroy on delete */ );
  
  free(bulb_matrix_file);

  // scanline model
  cart_coord_t center;
  center = mirror->getCenter();

  scanline = new ScanlineGrid( img_width, img_height, 5, 5 );
  
  // color model
  char* lut_file;

  try
    {
      lut_file = strdup( config->get_string("/omnivision/ball/colormap").c_str() );
    }
  catch (Exception &e)
    {
      e.append("OmniBallPipeline>::init() failed since config parameters are missing");
      throw;
    }

  cm = new ColorModelLookupTable(lut_file, 256 /* lut_width */, 256 /* lut_height */,
				 "omni-ball-colormap",
				 true /* destroy on delete */ );

  free(lut_file);
  
  // position models
  rel_pos = new OmniRelative(mirror);
  glob_pos = new OmniGlobal(mirror);

  // classifier
  classifier = new ReallySimpleClassifier(img_width, img_height, scanline, cm, 0, 30);

  // TODO: see above
  scaler = new LossyScaler();
  scaler->set_original_dimensions(cam->pixel_width(), cam->pixel_height());
  scaler->set_scaled_dimensions(cam->pixel_width() / 2, cam->pixel_height() / 2);
  scaler->set_scale_factor(0.5);

  // interface
  try
    {
      ball_interface = interface_manager->open_for_writing<ObjectPositionInterface>("OmniBall");
      ball_interface->set_object_type( ObjectPositionInterface::BALL );
    }
  catch (Exception &e)
    {
      e.append("Opening ball interface for writing failed");
      throw;
    }
}


/** Thread finalization. */
void
FvOmniBallPipelineThread::finalize()
{
  try
    {
      interface_manager->close(ball_interface);
    }
  catch (Exception &e)
    {
      e.append("Closing ball interface failed");
      throw;
    }

  logger->log_debug(name(), "Unregistering from vision master");
  vision_master->unregister_thread(this);
  delete cam;
}


/** A new image is retrieved from the camera and the classifier looks for a ball
 * in the image */
void
FvOmniBallPipelineThread::loop()
{
  cam->capture();
  // TODO: see above
  //  convert(cspace_from, cspace_to, cam->buffer(), buffer, img_width, img_height);
  scaler->set_original_buffer(cam->buffer());
  scaler->set_scaled_buffer(shm_buffer->buffer());
  scaler->scale();
  cam->dispose_buffer();

  ball_visible = false;

  // run classifier
  classifier->setSrcBuffer( buffer );

  rois = classifier->classify();

  // post-process ROIs
  if (rois->empty()) 
    {
      logger->log_warn(name(), "Could not find any ROIs in image");
    } 
  else 
    {
      // if we have at least one ROI 
      ball_visible = true;
      
      // find the ball candidate that is closest to the robot
      min_dist = 1000000.f;
      // for each ROI
      std::list< ROI >::iterator winner_roi = rois->end();
      for (r = rois->begin(); r != rois->end(); r++)
	{
	  // if ROI contains ball
	  if (r->hint == H_BALL)
	    {
	      // calculate mass point of ball
	      classifier->getMassPointOfBall( &(*r), &mass_point );
	      // update ball position
	      rel_pos->setCenter( mass_point.x, mass_point.y );
	      rel_pos->calc_unfiltered();
	      
	      if (rel_pos->getDistance() < min_dist) 
		{
		  min_dist = rel_pos->getDistance();
		  ball_image_x = mass_point.x;
		  ball_image_y = mass_point.y;
		  winner_roi = r;
		}
	    }
	}

    if ( ball_visible ) 
      {
	rel_pos->setCenter( ball_image_x, ball_image_y );

	if ( rel_pos->isPosValid() ) 
	  {
	    rel_pos->calc();
	    glob_pos->setPositionInImage( ball_image_x, ball_image_y );
	    glob_pos->calc();
	  } 
	else 
	  {
	    ball_visible = false;
	  }
      }
    
    if ( ball_visible && (winner_roi != rois->end())) 
      {
	shm_buffer->set_circle_found( true );
	shm_buffer->set_circle( ball_image_x, ball_image_y, 10 );
	// TODO: see above
	shm_buffer->set_roi( winner_roi->start.x/2,
			     winner_roi->start.y/2,
			     winner_roi->width/2,
			     winner_roi->height/2 );
      } 
    else 
      {
	shm_buffer->set_circle_found( false );
	shm_buffer->set_roi( 0, 0, 0, 0 );
	shm_buffer->set_circle( 0, 0, 0 );
      }
    
    // clean up
    rois->clear();
    delete rois;
    }

  // write data to interface
  if (ball_visible)
    {
      ball_interface->set_visible(true);
      ball_interface->set_relative_x( rel_pos->getX() );
      ball_interface->set_relative_y( rel_pos->getY() );
      ball_interface->set_distance( rel_pos->getDistance() );
      ball_interface->set_yaw( rel_pos->getBearing() );
    }
  else
    {
      ball_interface->set_visible(false);
    }
}

