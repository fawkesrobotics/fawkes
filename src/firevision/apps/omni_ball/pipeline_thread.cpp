
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

#include <string>

using namespace std;

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
  delete classifier;
}


/** Initialize the pipeline thread.
 * Camera is requested, config parameters are obtained from the config db, and
 * other miscellaneous init stuff is done here.
 */
void
FvOmniBallPipelineThread::init()
{
  // -- config --
  // camera, mirror and color model
  string camera;
  string mirror_file;
  string colormap_file;
  try
    {
      camera = config->get_string("/firevision/omni/camera");
      mirror_file = config->get_string("/firevision/omni/mirror");
      colormap_file = config->get_string("/firevision/omni/ball/colormap");
    }
  catch (Exception &e)
    {
      e.append("OmniBallPipeline::init() failed since config parameters are missing");
      throw;
    }

  // camera
  try 
    {
      cam = vision_master->register_for_camera( camera.c_str(), this );
    } 
  catch (Exception& e) 
    {
      e.append("FvOmniBallPipelineThread::init() failed since no camera is specified");
      throw;
    }

  // interface
  try
    {
      ball_interface = blackboard->open_for_writing<ObjectPositionInterface>("OmniBall");
      ball_interface->set_object_type( ObjectPositionInterface::BALL );
      ball_interface->set_visible(false);
      ball_interface->write();
    }
  catch (Exception &e)
    {
      vision_master->unregister_thread(this);
      delete cam;
      cam = NULL;
      e.append("Opening ball interface for writing failed");
      throw;
    }

  string mirror_calib_path = string(CONFDIR) + "/firevision/mirror/" + mirror_file;
  mirror = new Bulb( mirror_calib_path.c_str(), "omni-ball-bulb",
		     true /* destroy on delete */ );

  string colormap_path = string(CONFDIR) + "/firevision/colormaps/" + colormap_file;
  cm = new ColorModelLookupTable(colormap_path.c_str(), 256 /* lut_width */, 256 /* lut_height */,
				 "omni-ball-colormap", true /* destroy on delete */ );
  
  // image properties
  img_width = cam->pixel_width();
  img_height = cam->pixel_height();
  cspace_from = cam->colorspace();
  
  buffer_size = colorspace_buffer_size(cspace_to, img_width, img_height);
  try
    {
      shm_buffer = new SharedMemoryImageBuffer("omni-ball-processed", cspace_to,
					       img_width, img_height);
      if (! shm_buffer->is_valid() )
	{
	  throw Exception("Shared memeor segment not valid");
	}
    }
  catch (Exception& e)
    {
      vision_master->unregister_thread(this);
      delete cam;
      cam = NULL;
      blackboard->close(ball_interface);
      ball_interface = NULL;
      throw;
    }
  buffer = shm_buffer->buffer();
  
  
  // scanline model
  scanline = new ScanlineGrid( img_width, img_height, 5, 5 );
  
  // position model
  rel_pos = new OmniRelative(mirror);

  // classifier
  classifier = new SimpleColorClassifier(scanline, cm, 0, 30);
}


/** Thread finalization. */
void
FvOmniBallPipelineThread::finalize()
{
  try
    {
      ball_interface->set_visible(false);
      ball_interface->write();
      blackboard->close(ball_interface);
    }
  catch (Exception &e)
    {
      e.append("Closing ball interface failed");
      throw;
    }

  logger->log_debug(name(), "Unregistering from vision master");
  vision_master->unregister_thread(this);
  delete cam;
  delete shm_buffer;
  shm_buffer = NULL;
}


/** A new image is retrieved from the camera and the classifier looks for a ball
 * in the image */
void
FvOmniBallPipelineThread::loop()
{
  cam->capture();
  convert(cspace_from, cspace_to, cam->buffer(), buffer, img_width, img_height);
  cam->dispose_buffer();

  ball_visible = false;

  // run classifier
  classifier->set_src_buffer( buffer, img_width, img_height );

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
	      classifier->get_mass_point_of_ball( &(*r), &mass_point );
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

	if ( rel_pos->isPosValid() ) { rel_pos->calc(); } 
	else { ball_visible = false; }
      }
    
    if ( ball_visible && (winner_roi != rois->end())) 
      {
	shm_buffer->set_circle_found( true );
	shm_buffer->set_circle( ball_image_x, ball_image_y, 10 );
	shm_buffer->set_roi( winner_roi->start.x,
			     winner_roi->start.y,
			     winner_roi->width,
			     winner_roi->height );
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
  else { ball_interface->set_visible(false); }

  ball_interface->write();
}

