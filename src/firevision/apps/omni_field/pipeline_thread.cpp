
/***************************************************************************
 *  omni_field_pipeline_thread.cpp - Omni Field Pipeline Thread
 *
 *  Created: Thu Nov 01 17:59:40 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
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

#include <apps/omni_field/pipeline_thread.h>

#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/readers/pnm.h>

#include <fvutils/draw/drawer.h>

#include <models/mirror/mirrormodel.h>
#include <models/scanlines/star.h>
#include <models/color/lookuptable.h>
#include <models/mirror/bulb.h>
#include <models/relative_position/omni_relative.h>

#include <cams/camera.h>
#include <interfaces/object.h>
#include <utils/math/angle.h>
#include <utils/system/hostinfo.h>

#include <stdlib.h>
#include <cstdio>

using namespace std;

/** @class FvOmniFieldPipelineThread <apps/omni_field/pipeline_thread.h>
 * Field detector thread.
 *
 * @author Daniel Beck
 */


/** Constructor. */
FvOmniFieldPipelineThread::FvOmniFieldPipelineThread()
  : Thread("FvOmniFieldThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC)
{
  m_scanline_model = 0;
  m_colormodel = 0;
  m_mirror = 0;
  m_rel_pos = 0;
  m_shm_buffer = 0;
  m_mask = 0;
  m_obstacle_interfaces = 0;

  m_cfg_prefix = "/firevision/omni/field/";
  m_cfgfile_prefix = "../cfg/firevision/";

  m_cspace_to = YUV422_PLANAR;

  m_drawer = 0;
}


/** Destructor. */
FvOmniFieldPipelineThread::~FvOmniFieldPipelineThread()
{
}


/** Initialize the pipeline thread.
 * Camera is requested, config parameters are obtained from the config db, and
 * other miscellaneous init stuff is done here.
 */
void
FvOmniFieldPipelineThread::init()
{
  // camera
  try
    {
      m_camera = vision_master->register_for_camera( config->get_string("/firevision/omni/camera").c_str(), this );
    }
  catch (Exception& e)
    {
      e.append("FvOmniFieldPipelineThread::init() failed since no camera is specified");
      throw;
    }

  m_img_width = m_camera->pixel_width();
  m_img_height = m_camera->pixel_height();
  m_cspace_from = m_camera->colorspace();

  // config values
  unsigned int num_rays;
  unsigned int radius_incr;
  unsigned int dead_radius;
  unsigned int max_radius;
  unsigned int margin;
  char* colormodel_file = NULL;
  char* mirror_file = NULL;
  char* mask_file = NULL;

  // omni-field config values
  try
    {
      num_rays        = config->get_uint( (m_cfg_prefix + string("num_rays")).c_str() );
      radius_incr     = config->get_uint( (m_cfg_prefix + string("radius_incr")).c_str() );
      dead_radius     = config->get_uint( (m_cfg_prefix + string("dead_radius")).c_str() );
      max_radius      = config->get_uint( (m_cfg_prefix + string("max_radius")).c_str() );
      margin          = config->get_uint( (m_cfg_prefix + string("margin")).c_str() );
      colormodel_file = strdup( ( m_cfgfile_prefix + config->get_string( (m_cfg_prefix + string("colormap")).c_str() ) ).c_str() );
    }
  catch (Exception &e)
    {
      free(colormodel_file);
      delete m_camera;
      e.append("OmniFieldPipelineThread::init() failed since config parameters are missing");
      throw;
    }

  // mirror config values
  try
    {
      mirror_file = strdup( ( m_cfgfile_prefix + config->get_string( "/firevision/omni/mirror" ) ).c_str() );
    }
  catch (Exception &e)
    {
      free(mirror_file);
      logger->log_warn(name(), "No mirror specified in config. Using default.");
      HostInfo hi;
      asprintf(&mirror_file, "%s%s.mirror", m_cfgfile_prefix.c_str(), hi.short_name());
    }

  // mask config values
  try
    {
      mask_file = strdup( ( m_cfgfile_prefix + config->get_string( "/firevision/omni/mask" ) ).c_str() );
    }
  catch (Exception &e)
    {
      free(mask_file);
      logger->log_warn(name(), "No mask specified in config. Using default.");
      HostInfo hi;
      asprintf(&mask_file, "%s%s_mask.pnm", m_cfgfile_prefix.c_str(), hi.short_name());
    }

  // mask
  logger->log_debug(name(), "Loading mask from file %s", mask_file);
  PNMReader reader(mask_file);
  free(mask_file);
  m_mask = (unsigned char*) malloc( colorspace_buffer_size( reader.colorspace(),
				reader.pixel_width(), reader.pixel_height() ) );
  reader.set_buffer(m_mask);
  reader.read();

  // mirror
  logger->log_debug(name(), "Creating mirror model from file %s", mirror_file);
  m_mirror = new Bulb( mirror_file,
		       "omni-mirror",
		       true /* destroy on delete */ );
  free(mirror_file);

  // scanline model
  cart_coord_t center;
  center = m_mirror->getCenter();

  m_scanline_model = new ScanlineStar( m_img_width, m_img_height,
				       center.x, center.y,
				       num_rays, radius_incr,
				       m_mask,
				       dead_radius, max_radius,
				       margin );

  // color model
  logger->log_debug(name(), "Creating colormodel from colormap %s", colormodel_file);
  m_colormodel = new ColorModelLookupTable( colormodel_file,
					    "omni-field-colormap",
					    true /* destroy on delete */ );
  free(colormodel_file);

  // interfaces
  m_num_interfaces = m_scanline_model->num_rays();
  m_obstacle_interfaces = (ObjectPositionInterface**)malloc(m_num_interfaces * sizeof(ObjectPositionInterface*));
  unsigned int i;
  unsigned int flags = 
    ObjectPositionInterface::FLAG_HAS_RELATIVE_CARTESIAN |
    ObjectPositionInterface::FLAG_HAS_EXTENT |
    ObjectPositionInterface::FLAG_HAS_CIRCULAR_EXTENT;

  try
    {
      for ( i = 0; i < m_num_interfaces; ++i)
	{
	  char* id;
	  asprintf(&id, "OmniObstacle%d", i);
	  m_obstacle_interfaces[i] = blackboard->open_for_writing<ObjectPositionInterface>(id);
	  m_obstacle_interfaces[i]->set_object_type(ObjectPositionInterface::TYPE_OTHER);
	  m_obstacle_interfaces[i]->set_flags(flags);
	  m_obstacle_interfaces[i]->write();
	  free(id);
	}
    }
  catch (Exception &e)
    {
      for ( unsigned int j = 0; j < i; ++j)
	{
	  blackboard->close( m_obstacle_interfaces[j] );
	}
      delete m_scanline_model;
      delete m_mirror;
      free(m_mask);
      free(m_obstacle_interfaces);
      delete m_camera;
      e.append("%s: Opening object position interface for writing failed", name());
      throw;
    }

  // shm buffer
  m_buffer_size = colorspace_buffer_size(m_cspace_to, m_img_width, m_img_height);
  m_shm_buffer = new SharedMemoryImageBuffer( "omni-field-processed",
				m_cspace_to, m_img_width, m_img_height );
  m_buffer = m_shm_buffer->buffer();

  // position models
  m_rel_pos = new OmniRelative(m_mirror);

  // drawer
  m_drawer = new Drawer();
  m_drawer->setBuffer(m_buffer, m_img_width, m_img_height);
  m_drawer->setColor(127, 220, 220);
}


/** Thread finalization. */
void
FvOmniFieldPipelineThread::finalize()
{
  try
    {
      for (unsigned int i = 0; i < m_num_interfaces; ++i)
	{
	  blackboard->close( m_obstacle_interfaces[i] );
	}
    }
  catch (Exception &e)
    {
      e.append("%s: Closing object position interface failed", name());
      throw;
    }
  logger->log_debug(name(), "Unregistering form vision master");
  vision_master->unregister_thread(this);

  delete m_camera;
  free(m_obstacle_interfaces);
  delete m_colormodel;
  delete m_mirror;
  delete m_shm_buffer;
  free(m_mask);
  delete m_scanline_model;
  delete m_rel_pos;
  delete m_drawer;

  m_camera = 0;
  m_obstacle_interfaces = 0;
  m_colormodel = 0;
  m_mirror = 0;
  m_shm_buffer = 0;
  m_mask = 0;
  m_scanline_model = 0;
  m_rel_pos = 0;
  m_drawer = 0;
}


/** A new image is retrieved from the camera and the classifier looks
 * for a ball in the image */
void
FvOmniFieldPipelineThread::loop()
{
  m_camera->capture();
  if ( 0 == m_camera->buffer() ) { return; }

  convert( m_cspace_from, m_cspace_to, m_camera->buffer(), m_buffer,
	   m_img_width, m_img_height );
  m_camera->dispose_buffer();

  m_scanline_model->reset();

  // reset interfaces
  for (unsigned int i = 0; i < m_num_interfaces; ++i)
    {
      m_obstacle_interfaces[i]->set_visible(false);
      m_obstacle_interfaces[i]->write();
    }

  m_num_whites = 0;

  // search for obstacles
  unsigned int index;
  while ( !m_scanline_model->finished() )
    {
      point_t cur_point;
      cur_point.x = (*m_scanline_model)->x;
      cur_point.y = (*m_scanline_model)->y;

      hint_t object_type;
      bool _is_field = is_field(&cur_point, &object_type);

      // neither H_FIELD nor H_LINE
      if (!_is_field)
	{
	  unsigned int cur_x = cur_point.x;
	  unsigned int cur_y = cur_point.y;

	  // determine relative world coordinates
	  m_rel_pos->set_center(cur_x, cur_y);
	  m_rel_pos->calc_unfiltered();

	  index = m_scanline_model->ray_index();

	  // write data to interface
	  m_obstacle_interfaces[index]->set_visible( true );
	  m_obstacle_interfaces[index]->set_extent_x( 0.1 );
	  m_obstacle_interfaces[index]->set_relative_x( m_rel_pos->get_x() );
	  m_obstacle_interfaces[index]->set_relative_y( -m_rel_pos->get_y() );
	  m_obstacle_interfaces[index]->write();

	  // draw circle in image
	  m_drawer->drawCircle(cur_x, cur_y, 4);

	  // continue with next ray
	  m_scanline_model->skip_current_ray();
	}

      // H_LINE
      else if (object_type == H_LINE)
	{
	  if (m_last_seen_object == H_LINE)
	    {
	      if (4 < ++m_num_whites)
		// white obstacle
		{
		  m_rel_pos->set_center(m_first_white.x, m_first_white.y);
		  m_rel_pos->calc_unfiltered();

		  index = m_scanline_model->ray_index();
		  
		  m_obstacle_interfaces[index]->set_visible( true );
		  m_obstacle_interfaces[index]->set_extent_x( 0.1 );
		  m_obstacle_interfaces[index]->set_relative_x( m_rel_pos->get_x() );
		  m_obstacle_interfaces[index]->set_relative_y( -m_rel_pos->get_y() );
		  m_obstacle_interfaces[index]->write();

		  // draw circle in image
		  m_drawer->drawCircle(m_first_white.x, m_first_white.y, 4);
		  
		  // continue with next ray
		  m_scanline_model->skip_current_ray();

		  m_last_seen_object = H_UNKNOWN;
		  m_num_whites = 0;
		}
	    }
	  else
	    { 
	      m_last_seen_object = H_LINE;
	      m_first_white = cur_point; 

	      // continue with next scanline point
	      ++(*m_scanline_model); 
	    }
	}
      
      // H_FIELD
      else
	{ 
	  // continue with next scanline point
	  ++(*m_scanline_model);
	}
    }
}


/** Determines whether it's justified to classify a pixel at
 * a given pixel as field.
 * @param point the pixel coordinates
 * @param object_type the type of the object the current pixel has been classified as
 * @return true if pixel at given coordinate is either classified as H_FIELD or as
 *         H_LINE
 */
bool
FvOmniFieldPipelineThread::is_field(point_t* point, hint_t* object_type)
{
  unsigned int x_center;
  unsigned int y_center;
  int x;
  int y;
  unsigned char yp = 0;
  unsigned char up = 0;
  unsigned char vp = 0;
  color_t color;
  short offsets[3] = {-2, 0, 2};
  unsigned int num_offsets = 3;
  unsigned int num_green = 0;
  unsigned int num_white = 0;
  unsigned int num_pixels = num_offsets * num_offsets;

  x_center = point->x; 
  y_center = point->y;

  for ( unsigned int i = 0; i < num_offsets; ++i )
    {
      for ( unsigned int j = 0; j < num_offsets; ++j )
	{
	  x = x_center + offsets[i];
	  y = y_center + offsets[j];

	  if ( x < 0 || (unsigned int)x >= m_img_width ||
	       y < 0 || (unsigned int)y >= m_img_height )
	    {
	      --num_pixels;
	      continue;
	    }

	  YUV422_PLANAR_YUV(m_buffer, m_img_width, m_img_height,
			    x, y, yp, up, vp);

	  color = m_colormodel->determine(yp, up, vp);

	  if ( C_GREEN == color ) 
	    { ++num_green; }
	  if ( C_WHITE == color )
	    { ++num_white; }
	}
    }

  float ratio = (num_green + num_white) / float(num_pixels);

  bool is_field = false;
  if (object_type)
    { *object_type = H_UNKNOWN; }

  if (ratio > 0.8)
    { 
      is_field = true;
      
      if (object_type)
	{
	  if (num_green > num_white)
	    { *object_type = H_FIELD; }
	  else
	    { *object_type = H_LINE; }
	}
    }

  return is_field;
}
