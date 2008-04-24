
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

#include <geometry/hom_point.h>
#include <geometry/hom_vector.h>
#include <geometry/hom_polar.h>
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

  m_num_non_field = 0;
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
  m_obstacles.clear();

  m_num_whites = 0;

  // search for obstacles
  while ( !m_scanline_model->finished() )
    {
      point_t cur_point;
      cur_point.x = (*m_scanline_model)->x;
      cur_point.y = (*m_scanline_model)->y;

      hint_t object_type;
      bool _is_field = is_field(&cur_point, &object_type);

      // neither H_FIELD nor H_LINE nor H_BALL
      if ( !_is_field )
	{ 
	  ++m_num_non_field;

	  if (m_num_non_field > 4)
	    {
	      unsigned int obstacle_x = m_last_field.x;
	      unsigned int obstacle_y = m_last_field.y;
	      
	      // determine relative world coordinates
	      m_rel_pos->set_center(obstacle_x, obstacle_y);
	      m_rel_pos->calc_unfiltered();
	      
	      unsigned int index;
	      index = m_scanline_model->ray_index();
	      
	      // save detected obstacles for post-processing
	      HomPolar obstacle( m_rel_pos->get_distance(), m_rel_pos->get_bearing() );
	      m_obstacles[obstacle.phi()] = obstacle;

	      // continue with next ray
	      m_scanline_model->skip_current_ray();
	      
	      // DEBUG
	      cart_coord_t center = m_mirror->getCenter();
	      m_drawer->setColor(0, 127, 127);
	      m_drawer->drawLine(center.x, center.y, obstacle_x, obstacle_y);
	    }
	  else
	    { ++(*m_scanline_model); }
	}
      else
	{
	  m_num_non_field = 0;
	  m_last_field = cur_point;

	  // continue with next scanline point
	  ++(*m_scanline_model);
	}
    }

  if (m_obstacles.size() == 0)
    // no obstacles detected
    {
      for (unsigned int i = 0; i < m_num_interfaces; ++i)
	{ 
	  m_obstacle_interfaces[i]->set_visible(false);
	  m_obstacle_interfaces[i]->write();
	}

      return;
    }

  else if (m_obstacles.size() == 1)
    // exactly 1 obstacle was detected
    {
      std::map<float, HomPolar>::iterator i;
      i = m_obstacles.begin();
      HomPolar p = i->second;
      m_obstacle_interfaces[0]->set_visible(true);
      m_obstacle_interfaces[0]->set_distance( p.r() );
      m_obstacle_interfaces[0]->set_bearing( p.phi() );
      m_obstacle_interfaces[0]->write();

      for (unsigned int i = 1; i < m_num_interfaces; ++i)
	{
	  m_obstacle_interfaces[i]->set_visible(false);
	  m_obstacle_interfaces[i]->write();
	}

      return;
    }
  
  // find start of first cluster
  std::map<float, HomPolar>::iterator loit;
  std::map<float, HomPolar>::iterator coit;
  std::map<float, HomPolar>::iterator start_oit = m_obstacles.begin();
  loit = m_obstacles.end();
  --loit;
  for ( coit = m_obstacles.begin(); coit != m_obstacles.end(); ++coit )
    {
      float dx = coit->second.x() - loit->second.x();
      float dy = coit->second.y() - loit->second.y();
      float delta = sqrt( dx * dx + dy * dy ); 

      loit = coit;

      if ( delta > 0.5 )
	{
	  start_oit = coit;
	  break;
	}
    }

  typedef std::vector<HomPolar> Cluster;
  std::vector<Cluster> clusters;
  Cluster cur_cluster;
  cur_cluster.push_back(start_oit->second);
  clusters.push_back(cur_cluster);
  
  // It is assumed that at the beginning of the while-loop we are at the first point
  // of a cluster, i.e., it is visible and the point before is not visible or it is
  // too far away.
  HomPolar cluster_start = start_oit->second;
  coit = start_oit;
  ++coit;
  while ( coit != start_oit )
    {
      Cluster  cur_cluster  = clusters.back();
      HomPolar cur_obstacle = coit->second;

      // compute the center of the obstacles in the current cluster and the current obstacle
      HomPolar center;
      float avg_r = cur_obstacle.r();
      for (unsigned int c = 0; c < cur_cluster.size(); ++c)
	{ avg_r += cur_cluster[c].r(); }
      avg_r /= float(cur_cluster.size() + 1);

      float avg_phi = fabs(cur_obstacle.phi() - cluster_start.phi()) / 2.0 + cluster_start.phi();

      center.r(avg_r);
      center.phi(avg_phi);
      
      float dist_to_center;
      unsigned int c = 0;
      bool start_new_cluster = false;
      
      // Check distance of all points in the cluster to the point in the middle between
      // the first point of the cluster and the current point. If all points lie within
      // a certain radius around the center the new point is added to the cluster.
      // Otherwise a new cluster is started.
      while ( c < cur_cluster.size() && !start_new_cluster)
	{
	  dist_to_center = HomVector(cur_cluster[c] - center).length();
	  if (dist_to_center > 0.3)
	    {
	      ObjectPositionInterface* opi;
	      opi = m_obstacle_interfaces[ clusters.size() - 1 ];
	      opi->set_visible(true);
	      opi->set_relative_x( center.x() );
	      opi->set_relative_y( center.y() );
	      opi->set_bearing( center.phi() );
	      opi->set_distance( center.r() );
	      HomPolar cluster_end = cur_cluster.back();
	      opi->set_extent_x( HomVector(cluster_end - cluster_start).length() / 2.0 );
	      
	      Cluster new_cluster;
	      new_cluster.push_back(cur_obstacle);
	      clusters.push_back(new_cluster);
	      cluster_start = cur_obstacle;
	      
	      start_new_cluster = true;
	    }
	  ++c;
	}

      if ( !start_new_cluster )
	{
	  cur_cluster.push_back(cur_obstacle);
	}

      if ( ++coit == m_obstacles.end() )
	{ coit = m_obstacles.begin(); }
    }

  for (unsigned int i = 0; i < m_num_interfaces; ++i)
    {
      if ( i >= clusters.size() )
	{ m_obstacle_interfaces[i]->set_visible(false); }

      m_obstacle_interfaces[i]->write();
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

  short offsets[3] = {-4, 0, 4};
  unsigned int num_offsets = 3;
  unsigned int num_pixels  = num_offsets * num_offsets;

  unsigned int num_green  = 0;
  unsigned int num_white  = 0;
  unsigned int num_orange = 0;

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

	  switch(color)
	    {
	    case C_GREEN:
	      ++num_green;
	      break;

	    case C_WHITE:
	      ++num_white;
	      break;
	      
	    case C_ORANGE:
	      ++num_orange;
	      break;

	    default:
	      break;
	    }
	}
    }

  float ratio = (num_green + num_white + num_orange) / float(num_pixels);

  bool is_field;

  if (ratio > 0.6)
    { 
      if (object_type)
	{
	  *object_type = H_FIELD;
	  unsigned int max = num_green;

	  if (num_white > max)
	    { *object_type = H_LINE; }
	  
	  if (num_orange > max)
	    { *object_type = H_BALL; }
	}

      is_field = true;      
    }
  else
    {
      if (object_type)
	{ *object_type = H_UNKNOWN; }

      is_field = false;
    }


  return is_field;
}
