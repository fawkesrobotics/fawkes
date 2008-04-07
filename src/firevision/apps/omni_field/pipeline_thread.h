
/***************************************************************************
 *  omni_field_pipeline_thread.cpp - Omni Field Pipeline Thread
 *
 *  Created: Thu Nov 01 17:56:12 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __FIREVISION_APPS_OMNIFIELD_PIPELINE_THREAD_H_
#define __FIREVISION_APPS_OMNIFIELD_PIPELINE_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <aspect/blackboard.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/types.h>

class Camera;
class ScanlineStar;
class ColorModel;
class MirrorModel;
class RelativePositionModel;
class SharedMemoryImageBuffer;
class ObjectPositionInterface;
/*
// DEBUG
class Drawer;
class Writer;
*/

class FvOmniFieldPipelineThread
: public Thread,
  public LoggingAspect,
  public VisionAspect,
  public ConfigurableAspect,
  public BlackBoardAspect
{
 public:
  FvOmniFieldPipelineThread();
  virtual ~FvOmniFieldPipelineThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  bool is_field(point_t* point);

  std::string m_cfg_prefix;
  std::string m_cfgfile_prefix;

  Camera* m_camera;
  ScanlineStar* m_scanline_model;
  ColorModel* m_colormodel;
  MirrorModel* m_mirror;
  RelativePositionModel* m_rel_pos;
  SharedMemoryImageBuffer* m_shm_buffer;
  ObjectPositionInterface** m_obstacle_interfaces;

  size_t m_buffer_size;
  unsigned char* m_buffer;

  unsigned char* m_mask;

  unsigned int m_img_width;
  unsigned int m_img_height;

  colorspace_t m_cspace_from;
  colorspace_t m_cspace_to;

  unsigned int  m_field_image_x;
  unsigned int  m_field_image_y;
  cart_coord_t  m_mass_point;
  float         m_min_dist;

  unsigned int m_num_interfaces;

  /*
  // DEBUG
  Drawer* m_drawer;
  Writer* m_writer;
  */
};

#endif /* __FIREVISION_APPS_OMNIFIELD_PIPELINE_THREAD_H_ */
