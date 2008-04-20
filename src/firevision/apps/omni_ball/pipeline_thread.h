
/***************************************************************************
 *  omni_ball_pipeline_thread.cpp - Omni Ball Pipeline Thread
 *
 *  Created: Thu July 05 19:00:19 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *             2007       Daniel Beck
 *             2005       Martin Heracles
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

#ifndef __FIREVISION_APPS_OMNI_BALL_PIPELINE_THREAD_H_
#define __FIREVISION_APPS_OMNI_BALL_PIPELINE_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <aspect/blackboard.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/types.h>
#include <fvutils/base/roi.h>

class Camera;
class ScanlineModel;
class ColorModel;
class MirrorModel;
class SimpleColorClassifier;
class RelativePositionModel;
class SharedMemoryImageBuffer;
class ObjectPositionInterface;
class Drawer;

class FvOmniBallPipelineThread
: public Thread,
  public LoggingAspect,
  public VisionAspect,
  public ConfigurableAspect,
  public BlackBoardAspect
{
 public:
  FvOmniBallPipelineThread();
  virtual ~FvOmniBallPipelineThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  Camera* cam;
  ScanlineModel* scanline;
  ColorModel* cm;
  MirrorModel* mirror;
  RelativePositionModel* rel_pos;
  SimpleColorClassifier* classifier;
  SharedMemoryImageBuffer* shm_buffer;

  size_t buffer_size;
  unsigned char* buffer;

  unsigned int img_width;
  unsigned int img_height;

  colorspace_t cspace_from;
  colorspace_t cspace_to;

  bool ball_visible;
  unsigned int  ball_image_x;
  unsigned int  ball_image_y;
  cart_coord_t  mass_point;
  float         min_dist;

  ObjectPositionInterface* ball_interface;

  std::list< ROI > *rois;
  std::list< ROI >::iterator r;

  char* cfgfile_prefix;

  Drawer* drawer;
};

#endif /* __FIREVISION_APPS_OMNI_BALL_PIPELINE_THREAD_H_ */
