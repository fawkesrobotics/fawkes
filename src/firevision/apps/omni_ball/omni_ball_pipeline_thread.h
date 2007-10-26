
/***************************************************************************
 *  omni_ball_pipeline_thread.cpp - Omni Ball Pipeline Thread
 *
 *  Created: Thu July 05 19:00:19 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *             2007  Daniel Beck
 *
 *  $Id: base_thread.cpp 344 2007-10-04 16:37:23Z tim $
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

#ifndef __FIREVISION_APPS_OMNIBALL_OMNIBALLPIPELINETHREAD_H_
#define __FIREVISION_APPS_OMNIBALL_OMNIBALLPIPELINETHREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/types.h>
#include <fvutils/base/roi.h>

class Camera;
class ScanlineModel;
class ColorModel;
class MirrorModel;
class ReallySimpleClassifier;
class RelativePositionModel;
class GlobalPositionModel;
class SharedMemoryImageBuffer;
class Scaler;

class FvOmniBallPipelineThread
: public Thread,
  public LoggingAspect,
  public VisionAspect,
  public ConfigurableAspect
{
 public:
  FvOmniBallPipelineThread();
  virtual ~FvOmniBallPipelineThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  std::string cfg_component;
  std::string cfg_path;

  Camera* cam;
  ScanlineModel* scanline;
  ColorModel* cm;
  MirrorModel* mirror;
  RelativePositionModel* rel_pos;
  GlobalPositionModel* glob_pos;
  ReallySimpleClassifier* classifier;
  SharedMemoryImageBuffer* shm_buffer;
  Scaler* scaler;

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

  std::list< ROI > *rois;
  std::list< ROI >::iterator r;
};

#endif /* __FIREVISION_APPS_OMNIBALL_OMNIBALLPIPELINETHREAD_H_ */
