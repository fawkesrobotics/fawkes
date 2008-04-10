
/***************************************************************************
 *  pipeline_thread.h - Front Pipeline Thread
 *
 *  Generated: Wed Jun 15 16:30:22 2005
 *  Ported to Fawkes FireVision: Sun Dec 09 23:40:04 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_APPS_FRONT_PIPELINE_THREAD_H_
#define __FIREVISION_APPS_FRONT_PIPELINE_THREAD_H_

#include <core/threading/thread.h>

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <aspect/blackboard.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/types.h>
#include <fvutils/base/roi.h>

#include <string>

class Camera;
class ScanlineModel;
class ColorModel;
class SimpleColorClassifier;
class SharedMemoryImageBuffer;
class ObjectPositionInterface;
class CameraControl;
class Shrinker;
class FrontBallRelativePos;
class GlobalFromRelativePos;
class FilterHVSearch;
class ShapeModel;

class FvFrontPipelineThread
: public Thread,
  public LoggingAspect,
  public VisionAspect,
  public ConfigurableAspect,
  public BlackBoardAspect
{
 public:
  FvFrontPipelineThread();
  virtual ~FvFrontPipelineThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  Camera          *__cam;
  CameraControl   *__camctrl;

  colorspace_t     __cspace_from;
  colorspace_t     __cspace_to;

  struct timeval   __data_taken_time;

  unsigned int              __img_width;
  unsigned int              __img_height;

  unsigned int              __buffer_size;
  unsigned char            *__buffer;
  unsigned char            *__buffer_src;
  SharedMemoryImageBuffer  *__shm_buffer;
  SharedMemoryImageBuffer  *__shm_buffer_src;

  ScanlineModel            *__scanlines;
  ColorModel               *__cm;
  ShapeModel               *__circle_model;
  FrontBallRelativePos     *__ball_rel;
  GlobalFromRelativePos    *__ball_glob;
  SimpleColorClassifier    *__classifier;
  Shrinker                 *__shrinker;

  ObjectPositionInterface  *__ball_interface;

  bool                      __ball_visible;
  bool                      __already_fetched_pantilt;

  FilterHVSearch           *__hv_search;

  // Classifier results and iterators
  std::list< ROI >         *__rois;
  std::list< ROI >::iterator __r;

  bool             __generate_output;
  bool             __show_velo_info;

  float        __cfg_ball_circumfer;
  std::string  __cfg_scanline_model;
  std::string  __cfg_shrinker_type;
  std::string  __cfg_colormap;
  unsigned int __cfg_colormap_width;
  unsigned int __cfg_colormap_height;
  float        __cfg_cam_height;
  float        __cfg_cam_pan;
  float        __cfg_cam_tilt;
  float        __cfg_cam_offset_x;
  float        __cfg_cam_offset_y;
  float        __cfg_cam_hor_va;
  float        __cfg_cam_ver_va;
  float        __cfg_field_length;
  float        __cfg_field_width;
  float        __cfg_field_border;
  unsigned int __cfg_grid_offset_x;
  unsigned int __cfg_grid_offset_y;

  unsigned int __cfg_rcd_max_failures;
  unsigned int __cfg_rcd_min_pixels;
  unsigned int __cfg_rcd_min_interpix_dist;
  unsigned int __cfg_rcd_max_dist_p4;
  unsigned int __cfg_rcd_max_dist_all;
  float        __cfg_rcd_hwratio;
  float        __cfg_rcd_hollowrate;
  unsigned int __cfg_rcd_max_runtime;

  /* private methods */
  void fetch_pantilt_and_update_models();
  bool detect_ball_and_update_models( ROI *roi );
};

#endif /* __FIREVISION_APPS_FRONT_PIPELINE_THREAD_H_ */
