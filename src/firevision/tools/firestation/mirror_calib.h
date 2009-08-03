
/***************************************************************************
 *  mirror_calib.h - Mirror calibration tool
 *
 *  Created: Fri Dec 07 18:34:50 2007
 *  Copyright  2007  Daniel Beck
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

#ifndef __FIREVISION_TOOLS_FIRESTATION_MIRROR_CALIB_H_
#define __FIREVISION_TOOLS_FIRESTATION_MIRROR_CALIB_H_

#include <geometry/hom_point.h>

#ifdef HAVE_BULB_CREATOR
#include <models/mirror/bulb/bulb_generator.h>
class Bulb;
class BulbSampler;

class MirrorCalibTool : public BulbGeneratorProgressHandler
#else
class MirrorCalibTool
#endif
{
 public:
  MirrorCalibTool();
  MirrorCalibTool(unsigned int img_width, unsigned int img_height
		  /*, unsigned int num_dists, unsigned int num_oris*/);
  ~MirrorCalibTool();

  void start();
  void abort();
  void step(unsigned int x, unsigned int y);
  bool get_next(float* dist, float* ori);
  
  void eval(unsigned int x, unsigned int y, float* x_ret, float* y_ret);

  void load(const char* filename);
  void save(const char* filename);

  void set_img_dimensions(unsigned int width, unsigned int height);

  void set_dists(float dists[], unsigned int num_dists);
  void set_oris(float oris[], unsigned int num_oris);

  void setTotalSteps(unsigned int total_steps);
  void setProgress(unsigned int progress);
  void finished();
  
 private:
  unsigned int m_img_width;
  unsigned int m_img_height;

  unsigned int m_center_x;
  unsigned int m_center_y;

  unsigned int m_next_x;
  unsigned int m_next_y;
  unsigned int m_next_ori;

  static float m_sample_dist[];
  static float m_sample_ori[];

  unsigned int m_sample_step;
  unsigned int m_sample_dist_step;
  unsigned int m_sample_ori_step;
  unsigned int m_num_dists;
  unsigned int m_num_oris;

  fawkes::HomPoint m_next_sample_point;

  bool m_calib_done;
  bool m_step_two;

#ifdef HAVE_BULB_CREATOR
  Bulb* m_bulb;
  BulbSampler* m_sampler;
  BulbGenerator* m_generator;
#endif
};


#endif /*  __FIREVISION_TOOLS_IMAGE_VIEWER_MIRROR_CALIB_H_ */
