
/***************************************************************************
 *  beams.h - Scanline model implementation: beams
 *
 *  Created: Tue Apr 17 20:59:58 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __FIREVISION_SCANLINE_BEAMS_H_
#define __FIREVISION_SCANLINE_BEAMS_H_

#include <fvmodels/scanlines/scanlinemodel.h>
#include <fvutils/base/types.h>

#include <vector>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ScanlineBeams : public ScanlineModel
{

 public:

  ScanlineBeams(unsigned int image_width, unsigned int image_height,
    unsigned int start_x, unsigned int start_y,
    unsigned int stop_y, unsigned int offset_y,
                bool distribute_start_x,
    float angle_from, float angle_range, unsigned int num_beams);

  fawkes::upoint_t    operator*();
  fawkes::upoint_t *  operator->();
  fawkes::upoint_t *  operator++();
  fawkes::upoint_t *  operator++(int);

  bool          finished();
  void          reset();
  const char *  get_name();
  unsigned int  get_margin();

  virtual void  set_robot_pose(float x, float y, float ori) {}
  virtual void  set_pan_tilt(float pan, float tilt) {}

 private:
  void advance();


  bool _finished;

  std::vector<fawkes::upoint_t> beam_current_pos;
  std::vector<fawkes::upoint_t> beam_end_pos;

  unsigned int start_x;
  unsigned int start_y;
  float angle_from;
  float angle_range;
  unsigned int num_beams;
  unsigned int stop_y;
  unsigned int offset_y;
  unsigned int image_width;
  unsigned int image_height;
  bool distribute_start_x;

  fawkes::upoint_t coord;
  fawkes::upoint_t tmp_coord;

  unsigned int next_beam;
  unsigned int first_beam;
  unsigned int last_beam;
};

} // end namespace firevision

#endif
