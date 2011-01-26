
/***************************************************************************
 *  pike.h - AlliedVision Pike camera
 *
 *  Generated: Tue Mar 16 15:24:57 2010
 *  Copyright  2010  Daniel Beck
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

#ifndef __FIREVISION_CAMS_PIKE_H_
#define __FIREVISION_CAMS_PIKE_H_

#include <fvcams/firewire.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class PikeCamera : public FirewireCamera
{
 public:
  PikeCamera(const CameraArgumentParser* cap);
  virtual ~PikeCamera();

  virtual void open();

  virtual void print_info();

  virtual bool set_autofunction_aoi( unsigned int left,
				     unsigned int top,
				     unsigned int width,
				     unsigned int height,
				     bool show_work_area = false );

  virtual void parse_set_autofnc_aoi( const char* aoi );

 private:
  bool __set_autofnc_aoi;
  unsigned int __aoi_left;
  unsigned int __aoi_top;
  unsigned int __aoi_width;
  unsigned int __aoi_height;
  bool __aoi_show_work_area;
};

} // end namespace firevision

#endif /* __FIREVISION_CAMS_PIKE_H_ */
