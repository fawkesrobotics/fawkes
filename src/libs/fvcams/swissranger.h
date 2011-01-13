
/***************************************************************************
 *  swissranger.h - SwissRanger SR4000 Camera
 *
 *  Created: Wed Jan 13 17:04:51 2010
 *  Copyright  2005-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CAMS_SWISSRANGER_H_
#define __FIREVISION_CAMS_SWISSRANGER_H_

#include <fvcams/camera.h>
#include <fvcams/control/focus.h>

// libmesasr header defining basic types and enums
#include <definesSR.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;

class SwissRangerCamera
: public Camera
{
 public:
  /** Operation mode of the camera. */
  typedef enum {
    DISTANCE,		/**< raw distance image, unsigned short distance values */
    DISTANCE_GRAY_8,	/**< gray distance image, plain gray 8bpp buffer */
    AMPLITUDE,		/**< raw amplitude image, unsigned short values */
    AMPLITUDE_GRAY,	/**< amplitude gray image, 16bpp */
    AMPLITUDE_GRAY_8,	/**< amplitude gray image, 8bpp */
    CONF_MAP,		/**< confidence map, 16bpp*/
    CARTESIAN_UINT16,	/**< Cartesian coordinates, three consecutive planes for
			 * X, Y, Z data, each with unsigned short values (mm) */
    CARTESIAN_FLOAT,	/**< Cartesian coordinates, three consecutive planes for
			 * X, Y, Z data, each with float values (meters) */
    CARTESIAN_DOUBLE	/**< Cartesian coordinates, three consecutive planes for
			 * X, Y, Z data, each with double values (meters) */
  } mode_t;

  SwissRangerCamera(const CameraArgumentParser *cap);
  virtual ~SwissRangerCamera();

  virtual void open();
  virtual void start();
  virtual void stop();
  virtual void close();
  virtual void flush();
  virtual void capture();

  virtual void print_info();
  virtual bool ready();

  virtual unsigned char* buffer();
  virtual unsigned int   buffer_size();
  virtual void           dispose_buffer();

  virtual unsigned int   pixel_width();
  virtual unsigned int   pixel_height();
  virtual colorspace_t   colorspace();

  virtual void           set_image_number(unsigned int n);

  virtual const char *   model() const;

  static  void           print_available_cams();

 protected:
  /** true if camera has been opened, false otherwise */
  bool _opened;
  /** true if camera has been started, false otherwise */
  bool _started;
  /** true, if a valid frame has been received, false otherwise */
  bool _valid_frame_received;

 private:
  SRCAM __cam;

  /** Camera model, used in open to identify the camera,
   * if empty first found camera is used */
  char *__model;
  char *__vendor;
  unsigned int __vendor_id;
  unsigned int __product_id;
  unsigned int __serial;

  mode_t __mode;
  bool   __set_modfreq;
  bool   __use_median;
  bool   __use_denoise;
  unsigned int __integration_time;
  unsigned int __amplitude_threshold;

  ModulationFrq __modulation_freq;

  unsigned int __width;
  unsigned int __height;

  size_t          __buffer_size;
  unsigned char  *__buffer;
  unsigned char  *__gray_buffer;
  void           *__coord_uint16_buf;
  float          *__coord_float_buf;
  double         *__coord_double_buf;

  short          *__xu;
  short          *__yu;
  unsigned short *__zu;

  float          *__xf;
  float          *__yf;
  float          *__zf;

  double         *__xd;
  double         *__yd;
  double         *__zd;

};

} // end namespace firevision

#endif
