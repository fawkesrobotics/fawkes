
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

#ifndef _FIREVISION_CAMS_SWISSRANGER_H_
#define _FIREVISION_CAMS_SWISSRANGER_H_

#include <fvcams/camera.h>
#include <fvcams/control/focus.h>

// libmesasr header defining basic types and enums
#include <definesSR.h>

namespace firevision {

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
  SRCAM cam_;

  /** Camera model, used in open to identify the camera,
   * if empty first found camera is used */
  char *model_;
  char *vendor_;
  unsigned int vendor_id_;
  unsigned int product_id_;
  unsigned int serial_;

  mode_t mode_;
  bool   set_modfreq_;
  bool   use_median_;
  bool   use_denoise_;
  unsigned int integration_time_;
  unsigned int amplitude_threshold_;

  ModulationFrq modulation_freq_;

  unsigned int width_;
  unsigned int height_;

  size_t          buffer_size_;
  unsigned char  *buffer_;
  unsigned char  *gray_buffer_;
  void           *coord_uint16_buf_;
  float          *coord_float_buf_;
  double         *coord_double_buf_;

  short          *xu_;
  short          *yu_;
  unsigned short *zu_;

  float          *xf_;
  float          *yf_;
  float          *zf_;

  double         *xd_;
  double         *yd_;
  double         *zd_;

};

} // end namespace firevision

#endif
