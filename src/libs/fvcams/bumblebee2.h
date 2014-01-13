
/***************************************************************************
 *  bumblebee2.h - Point Grey Bumblebee 2 camera
 *
 *  Generated: Sat Apr 14 20:49:20 2007 (watching Ghostbusters)
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

#ifndef __FIREVISION_CAMS_BUMBLEBEE2_H_
#define __FIREVISION_CAMS_BUMBLEBEE2_H_

#include <fvcams/firewire.h>
#include <fvutils/color/bayer.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Bumblebee2CameraData;

class Bumblebee2Camera : public FirewireCamera
{
 public:

  static const unsigned int ORIGINAL;
  static const unsigned int DEINTERLACED;
  static const unsigned int RGB_IMAGE;

  Bumblebee2Camera(const CameraArgumentParser *cap);
  Bumblebee2Camera();
  virtual ~Bumblebee2Camera();

  virtual void open_device();
  virtual void open();
  virtual void close();
  virtual void capture();

  virtual unsigned char* buffer();

  virtual void set_image_number(unsigned int image_num);

  bool is_bumblebee2();
  void write_triclops_config_from_camera_to_file(const char *filename);

  void deinterlace_stereo();
  void decode_bayer();

  virtual void     print_info();
  virtual uint32_t serial_no() const;
  virtual bool     verify_guid(uint64_t ver_guid) const;

  static void deinterlace_stereo(unsigned char *raw16, unsigned char *deinterlaced,
				 unsigned int width, unsigned int height);
  static void decode_bayer(unsigned char *deinterlaced, unsigned char *rgb,
			   unsigned int width, unsigned int height,
			   bayer_pattern_t bayer_pattern);

 private:
  void get_sensor_info();
  void get_triclops_context_from_camera();
  void get_bayer_tile();
  void deinterlace_green( unsigned char* src,  unsigned char* dest, 
			  unsigned int width,  unsigned int height);


  /** Bayer pattern */
  dc1394color_filter_t __bayer_pattern;

  bool _auto_acquire_sensor_info;

  unsigned int   __image_num;
  unsigned char *__buffer;
  unsigned char *__buffer_deinterlaced;
  unsigned char *__buffer_rgb;

  bool          _supports_color;
};

} // end namespace firevision

#endif
