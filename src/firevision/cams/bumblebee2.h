
/***************************************************************************
 *  bumblebee2.h - Point Grey Bumblebee 2 camera
 *
 *  Generated: Sat Apr 14 20:49:20 2007 (watching Ghostbusters)
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __FIREVISION_CAMS_BUMBLEBEE2_H_
#define __FIREVISION_CAMS_BUMBLEBEE2_H_

#include <cams/firewire.h>

class Bumblebee2CameraData;

class Bumblebee2Camera : public FirewireCamera
{
 public:

  static const unsigned int LEFT_ORIGINAL;
  static const unsigned int RIGHT_ORIGINAL;

  Bumblebee2Camera(CameraArgumentParser *cap);
  virtual ~Bumblebee2Camera();

  virtual void open();
  virtual void close();

  virtual void flush();
  virtual void capture();

  virtual unsigned char* buffer();
  virtual unsigned int   buffer_size();
  unsigned char * buffer_disparity();

  virtual colorspace_t   colorspace();

  virtual void set_image_number(unsigned int n);

  bool is_bumblebee2();

  bool get_xyz(unsigned int px, unsigned int py, float *x, float *y, float *z);
 protected:

  void get_triclops_context_from_camera();
  void write_triclops_config_from_camera_to_file(const char *filename);
  void get_bayer_tile();
  void deinterlace_green( unsigned char* src,  unsigned char* dest, 
			  unsigned int width,  unsigned int height);


  /** Bayer pattern */
  dc1394color_filter_t bayer_pattern;

  Bumblebee2CameraData *data;

  unsigned char *_buffer;
  unsigned char *buffer_deinterlaced;
  unsigned char *buffer_rgb;
  unsigned char *buffer_green;
  unsigned char *buffer_rgb_left;
  unsigned char *buffer_rgb_right;
  unsigned char *buffer_rgb_center;
  unsigned char *buffer_yuv_left;
  unsigned char *buffer_yuv_right;
  unsigned char *buffer_yuv_center;

  bool done;

};

#endif
