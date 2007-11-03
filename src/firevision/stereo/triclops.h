
/***************************************************************************
 *  triclops.h - Stereo processor using the TriclopsSDK
 *
 *  Created: Fri May 18 16:25:26 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_STEREO_TRICLOPS_H_
#define __FIREVISION_STEREO_TRICLOPS_H_

#include <stereo/stereo_processor.h>
#include <sys/types.h>

class Bumblebee2Camera;
class Camera;
class TriclopsStereoProcessorData;
class RectificationLutInfoBlock;

class TriclopsStereoProcessor : public StereoProcessor
{
 public:
  TriclopsStereoProcessor(unsigned int width, unsigned int height,
			  const char *context_file);
  TriclopsStereoProcessor(Camera *camera);
  virtual ~TriclopsStereoProcessor();

  virtual bool          subpixel_interpolation();
  virtual bool          edge_correlation();
  virtual bool          lowpass();
  virtual int           disparity_range_min();
  virtual int           disparity_range_max();
  virtual unsigned int  edge_masksize();
  virtual unsigned int  stereo_masksize();
  virtual bool          surface_validation();
  virtual bool          texture_validation();
  virtual unsigned char disparity_mapping_min();
  virtual unsigned char disparity_mapping_max();
  virtual bool          disparity_mapping();

  virtual void          set_subpixel_interpolation(bool enabled);
  virtual void          set_edge_correlation(bool enabled);
  virtual void          set_lowpass(bool enabled);
  virtual void          set_disparity_range(int min, int max);
  virtual void          set_edge_masksize(unsigned int mask_size); // 3-13
  virtual void          set_stereo_masksize(unsigned int mask_size); // 1-15
  virtual void          set_surface_validation(bool enabled);
  virtual void          set_texture_validation(bool enabled);
  virtual void          set_disparity_mapping_range(unsigned char min, unsigned char max);
  virtual void          set_disparity_mapping(bool enabled);

  virtual bool          get_xyz(unsigned int px, unsigned int py,
				float *x, float *y, float *z);

  virtual bool          get_world_xyz(unsigned int px, unsigned int py,
				      float *x, float *y, float *z);

  virtual void             set_raw_buffer(unsigned char *raw16_buffer);
  virtual void             preprocess_stereo();
  virtual void             calculate_disparity(ROI *roi = 0);
  virtual void             calculate_yuv(bool both = false);
  virtual unsigned char *  disparity_buffer();
  virtual size_t           disparity_buffer_size() const;
  virtual unsigned char *  yuv_buffer();
  virtual unsigned char *  auxiliary_yuv_buffer();

  void    generate_rectification_lut(const char *lut_file);
  bool    verify_rectification_lut(const char *lut_file);

 private:
  void get_triclops_context_from_camera();
  void deinterlace_green( unsigned char* src, 
			  unsigned char* dest, 
			  unsigned int width, 
			  unsigned int height);

  void create_buffers();
  void setup_triclops();


 private:
  Bumblebee2Camera             *bb2;
  TriclopsStereoProcessorData  *data;

  unsigned char *buffer_rgb;
  unsigned char *buffer_green;
  unsigned char *buffer_rgb_left;
  unsigned char *buffer_rgb_right;
  unsigned char *buffer_rgb_center;
  unsigned char *buffer_yuv_left;
  unsigned char *buffer_yuv_right;
  unsigned char *buffer_yuv_center;
  unsigned char *_buffer;
  unsigned char *buffer_deinterlaced;
  unsigned char *buffer_raw16;

  unsigned int _width;
  unsigned int _height;

  char  *_context_file;
};

#endif
