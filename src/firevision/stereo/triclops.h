
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

#ifndef __FIREVISION_FVUTILS_STEREO_TRICLOPS_H_
#define __FIREVISION_FVUTILS_STEREO_TRICLOPS_H_

#include <stereo/stereo_processor.h>

class Bumblebee2Camera;
class Camera;
class TriclopsStereoProcessorData;

class TriclopsStereoProcessor : public StereoProcessor
{
 public:
  TriclopsStereoProcessor(Camera *camera);
  virtual ~TriclopsStereoProcessor();

  virtual bool get_xyz(unsigned int px, unsigned int py,
		       float *x, float *y, float *z);

  virtual bool get_world_xyz(unsigned int px, unsigned int py,
			     float *x, float *y, float *z);

  virtual void             preprocess_stereo();
  virtual void             calculate_disparity();
  virtual void             calculate_yuv(bool both = false);
  virtual unsigned char *  disparity_buffer();
  virtual unsigned char *  yuv_buffer();
  virtual unsigned char *  auxiliary_yuv_buffer();

 private:
  void get_triclops_context_from_camera();
  void deinterlace_green( unsigned char* src, 
			  unsigned char* dest, 
			  unsigned int width, 
			  unsigned int height);



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

  unsigned int _width;
  unsigned int _height;
};

#endif
