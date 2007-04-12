
/***************************************************************************
 *  shmem.h - This header defines a reader for shared memory images
 *
 *  Generated: Thu Jan 12 19:41:17 2006
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

#ifndef __FIREVISION_CAMS_SHMEM_H_
#define __FIREVISION_CAMS_SHMEM_H_

#include <cams/camera.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/ipc/shm_image.h>

class CameraArgumentParser;

class SharedMemoryCamera : public Camera
{

 public:

  SharedMemoryCamera(unsigned int image_num=0);
  SharedMemoryCamera(CameraArgumentParser *cap);

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

  virtual unsigned int   number_of_images();
  virtual void           set_image_number(unsigned int n);

  SharedMemoryImageBuffer *  shared_memory_image_buffer();

 private:
  void init();

  bool opened;
  unsigned int  width;
  unsigned int  height;
  unsigned int  image_num;

  SharedMemoryImageBuffer  *shm_buffer;
};

#endif
