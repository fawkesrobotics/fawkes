
/***************************************************************************
 *  shmem.h - This header defines a reader for shared memory images
 *
 *  Generated: Thu Jan 12 19:41:17 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
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

  SharedMemoryCamera(const char *image_id, bool deep_copy = false);
  SharedMemoryCamera(const CameraArgumentParser *cap);
  ~SharedMemoryCamera();

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

  SharedMemoryImageBuffer *  shared_memory_image_buffer();

  virtual void           lock_for_read();
  virtual bool           try_lock_for_read();
  virtual void           lock_for_write();
  virtual bool           try_lock_for_write();
  virtual void           unlock();

 private:
  void init();

  bool          deep_copy;
  bool          opened;
  unsigned int  width;
  unsigned int  height;
  char *        image_id;

  SharedMemoryImageBuffer  *shm_buffer;

  unsigned char *deep_buffer;
};

#endif
