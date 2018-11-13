
/**************************************************************************
 *  yuvcm.h - YUV colormap
 *
 *  Created: Sat Mar 29 12:45:29 2008
 *  Copyright  2005-2008  Tim Niemueller  [www.niemueller.de]
 *
 ***************************************************************************/

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

#ifndef _FIREVISION_FVUTILS_COLORMAP_YUVCM_H_
#define _FIREVISION_FVUTILS_COLORMAP_YUVCM_H_

#include <fvutils/colormap/colormap.h>

#include <sys/types.h>
#include <fvutils/base/types.h>

namespace firevision {

class SharedMemoryLookupTable;

class YuvColormap : public Colormap
{
 public:
	YuvColormap(unsigned int depth = 1, unsigned int width = 256, unsigned int height = 256);
  YuvColormap(const char *shmem_lut_id, unsigned int depth = 1,
              unsigned int width = 256, unsigned int height = 256);
  YuvColormap(const char *shmem_lut_id, bool destroy_on_free,
              unsigned int depth = 1, unsigned int width = 256, unsigned int height = 256);
  YuvColormap(YuvColormap *cm, const char *shmem_lut_id, bool destroy_on_free = false);
	YuvColormap(const YuvColormap& cm);
	virtual ~YuvColormap();

  virtual color_t          determine(unsigned int y, unsigned int u, unsigned int v) const;
  virtual void             set(unsigned int y, unsigned int u, unsigned int v, color_t c);

  virtual void             reset();
  virtual void             set(unsigned char *buffer);

  virtual size_t           size();

  virtual unsigned char *  get_buffer() const;

  virtual Colormap &  operator+=(const Colormap & cmlt);
  virtual Colormap &  operator+=(const char *filename);
  virtual Colormap &  operator=(const YuvColormap &yuvcm);

  virtual unsigned int     width() const;
  virtual unsigned int     height() const;
  virtual unsigned int     depth() const;
  virtual unsigned int     deepness() const;
  unsigned int             plane_size() const;

  virtual std::list<ColormapFileBlock *>  get_blocks();

  void copy_uvplane(unsigned char *uvplane, unsigned int level);

  void replace_color(color_t from, color_t to);

 private:
  void constructor(unsigned int depth, unsigned int width, unsigned int height,
		   const char *shmem_lut_id = 0, bool destroy_on_free = false);


  SharedMemoryLookupTable *shm_lut_;
  unsigned char *lut_;
  size_t         lut_size_;

  unsigned int width_;
  unsigned int height_;
  unsigned int depth_;
  unsigned int depth_div_;
  unsigned int width_div_;
  unsigned int height_div_;
  unsigned int plane_size_;
};


inline color_t
YuvColormap::determine(unsigned int y, unsigned int u, unsigned int v) const
{
  return (color_t) *(lut_ + (y / depth_div_) * plane_size_ + (v / height_div_) * width_ + (u / width_div_));
}

} // end namespace firevision

#endif
