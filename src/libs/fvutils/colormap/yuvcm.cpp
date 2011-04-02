
/**************************************************************************
 *  colormap.cpp - colormap
 *
 *  Created: Sat Mar 29 18:11:38 2008
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

#include <fvutils/colormap/yuvcm.h>

#include <fvutils/colormap/cmfile.h>
#include <fvutils/colormap/cmfile_yuvblock.h>
#include <fvutils/ipc/shm_lut.h>
#include <core/exceptions/software.h>

#include <cstdlib>
#include <cstring>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class YuvColormap <fvutils/colormap/colormap.h>
 * YUV Colormap.
 * This class is the implementation of a 3D YUV colormap. The U/V planes are always
 * sampled in full. In general for colormaps we assume that in many cases the luminance
 * can be ignored completely. This allows for small datasets with speedy access and
 * sufficient discriminatory power. However, in some situations this is not enough.
 * In that case you can give a depth for the Y value. The Y axis is then separated
 * in the given number of ranges, each range is a stacked complete U/V plane.
 * Note, only depth values where depth = 2^n, n from natural numbers holds will provide
 * with equal ranges. Other values will lead to one bigger range, being the one with
 * the highest Y values which will be filled with the whole rest.
 *
 * You can see such a colormap as a colormap that consists of UV planes that represent
 * a certain Y range stacked on top of each other.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param depth Y resolution depth
 * @param width U depth
 * @param height V depth
 */
YuvColormap::YuvColormap(unsigned int depth, unsigned int width, unsigned int height)
{
  constructor(depth, width, height);
}


/** Constructor.
 * Creates a colormap in shared memory for the given LUT ID.
 * @param shmem_lut_id shared memory LUT ID
 * @param depth Y depth
 * @param width U depth
 * @param height V depth
 */
YuvColormap::YuvColormap(const char *shmem_lut_id, unsigned int depth, unsigned int width, unsigned int height)
{
  constructor(depth, width, height, shmem_lut_id);
}


/** Constructor.
 * Creates a colormap in shared memory for the given LUT ID.
 * @param shmem_lut_id shared memory LUT ID
 * @param destroy_on_free true to delete the shared memory segment to delete, false to keep the segment
 * @param depth Y depth
 * @param width U depth
 * @param height V depth
 */
YuvColormap::YuvColormap(const char *shmem_lut_id, bool destroy_on_free, unsigned int depth, unsigned int width, unsigned int height)
{
  constructor(depth, width, height, shmem_lut_id, destroy_on_free);
}


/** Constructor.
 * Creates a colormap in shared memory for the given LUT ID and copies the data of the
 * given existing colormap.
 * @param cm existing colormap to copy data from
 * @param shmem_lut_id shared memory LUT ID
 * @param destroy_on_free true to delete the shared memory segment to delete, false to keep the segment
 */
YuvColormap::YuvColormap(YuvColormap *cm, const char *shmem_lut_id, bool destroy_on_free)
{
  constructor(cm->depth(), cm->width(), cm->height(), shmem_lut_id, destroy_on_free);
  memcpy(__lut, cm->__lut, __lut_size);
}


/** Internal constructor.
 * @param shmem_lut_id shared memory LUT ID
 * @param destroy_on_free true to delete the shared memory segment to delete, false to keep the segment
 * @param depth Y depth
 * @param width U depth
 * @param height V depth
 */
void
YuvColormap::constructor(unsigned int depth, unsigned int width, unsigned int height,
                         const char *shmem_lut_id, bool destroy_on_free)
{
  if ( depth > 256 ) {
    throw OutOfBoundsException("YuvColormap depth out of bounds", depth, 1, 256);
  }
  if ( (depth != 1) && (depth != 2) && (depth != 4) && (depth != 8) && (depth != 16) &&
       (depth != 32) && (depth != 64) && (depth != 128) && (depth != 256) ) {
    throw IllegalArgumentException("Depth must be of the form d=2^n with n from [1,8]");
  }

  if ( width > 256 ) {
    throw OutOfBoundsException("YuvColormap width out of bounds", width, 1, 256);
  }
  if ( (width != 1) && (width != 2) && (width != 4) && (width != 8) && (width != 16) &&
       (width != 32) && (width != 64) && (width != 128) && (width != 256) ) {
    throw IllegalArgumentException("Width must be of the form d=2^n with n from [1,8]");
  }

  if ( height > 256 ) {
    throw OutOfBoundsException("YuvColormap height out of bounds", height, 1, 256);
  }
  if ( (height != 1) && (height != 2) && (height != 4) && (height != 8) && (height != 16) &&
       (height != 32) && (height != 64) && (height != 128) && (height != 256) ) {
    throw IllegalArgumentException("Height must be of the form d=2^n with n from [1,8]");
  }

  __width  = width;
  __height = height;
  __depth  = depth;
  __depth_div  = 256 / __depth;
  __width_div  = 256 / __width;
  __height_div  = 256 / __height;
  __plane_size = __width * __height;

  if ( shmem_lut_id != NULL ) {
    __shm_lut  = new SharedMemoryLookupTable(shmem_lut_id, __width, __height, __depth, /* bytes p. cell */ 1);
    __shm_lut->set_destroy_on_delete( destroy_on_free );
    __lut      = __shm_lut->buffer();
    __lut_size = __shm_lut->data_size();
  } else {
    __shm_lut = NULL;
    __lut_size = __width * __height * __depth;
    __lut = (unsigned char *)malloc( __lut_size );
  }
  memset(__lut, C_OTHER, __lut_size);
}


/** Destructor. */
YuvColormap::~YuvColormap()
{
  if ( __shm_lut ) {
    delete __shm_lut;
  } else {
    free(__lut);
  }
  __lut = NULL;
  __lut_size = 0;
}


void
YuvColormap::set(unsigned int y, unsigned int u, unsigned int v, color_t c)
{
  *(__lut + (y / __depth_div) * __plane_size + (v / __height_div) * __width + (u / __width_div)) = c;
}


void
YuvColormap::reset()
{
  memset(__lut, C_OTHER, __lut_size);
}


void
YuvColormap::set(unsigned char *buffer)
{
  memcpy(__lut, buffer, __lut_size);
}


size_t
YuvColormap::size()
{
  return __lut_size;
}


std::list<ColormapFileBlock *>
YuvColormap::get_blocks()
{
  std::list<ColormapFileBlock *> rv;

  for (unsigned int i = 0; i < __depth; ++i) {
    ColormapFileYuvBlock *yuvb = new ColormapFileYuvBlock(this, i);
    rv.push_back(yuvb);
  }

  return rv;
}


unsigned char *
YuvColormap::get_buffer() const
{
  return __lut;
}


/** Copy single U/V plane.
 * This will copy the given U/V plane to the given level in this colormap.
 * @param uvplane buffer of U/V plane to copy
 * @param level level to copy the plane to
 * @exception OutOfBoundsException thrown if level > depth()
 */
void
YuvColormap::copy_uvplane(unsigned char *uvplane, unsigned int level)
{
  if ( level > __depth ) {
    throw OutOfBoundsException("YuvColormap::copy_uvplane(): Invalid level", level, 0, __depth);
  }

  memcpy(__lut + level * __plane_size, uvplane, __plane_size);
}


/** Adds the given colormap to this colormap.
 * This operator takes the given colormap and compares it to this colormap. If
 * this colormap has C_OTHER or C_BACKGROUND the value is compied from the other
 * LUT, otherwise the value is kept as is.
 * @param cmlt other colormap to add
 * @return reference to this
 */
Colormap &
YuvColormap::operator+=(const Colormap & cmlt)
{
  const YuvColormap *tc = dynamic_cast<const YuvColormap *>(&cmlt);
  if ( tc == NULL ) {
    throw TypeMismatchException("Only YUV colormaps can be added to a YUV colormap");
  }

  if ( (__width != tc->__width) || (__height != tc->__height) || (__depth != tc->__depth) ) {
    throw TypeMismatchException("YuvColormaps are of different sizes");
  }

  unsigned char *this_lut = __lut;
  unsigned char *other_lut = tc->__lut;

  for (unsigned int i = 0; i < __plane_size * __depth; ++i) {
    if ( (*this_lut == C_OTHER) || (*this_lut == C_BACKGROUND) ) {
      // can be overridden
      if ( (*other_lut != C_OTHER) && (*other_lut != C_BACKGROUND) ) {
	// there is something that is worth overriding this value
	*this_lut = *other_lut;
      }
    }
    ++this_lut;
    ++other_lut;
  }

  return *this;
}


/** Assign operation.
 * Copies all values from the given colormap.
 * @param yuvcm colormap which's data to copy to this instance
 * @exception TypeMismatchException thrown if depth of colormaps does not match.
 * @return reference to this
 */
Colormap &
YuvColormap::operator=(const YuvColormap & yuvcm)
{
  if ( __lut_size != yuvcm.__lut_size ) {
    throw TypeMismatchException("Size of colormaps does not match");
  }

  memcpy(__lut, yuvcm.__lut, __lut_size);

  return *this;
}


Colormap &
YuvColormap::operator+=(const char *filename)
{
  ColormapFile cmf;
  cmf.read(filename);
  Colormap *tcm = cmf.get_colormap();
  YuvColormap *tycm = dynamic_cast<YuvColormap *>(tcm);
  if ( ! tycm ) {
    delete tcm;
    throw TypeMismatchException("File does not contain a YUV colormap");
  }
  *this += *tycm;
  delete tcm;
  return *this;
}


unsigned int
YuvColormap::width() const
{
  return __width;
}


unsigned int
YuvColormap::height() const
{
  return __height;
}


unsigned int
YuvColormap::depth() const
{
  return __depth;
}


unsigned int
YuvColormap::deepness() const
{
  return 256;
}


/** Get U/V plane size.
 * @return size of a single U/V plane
 */
unsigned int
YuvColormap::plane_size() const
{
  return __plane_size;
}


/** Replace a given color with another one.
 * @param from color to replace
 * @param to color to replace @p from with
 */
void
YuvColormap::replace_color(color_t from, color_t to)
{
  unsigned char *this_lut = __lut;

  for (unsigned int i = 0; i < __plane_size * __depth; ++i, ++this_lut) {
    if (*this_lut == from)  *this_lut = to;
  }
}

} // end namespace firevision
