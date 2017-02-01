
/***************************************************************************
 *  lookuptable.cpp - Implementation of a lookup table color model
 *
 *  Generated: Wed May 18 13:59:18 2005
 *  Copyright  2005  Tim Niemueller  [www.niemueller.de]
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

#include <fvmodels/color/lookuptable.h>

#include <fvutils/color/yuv.h>
#include <fvutils/colormap/yuvcm.h>
#include <fvutils/colormap/cmfile.h>
#include <fvutils/ipc/shm_lut.h>

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>

#include <iostream>
#include <sys/utsname.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <cstring>
#include <cstdlib>
#include <cmath>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColorModelLookupTable <fvmodels/color/lookuptable.h>
 * Color model based on a lookup table.
 * Very fast and easy implementation of a lookup table. It ignores
 * the luminance and determines the classification just based on the U and
 * V chrominance values. This model is very versatile as you can generate
 * the lookuptable with many different methods.
 */

/** Create a lookup table with given dimensions _not_ using shared memory.
 * @param colormap colormap to use, the colormap is consumed, meaning that the color model
 * takes ownership of the colormap and deletes it in its dtor.
 */
ColorModelLookupTable::ColorModelLookupTable(YuvColormap *colormap)
{
  __colormap = colormap;
}

/** Create a lookup table with given dimensions using shared memory
 * @param lut_id ID of the LUT in shared memory
 * @param destroy_on_free true to destroy lookup table in shmem on delete
 */
ColorModelLookupTable::ColorModelLookupTable(const char *lut_id, bool destroy_on_free)
{
  __colormap = new YuvColormap(lut_id, destroy_on_free);
}


/** Create a lookup table with given dimensions using shared memory
 * @param depth depth of the lookup table
 * @param lut_id ID of the LUT in shared memory
 * @param destroy_on_free true to destroy lookup table in shmem on delete
 */
ColorModelLookupTable::ColorModelLookupTable(unsigned int depth,
					     const char *lut_id, bool destroy_on_free)
{
  __colormap = new YuvColormap(lut_id, destroy_on_free, depth);
}


/** Create a lookup table using shared memory, load contents from file.
 * @param file name of the file to load from
 * @param lut_id ID of the LUT in shared memory, use a constant from utils/shm_registry.h
 * @param destroy_on_free true to destroy lookup table in shmem on delete
 */
ColorModelLookupTable::ColorModelLookupTable(const char *file,
					     const char *lut_id, bool destroy_on_free)
{
  ColormapFile cmf;
  cmf.read(file);
  Colormap *tcm = cmf.get_colormap();
  YuvColormap *tycm = dynamic_cast<YuvColormap *>(tcm);
  if ( ! tycm ) {
    delete tcm;
    throw TypeMismatchException("File does not contain a YUV colormap");
  }
  __colormap = new YuvColormap(tycm, lut_id, destroy_on_free);
  delete tcm;
}


/** Create a lookup table, load contents from file.
 * @param file name of the file to load from
 */
ColorModelLookupTable::ColorModelLookupTable(const char *file)
{
  ColormapFile cmf;
  cmf.read(file);
  Colormap *tcm = cmf.get_colormap();
  __colormap = dynamic_cast<YuvColormap *>(tcm);
  if ( ! __colormap ) {
    delete tcm;
    throw TypeMismatchException("File does not contain a YUV colormap");
  }
}


/** Destructor. */
ColorModelLookupTable::~ColorModelLookupTable()
{
  delete __colormap;
}

color_t
ColorModelLookupTable::determine(unsigned int y, unsigned int u, unsigned int v) const
{
  return __colormap->determine(y, u, v);
}

const char *
ColorModelLookupTable::get_name()
{
  return "ColorModelLookupTable";
}

/** Get colormap.
 * @return a pointer to the YUV colormap used internally.
 */
YuvColormap *
ColorModelLookupTable::get_colormap() const
{
  return __colormap;
}


/** Set colormap.
 * @param yuvcm colormap to assign. The content of the colormap is copied
 * into the internal one.
 */
void
ColorModelLookupTable::set_colormap(const YuvColormap &yuvcm)
{
  *__colormap = yuvcm;
}


/** Load colormap from file.
 * @param filename name of colormap file
 */
void
ColorModelLookupTable::load(const char *filename)
{
  ColormapFile cmf;
  cmf.read(filename);
  Colormap *tcm = cmf.get_colormap();
  YuvColormap *tycm = dynamic_cast<YuvColormap *>(tcm);
  if ( ! tycm ) {
    delete tcm;
    throw TypeMismatchException("File does not contain a YUV colormap");
  }
  *__colormap = *tycm;
  delete tcm;
}


/** Add colormaps.
 * This adds the colormap of the given lookuptable color model to internals colormap.
 * @param cmlt lookup table color model to copy data from
 * @return this
 */
ColorModelLookupTable &
ColorModelLookupTable::operator+=(const ColorModelLookupTable &cmlt)
{
  *__colormap += *(cmlt.__colormap);
  return *this;
}


/** Reset colormap. */
void
ColorModelLookupTable::reset()
{
  __colormap->reset();
}

/** Compose filename.
 * @param format format string
 * @return composed filename
 * @see ColormapFile::compose_filename()
 */
std::string
ColorModelLookupTable::compose_filename(const std::string format)
{
  return ColormapFile::compose_filename(format);
}

} // end namespace firevision
