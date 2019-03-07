
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

#include <core/exceptions/software.h>
#include <fvutils/colormap/cmfile.h>
#include <fvutils/colormap/cmfile_yuvblock.h>
#include <fvutils/colormap/yuvcm.h>
#include <fvutils/ipc/shm_lut.h>

#include <cstdlib>
#include <cstring>

using namespace fawkes;

namespace firevision {

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
YuvColormap::YuvColormap(const char * shmem_lut_id,
                         unsigned int depth,
                         unsigned int width,
                         unsigned int height)
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
YuvColormap::YuvColormap(const char * shmem_lut_id,
                         bool         destroy_on_free,
                         unsigned int depth,
                         unsigned int width,
                         unsigned int height)
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
	memcpy(lut_, cm->lut_, lut_size_);
}

/** Copy constructor.
 * Creates a colormap in shared memory for the given LUT ID and copies the data of the
 * given existing colormap.
 * @param cm color mape to copy from
 */
YuvColormap::YuvColormap(const YuvColormap &cm)
{
	constructor(cm.depth(), cm.width(), cm.height());
	memcpy(lut_, cm.lut_, lut_size_);
}

/** Internal constructor.
 * @param shmem_lut_id shared memory LUT ID
 * @param destroy_on_free true to delete the shared memory segment to delete, false to keep the segment
 * @param depth Y depth
 * @param width U depth
 * @param height V depth
 */
void
YuvColormap::constructor(unsigned int depth,
                         unsigned int width,
                         unsigned int height,
                         const char * shmem_lut_id,
                         bool         destroy_on_free)
{
	if (depth > 256) {
		throw OutOfBoundsException("YuvColormap depth out of bounds", depth, 1, 256);
	}
	if ((depth != 1) && (depth != 2) && (depth != 4) && (depth != 8) && (depth != 16) && (depth != 32)
	    && (depth != 64) && (depth != 128) && (depth != 256)) {
		throw IllegalArgumentException("Depth must be of the form d=2^n with n from [1,8]");
	}

	if (width > 256) {
		throw OutOfBoundsException("YuvColormap width out of bounds", width, 1, 256);
	}
	if ((width != 1) && (width != 2) && (width != 4) && (width != 8) && (width != 16) && (width != 32)
	    && (width != 64) && (width != 128) && (width != 256)) {
		throw IllegalArgumentException("Width must be of the form d=2^n with n from [1,8]");
	}

	if (height > 256) {
		throw OutOfBoundsException("YuvColormap height out of bounds", height, 1, 256);
	}
	if ((height != 1) && (height != 2) && (height != 4) && (height != 8) && (height != 16)
	    && (height != 32) && (height != 64) && (height != 128) && (height != 256)) {
		throw IllegalArgumentException("Height must be of the form d=2^n with n from [1,8]");
	}

	width_      = width;
	height_     = height;
	depth_      = depth;
	depth_div_  = 256 / depth_;
	width_div_  = 256 / width_;
	height_div_ = 256 / height_;
	plane_size_ = width_ * height_;

	if (shmem_lut_id != NULL) {
		shm_lut_ =
		  new SharedMemoryLookupTable(shmem_lut_id, width_, height_, depth_, /* bytes p. cell */ 1);
		shm_lut_->set_destroy_on_delete(destroy_on_free);
		lut_      = shm_lut_->buffer();
		lut_size_ = shm_lut_->data_size();
	} else {
		shm_lut_  = NULL;
		lut_size_ = (size_t)width_ * (size_t)height_ * (size_t)depth_;
		lut_      = (unsigned char *)malloc(lut_size_);
	}
	memset(lut_, C_OTHER, lut_size_);
}

/** Destructor. */
YuvColormap::~YuvColormap()
{
	if (shm_lut_) {
		delete shm_lut_;
	} else {
		free(lut_);
	}
	lut_      = NULL;
	lut_size_ = 0;
}

void
YuvColormap::set(unsigned int y, unsigned int u, unsigned int v, color_t c)
{
	*(lut_ + (y / depth_div_) * plane_size_ + (v / height_div_) * width_ + (u / width_div_)) = c;
}

void
YuvColormap::reset()
{
	memset(lut_, C_OTHER, lut_size_);
}

void
YuvColormap::set(unsigned char *buffer)
{
	memcpy(lut_, buffer, lut_size_);
}

size_t
YuvColormap::size()
{
	return lut_size_;
}

std::list<ColormapFileBlock *>
YuvColormap::get_blocks()
{
	std::list<ColormapFileBlock *> rv;

	for (unsigned int i = 0; i < depth_; ++i) {
		ColormapFileYuvBlock *yuvb = new ColormapFileYuvBlock(this, i);
		rv.push_back(yuvb);
	}

	return rv;
}

unsigned char *
YuvColormap::get_buffer() const
{
	return lut_;
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
	if (level > depth_) {
		throw OutOfBoundsException("YuvColormap::copy_uvplane(): Invalid level", level, 0, depth_);
	}

	memcpy(lut_ + level * plane_size_, uvplane, plane_size_);
}

/** Adds the given colormap to this colormap.
 * This operator takes the given colormap and compares it to this colormap. If
 * this colormap has C_OTHER or C_BACKGROUND the value is compied from the other
 * LUT, otherwise the value is kept as is.
 * @param cmlt other colormap to add
 * @return reference to this
 */
Colormap &
YuvColormap::operator+=(const Colormap &cmlt)
{
	const YuvColormap *tc = dynamic_cast<const YuvColormap *>(&cmlt);
	if (tc == NULL) {
		throw TypeMismatchException("Only YUV colormaps can be added to a YUV colormap");
	}

	if ((width_ != tc->width_) || (height_ != tc->height_) || (depth_ != tc->depth_)) {
		throw TypeMismatchException("YuvColormaps are of different sizes");
	}

	unsigned char *this_lut  = lut_;
	unsigned char *other_lut = tc->lut_;

	for (unsigned int i = 0; i < plane_size_ * depth_; ++i) {
		if ((*this_lut == C_OTHER) || (*this_lut == C_BACKGROUND)) {
			// can be overridden
			if ((*other_lut != C_OTHER) && (*other_lut != C_BACKGROUND)) {
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
YuvColormap::operator=(const YuvColormap &yuvcm)
{
	if (lut_size_ != yuvcm.lut_size_) {
		throw TypeMismatchException("Size of colormaps does not match");
	}

	memcpy(lut_, yuvcm.lut_, lut_size_);

	return *this;
}

Colormap &
YuvColormap::operator+=(const char *filename)
{
	ColormapFile cmf;
	cmf.read(filename);
	Colormap *   tcm  = cmf.get_colormap();
	YuvColormap *tycm = dynamic_cast<YuvColormap *>(tcm);
	if (!tycm) {
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
	return width_;
}

unsigned int
YuvColormap::height() const
{
	return height_;
}

unsigned int
YuvColormap::depth() const
{
	return depth_;
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
	return plane_size_;
}

/** Replace a given color with another one.
 * @param from color to replace
 * @param to color to replace @p from with
 */
void
YuvColormap::replace_color(color_t from, color_t to)
{
	unsigned char *this_lut = lut_;

	for (unsigned int i = 0; i < plane_size_ * depth_; ++i, ++this_lut) {
		if (*this_lut == from)
			*this_lut = to;
	}
}

} // end namespace firevision
