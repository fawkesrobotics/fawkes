
/***************************************************************************
 *  lookuptable.cpp - Implementation of a lookup table color model
 *
 *  Generated: Wed May 18 13:59:18 2005
 *  Copyright  2005  Tim Niemueller  [www.niemueller.de]
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

#include <models/color/lookuptable.h>

#include <fvutils/color/yuv.h>
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

/** @class ColorModelLookupTable <models/color/lookuptable.h>
 * Color model based on a lookup table.
 * Very fast and easy implementation of a lookup table. It ignores
 * the luminance and determines the classification just based on the U and
 * V chrominance values. This model is very versatile as you can generate
 * the lookuptable with many different methods.
 *
 * VERY IMPORTANT NOTE:
 * The given width and height are _not_ the dimensions of the image but they are the size
 * of the lookup table. For example if you want to generate a lut for the UV values of the YUV color
 * space and you have one byte per U and V value then you would have width = height = 256!
 */

/** Create a lookup table with given dimensions _not_ using shared memory.
 * @param width width of lookup table
 * @param height height of lookup table
 * @param depth depth of the lookup table
 */
ColorModelLookupTable::ColorModelLookupTable(unsigned int width, unsigned int height,
					     unsigned int depth)
{
  this->width            = width;
  this->height           = height;
  this->depth            = depth;
  this->lut_id           = NULL;
  this->destroy_on_free  = false;

  create();

  x_max = 256;
  y_max = 256;
  z_max = 256;
}

/** Create a lookup table with given dimensions _not_ using shared memory, load contents
 * from file
 * @param file name of the file to load from
 * @param width width of lookup table
 * @param height height of lookup table
 * @param depth depth of the lookup table
 */
ColorModelLookupTable::ColorModelLookupTable(const char *file,
					     unsigned int width, unsigned int height,
					     unsigned int depth)
{
  this->width            = width;
  this->height           = height;
  this->depth            = depth;
  this->lut_id           = NULL;
  this->destroy_on_free  = false;

  create();
  load(file);

  x_max = 256;
  y_max = 256;
  z_max = 256;
}


/** Create a lookup table with given dimensions using shared memory
 * @param width width of lookup table
 * @param height height of lookup table
 * @param lut_id ID of the LUT in shared memory
 * @param destroy_on_free true to destroy lookup table in shmem on delete
 */
ColorModelLookupTable::ColorModelLookupTable(unsigned int width, unsigned int height,
					     const char *lut_id, bool destroy_on_free)
{
  this->width            = width;
  this->height           = height;
  this->depth            = 1;
  this->lut_id           = strdup(lut_id);
  this->destroy_on_free  = destroy_on_free;

  create();

  x_max = 256;
  y_max = 256;
  z_max = 256;
}


/** Create a lookup table with given dimensions using shared memory
 * @param width width of lookup table
 * @param height height of lookup table
 * @param depth depth of the lookup table
 * @param lut_id ID of the LUT in shared memory
 * @param destroy_on_free true to destroy lookup table in shmem on delete
 */
ColorModelLookupTable::ColorModelLookupTable(unsigned int width, unsigned int height,
					     unsigned int depth,
					     const char *lut_id, bool destroy_on_free)
{
  this->width            = width;
  this->height           = height;
  this->depth            = depth;
  this->lut_id           = strdup(lut_id);
  this->destroy_on_free  = destroy_on_free;

  create();

  x_max = 256;
  y_max = 256;
  z_max = 256;
}


/** Create a lookup table with given dimensions using shared memory, load contents
 * from file
 * @param file name of the file to load from
 * @param width width of lookup table
 * @param height height of lookup table
 * @param lut_id ID of the LUT in shared memory, use a constant from utils/shm_registry.h
 * @param destroy_on_free true to destroy lookup table in shmem on delete
 */
ColorModelLookupTable::ColorModelLookupTable(const char *file,
					     unsigned int width, unsigned int height,
					     const char *lut_id, bool destroy_on_free)
{
  this->width            = width;
  this->height           = height;
  this->depth            = 1;
  this->lut_id           = strdup(lut_id);
  this->destroy_on_free  = destroy_on_free;

  create();
  load(file);

  x_max = 256;
  y_max = 256;
  z_max = 256;
}


/** Create a lookup table with given dimensions using shared memory, load contents
 * from file
 * @param file name of the file to load from
 * @param width width of lookup table
 * @param height height of lookup table
 * @param depth depth of the lookup table
 * @param lut_id ID of the LUT in shared memory, use a constant from utils/shm_registry.h
 * @param destroy_on_free true to destroy lookup table in shmem on delete
 */
ColorModelLookupTable::ColorModelLookupTable(const char *file,
					     unsigned int width, unsigned int height,
					     unsigned int depth,
					     const char *lut_id, bool destroy_on_free)
{
  this->width            = width;
  this->height           = height;
  this->depth            = depth;
  this->lut_id           = strdup(lut_id);
  this->destroy_on_free  = destroy_on_free;

  create();
  load(file);

  x_max = 256;
  y_max = 256;
  z_max = 256;
}


/** Creates the memory segment.
 */
void
ColorModelLookupTable::create()
{
  bytes_per_sample = 1;

  if ( lut_id != NULL ) {
    shm_lut   = new SharedMemoryLookupTable( lut_id, width, height, depth, bytes_per_sample);
    shm_lut->set_destroy_on_delete( destroy_on_free );
    lut       = shm_lut->buffer();
    lut_bytes = shm_lut->data_size();
  } else {
    lut_bytes = width * height * depth * bytes_per_sample;
    lut = (unsigned char *)malloc( lut_bytes );
  }
  memset(lut, C_OTHER, lut_bytes);
}


/** Load colormap from file to specific buffer.
 * @param file file to load from
 * @param buffer buffer to write data to
 * @param buffer_size size of buffer
 * @exception TypeMismatchException thrown if the colormaps do not have the
 * same size.
 * @exception CouldNotOpenFileException failed to open the given file
 * @exceptoin FileReadException thrown if reading failed
 */
void
ColorModelLookupTable::load_to_buffer(const char *file, unsigned char *buffer,
				      off_t buffer_size)
{
  struct stat st;
  if ( stat(file, &st) != 0 ) {
    throw CouldNotOpenFileException(file, errno, "Could not stat colormap file");
  }

  if ( st.st_size != buffer_size ) {
    TypeMismatchException tme("File does not have the required size");
    tme.append("Got %lu bytes but expected %lu bytes", st.st_size, buffer_size);
    throw tme;
  }

  FILE *f = fopen(file, "r");
  if (f == NULL) {
    throw CouldNotOpenFileException(file, errno, "Could not open colormap file");
  }


  if ( (fread(buffer, buffer_size, 1, f) == 0) && (! feof(f)) && (ferror(f) != 0)) {
    fclose(f);
    throw FileReadException(file, errno, "Could not read colormap data from file");
  }

  fclose(f);
}

/** Load LUT from file.
 * @param file filename
 * @exception CouldNotOpenFileException file could not be opened for whatever reason
 * @exception TypeMismatchException file does not have the exact size of this colormap
 * @exception FileReadException data could not be read from the file
 */
void
ColorModelLookupTable::load(const char *file)
{
  load_to_buffer(file, lut, lut_bytes);
}


/** Save LUT to file.
 * @param file filename
 */
void
ColorModelLookupTable::save(const char *file)
{
  FILE *f = fopen(file, "w");

  if (f == NULL) {
    cout << "ColorModelLookupTable: Cannot open file '" << file << "' for writing" << endl;
    return;
  }

  if ( fwrite(lut, lut_bytes, 1, f) == 0) {
    cout << "ColorModelLookupTable: Could not write!" << endl;
  } 

  fclose(f);
}


/** Reset lookup table.
 * This will set all entries to C_OTHER.
 */
void
ColorModelLookupTable::reset()
{
  memset(lut, C_OTHER, lut_bytes);
}


/** Set the maximal input value for the three dimensions.
 * For a YUV lookup table this would be 256 for all dimensions.
 * @param x_max the maximal value in the 1st dimension
 * @param y_max the maximal value in the 2nd dimension
 * @param z_max the maximal value in the 3rd dimension
 */
void
ColorModelLookupTable::set_range(unsigned int x_max, 
				 unsigned int y_max, 
				 unsigned int z_max)
{
  this->x_max = x_max;
  this->y_max = y_max;
  this->z_max = z_max;
}


/** Destructor. */
ColorModelLookupTable::~ColorModelLookupTable()
{
  if ( lut_id != NULL ) {
    delete shm_lut;
    free(lut_id);
  } else {
    free(lut);
  }
  lut = NULL;
  lut_id = NULL;
  lut_bytes = 0;
}


color_t
ColorModelLookupTable::determine(unsigned int x,
				 unsigned int y,
				 unsigned int z) const
{
  unsigned int x_index;
  unsigned int y_index;
  unsigned int z_index;

  x_index = (unsigned int)( x / float(x_max) * float(width) );
  y_index = (unsigned int)( y / float(y_max) * float(height) );
  z_index = (unsigned int)( z / float(z_max) * float(depth) );

  return determine_cr(x_index, y_index, z_index);
}


/** Similiar to determine() but assumes that the x, y, z values
 * are in the range 0 <= x < lut_width, etc.
 * The suffix 'cr' stands for compressed range.
 * @param x the x coordinate
 * @param y the y coordinate
 * @param z the z coordinate
 */
color_t
ColorModelLookupTable::determine_cr(unsigned int x,
				    unsigned int y,
				    unsigned int z) const
{
  if ( x >= width || y >= height || z >= depth ) {
    throw Exception("LookupTable::determine: height=%d, width=%d, or depth=%d out of bounds", x, y, z);
  }

  return (color_t) *(lut + (x * height * depth * bytes_per_sample) + 
		     (y * depth * bytes_per_sample) + (z * bytes_per_sample));
}


const char *
ColorModelLookupTable::get_name()
{
  return "ColorModelLookupTable";
}


/** Set the appropriate value.
 * @param x x value of entry to set
 * @param y y value of entry to set
 * @param z z value of entry to set
 * @param c color class to set this entry to
 * @param full_range if true, the x, y, z are assumed to be in the interval
 * 0 <= x < max_x, etc. if false they are assumed to be in the interval
 * 0 <= x < lut_width, etc.
 */
void
ColorModelLookupTable::set(unsigned int x,
			   unsigned int y,
			   unsigned int z,
			   color_t      c,
			   bool full_range)
{
  unsigned int x_index;
  unsigned int y_index;
  unsigned int z_index;

  if (full_range) {
    x_index = (unsigned int)( x / float(x_max) * float(width) );
    y_index = (unsigned int)( y / float(y_max) * float(height) );
    z_index = (unsigned int)( z / float(z_max) * float(depth) );
  } else {
    x_index = x;
    y_index = y;
    z_index = z;
  }

  if ( x_index >= width || y_index >= height || z_index >= depth ) {
    throw OutOfBoundsException("Lookuptable::set: height, width, or depth out of bounds");
  }
  
  *( lut + (x_index * height * depth * bytes_per_sample) 
     + (y_index * depth * bytes_per_sample) + (z_index * bytes_per_sample) ) = c;
}


/** Copy another LUT buffer.
 * @param buffer buffer to copy, must be at least of the size as this
 * buffer. */
void
ColorModelLookupTable::set(unsigned char *buffer)
{
  memcpy(lut, buffer, lut_bytes);
}


/** Get size of LUT in bytes.
 * @return size of LUT in bytes. */
unsigned int
ColorModelLookupTable::size()
{
  return lut_bytes;
}

/** Get the raw lut buffer
 * this should only be used if absolutely necessary, the buffer will persist
 * as long as this instance exists.
 * @return the raw buffer, aligned as height consecutive lines of
 *         size width * bytes_per_cell which consist of width number of cells which
 *         are each bytes_per_cell wide and consecutive
 *      
 */
unsigned char *
ColorModelLookupTable::get_buffer()
{
  return lut;
}


/** Add another colormap to this one.
 * The other colormap is traversed. If this colormap has an entry set to background
 * or other but the supplied colormap has another specific object entry the entry
 * in this colormap is set to the object. If the colormaps disagree this colormap
 * takes precedence.
 * @param cmlt color model lookup table to add
 * @return this instance with the other color model lookup table added
 * @exception TypeMismatchException thrown if the colormaps do not have the
 * same size.
 */
ColorModelLookupTable &
ColorModelLookupTable::operator+=(const ColorModelLookupTable &cmlt)
{
  if ( (width != cmlt.width) || (height != cmlt.height) || (depth != cmlt.depth) ) {
    throw TypeMismatchException("Colormaps are of different sizes");
  }

  unsigned char *this_lut = lut;
  unsigned char *other_lut = cmlt.lut;

  for (unsigned int i = 0; i < width * height * depth; ++i) {
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


/** Add another colormap to this one.
 * This is an overloaded member function for convenience. It adds the lookup table
 * stored in the given file.
 * @param file filename of the file with the LUT to add
 * @return this instance with the other color model lookup table added
 * @exception TypeMismatchException thrown if the colormaps do not have the
 * same size.
 * @exception CouldNotOpenFileException failed to open the given file
 * @exception OutOfMemoryException not enough memory for the temporary LUT buffer
 */
ColorModelLookupTable &
ColorModelLookupTable::operator+=(const char *file)
{
  unsigned char *tmp = (unsigned char *)malloc( lut_bytes );
  
  try {
    load_to_buffer(file, tmp, lut_bytes);
  } catch (Exception &e) {
    throw;
  }

  unsigned char *this_lut = lut;
  unsigned char *other_lut = tmp;

  for (unsigned int i = 0; i < width * height * depth; ++i) {
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

  free(tmp);

  return *this;
}


/** Create image from LUT.
 * Create image from LUT, useful for debugging and analysing.
 * @param yuv422_planar_buffer contains the image upon return, must be initialized
 * with the appropriate size beore.
 */
void
ColorModelLookupTable::to_image(unsigned char *yuv422_planar_buffer)
{
  unsigned int y_plane = width / 2;

  if (y_plane >= width) {
    throw OutOfBoundsException("LookupTable::to_image: y_plane");
  }

  unsigned char *yp = yuv422_planar_buffer;
  unsigned char *up = YUV422_PLANAR_U_PLANE(yuv422_planar_buffer, 512, 512);
  unsigned char *vp = YUV422_PLANAR_V_PLANE(yuv422_planar_buffer, 512, 512);

  color_t c;
  for (unsigned int v = depth; v > 0 ; --v) {
    for (unsigned int u = 0; u < height; ++u) {

      c = determine_cr(y_plane, u, v-1);

      switch (c) {
      case C_ORANGE:
	*yp++ = 128;
	*yp++ = 128;
	*up++ =  30;
	*vp++ = 220;
	break;

      case C_BLUE:
	*yp++ =   0;
	*yp++ =   0;
	*up++ = 255;
	*vp++ = 128;
	break;

      case C_YELLOW:
	*yp++ = 255;
	*yp++ = 255;
	*up++ =   0;
	*vp++ = 128;
	break;

      case C_CYAN:
	*yp++ = 255;
	*yp++ = 255;
	*up++ = 128;
	*vp++ =   0;
	break;

      case C_MAGENTA:
	*yp++ = 128;
	*yp++ = 128;
	*up++ = 128;
	*vp++ = 255;
	break;

      case C_WHITE:
	*yp++ = 255;
	*yp++ = 255;
	*up++ = 128;
	*vp++ = 128;
	break;

      case C_BLACK:
      case C_BACKGROUND:
	*yp++ =   0;
	*yp++ =   0;
	*up++ = 128;
	*vp++ = 128;
	break;

      case C_GREEN:
	*yp++ = 128;
	*yp++ = 128;
	*up++ =   0;
	*vp++ =   0;
	break;

      case C_RED:
	*yp++ = 128;
	*yp++ = 128;
	*up++ =   0;
	*vp++ = 255;
	break;

      default:
	*yp++ = 128;
	*yp++ = 128;
	*up++ = 128;
	*vp++ = 128;
	break;
      }
    }
    // Double line
    memcpy(yp, (yp - 512), 512);
    yp += 512;
    memcpy(up, (up - 256), 256);
    memcpy(vp, (vp - 256), 256);
    up += 256;
    vp += 256;
  }

}


/** Compose filename.
 * In the format %g is replaced with the hostname.
 * @param format format for the filename
 */
std::string
ColorModelLookupTable::compose_filename(const std::string format)
{
  string rv = format;

  struct utsname uname_info;
  uname( &uname_info );

  size_t loc = rv.find( "%h" );
  while (loc != string::npos) {
    rv.replace( loc, 2, uname_info.nodename );
    loc = rv.find( "%h" );
  }

  return rv;
}
