
/***************************************************************************
 *  fileloader.cpp - A camera which obtains its images from a single image
 *                   file or from several image files in a directory
 *
 *  Generated: Tue Feb 22 13:28:08 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *             2008       Daniel Beck
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

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <fvcams/fileloader.h>
#include <fvutils/writers/fvraw.h>
#include <fvutils/system/filetype.h>
#include <fvutils/system/camargp.h>
#include <fvutils/colormap/cmfile.h>
#include <fvutils/colormap/colormap.h>

#include <fvutils/readers/fvraw.h>
#ifdef HAVE_LIBJPEG
#include <fvutils/readers/jpeg.h>
#endif
#ifdef HAVE_LIBPNG
#include <fvutils/readers/png.h>
#endif

#include <cstring>
#include <cstdlib>
#include <cstdio>

#include <sys/types.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FileLoader <fvcams/fileloader.h>
 * Load images from files.
 * The file loader tries to determine the image format of the given image using
 * the the file type utility. Currently it recognizes JPEG and FvRaw image files.
 *
 * @author Tim Niemueller
 * @author Daniel Beck
 */

char* FileLoader::extension = NULL;

#if defined(__GLIBC__) || defined(__FreeBSD__)
int file_select(const struct dirent* ent)
#else
int file_select(struct dirent *ent)
#endif
{
  if ( !FileLoader::extension ) { return 1; }

  // NOTE: this only checks whether the filename contains the
  // extension and not whether it ends with it.
  if ( NULL != strstr(ent->d_name, FileLoader::extension) ) {
    return 1; 
  }

  return 0;
}

/** Constructor.
 * @param filename name of file to open, full path or relative to working directory
 */
FileLoader::FileLoader(const char *filename)
{
  this->filename = strdup(filename);
  this->dirname = NULL;
  this->extension = NULL;
  this->file_list = NULL;
  num_files = 0;
  cur_file = 0;
  opened = started = false;
  width = height = 0;
  file_buffer = NULL;
  this->cspace = CS_UNKNOWN;
}


/** Constructor.
 * Initialize with the parameters from the given camera argument parser. The following
 * parameters are supported:
 * - file=FILENAME: open the given file
 * - dir=DIRECTORY: sequentially open files in this directory
 * - ext=EXTENSION: only open files with this extension
 * - width=W: width in pixels of image
 * - height=H: height in pixels of image
 * - colorspace=C: colorspace of image
 *
 * Width, height, and colorspace are used for raw images without an header, e.g if
 * captured by v4l2-ctl.
 *
 * @param cap camera argument parser
 */
FileLoader::FileLoader(const CameraArgumentParser *cap)
{
  filename = NULL;
  dirname = NULL;

  file_list = NULL;
  num_files = 0;
  cur_file = 0;
  width = height = 0;
  file_buffer = NULL;
  this->cspace = CS_UNKNOWN;
  opened = started = false;

  if ( cap->has("file") ) {
    this->filename = strdup(cap->get("file").c_str());
    if (cap->has("width")) {
      width = cap->get_int("width");
    }
    if (cap->has("height")) {
      height = cap->get_int("height");
    }
    if (cap->has("colorspace")) {
      cspace = colorspace_by_name(cap->get("colorspace").c_str());
    }
  } else if ( cap->has("dir") ) {
    this->dirname = strdup( cap->get("dir").c_str() );
    if ( cap->has("ext") ) {
      this->extension = strdup( cap->get("ext").c_str() );
    }
  } else {
    throw MissingParameterException("Neither parameter file nor parameter directory are present");
  }

}


/** Legacy constructor.
 * Before FvRaw FireVision had the ability to store the pure buffer of an image
 * without any header. Because of this additional information like colorspace,
 * width and height of the image had to be supplied. The number of bytes that
 * has to be read for the image is calculated from the given parameters.
 * @param cspace color space of image
 * @param filename filename to open
 * @param width width of image
 * @param height height of image
 */
FileLoader::FileLoader(colorspace_t cspace, const char *filename,
		       unsigned int width, unsigned int height)
{
  started = opened = false;
  this->cspace = cspace;
  this->width = width;
  this->height = height;
  this->filename = strdup(filename);
  this->dirname = NULL;
  this->extension = NULL;
  this->file_list = NULL;
  num_files = 0;
  cur_file = 0;
  file_buffer = NULL;
}


/** Destructor. */
FileLoader::~FileLoader()
{
  for (int i = 0; i < num_files; ++i) {
    free(file_list[i]);
  }
  free(file_list);
  free(dirname);
  free(extension);
  free(filename);
}


void
FileLoader::open()
{
  if (opened) return;

  if (dirname) {
    num_files = scandir(dirname, &file_list, file_select, alphasort);
    
    if ( -1 == num_files ) {
      throw Exception("Error while scanning directory %s", dirname);
    }
  }

  read_file();
  opened = true;
}


void
FileLoader::start()
{
  if (started) return;

  if (!opened) {
    throw Exception("Trying to start closed file");
  }

  started = true;
}

void
FileLoader::stop()
{
  started = false;
}


void
FileLoader::print_info()
{
}


void
FileLoader::capture()
{
  if (0 != num_files) {
    if (file_buffer) {
      free(file_buffer);
    }

    read_file();

    if (++cur_file == num_files) {
      cur_file = 0;
    }
  }
}


unsigned char*
FileLoader::buffer()
{
  return file_buffer;
}


unsigned int
FileLoader::buffer_size()
{
  return _buffer_size;
}


void
FileLoader::close()
{
  if (file_buffer != NULL) {
    free(file_buffer);
    file_buffer = NULL;
  }
  opened = false;
}


void
FileLoader::dispose_buffer()
{
}


void
FileLoader::flush()
{
}


bool
FileLoader::ready()
{
  return started;
}


void
FileLoader::set_image_number(unsigned int n)
{
}


unsigned int
FileLoader::pixel_width()
{
  return width;
}


unsigned int
FileLoader::pixel_height()
{
  return height;
}


colorspace_t
FileLoader::colorspace()
{
  return cspace;
}


/** Set the colorspace of the image.
 * @param c colorspace
 */
void
FileLoader::set_colorspace(colorspace_t c)
{
  cspace = c;
}


/** Set width.
 * @param w image width in pixels
 */
void
FileLoader::set_pixel_width(unsigned int w)
{
  width = w;
}


/** Set height.
 * @param h image height in pixels
 */
void
FileLoader::set_pixel_height(unsigned int h)
{
  height = h;
}

void
FileLoader::read_file()
{
  char* fn;
  if (0 != num_files) {
    if (asprintf(&fn, "%s/%s", dirname, file_list[cur_file]->d_name) == -1) {
      throw OutOfMemoryException("FileLoader::read_file(): asprintf() failed (2)");
    }
  } else {
    fn = strdup(filename);
  }

  std::string ft = fv_filetype_file( fn );

  if ( ft == "FvRaw" ) {
    FvRawReader *fvrr = new FvRawReader( fn );
    cspace = fvrr->colorspace();
    width  = fvrr->pixel_width();
    height = fvrr->pixel_height();
    _buffer_size = colorspace_buffer_size( cspace, width, height );
    file_buffer = (unsigned char*)malloc(_buffer_size);
    fvrr->set_buffer( file_buffer );
    try {
      fvrr->read();
    } catch (Exception &e) {
      delete fvrr;
      e.append("FileLoader::open() failed");
      throw;
    }
    delete fvrr;

#ifdef HAVE_LIBJPEG
  } else if ( ft.find( "JPEG" ) != std::string::npos ) {
    JpegReader *jr = new JpegReader( fn );
    cspace = jr->colorspace();
    width  = jr->pixel_width();
    height = jr->pixel_height();
    _buffer_size = colorspace_buffer_size( cspace, width, height );
    file_buffer = (unsigned char*)malloc(_buffer_size);
    jr->set_buffer( file_buffer );
    try {
      jr->read();
    } catch (Exception &e) {
      delete jr;
      e.append("FileLoader::open() failed");
      throw;
    }
    delete jr;
#endif

#ifdef HAVE_LIBPNG
  } else if ( ft.find( "PNG" ) != std::string::npos ) {
    PNGReader *pr = new PNGReader( fn );    cspace = pr->colorspace();
    width  = pr->pixel_width();
    height = pr->pixel_height();
    _buffer_size = colorspace_buffer_size( cspace, width, height );
    file_buffer = (unsigned char*)malloc(_buffer_size);
    pr->set_buffer( file_buffer );
    try {
      pr->read();
    } catch (Exception &e) {
      delete pr;
      e.append("FileLoader::open() failed for PNG");
      throw;
    }
    delete pr;
#endif

  } else if ( ft == "FvColormap" ) {
    ColormapFile cmf;
    cmf.read(fn);

    Colormap *colormap = cmf.get_colormap();
    cspace = YUV422_PLANAR;
    width  = colormap->width() * 2;
    height = colormap->height() * 2;
    _buffer_size = colorspace_buffer_size( cspace, width, height );
    file_buffer = (unsigned char*)malloc(_buffer_size);
    colormap->to_image(file_buffer);

    delete colormap;

  } else {
    _buffer_size = colorspace_buffer_size( cspace, width, height );

    if (_buffer_size > 0) {
      FILE *f;
      f = fopen( fn, "rb" );
      file_buffer = (unsigned char*)malloc(_buffer_size);
      if (fread(file_buffer, _buffer_size, 1, f) != 1) {
	// cout << "FileLoader: Could not read data." << endl;
	fclose(f);
	throw Exception("Could not read data");
      }
      fclose(f);
    } else {
      throw Exception("Invalid color space (buffer size is 0)");
    }
  }

  free(fn);
}

} // end namespace firevision
