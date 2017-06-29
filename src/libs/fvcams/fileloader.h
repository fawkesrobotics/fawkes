
/***************************************************************************
 *  fileloader.h - A camera which obtains its images from a single image
 *                 file or from several image files in a directory
 *
 *  Generated: Tue Mar  2 12:26:44 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CAMS_FILELOADER_H_
#define __FIREVISION_CAMS_FILELOADER_H_

#include <fvcams/camera.h>
#include <dirent.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;

class FileLoader : public Camera
{
/// @cond DOXYGEN_BUG
#if defined(__GLIBC__) || defined(__FreeBSD__)
  friend int file_select(const struct dirent*);
#else
  friend int file_select(struct dirent*);
#endif
/// @endcond

 public:

  FileLoader(const char *filename);
  FileLoader(colorspace_t cspace, const char* filename, unsigned int width, unsigned int height);
  FileLoader(const CameraArgumentParser *cap);
  ~FileLoader();

  virtual void             open();
  virtual void             start();
  virtual void             stop();
  virtual void             close();
  virtual void             capture();
  virtual void             flush();

  virtual bool             ready();

  virtual void             print_info();

  virtual unsigned char *  buffer();
  virtual unsigned int     buffer_size();
  virtual void             dispose_buffer();

  virtual unsigned int     pixel_width();
  virtual unsigned int     pixel_height();
  virtual colorspace_t     colorspace();

  virtual void             set_image_number(unsigned int n);

  void                     set_colorspace(colorspace_t c);
  void                     set_pixel_width(unsigned int w);
  void                     set_pixel_height(unsigned int h);

 private:
  void                     read_file();

  bool started;
  bool opened;
  unsigned char* file_buffer;
  int _buffer_size;
  unsigned int width;
  unsigned int height;
  colorspace_t cspace;
  char *filename;
  char *dirname;
  static char *extension;
  int num_files;
  int cur_file;
  struct dirent **file_list;
};

} // end namespace firevision

#endif
