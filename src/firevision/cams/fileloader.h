
/***************************************************************************
 *  fireloader.h - This header defines a dummy file loader camera
 *
 *  Generated: Tue Mar  2 12:26:44 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CAMS_FILELOADER_H_
#define __FIREVISION_CAMS_FILELOADER_H_

#include <cams/camera.h>

class CameraArgumentParser;

class FileLoader : public Camera
{

 public:

  FileLoader(const char *filename);
  FileLoader(colorspace_t cspace, const char* filename, unsigned int width, unsigned int height);
  FileLoader(const CameraArgumentParser *cap);

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
  bool started;
  bool opened;
  unsigned char* file_buffer;
  int _buffer_size;
  unsigned int width;
  unsigned int height;
  colorspace_t cspace;
  const char *filename;
};

#endif
