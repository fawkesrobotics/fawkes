
/***************************************************************************
 *  seq_writer.h - Writes sequences of images
 *
 *  Generated: Tue Jul 03 08:14:56 2007
 *  Copyright  2007  Daniel Beck
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

#ifndef __FIREVISION_FVUTILS_WRITERS_SEQ_WRITER_H_
#define __FIREVISION_FVUTILS_WRITERS_SEQ_WRITER_H_

#include <fvutils/color/colorspaces.h>
#include <fvutils/writers/writer.h>
#include <core/exception.h>

class SeqWriter {
  
 public:
  SeqWriter(Writer* writer);
  ~SeqWriter();

  void set_path(const char* img_path);
  void set_filename(const char* file_name);

  void set_dimensions(unsigned int width, unsigned int height);
  void set_colorspace(colorspace_t _cspace);

  void write(unsigned char* buffer);

 private:
  Writer* writer;
  char* filename;
  char* img_path;
  colorspace_t cspace;
  unsigned int frame_number;

};


#endif /*  __FIREVISION_FVUTILS_WRITERS_SEQ_WRITER_H_ */
