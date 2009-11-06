
/***************************************************************************
 *  seq_writer.h - Writes sequences of images
 *
 *  Generated: Tue Jul 03 08:14:56 2007
 *  Copyright  2007  Daniel Beck
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

#ifndef __FIREVISION_FVUTILS_WRITERS_SEQ_WRITER_H_
#define __FIREVISION_FVUTILS_WRITERS_SEQ_WRITER_H_

#include <fvutils/color/colorspaces.h>
#include <fvutils/writers/writer.h>
#include <core/exception.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SeqWriter {
  
 public:
  SeqWriter(Writer* writer);
  ~SeqWriter();

  void set_path(const char* img_path);
  void set_filename(const char* filename);

  void set_dimensions(unsigned int width, unsigned int height);
  void set_colorspace(colorspace_t cspace);

  void write(unsigned char* buffer);

 private:
  Writer* writer;
  char* filename;
  char* img_path;
  colorspace_t cspace;
  unsigned int frame_number;

};

} // end namespace firevision

#endif /*  __FIREVISION_FVUTILS_WRITERS_SEQ_WRITER_H_ */
