
/**************************************************************************
 *  cmfile.h - FVFF Colormap File Format
 *
 *  Created: Sat Mar 29 12:49:48 2008
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

#ifndef __FIREVISION_FVUTILS_COLORMAP_CMFILE_H_
#define __FIREVISION_FVUTILS_COLORMAP_CMFILE_H_

#include <fvutils/fileformat/fvfile.h>
#include <fvutils/colormap/cmfile_block.h>
#include <vector>
#include <string>
#include <stdint.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Colormap;

#define CMFILE_MAGIC_TOKEN  0xFF01
#define CMFILE_CUR_VERSION  2

#define CMFILE_TYPE_YUV     1

#pragma pack(push,4)
/** Block header for a Colormap header block in a ColormapFile. */
typedef struct {
  uint16_t depth;      /**< Y resolution */
  uint16_t width;      /**< U resolution */
  uint16_t height;     /**< V resolution */
  uint16_t reserved;   /**< reserved for future use, padding */
} cmfile_header_t;
#pragma pack(pop)

class ColormapFile : public FireVisionDataFile
{
 public:
  ColormapFile();
  ColormapFile(uint16_t depth, uint16_t width, uint16_t height);

  class ColormapBlockVector : public std::vector<ColormapFileBlock *>
  {
   public:
    ~ColormapBlockVector();
  };

  void                   add_colormap(Colormap *colormap);
  ColormapBlockVector *  colormap_blocks();
  Colormap *             get_colormap();

  uint16_t                get_depth();
  uint16_t                get_width();
  uint16_t                get_height();

  static bool            is_colormap_file(const char *filename);
  static std::string     compose_filename(const std::string format);

  virtual void           clear();

 private:
  inline void assert_header();
 private:
  cmfile_header_t  *__header;
};

} // end namespace firevision

#endif
