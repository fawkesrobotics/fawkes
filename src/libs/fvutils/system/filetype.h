
/***************************************************************************
 *  filetype.h - little utility to decide on filetype, this FireVision
 *               version can also detect FvRaw images.
 *
 *  Generated: Tue Feb 23 13:49:38 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_FILETYPE_H_
#define __FIREVISION_FVUTILS_FILETYPE_H_

#include <core/exception.h>
#include <utils/system/filetype.h>
#include <fvutils/readers/fvraw.h>
#include <fvutils/colormap/cmfile.h>
#include <string>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

inline std::string
fv_filetype_file(const char *filename)
{
  std::string rv;

  try {
    rv = fawkes::filetype_file(filename);
  } catch (fawkes::Exception &e) {
    // Some error occured, most likely libmagic was not available at compile
    // time, assume data to make our own checks, we just cannot detect JPG
    // and PNG in this case.
    rv = "data";
  }

  if ( rv == "data" ) {
    if ( FvRawReader::is_FvRaw(filename) ) {
      rv = "FvRaw";
    } else if ( ColormapFile::is_colormap_file(filename) ) {
      rv = "FvColormap";
    }
  }

  return rv;
}

} // end namespace firevision


#endif
