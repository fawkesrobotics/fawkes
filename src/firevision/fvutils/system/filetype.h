
/***************************************************************************
 *  filetype.h - little utility to decide on filetype, this FireVision
 *               version can also detect FvRaw images.
 *
 *  Generated: Tue Feb 23 13:49:38 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __FIREVISION_FVUTILS_FILETYPE_H_
#define __FIREVISION_FVUTILS_FILETYPE_H_

#include <utils/system/filetype.h>
#include <fvutils/readers/fvraw.h>
#include <string>

inline std::string
fv_filetype_file(const char *filename)
{
  std::string rv = filetype_file(filename);

  if ( rv == "data" ) {
    if ( FvRawReader::is_FvRaw(filename) ) {
      rv = "FvRaw";
    }
  }

  return rv;
}


#endif
