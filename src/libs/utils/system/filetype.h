
/***************************************************************************
 *  filetype.h - little utility to decide on filetype
 *
 *  Generated: Tue Feb 23 13:49:38 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __UTILS_FILETYPE_H_
#define __UTILS_FILETYPE_H_

#include <magic.h>
#include <string>

inline std::string
filetype_file(const char *filename)
{
  std::string rv;

  magic_t m = magic_open( MAGIC_ERROR );
  magic_load( m, NULL );

  const char * res = magic_file( m, filename );
  if ( res == NULL ) {
    res = magic_error( m );
  }

  rv = res;

  magic_close( m );

  return rv;
}


#endif
