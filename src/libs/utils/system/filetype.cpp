
/***************************************************************************
 *  filetype.cpp - little utility to decide on filetype
 *
 *  Generated: Sun Oct 26 10:52:59 2008 (split off cpp file)
 *  Copyright  2005-2008  Tim Niemueller [www.niemueller.de]
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

#include <utils/system/filetype.h>
#include <core/exception.h>

#include <magic.h>

namespace fawkes {

/** Get filetype of file.
 * Returns a long decriptive string of the filetype, similar to the file
 * console utility.
 * @param filename path to the file whose type should be determined
 * @return descriptive string
 */
std::string
filetype_file(const char *filename)
{
  std::string rv;

  magic_t m = magic_open( MAGIC_ERROR );
  magic_load( m, NULL );

  const char * res = magic_file( m, filename );
  if ( res == NULL ) {
    fawkes::Exception e("Failed to determine file type of %s: %s", filename, magic_error(m));
    magic_close(m);
    throw e;
  }

  rv = res;
  magic_close( m );

  return rv;
}


/** Get mime-type of file.
 * This function gives a brief mime-type for the given file.
 * @param filename path to the file whose type should be determined
 * @return descriptive string
 * @param filename 
 */
std::string
mimetype_file(const char *filename)
{
  std::string rv;

#ifdef MAGIC_MIME_TYPE
  magic_t m = magic_open( MAGIC_ERROR | MAGIC_MIME_TYPE );
#else
  magic_t m = magic_open( MAGIC_ERROR | MAGIC_MIME );
#endif
  magic_load( m, NULL );

  const char * res = magic_file( m, filename );
  if ( res == NULL ) {
    fawkes::Exception e("Failed to determine mime type of %s: %s", filename, magic_error(m));
    magic_close(m);
    throw e;
  }

  rv = res;
#ifndef MAGIC_MIME_TYPE
  rv = rv.substr(0, rv.find(","));
#endif
  magic_close(m);
  return rv;
}

} // end namespace fawkes

