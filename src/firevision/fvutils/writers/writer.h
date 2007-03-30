
/***************************************************************************
 *  writer.h - Writer interface
 *
 *  Generated: Thu Jun 02 18:24:35 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_WRITERS_WRITER_H_
#define __FIREVISION_FVUTILS_WRITERS_WRITER_H_

#include <fvutils/color/colorspaces.h>

class Writer {

 public:
  virtual ~Writer();

  virtual void             set_filename(const char *filename)                      = 0;
  virtual void             set_dimensions(unsigned int width, unsigned int height) = 0;
  virtual void             set_buffer(colorspace_t cspace, unsigned char *buffer)  = 0;
  virtual void             write()                                                 = 0;

};



#endif
