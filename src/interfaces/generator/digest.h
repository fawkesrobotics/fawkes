 
/***************************************************************************
 *  digest.h - Interface digest generator
 *
 *  Created: Thu Feb 28 15:47:45 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __INTERFACES_GENERATOR_DIGEST_H_
#define __INTERFACES_GENERATOR_DIGEST_H_

#include <string>
#include <cstddef>

class InterfaceDigest
{
 public:
  InterfaceDigest(std::string config_filename);
  ~InterfaceDigest();

  const unsigned char * get_hash();
  size_t                get_hash_size();

 private:
  unsigned char *digest;
  size_t         digest_size;
};


#endif
