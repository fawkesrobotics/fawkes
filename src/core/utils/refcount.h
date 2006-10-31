
/***************************************************************************
 *  refcount.h - reference counting base class
 *
 *  Created: Fri Oct 27 13:43:50 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __CORE_UTILS_REFCOUNT_H_
#define __CORE_UTILS_REFCOUNT_H_

class Mutex;

class RefCount
{
 public:
  RefCount();
  virtual ~RefCount();

  void              ref();
  void              unref();
  unsigned int      refcount();

 private:

  unsigned int  refc;
  Mutex        *ref_mutex;
};


#endif
