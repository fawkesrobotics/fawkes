
/***************************************************************************
 *  sdl_keeper.h - utility to keep track of SDL initialization state
 *
 *  Created: Mon Nov 05 14:33:52 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVWIDGETS_SDL_KEEPER_H_
#define __FIREVISION_FVWIDGETS_SDL_KEEPER_H_

#include <core/threading/mutex.h>

class SDLKeeper
{
 public:
  static void init(unsigned int flags);
  static void quit() throw();

  static void force_quit();

 private:
  SDLKeeper();

  static unsigned int  _refcount;
  static Mutex         _mutex;
};

#endif
