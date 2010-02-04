
/***************************************************************************
 *  sdl_keeper.h - utility to keep track of SDL initialization state
 *
 *  Created: Mon Nov 05 14:33:52 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVWIDGETS_SDL_KEEPER_H_
#define __FIREVISION_FVWIDGETS_SDL_KEEPER_H_

#include <core/threading/mutex.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SDLKeeper
{
 public:
  static void init(unsigned int flags);
  static void quit() throw();

  static void force_quit();

 private:
  SDLKeeper();

  static unsigned int  _refcount;
  static fawkes::Mutex         _mutex;
};

} // end namespace firevision

#endif
