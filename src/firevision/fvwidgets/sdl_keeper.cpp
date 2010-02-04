
/***************************************************************************
 *  sdl_keeper.cpp - utility to keep track of SDL initialization state
 *
 *  Created: Mon Nov 05 14:34:36 2007
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

#include <fvwidgets/sdl_keeper.h>

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/exception.h>

#include <SDL.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

unsigned int SDLKeeper::_refcount = 0;
Mutex        SDLKeeper::_mutex;


/** @class SDLKeeper <fvwidgets/sdl_keeper.h>
 * SDL Reference keeper.
 *
 * Use this keeper to initialize and quit the SDL library. As there may be many
 * modules using the SDL a central place for reference counting is needed.
 *
 * @author Tim Niemueller
 */

/** Private inaccessible constructor. */
SDLKeeper::SDLKeeper()
{
}


/** Init SDL.
 * Keeps track of SDL_Init calls and only calls SDL_InitSubSystem on consecutive
 * calls.
 * @param flags Same flags as for SDL_Init
 */
void
SDLKeeper::init(unsigned int flags)
{
  MutexLocker lock(&_mutex); 

  unsigned int alive_subsys = SDL_WasInit(SDL_INIT_EVERYTHING);
  if ( (alive_subsys & flags) != flags ) {
    // Subsystem has not been initialized, yet
    if ( _refcount == 0 ) {
      if ( SDL_Init(flags) != 0 ) {
	throw Exception("SDL: initialization failed");
      }
    } else {
      unsigned int still_to_init = ~alive_subsys & flags;
      if ( SDL_Init(still_to_init) != 0 ) {
	throw Exception("SDL: initialization failed");
      }
    }
  }

  ++_refcount;
}


/** Conditionally quit SDL.
 * Use this after you are done with the SDL. No subsystem will be closed after all
 * users of SDL quit the usage. Then the whole SDL will be released at once.
 */
void
SDLKeeper::quit() throw()
{
  MutexLocker lock(&_mutex); 

  if ( (_refcount > 0) && (--_refcount == 0) ) {
    SDL_Quit();
  }
}


/** Force quit of SDL.
 * This will quit the SDL no matter of the reference count. Use with extreme care.
 */
void
SDLKeeper::force_quit()
{
  MutexLocker lock(&_mutex); 

  SDL_Quit();
  _refcount = 0;
}

} // end namespace firevision
