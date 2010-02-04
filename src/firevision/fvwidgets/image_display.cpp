
/***************************************************************************
 *  image_display.cpp - widget to display an image based on SDL
 *
 *  Created: Mon Nov 05 14:19:26 2007
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

#include <fvwidgets/image_display.h>

#include <fvwidgets/sdl_keeper.h>
#include <SDL.h>

#include <core/exception.h>
#include <fvutils/color/conversions.h>
#include <fvutils/color/yuv.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ImageDisplay <fvwidgets/image_display.h>
 * Simple image display.
 * This is a simple thin wrapper around the SDL to display images in a standalone
 * window. Use this for instance for easy verification of vision results.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param width width of image
 * @param height height of image
 * @param title window title
 */
ImageDisplay::ImageDisplay(unsigned int width, unsigned int height, const char* title)
{

  SDLKeeper::init(SDL_INIT_VIDEO);
  if (title) SDL_WM_SetCaption (title, NULL);

  _width  = width;
  _height = height;

  int bpp = SDL_VideoModeOK(_width, _height, 16, SDL_ANYFORMAT);
  _surface = SDL_SetVideoMode(width, height, bpp, /* flags */ SDL_HWSURFACE | SDL_ANYFORMAT);
  if ( ! _surface ) {
    throw Exception("SDL: cannot create surface");
  }

  // SDL_UYVY_OVERLAY
  _overlay = SDL_CreateYUVOverlay(width, height, SDL_UYVY_OVERLAY, _surface);
  if ( ! _overlay ) {
    throw Exception("Cannot create overlay");
  }

  _rect = new SDL_Rect;

  _rect->x = 0;
  _rect->y = 0;
  _rect->w = _width;
  _rect->h = _height;
}


/** Destructor. */
ImageDisplay::~ImageDisplay()
{
  delete _rect;

  SDL_FreeYUVOverlay(_overlay);
  SDL_FreeSurface(_surface);

  SDLKeeper::quit();
}


/** Show image from given colorspace.
 * @param colorspace colorspace of the supplied buffer
 * @param buffer image buffer
 */
void
ImageDisplay::show(colorspace_t colorspace, unsigned char *buffer)
{
  SDL_LockYUVOverlay(_overlay);
  convert(colorspace, YUV422_PACKED, buffer, _overlay->pixels[0], _width, _height);
  SDL_UnlockYUVOverlay(_overlay);
  SDL_DisplayYUVOverlay(_overlay, _rect);
}


/** Show image from YUV422_PLANAR colorspace.
 * @param yuv422_planar_buffer YUV422_PLANAR encoded image.
 */
void
ImageDisplay::show(unsigned char *yuv422_planar_buffer)
{
  SDL_LockYUVOverlay(_overlay);

  yuv422planar_to_yuv422packed(yuv422_planar_buffer, _overlay->pixels[0],
			       _width, _height);

  SDL_UnlockYUVOverlay(_overlay);
  SDL_DisplayYUVOverlay(_overlay, _rect);
}

/** Process a few SDL events.
 * @param max_num_events maximum number of events to process.
 */
void
ImageDisplay::process_events(unsigned int max_num_events)
{
  unsigned int proc = 0;
  SDL_Event event;
  while ( (proc++ < max_num_events) && (SDL_PollEvent(&event)) ) {
    // nothing to do here
  }
}


/** Process SDL events until quit.
 * Process SDL events and keeps the window responsive until either
 * the key "q" or "Esc" are pressed.
 */
void
ImageDisplay::loop_until_quit()
{
  bool quit = false;
  while (! quit) {
    SDL_Event event;
    if ( SDL_WaitEvent(&event) ) {
      switch (event.type) {
      case SDL_QUIT:
	quit = true;
	break;
      case SDL_KEYUP:
	if ( (event.key.keysym.sym == SDLK_ESCAPE) ||
	     (event.key.keysym.sym == SDLK_q) ) {
	  quit = true;
	}
	break;
      }
    }
  }
}

} // end namespace firevision
