
/***************************************************************************
 *  show_yuv.cpp - Show YUV color space
 *
 *  Created: Tue Feb 23 13:49:38 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <unistd.h>
#include <iostream>

#include <fvwidgets/image_display.h>
#include <fvutils/color/conversions.h>
#include <fvutils/color/yuv.h>

#include <SDL.h>

using namespace std;
using namespace firevision;

/** YUV color space demo.
 * This class fills the given buffer of the size 512x512.
 * @author Tim Niemueller
 */
class YUVSpaceDemo
{
 public:
  /** Constructor.
   * @param yuv_buffer YUV422_PLANAR encoded buffer.
   */
  YUVSpaceDemo(unsigned char *yuv_buffer)
  {
    brightness = 128;
    buffer = yuv_buffer;
  }

  /** Fill buffer. */
  void
  fill()
  {
    unsigned char *yp = buffer;
    unsigned char *up = YUV422_PLANAR_U_PLANE(buffer, 512, 512);
    unsigned char *vp = YUV422_PLANAR_V_PLANE(buffer, 512, 512);

    for (int v = 255; v >= 0 ; --v) {
      for (int u = 0; u < 256; ++u) {
	*yp++ = brightness;
	*yp++ = brightness;
	*up++ = u;
	*vp++ = v;
      }
      // Double line
      memcpy(yp, (yp - 512), 512);
      yp += 512;
      memcpy(up, (up - 256), 256);
      memcpy(vp, (vp - 256), 256);
      up += 256;
      vp += 256;
    }
  }

  /** Increase brightness.
   * @param val value to increase brightness by
   */
  void brightness_up(unsigned int val = 1)
  {
    if ( brightness != 255 ) {
      if ( (brightness + val) < 255 ) {
	brightness += val;
      } else {
	brightness = 255;
      }
      printf("New brightness: %i\n", brightness);
      fill();
    }
  }

  /** Decrease brightness.
   * @param val value to decrease brightness by
   */
  void brightness_down(unsigned int val = 1) {
    if ( brightness != 0 ) {
      if ( (brightness - (int)val) > 0 ) {
	brightness -= val;
      } else {
	brightness = 0;
      }
      printf("New brightness: %i\n", brightness);
      fill();
    }
  }

  /** Get Brightness.
   * @return current brightness
   */
  int get_brightness() const
  {
    return brightness;
  }

 private:
  unsigned char *buffer;
  int brightness;

};


int
main( int argc, char **argv )
{

  unsigned int width = 512;
  unsigned int height = 512;

  unsigned char *yuv_buffer = malloc_buffer(YUV422_PLANAR, width, height);
  YUVSpaceDemo *yuvspace = new YUVSpaceDemo(yuv_buffer);
  ImageDisplay *display = new ImageDisplay(width, height);

  cout << endl << endl
       << " V" << endl
       << " ^" << endl
       << " |" << endl
       << " +--> U" << endl << endl;

  yuvspace->fill();
  display->show(yuv_buffer);

  SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL);

  bool quit = false;
  while (! quit) {
    SDL_Event event;
    if ( SDL_WaitEvent(&event) ) {
      switch (event.type) {
      case SDL_QUIT:
	quit = true;
	break;
      case SDL_KEYDOWN:
	if ( event.key.keysym.sym == SDLK_UP ) {
	  yuvspace->brightness_up();
	  display->show(yuv_buffer);
	} else if ( event.key.keysym.sym == SDLK_DOWN ) {
	  yuvspace->brightness_down();
	  display->show(yuv_buffer);
	} else if ( event.key.keysym.sym == SDLK_PAGEUP ) {
	  yuvspace->brightness_up(20);
	  display->show(yuv_buffer);
	} else if ( event.key.keysym.sym == SDLK_PAGEDOWN ) {
	  yuvspace->brightness_down(20);
	  display->show(yuv_buffer);

	} else if ( event.key.keysym.sym == SDLK_ESCAPE ) {
	  quit = true;
	} else if ( event.key.keysym.sym == SDLK_q ) {
	  quit = true;
	}
	break;

      case SDL_MOUSEBUTTONDOWN:
	{
	  int x = event.button.x;
	  int y = event.button.y;

	  printf("YUV: %i %u %u\n", yuvspace->get_brightness(),
		 x / 2, y / 2);
	}
	break;

      default:
	break;
      }
    }
  }

  free(yuv_buffer);
  delete display;
  delete yuvspace;

  return 0;
}
