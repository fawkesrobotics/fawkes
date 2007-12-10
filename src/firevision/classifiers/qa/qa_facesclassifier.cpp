
/***************************************************************************
 *  qa_camargp.h - QA for camera argument parser
 *
 *  Generated: Wed Apr 11 16:02:33 2007
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

/// @cond QA

#include <SDL.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/readers/jpeg.h>
#include <classifiers/faces.h>
#include <filters/roidraw.h>
#include <fvwidgets/image_display.h>

#include <cstdio>

int
main(int argc, char **argv)
{
  if ( argc < 3 ) {
    printf("Usage: %s <Haar cascade file> <Image file as JPEG>\n", argv[0]);
    exit(-1);
  }

  const char *cascade_file = argv[1];
  const char *image_file = argv[2];

  JpegReader *reader = new JpegReader(image_file);
  unsigned char *buffer = malloc_buffer(YUV422_PLANAR,
					reader->pixel_width(), reader->pixel_height());

  reader->set_buffer(buffer);
  reader->read();

  FacesClassifier *classifier = new FacesClassifier(cascade_file, reader->pixel_width(),
						    reader->pixel_height());


  classifier->set_src_buffer(buffer, reader->pixel_width(), reader->pixel_height());
  std::list< ROI > *rois = classifier->classify();

  FilterROIDraw *roi_draw = new FilterROIDraw();
  for (std::list< ROI >::iterator i = rois->begin(); i != rois->end(); ++i) {
    printf("ROI: start (%u, %u)  extent %u x %u\n", (*i).start.x, (*i).start.y,
	   (*i).width, (*i).height);

    roi_draw->set_dst_buffer(buffer, &(*i));
    roi_draw->apply();
  }

  ImageDisplay *display = new ImageDisplay(reader->pixel_width(), reader->pixel_height());
  display->show(buffer);

  bool quit = false;
  while (! quit) {
    SDL_Event event;
    if ( SDL_WaitEvent(&event) ) {
      if (event.type == SDL_QUIT) {
	quit = true;
      }
    }
  }

  delete display;

  delete rois;
  free(buffer);
  delete reader;
  delete classifier;
}

/// @endcond
