
/***************************************************************************
 *  viewer.cpp - Generic viewer tool
 *
 *  Created: Tue Nov 06 15:02:51 2007
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


#include <core/exceptions/software.h>
#include <utils/system/argparser.h>

#include <cams/shmem.h>
#include <cams/factory.h>
#include <cams/net.h>
#include <cams/fileloader.h>

#include <fvwidgets/image_display.h>

#include <cstring>
#include <cstdio>
#include <stdint.h>

#include <SDL.h>

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-s shmem_id] [-n host[:port]/image_id] [-f file] [cam arg string]\n"
	 "  -s shmem_id    Open shared memory image with given ID\n"
	 "  -n net_string  Open network camera, the camera string is of the form\n"
	 "                 host[:port]/image_id. You have to specify at least the host\n"
	 "                 and the image_id, the port is optional and defaults to 5000\n"
	 "  -f file        Open file loader camera with given file\n"
	 "  cam arg string Can be an arbitrary camera argument string that is understood\n"
	 "                 by CameraFactory and the desired camera.\n",
	 program_name);
}


int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "Hs:f:n:");

  Camera *cam;

  if ( argp.has_arg("H") ) {
    print_usage(argp.program_name());
    exit(0);
  } else if ( argp.has_arg("s") ) {
    cam = new SharedMemoryCamera(argp.arg("s"));
  } else if ( argp.has_arg("f") ) {
    cam = new FileLoader(argp.arg("f"));
  } else if ( argp.has_arg("n") ) {
    char *net_string = strdup(argp.arg("n"));
    char *image_id = NULL, *host = NULL, *port = NULL, *save_ptr = NULL;
    int port_num = 5000;
    int image_num = 0;

    if ( strchr(net_string, ':') != NULL ) {
      host = strtok_r(net_string, ":", &save_ptr);
      port = strtok_r(NULL, "/", &save_ptr);
    } else {
      host = strtok_r(net_string, "/", &save_ptr);
    }
    image_id = strtok_r(NULL, "", &save_ptr);

    if ( port != NULL ) {
      port_num = atoi(port);
      if ( (port_num < 0) || (port_num > 0xFFFF) ) {
	throw OutOfBoundsException("Invalid port", port_num, 0, 0xFFFF);
      }
    }

    if( image_id == NULL ) {
      throw IllegalArgumentException("Image ID must be specified");
    }

    image_num = atoi(image_id);
    if ( image_num < 0 ) {
      throw OutOfBoundsException("Invalid Image Number", image_num, 0, INT_MAX);
    }

    cam = new NetworkCamera(host, port_num, image_num);
  } else {
    if ( argp.num_items() == 0 ) {
      print_usage(argp.program_name());
      printf("\n\nNeither camera option nor camera string given. Aborting.\n\n");
      exit(-3);
    }
    cam = CameraFactory::instance(argp.items()[0]);
  }

  if ( cam == NULL ) {
    throw Exception("Failed to initialize camera for unknown reason");
  }


  try {
    cam->open();
    cam->start();
  } catch (Exception &e) {
    printf("Failed to open camera\n");
    e.print_trace();
    delete cam;
    exit(-2);
  }

  cam->capture();

  ImageDisplay *display = new ImageDisplay(cam->pixel_width(), cam->pixel_height());
  display->show(cam->colorspace(), cam->buffer());

  cam->dispose_buffer();

  bool quit = false;
  while (! quit) {
    SDL_Event event;
    if ( SDL_WaitEvent(&event) ) {
      switch (event.type) {
      case SDL_QUIT:
	quit = true;
	break;
      case SDL_KEYUP:
	if ( event.key.keysym.sym == SDLK_SPACE ) {
	  cam->capture();
	  display->show(cam->colorspace(), cam->buffer());
	  cam->dispose_buffer();
	} else if ( event.key.keysym.sym == SDLK_ESCAPE ) {
	  quit = true;
	} else if ( event.key.keysym.sym == SDLK_q ) {
	  quit = true;
	}
	break;
      default:
	break;
      }
    }
  }


  cam->close();
  delete cam;

  return 0;
}
