
/***************************************************************************
 *  rcxretriever.cpp - FireVision Retriever for RCSoftX
 *
 *  Created: Tue Jul 15 13:38:52 2008 (RoboCup 2008, Suzhou)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <cams/factory.h>
#include <fvutils/ipc/shm_image.h>

#include <cstring>
#include <signal.h>
#include <cstdio>

using namespace fawkes;

Camera *cam;
SharedMemoryImageBuffer *shm;
bool quit = false;

void
handle_signal(int signum)
{
  quit = true;
}

int
main(int argc, char **argv)
{
  signal(SIGINT, handle_signal);

  printf("Instantiating camera\n");
  cam = CameraFactory::instance("leutron:leutron");
  printf("Opening camera\n");
  cam->open();
  printf("Starting camera\n");
  cam->start();

  printf("Creating shm segment rcxretriever\n");
  shm = new SharedMemoryImageBuffer("rcxretriever", cam->colorspace(),
				    cam->pixel_width(), cam->pixel_height());

  printf("Running capture loop\n");
  while (! quit) {
    cam->capture();
    memcpy(shm->buffer(), cam->buffer(), cam->buffer_size());
    cam->dispose_buffer();
  }

  printf("Cleaning up\n");
  cam->stop();
  cam->close();

  delete shm;
  delete cam;

}

