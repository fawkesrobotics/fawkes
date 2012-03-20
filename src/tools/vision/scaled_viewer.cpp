
/***************************************************************************
 *  scale_viewer.cpp - Generic scale viewer tool
 *
 *  Created: Thu Aug 25 16:13:34 2011 (based on fvviewer)
 *  Copyright  2005-2011  Tim Niemueller [www.niemueller.de]
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


#include <core/exceptions/software.h>
#include <utils/system/argparser.h>
#include <utils/time/tracker.h>

#include <fvcams/factory.h>
#ifdef HAVE_SHMEM_CAM
#include <fvcams/shmem.h>
#endif
#ifdef HAVE_NETWORK_CAM
#include <fvcams/net.h>
#endif
#ifdef HAVE_FILELOADER_CAM
#include <fvcams/fileloader.h>
#endif

#include <cstring>
#include <cstdio>
#include <stdint.h>

#include <gtkmm.h>

#include <fvutils/color/conversions.h>

using namespace fawkes;
using namespace firevision;

Gtk::Image *img_image;
Camera     *cam;

TimeTracker tt;
int ttc_capture;
int ttc_convert;
int ttc_draw;
int ttc_interloop;
unsigned int loop_count = 0;

static bool
timeout_handler()
{
  tt.ping_end(ttc_interloop);

  tt.ping_start(ttc_capture);
  cam->capture();
  tt.ping_end(ttc_capture);

  tt.ping_start(ttc_convert);
  unsigned int orig_width  = cam->pixel_width();
  unsigned int orig_height = cam->pixel_height();

  unsigned char *rgb_buffer = malloc_buffer(RGB, orig_width, orig_height);

  convert(cam->colorspace(), RGB, cam->buffer(), rgb_buffer,
          orig_width, orig_height);
  tt.ping_end(ttc_convert);

  tt.ping_start(ttc_draw);
  Glib::RefPtr<Gdk::Pixbuf> image =
    Gdk::Pixbuf::create_from_data( rgb_buffer, Gdk::COLORSPACE_RGB, false,
                                   8, orig_width, orig_height, 3 * orig_width);

  int width = img_image->get_width();
  int height = img_image->get_height();
  Glib::RefPtr<Gdk::Pixbuf> scaled = image->scale_simple(width, height,
                                                         Gdk::INTERP_NEAREST);

  img_image->set(scaled);
  img_image->queue_draw();

  tt.ping_end(ttc_draw);

  cam->dispose_buffer();

  free(rgb_buffer);

  if (++loop_count >= 10) {
    loop_count = 0;
    tt.print_to_stdout();
  }

  tt.ping_start(ttc_interloop);
  return true;
}



void
print_usage(const char *program_name)
{
  printf("Usage: %s -n host[:port]/image_id [-j] [-d delay] [-v]\n\n"
	 "  -n net_string  Open network camera, the camera string is of the form\n"
	 "                 host[:port]/image_id. You have to specify at least the host\n"
	 "                 and the image_id, the port is optional and defaults to 5000\n"
	 "  -j             Receive JPEG images, only valid with -n\n"
	 "  -d delay       Delay in ms before a new image is capture.\n",
	 program_name);
}



int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "hn:jd:");

  Gtk::Main gtk_main(argc, argv);

  //bool verbose = argp.has_arg("v");
  int delay = 300;

  if ( argp.has_arg("d") ) {
    delay = atoi(argp.arg("d"));
    if ( delay < 0 )  delay = 300;
  }

  if ( argp.has_arg("h") ) {
    print_usage(argp.program_name());
    exit(0);
  } else if ( argp.has_arg("n") ) {
    char *net_string = strdup(argp.arg("n"));
    char *image_id = NULL, *host = NULL, *port = NULL, *save_ptr = NULL;
    int port_num = 2208;
    char *hostport;

    hostport = strtok_r(net_string, "/", &save_ptr);
    image_id = strtok_r(NULL, "", &save_ptr);

    if ( strchr(hostport, ':') != NULL ) {
      host = strtok_r(hostport, ":", &save_ptr);
      port = strtok_r(NULL, "", &save_ptr);
    } else {
      host = hostport;
    }

    if ( port != NULL ) {
      port_num = atoi(port);
      if ( (port_num < 0) || (port_num > 0xFFFF) ) {
	throw OutOfBoundsException("Invalid port", port_num, 0, 0xFFFF);
      }
    }

    if( image_id == NULL ) {
      throw IllegalArgumentException("Image ID must be specified");
    }

    cam = new NetworkCamera(host, port_num, image_id, argp.has_arg("j"));
    free(net_string);
  } else {
    print_usage(argp.program_name());
    exit(1);
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

  ttc_capture   = tt.add_class("Capture");
  ttc_convert   = tt.add_class("Convert");
  ttc_draw      = tt.add_class("Draw");
  ttc_interloop = tt.add_class("InterLoop");
  tt.ping_start(ttc_interloop);

  Glib::RefPtr<Gtk::Builder> builder;
  builder =
    Gtk::Builder::create_from_file(RESDIR"/guis/scale_viewer/scale_viewer.ui");

  Gtk::Window *window;

  builder->get_widget("wnd_main", window);
  builder->get_widget("img_image", img_image);

  Glib::signal_timeout().connect(sigc::ptr_fun(&timeout_handler), delay);

  window->set_size_request(320, 240);
  Gtk::Main::run(*window);

  cam->close();
  delete cam;

  return 0;
}

