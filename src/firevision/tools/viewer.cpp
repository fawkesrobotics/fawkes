
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

#include <cams/factory.h>
#ifdef HAVE_SHMEM_CAM
#include <cams/shmem.h>
#endif
#ifdef HAVE_NETWORK_CAM
#include <cams/net.h>
#endif
#ifdef HAVE_FILELOADER_CAM
#include <cams/fileloader.h>
#endif

#include <fvwidgets/image_display.h>
#ifdef HAVE_RECTINFO
#include <fvutils/rectification/rectfile.h>
#include <fvutils/rectification/rectinfo_block.h>
#include <filters/rectify.h>
#endif

#include <cstring>
#include <cstdio>
#include <stdint.h>

#include <SDL.h>
#ifdef HAVE_GTKMM
#include <gtkmm.h>
#endif

#include <fvutils/color/conversions.h>

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-c] [-s shmem_id] [-n host[:port]/image_id] [-f file] [-v] [cam arg string]\n"
         "  -c             Start in continuous update mode\n"
	 "  -s shmem_id    Open shared memory image with given ID\n"
	 "  -n net_string  Open network camera, the camera string is of the form\n"
	 "                 host[:port]/image_id. You have to specify at least the host\n"
	 "                 and the image_id, the port is optional and defaults to 5000\n"
	 "  -j             Receive JPEG images, only valid with -n\n"
	 "  -f file        Open file loader camera with given file\n"
	 "  -v             Verbose output on console\n"
	 "  cam arg string Can be an arbitrary camera argument string that is understood\n"
	 "                 by CameraFactory and the desired camera.\n",
	 program_name);
}

void
print_keys()
{
  printf("Keys:\n"
	 "  c        continuous mode (automatic image updating as fast as possible)\n"
	 "  r        rectify image, will query for rectification info file and possibly\n"
	 "           for camera if there is more than one block.\n"
	 "  Shift-R  rectify image, use already loaded lut info file, do not query for\n"
	 "           new file\n"
	 "  Space    Refresh image\n"
	 "  q/Esc    Quit viewer\n");
}


/** Process all outstanding Gtk events. */
void
process_gtk_events()
{
  while ( Gtk::Main::events_pending() ) {
    Gtk::Main::iteration();
  }
}


int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "hs:f:n:vjc");

#ifdef HAVE_GTKMM
  Gtk::Main gtk_main(argc, argv);
#endif

  Camera *cam;
  bool verbose = argp.has_arg("v");

  if ( argp.has_arg("h") ) {
    print_usage(argp.program_name());
    exit(0);
  } else if ( argp.has_arg("s") ) {
#ifdef HAVE_SHMEM_CAM
    cam = new SharedMemoryCamera(argp.arg("s"));
#else
    throw Exception("SharedMemoryCamera not available at compile time");
#endif
  } else if ( argp.has_arg("f") ) {
#ifdef HAVE_FILELOADER_CAM
    cam = new FileLoader(argp.arg("f"));
#else
    throw Exception("FileLoader not available at compile time");
#endif
  } else if ( argp.has_arg("n") ) {
#ifdef HAVE_NETWORK_CAM
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
#else
    throw Exception("NetworkCamera not available at compile time");
#endif
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

  print_keys();

  if ( verbose ) {
    printf("Camera opened, settings:\n"
	   "  Colorspace:  %u (%s)\n"
	   "  Dimensions:  %u x %u\n"
	   "  Buffer size: %zu\n",
	   cam->colorspace(), colorspace_to_string(cam->colorspace()),
	   cam->pixel_width(), cam->pixel_height(),
	   colorspace_buffer_size(cam->colorspace(), cam->pixel_width(), cam->pixel_height()));
  }

  cam->capture();

  ImageDisplay *display = new ImageDisplay(cam->pixel_width(), cam->pixel_height());
  display->show(cam->colorspace(), cam->buffer());

  cam->dispose_buffer();

#ifdef HAVE_RECTINFO
  RectificationInfoFile *rectfile = new RectificationInfoFile();
  FilterRectify *rectify_filter = NULL;
  unsigned char *filtered_buffer = malloc_buffer(YUV422_PLANAR,
						 cam->pixel_width(), cam->pixel_height());
  unsigned char *unfiltered_buffer = malloc_buffer(YUV422_PLANAR,
						   cam->pixel_width(), cam->pixel_height());
  bool rectifying = false;
#endif
  bool continuous = argp.has_arg("c");

  SDL_Event redraw_event;
  redraw_event.type = SDL_KEYUP;
  redraw_event.key.keysym.sym = SDLK_SPACE;

  if ( continuous ) {
    SDL_PushEvent(&redraw_event);
  }

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
#ifdef HAVE_RECTINFO
	  if ( rectifying ) {
	    convert(cam->colorspace(), YUV422_PLANAR, cam->buffer(), unfiltered_buffer,
		    cam->pixel_width(), cam->pixel_height());
	    ROI *fir = ROI::full_image(cam->pixel_width(), cam->pixel_height());
	    rectify_filter->set_src_buffer(unfiltered_buffer, fir);
	    rectify_filter->set_dst_buffer(filtered_buffer, fir);
	    rectify_filter->apply();
	    display->show(YUV422_PLANAR, filtered_buffer);
	  } else {
#endif
	    display->show(cam->colorspace(), cam->buffer());
#ifdef HAVE_RECTINFO
          }
#endif

	  cam->dispose_buffer();
	  if ( continuous ) {
	    SDL_PushEvent(&redraw_event);
	  }
	} else if ( event.key.keysym.sym == SDLK_ESCAPE ) {
	  quit = true;
	} else if ( event.key.keysym.sym == SDLK_q ) {
	  quit = true;
	} else if ( event.key.keysym.sym == SDLK_c ) {
	  continuous = true;
	  SDL_PushEvent(&redraw_event);
	} else if ( event.key.keysym.sym == SDLK_r ) {
#ifdef HAVE_GTKMM
#  ifdef HAVE_RECTINFO
	  if ( rectifying ) {
	    rectifying = false;
	  } else {
	    if ( ! (SDL_GetModState() & KMOD_LSHIFT) &&
		 ! (SDL_GetModState() & KMOD_RSHIFT) ||
		 ! rectify_filter ) {
	      Gtk::FileChooserDialog fcd("Open Rectification Info File");

	      // Add response buttons the the dialog
	      fcd.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
	      fcd.add_button(Gtk::Stock::OPEN, Gtk::RESPONSE_OK);

	      Gtk::FileFilter filter_rectinfo;
	      filter_rectinfo.set_name("Rectification Info");
	      filter_rectinfo.add_pattern("*.rectinfo");
	      fcd.add_filter(filter_rectinfo);

	      Gtk::FileFilter filter_any;
	      filter_any.set_name("Any File");
	      filter_any.add_pattern("*");
	      fcd.add_filter(filter_any);

	      int result = fcd.run();

	      fcd.hide();
	      process_gtk_events();

	      if ( result == Gtk::RESPONSE_OK) {
		// Nice, we got a file
		try {
		  rectfile->read(fcd.get_filename().c_str());
		  if ( rectfile->num_blocks() == 0 ) {
		    throw Exception("Rectification info file does not contain any info blocks");
		  }
		  Gtk::HBox hbox;
		  Gtk::Label label("Camera: ");
		  Gtk::ComboBoxText cboxt;
		  hbox.add(label);
		  hbox.add(cboxt);
		  label.show();
		  cboxt.show();

		  RectificationInfoFile::RectInfoBlockVector & blocks = rectfile->blocks();
		  for (RectificationInfoFile::RectInfoBlockVector::iterator b = blocks.begin(); b != blocks.end(); ++b) {
		    Glib::ustring us = rectinfo_camera_strings[(*b)->camera()];
		    us += Glib::ustring(" (") + rectinfo_type_strings[(*b)->type()] + ")";
		  cboxt.append_text(us);
		  }
		  cboxt.set_active(0);

		  Gtk::Dialog dialog("Choose Camera", false, true);
		  dialog.add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);
		  dialog.get_vbox()->add(hbox);
		  hbox.show();
		  dialog.run();
		  dialog.hide();
		  process_gtk_events();
		
		  RectificationInfoBlock *chosen_block = blocks[cboxt.get_active_row_number()];

		  delete rectify_filter;
		  rectify_filter = new FilterRectify(chosen_block);
		} catch (Exception &e) {
		  Gtk::MessageDialog md(e.what(),
					/* use markup */ false,
					Gtk::MESSAGE_ERROR);
		  md.set_title("Reading Rectification Info failed");
		  md.run();
		  md.hide();
		  
		  process_gtk_events();
		}
	      }
	    }
	    rectifying =  (rectify_filter != NULL);
	  }
	  SDL_PushEvent(&redraw_event);
#  else
        printf("Rectification support not available at compile time\n");
#  endif
	}
#else
	printf("Rectification support requires gtkmm(-devel) to be installed "
	       " at compile time.\n");
#endif
	break;
      default:
	break;
      }
    }
  }

#ifdef HAVE_RECTINFO
  delete rectfile;
  delete rectify_filter;
  free(filtered_buffer);
  free(unfiltered_buffer);
#endif

  cam->close();
  delete cam;
  delete display;

  return 0;
}

