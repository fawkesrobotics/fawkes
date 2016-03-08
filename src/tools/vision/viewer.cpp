
/***************************************************************************
 *  viewer.cpp - Generic viewer tool
 *
 *  Created: Tue Nov 06 15:02:51 2007
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


#include <core/exceptions/software.h>
#include <utils/system/argparser.h>

#include <fvcams/factory.h>
#include <fvcams/buffer.h>
#ifdef HAVE_SHMEM_CAM
#include <fvcams/shmem.h>
#endif
#ifdef HAVE_NETWORK_CAM
#include <fvcams/net.h>
#endif
#ifdef HAVE_FILELOADER_CAM
#include <fvcams/fileloader.h>
#endif

#include <fvwidgets/image_display.h>
#ifdef HAVE_RECTINFO
#include <fvutils/rectification/rectfile.h>
#include <fvutils/rectification/rectinfo_block.h>
#include <fvfilters/rectify.h>
#endif
#include <fvutils/colormap/colormap.h>
#include <fvutils/colormap/cmfile.h>
#include <fvutils/system/filetype.h>

#include <cstring>
#include <cstdio>
#include <stdint.h>

#include <SDL.h>
#ifdef HAVE_GTKMM
#include <gtkmm.h>
#endif

#include <fvutils/color/conversions.h>

using namespace fawkes;
using namespace firevision;

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-c] [-s shmem_id] [-n host[:port]/image_id] [-f file] [-o shmem_id] [-v] \\\n"
	 "          [-d delay] [cam arg string]\n\n"
         "  -c             Start in continuous update mode\n"
	 "  -s shmem_id    Open shared memory image with given ID\n"
	 "  -n net_string  Open network camera, the camera string is of the form\n"
	 "                 host[:port]/image_id. You have to specify at least the host\n"
	 "                 and the image_id, the port is optional and defaults to 5000\n"
	 "  -j             Receive JPEG images, only valid with -n\n"
	 "  -d delay       Delay in ms before a new image is capture.\n"
	 "  -f file        Open file loader camera with given file (image, colormap)\n"
	 "  -o shmem_id    Output the image to a shared memory segment with given ID\n"
	 "  -v             Verbose output on console\n"
	 "  cam arg string Can be an arbitrary camera argument string that is understood\n"
	 "                 by CameraFactory and the desired camera.\n",
	 program_name);
}

void
print_keys()
{
  printf("Keys:\n"
	 "  c        Toggle continuous mode (automatic image updating as fast as possible)\n"
	 "  r        rectify image, will query for rectification info file and possibly\n"
	 "           for camera if there is more than one block.\n"
	 "  +        Increase delay by 5 ms\n"
	 "  -        Decrease delay by 5 ms\n"
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
  ArgumentParser argp(argc, argv, "hs:f:n:vjcd:o:");
  std::string title = "";

#ifdef HAVE_GTKMM
  Gtk::Main gtk_main(argc, argv);
#endif

  Camera *cam;
  SharedMemoryImageBuffer *buf = NULL;
  bool verbose = argp.has_arg("v");
  int delay = 0;
  Colormap *colormap = NULL;
  unsigned int colormap_y = 0;

  if ( argp.has_arg("d") ) {
    delay = atoi(argp.arg("d"));
    if ( delay < 0 )  delay = 0;
  }

  if ( argp.has_arg("h") ) {
    print_usage(argp.program_name());
    exit(0);
  } else if ( argp.has_arg("s") ) {
#ifdef HAVE_SHMEM_CAM
		title = std::string(argp.arg("s"));
    cam = new SharedMemoryCamera(argp.arg("s"));
#else
    throw Exception("SharedMemoryCamera not available at compile time");
#endif
  } else if ( argp.has_arg("f") ) {
#ifdef HAVE_FILELOADER_CAM
    std::string filename = argp.arg("f");
    title = std::string("File: ").append(filename);
    std::string ft = fv_filetype_file(filename.c_str());

    if (ft == "FvColormap") {
	    ColormapFile cm_file;
	    cm_file.read(filename.c_str());
	    colormap = cm_file.get_colormap();

	    cam = new BufferCamera(YUV422_PLANAR, 512, 512);
	    colormap->to_image(cam->buffer(), colormap_y);
    } else {
	    cam = new FileLoader(filename.c_str());
    }
#else
    throw Exception("FileLoader not available at compile time");
#endif
  } else if ( argp.has_arg("n") ) {
#ifdef HAVE_NETWORK_CAM
    title = std::string("Net cam: ").append(argp.arg("n"));
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

  if ( argp.has_arg("o") )
  {
    buf = new SharedMemoryImageBuffer(argp.arg("o"), cam->colorspace(), cam->pixel_width(), cam->pixel_height());
  }

  print_keys();

  if ( verbose ) {
    printf("Camera opened, settings:\n"
	   "  Colorspace:  %u (%s)\n"
	   "  Dimensions:  %u x %u\n"
	   "  Buffer size: %zu\n"
	   "  Delay:       %i ms\n",
	   cam->colorspace(), colorspace_to_string(cam->colorspace()),
	   cam->pixel_width(), cam->pixel_height(),
	   colorspace_buffer_size(cam->colorspace(), cam->pixel_width(), cam->pixel_height()),
	   delay);
  }

  ImageDisplay *display = new ImageDisplay(cam->pixel_width(), cam->pixel_height(), title.c_str());

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

  SDL_PushEvent(&redraw_event);

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
          if (cam->buffer() != NULL ) {
            if ( buf ) memcpy(buf->buffer(), cam->buffer(), cam->buffer_size());
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
          } else {
            printf("No valid frame received\n");
          }
	  if ( continuous ) {
	    usleep(delay * 1000);
	    SDL_PushEvent(&redraw_event);
	  }
	} else if ( event.key.keysym.sym == SDLK_ESCAPE ) {
	  quit = true;
	} else if ( event.key.keysym.sym == SDLK_q ) {
	  quit = true;
	} else if ( event.key.keysym.sym == SDLK_c ) {
	  continuous = ! continuous;
	  SDL_PushEvent(&redraw_event);
	} else if ( event.key.keysym.sym == SDLK_PLUS ) {
	  delay += 5;
	  printf("New delay: %i ms\n", delay);
	} else if ( event.key.keysym.sym == SDLK_MINUS ) {
	  if ( delay > 5 ) {
	    delay -= 5;
	  } else {
	    delay = 0;
	  }
	  printf("New delay: %i ms\n", delay);
	} else if ( event.key.keysym.sym == SDLK_UP ) {
		colormap_y = std::min(255u, colormap_y + 5);
		printf("Colormap new Y (+): %u\n", colormap_y);
		colormap->to_image(cam->buffer(), colormap_y);
	  SDL_PushEvent(&redraw_event);
	} else if ( event.key.keysym.sym == SDLK_DOWN ) {
		colormap_y = std::max(0, (int)colormap_y - 5);
		printf("Colormap new Y (-): %u\n", colormap_y);
		colormap->to_image(cam->buffer(), colormap_y);
	  SDL_PushEvent(&redraw_event);
	} else if ( event.key.keysym.sym == SDLK_r ) {
#ifdef HAVE_GTKMM
#  ifdef HAVE_RECTINFO
	  if ( rectifying ) {
	    rectifying = false;
	  } else {
	    if ( (! (SDL_GetModState() & KMOD_LSHIFT) &&
		  ! (SDL_GetModState() & KMOD_RSHIFT)) ||
		 ! rectify_filter ) {
	      Gtk::FileChooserDialog fcd("Open Rectification Info File");

	      // Add response buttons the the dialog
	      fcd.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
	      fcd.add_button(Gtk::Stock::OPEN, Gtk::RESPONSE_OK);

#if GTK_VERSION_GE(3,0)
              Glib::RefPtr<Gtk::FileFilter> filter_rectinfo =
                Gtk::FileFilter::create();
	      filter_rectinfo->set_name("Rectification Info");
	      filter_rectinfo->add_pattern("*.rectinfo");
#else
              Gtk::FileFilter filter_rectinfo;
	      filter_rectinfo.set_name("Rectification Info");
	      filter_rectinfo.add_pattern("*.rectinfo");
#endif	      
	      fcd.add_filter(filter_rectinfo);

#if GTK_VERSION_GE(3,0)
              Glib::RefPtr<Gtk::FileFilter> filter_any =
                Gtk::FileFilter::create();
	      filter_any->set_name("Any File");
	      filter_any->add_pattern("*");
#else
              Gtk::FileFilter filter_any;
	      filter_any.set_name("Any File");
	      filter_any.add_pattern("*");
#endif
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

		  RectificationInfoFile::RectInfoBlockVector *blocks = rectfile->rectinfo_blocks();
		  for (RectificationInfoFile::RectInfoBlockVector::iterator b = blocks->begin(); b != blocks->end(); ++b) {
		    Glib::ustring us = rectinfo_camera_strings[(*b)->camera()];
		    us += Glib::ustring(" (") + rectinfo_type_strings[(*b)->type()] + ")";
#if GTK_VERSION_GE(3,0)
                    cboxt.append(us);
#else
                    cboxt.append_text(us);
#endif
		  }
		  cboxt.set_active(0);

		  Gtk::Dialog dialog("Choose Camera", true);
		  dialog.add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);
		  dialog.get_vbox()->add(hbox);
		  hbox.show();
		  dialog.run();
		  dialog.hide();
		  process_gtk_events();
		
		  RectificationInfoBlock *chosen_block = (*blocks)[cboxt.get_active_row_number()];
		  RectificationInfoFile::RectInfoBlockVector::iterator bi = blocks->begin();
		  for(int i = 1; i < cboxt.get_active_row_number(); ++i) {
		    ++bi;
		  }
		  blocks->erase(bi); // needs to be erased because otherwise it would be deleted by following delete
		  delete blocks;

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
  delete colormap;
  
  return 0;
}

