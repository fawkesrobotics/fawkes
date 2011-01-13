
/***************************************************************************
 *  qa_camargp.h - QA for camera argument parser
 *
 *  Generated: Wed Apr 11 16:02:33 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *             2009  Daniel Beck
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

/// @cond QA

#include <fvutils/adapters/iplimage.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/readers/jpeg.h>
#include <fvutils/draw/drawer.h>
#include <classifiers/faces.h>
#include <filters/roidraw.h>
#include <fvwidgets/image_display.h>
#include <utils/system/argparser.h>
#include <utils/time/tracker.h>
#include <cams/factory.h>

#include <SDL.h>
#include <opencv/cv.h>

#include <cstdlib>
#include <cstdio>

using namespace fawkes;

int
main(int argc, char **argv)
{
  ArgumentParser* argp = new ArgumentParser( argc, argv, "h:f:c:" );

  if ( argp->has_arg( "h" ) && argp->has_arg( "f" ) )
    // read image from file
  {
    const char *cascade_file = argp->arg( "h" );
    const char *image_file = argp->arg( "f" );
    
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
    
    display->loop_until_quit();
    
    delete display;
    
    delete rois;
    free(buffer);
    delete reader;
    delete classifier;
  }

  else if ( argp->has_arg( "h" ) && argp->has_arg( "c" ) )
    // get images from camera
  {
    const char *cascade_file = argp->arg( "h" );

    Camera* camera = NULL;
    try
    {
      camera = CameraFactory::instance( argp->arg( "c" ) );
      camera->open();
      camera->start();
    }
    catch ( Exception& e )
    {
      printf( "Failed to open camera.\n" );
      delete camera;
      return( -1 );
    }

    printf( "successfully opened camera: w=%d h=%d\n",
	    camera->pixel_width(), camera->pixel_height() );

    TimeTracker* tt = new TimeTracker();
    unsigned int ttc_recognition = tt->add_class( "Face recognition" );
    unsigned int loop_count = 0;
    

    IplImage* image = cvCreateImage( cvSize( camera->pixel_width(),
					     camera->pixel_height() ),
				     IPL_DEPTH_8U, 3 );

    IplImage* scaled_image = cvCreateImage( cvSize( camera->pixel_width() / 2,
						    camera->pixel_height() / 2 ),
					    IPL_DEPTH_8U, 3 );

    FacesClassifier *classifier = new FacesClassifier( cascade_file,
						       camera->pixel_width(),
						       camera->pixel_height(),
						       scaled_image,
						       1.2 /* scale factor */,
						       2 /* min neighbours */,
						       CV_HAAR_DO_CANNY_PRUNING );
    
    unsigned char* display_buffer = (unsigned char*) malloc( camera->buffer_size() );

    ImageDisplay* display = new ImageDisplay( camera->pixel_width(),
					      camera->pixel_height(),
					      "QA Faces Classifier" );
    
    Drawer* drawer = new Drawer();
    drawer->set_buffer( display_buffer,
			camera->pixel_width(),
			camera->pixel_height() );
    
    SDL_Event redraw_event;
    redraw_event.type = SDL_KEYUP;
    redraw_event.key.keysym.sym = SDLK_SPACE;

    SDL_PushEvent( &redraw_event );

    bool quit = false;
    while ( !quit )
    {
      SDL_Event event;
      if ( SDL_WaitEvent( &event ) )
      {
	switch ( event.type )
	{
	case SDL_QUIT:
	  quit = true;
	  break;
	  
	case SDL_KEYUP:
	  if ( event.key.keysym.sym == SDLK_SPACE )
	  {
	    camera->capture();
	    
	    if ( camera->buffer() != NULL )
	    {
	      IplImageAdapter::convert_image_bgr( camera->buffer(), image );
	      cvResize( image, scaled_image, CV_INTER_LINEAR );
	      memcpy( display_buffer, camera->buffer(), camera->buffer_size() );

	      tt->ping_start( ttc_recognition );
	      std::list< ROI > *rois = classifier->classify();
	      tt->ping_end( ttc_recognition );
	      
	      camera->dispose_buffer();
    
	      bool first = true;
	      for ( std::list< ROI >::reverse_iterator i = rois->rbegin();
		    i != rois->rend();
		    ++i )
	      {
		if ( first ) { drawer->set_color( 127, 70, 200 ); }
		drawer->draw_rectangle( 2 * i->start.x, 2 * i->start.y, 2 * i->width, 2 * i->height );
		if ( first ) { drawer->set_color( 30, 30, 30 ); first = false; }
	      }

	      if ( ++loop_count % 15 == 0 ) { tt->print_to_stdout(); }

	      display->show( display_buffer );
	    }

	    SDL_PushEvent( &redraw_event );
	  }

	  else if ( event.key.keysym.sym == SDLK_ESCAPE )
	  { quit = true; }

	  break;

	default:
	  break;
	}
      }
    }

    camera->stop();
    camera->close();
    delete camera;
    delete display;
    delete drawer;
    free( display_buffer );
    cvReleaseImage( &image );
    cvReleaseImage( &scaled_image );
    delete tt;
  }

  else
  {
    printf("Usage: %s -h <Haar cascade file> -f <Image file as JPEG>\n", argv[0]);
    printf("    or %s -h <Haar cascade file> -c <Camera argument string>\n", argv[0]);
    exit(-1);
  }

  delete argp;
}

/// @endcond
