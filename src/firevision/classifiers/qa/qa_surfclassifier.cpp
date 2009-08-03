
/***************************************************************************
 *  qa_surfclassifier.cpp - QA for SURF classifier
 *
 *  Generated: Wed March 15 16:00:00 2008
 *  Copyright 2008 Stefan Schiffer [stefanschiffer.de]
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

/// @cond QA

#include <fvutils/color/colorspaces.h>
#include <fvutils/readers/jpeg.h>
#include <fvutils/readers/png.h>
#include <fvutils/writers/png.h>
#include <filters/roidraw.h>

#include <classifiers/surf.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <cstdio>

int
main(int argc, char **argv)
{
  if ( argc < 3 ) {
    printf("Usage: %s <object-image-file.png> <scene-image-file.png>\n", argv[0]);
    exit(-1);
  }

  const char *object_file = argv[1];
  const char *scene_file = argv[2];

  printf("QASurfClassifier: creating cvImages for object and scene\n");
  IplImage * obj_img = cvLoadImage( object_file, 1 );
  IplImage * scn_img = cvLoadImage( scene_file, 1 );
  //IplImage * stacked = stack_imgs( obj_img, scn_img );

  printf("QASurfClassifier: Load scene as image\n");
  /*
  JpegReader *reader = new JpegReader(scene_file);
  */
  PNGReader *reader = new PNGReader(scene_file);
  unsigned char *buffer = malloc_buffer(YUV422_PLANAR,
					reader->pixel_width(), reader->pixel_height());
  reader->set_buffer(buffer);
  reader->read();

  //  PNGWriter pngw("scenetest.png", reader->pixel_width(), reader->pixel_height());
  //  pngw.set_buffer(YUV422_PLANAR, buffer );
  //  pngw.write();


  printf("QASurfClassifier: Instantiate SurfClassifier\n");
  SurfClassifier *classifier = new SurfClassifier(object_file);

  classifier->set_src_buffer(buffer, reader->pixel_width(), reader->pixel_height());

  printf("QASurfClassifier: classify ...\n");
  std::list< ROI > *rois = classifier->classify();

  printf("QASurfClassifier: filterROI\n");
  FilterROIDraw *roi_draw = new FilterROIDraw();
  for (std::list< ROI >::iterator i = rois->begin(); i != rois->end(); ++i) {
    printf("QASurfClassifier: ROI: start (%u, %u)  extent %u x %u\n", 
	   (*i).start.x, (*i).start.y, (*i).width, (*i).height);
    // draw ROIs
    roi_draw->set_dst_buffer(buffer, &(*i));
    roi_draw->apply();
  }

  printf("QASurfClassifier: draw ROIs in cvWindow\n");
  for (std::list< ROI >::iterator i = rois->begin(); i != rois->end(); ++i) {
    if( (*i).height == 11 && (*i).width == 11 ) {
      cvRectangle( scn_img, 
		   cvPoint((*i).start.x, (*i).start.y), 
		   cvPoint((*i).start.x+(*i).width, (*i).start.y+(*i).height), 
		   CV_RGB( 0, 0, 180 ), 
		   2//, 4
		   );
    }
    else{
      cvRectangle( scn_img, 
		   cvPoint((*i).start.x, (*i).start.y), 
		   cvPoint((*i).start.x+(*i).width, (*i).start.y+(*i).height), 
		   CV_RGB( 180, 0, 0 ), 
		   2//, 4
		   );
    }
  }

  //display_big_img( stacked, "Matches" );
  cvNamedWindow( "Scene-Matches", 1 );
  cvShowImage( "Scene-Matches", scn_img );
  cvNamedWindow( "Object", 1 );
  cvShowImage( "Object", obj_img );
  cvWaitKey( 0 );

  //  ImageDisplay *display = new ImageDisplay(reader->pixel_width(), reader->pixel_height());
  //  display->show(buffer);
  //  display->loop_until_quit();
  //  delete display;

  delete rois;
  free(buffer);
  delete reader;
  delete classifier;
}

/// @endcond
