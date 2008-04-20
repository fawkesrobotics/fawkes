
/***************************************************************************
 *  pipeline.cpp - Implementation of the image processing pipeline for
 *                 cannikin
 *
 *  Generated: Tue Apr 10 13:44:45 2007 (based on suricate)
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

/// @cond RCSoftX

#include "pipeline.h"
#include "config.h"

#include <utils/system/console_colors.h>
#include <utils/system/argparser.h>

#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>
#include <utils/ipc/msg.h>
#include <fvutils/ipc/msg_registry.h>
#include <fvutils/draw/drawer.h>

#include <cams/factory.h>

#include <models/scanlines/grid.h>
#include <models/scanlines/radial.h>
#include <models/scanlines/cornerhorizon.h>
#include <models/color/thresholds.h>
#include <models/color/lookuptable.h>

#include <stereo/triclops.h>

#include <classifiers/simple.h>

#include <unistd.h>
#include <iostream>
#include <algorithm>  // for min/max

using namespace std;

class DisparityPoint {
 public:
  DisparityPoint(unsigned int px, unsigned int py,
		 float x, float y, float z) {
    this->px = px;
    this->py = py;
    this->x = x;
    this->y = y;
    this->z = z;
  }

  float x, y, z;
  unsigned int px, py;

  bool operator<(const DisparityPoint &p) const {
    return (z < p.z);
  }
};

CannikinPipeline::CannikinPipeline(ArgumentParser *argp, CannikinConfig *config)
{
  param_width = param_height = 0;
  msg_prefix = cblue + "CannikinPipeline: " + cnormal;
  // box_visible = false;
  quit = false;

  // This doesn't change because we do the conversion to YUV422_PLANAR
  // and then work on this image
  cspace_to   = YUV422_PLANAR;

  cup_visible = false;

  this->argp   = argp;
  this->config = config;

  scanlines     = NULL;
  cm            = NULL;
  disparity_scanlines = NULL;
  /*
  box_rel      = NULL;
  box_glob     = NULL;
  box_relvelo  = NULL;
  box_globvelo = NULL;
  */
  classifier    = NULL;
  drawer = NULL;

  if ( (use_fileloader = argp->has_arg("L")) ) {
    cout <<  msg_prefix << "Fileloader requested. Looking for needed parameters" << endl;
    if ( (file = argp->arg("f")) == NULL) {
      file = "../src/modules/robocup/firevision/images/office-ball-kabel-wirrwarr_748x572_leutron.yuv422packed.raw";
      cout << msg_prefix << cred << "No file given, using default (" << file << ")" << cnormal << endl;
    }
  }

  generate_output = argp->has_arg("o");
#ifdef HAVE_TRICLOPS_SDK
  camless_mode      = argp->has_arg("C");
#else
  camless_mode      = true;
#endif

  state = CANNIKIN_STATE_TEST_MODE;
  _mode = DETECT_CUP;

  msgq = new IPCMessageQueue( FIREVISION_MSGQ_CANNIKIN,
			      true /* destroy on delete */,
			      true /* create if it does not exist */);

  cam      = NULL;
  camctrl  = NULL;
  triclops = NULL;
  buffer1  = NULL;
  buffer2  = NULL;
  buffer3  = NULL;
  shm_buffer = NULL;
  shm_buffer_src = NULL;
  scanlines = NULL;
  disparity_scanlines = NULL;
  cm = NULL;
  classifier = NULL;
}


CannikinPipeline::~CannikinPipeline()
{
  cout << msg_prefix << "destructor called" << endl;

  finalize();

  delete cam;
  delete triclops;

  cam = NULL;
  camctrl = NULL;
  triclops = NULL;

  //cout << msg_prefix << "Deleting shared memory buffer for final image" << endl;
  delete shm_buffer;
  //cout << msg_prefix << "Deleting shared memory buffer for source image" << endl;
  delete shm_buffer_src;
  //cout << msg_prefix << "Freeing temporary buffers" << endl;
  if (buffer1)  free(buffer1);
  if (buffer2)  free(buffer2);
  if (buffer3)  free(buffer3);

  delete scanlines;
  delete disparity_scanlines;
  delete cm;
  /*
  delete box_rel;
  delete box_glob;
  delete box_relvelo;
  delete box_globvelo;
  */
  delete classifier;

  delete drawer;
}


void
CannikinPipeline::init()
{

  if ( ! camless_mode ) {
    cam = CameraFactory::instance( config->CameraString.c_str() );

    cam->open();
    cam->start();

#ifdef HAVE_TRICLOPS_SDK
    triclops = new TriclopsStereoProcessor(cam);

    triclops->set_subpixel_interpolation( false );
    triclops->set_edge_correlation( true );
    triclops->set_disparity_range(5, 100);
    triclops->set_edge_masksize(11);
    triclops->set_stereo_masksize(23);
    triclops->set_surface_validation( true );
    triclops->set_texture_validation( false );
    triclops->set_disparity_mapping( true );
    triclops->set_disparity_mapping_range(10, 85);
#endif

    width  = cam->pixel_width();
    height = cam->pixel_height();
    cspace_from = cam->colorspace();

    /* NOTE:
     * buffer_src is the place where the converted image is stored. 
     * After the processing of a given region is done the resulting image
     * has to be placed in buffer. Further steps are done on this buffer!
     * At the beginning of the loop buffer_src will contain the source image in
     * YUV422_PLANAR format that just arrived from the camera. In later runs
     * there are already some filtered ROIs. buffer_src is considered to be
     * read-only. Do not write to this buffer!
     * buffer2 and buffer3 are available for any operations you would
     * like to do for temporary storage. Avoid copying too much data around
     * and think about in-place operations. Two buffers should be sufficient
     * for most operations.
     */

    buffer_size = colorspace_buffer_size(YUV422_PLANAR, width, height);

    cout << msg_prefix << "Creating shared memory segment for final image" << endl;
    shm_buffer     = new SharedMemoryImageBuffer("cannikin-processed", YUV422_PLANAR, width, height);
    cout << msg_prefix << "Creating shared memory segment for source image" << endl;
    shm_buffer_src = new SharedMemoryImageBuffer("cannikin-raw", YUV422_PLANAR, width, height);

    buffer     = shm_buffer->buffer();
    buffer_src = shm_buffer_src->buffer();
    buffer1    = (unsigned char *)malloc( buffer_size );
    buffer2    = (unsigned char *)malloc( buffer_size );
    buffer3    = (unsigned char *)malloc( buffer_size );

    // models
    scanlines = new ScanlineGrid(width, height,
				 config->ScanlineGridXOffset, config->ScanlineGridYOffset);

    disparity_scanlines = new ScanlineRadial(width, height, width / 2, height / 2, 5, 10);
    //disparity_scanlines = new ScanlineGrid(width, height, 2, 2);

    drawer = new Drawer();
    drawer->setBuffer(buffer, width, height);
  } // end not camless mode



  string colormap_filestem = ColorModelLookupTable::compose_filename(config->ColormapDirectory
								     + "/" +
								     config->ColormapFilestem );

  cout << "Colormap filestem is '" << colormap_filestem << "'" << endl;

  colormap_filestem_cindex = colormap_filestem.find("%c");
  if ( colormap_filestem_cindex != string::npos ) {
    // remove %c
    colormap_filestem.erase(colormap_filestem_cindex, 2);

    string tmp;

    tmp = colormap_filestem;
    tmp.insert(colormap_filestem_cindex, "red");
    cout << "Adding colormap " << tmp << endl;
    colormaps[CC_RED] = strdup(tmp.c_str());

    /*
    tmp = colormap_filestem;
    tmp.insert(colormap_filestem_cindex, "yellow");
    cout << "Adding colormap " << tmp << endl;
    colormaps[CC_YELLOW] = strdup(tmp.c_str());
    tmp = colormap_filestem;
    tmp.insert(colormap_filestem_cindex, "blue");
    cout << "Adding colormap " << tmp << endl;
    colormaps[CC_BLUE] = strdup(tmp.c_str());
*/
    /*
    tmp = colormap_filestem;
    tmp.insert(colormap_filestem_cindex, "orange");
    cout << "Adding colormap " << tmp << endl;
    colormaps[CC_ORANGE] = strdup(tmp.c_str());
    tmp = colormap_filestem;
    tmp.insert(colormap_filestem_cindex, "green");
    cout << "Adding colormap " << tmp << endl;
    colormaps[CC_GREEN] = strdup(tmp.c_str());
    */
  }

  cm  = new ColorModelLookupTable( "front-color",
				   true /* destroy on free */);
  cm->reset();
  set_cup_color(CC_BLUE);


  /*
  // Position models for box
  box_rel      = new BoxRelative(width, height,
				 config->CameraHeight,
				 config->CameraOffsetX,
				 config->CameraOffsetY,
				 config->CameraOrientation,
				 config->HorizontalViewingAngle,
				 config->VerticalViewingAngle
				 );
  box_rel->setPanTilt(config->CameraBearing, config->CameraSlope);
  
  box_glob     = new BallGlobal( box_rel );
  */

  // Classifier
  if ( config->ClassifierType != "simple") {
    cout << msg_prefix << cyellow << "Only the really simple classifier is supporter at this time" << cnormal << endl;
  }
  classifier   = new SimpleColorClassifier(scanlines, cm, 20 /* min pixels to consider */, 30 /* initial box extent */);

}


void
CannikinPipeline::set_colormap(cup_color_t c, const char *file)
{
  if ( colormaps.find(c) != colormaps.end() ) {
    free(colormaps[c]);
  }
  if ( file == NULL ) {
    // remove
    colormaps.erase(c);
  } else {
    // set
    colormaps[c] = strdup(file);
  }
}


void
CannikinPipeline::finalize()
{
  if (cam != NULL)  cam->close();
}


void
CannikinPipeline::handle_signal(int signum)
{
  if ( (signum == SIGINT) ||
       (signum == SIGTERM) ) {
    quit = true;
  }
}

void
CannikinPipeline::run(unsigned int delay)
{
  SignalManager::instance()->register_handler(SIGINT, this);

  delay *= 1000;

  while ( !quit ) {
    loop();
    usleep(delay);
  }

  SignalManager::instance()->finalize();
}


void
CannikinPipeline::run(unsigned int delay, unsigned int times)
{
  SignalManager::instance()->register_handler(SIGINT, this);

  delay *= 1000;

  for (unsigned int i = 0; !quit && (i < times); ++i) {
    loop();
    usleep(delay);
  }

  SignalManager::instance()->finalize();
}



/*
RelativePositionModel *
CannikinPipeline::getRelativeBoxPosModel()
{
  return box_rel;
}


GlobalPositionModel *
CannikinPipeline::getGlobalBoxPosModel()
{
  return box_glob;
}


VelocityModel *
CannikinPipeline::getBoxRelativeVelocityModel()
{
  return box_relvelo;
}


VelocityModel *
CannikinPipeline::getBoxGlobalVelocityModel()
{
  return box_globvelo;
}
*/

CameraControl *
CannikinPipeline::getCameraControl()
{
  return camctrl;
}


ScanlineModel *
CannikinPipeline::getScanlineModel()
{
  return scanlines;
}


void
CannikinPipeline::getDataTakenTime(long int *sec, long int *usec)
{
  *sec  = data_taken_time.tv_sec;
  *usec = data_taken_time.tv_usec;
}


void
CannikinPipeline::set_mode(cannikin_mode_t m)
{
  _mode = m;
  switch (_mode) {
  case DETECT_CUP:
    state = CANNIKIN_STATE_REINITIALIZE_COLORMAP;
    break;
  case DETERMINE_CUP_COLOR:
    state = CANNIKIN_STATE_DETERMINE_CUP_COLOR;
    break;
  case TEST_MODE:
    state = CANNIKIN_STATE_TEST_MODE;
    break;
  case STEREO_TEST_MODE:
    state = CANNIKIN_STATE_STEREO_TEST_MODE;
    break;
  default: break;
  }
}


CannikinPipeline::cannikin_mode_t
CannikinPipeline::mode()
{
  return _mode;
}


void
CannikinPipeline::set_cup_color(cup_color_t c) {
  _cup_color = c;
  if ( state == CANNIKIN_STATE_DETECTION ) {
    state = CANNIKIN_STATE_REINITIALIZE_COLORMAP;
  }
}


CannikinPipeline::cup_color_t
CannikinPipeline::cup_color()
{
  return _cup_color;
}


bool
CannikinPipeline::is_cup_visible()
{
  return cup_visible;
}


bool
CannikinPipeline::done_determining_cup_color()
{
  return cup_color_determination_done;
}


CannikinPipeline::cup_color_t
CannikinPipeline::determined_cup_color()
{
  return _determined_cup_color;
}


void
CannikinPipeline::ipc_messaging()
{
  try {
    if ( msgq->recv(CANNIKIN_MTYPE_SET_STEREOPARAMS, (IPCMessageQueue::MessageStruct *)&stereo_params, sizeof(stereo_params)) ) {
      if ( generate_output ) {
	cout << "RECEIVED stereo parameters:" << endl
	     << "  Disparity Range:          [" << (unsigned int)stereo_params.min_disparity << ".." << (unsigned int)stereo_params.max_disparity << "]" << endl
	     << "  Disparity Mapping Range:  [" << (unsigned int)stereo_params.min_disparity_mapping << ".." << (unsigned int)stereo_params.max_disparity_mapping << "]" << endl
	     << "  Edge Mask Size:           " << (unsigned int)stereo_params.edge_masksize << endl
	     << "  Stereo Mask Size:         " << (unsigned int)stereo_params.stereo_masksize << endl
	     << "  Disparity mapping:        " << stereo_params.flag_disparity_mapping << endl
	     << "  Subpixel Interpolation:   " << stereo_params.flag_subpixel_interpolation << endl
	     << "  Lowpass filtering:        " << stereo_params.flag_lowpass_filter << endl
	     << "  Surface Validation:       " << stereo_params.flag_surface_validation << endl
	     << "  Texture Validation:       " << stereo_params.flag_texture_validation << endl;
      }
      if ( ! camless_mode ) {
#ifdef HAVE_TRICLOPS_SDK
	triclops->set_disparity_range(stereo_params.min_disparity, stereo_params.min_disparity);
	triclops->set_disparity_mapping_range(stereo_params.min_disparity_mapping, stereo_params.min_disparity_mapping);
	triclops->set_edge_masksize(stereo_params.edge_masksize);
	triclops->set_stereo_masksize(stereo_params.stereo_masksize);
	triclops->set_disparity_mapping(stereo_params.flag_disparity_mapping);
	triclops->set_subpixel_interpolation(stereo_params.flag_subpixel_interpolation);
	triclops->set_lowpass(stereo_params.flag_lowpass_filter);
	triclops->set_surface_validation(stereo_params.flag_surface_validation);
	triclops->set_texture_validation(stereo_params.flag_texture_validation);
#endif
      }
    }
    if ( msgq->recv(CANNIKIN_MTYPE_GET_STEREOPARAMS, (IPCMessageQueue::MessageStruct *)&mtype, sizeof(mtype)) ) {
      // Someone wants our params, send them!
      stereo_params.mtype = CANNIKIN_MTYPE_STEREOPARAMS;
      if ( ! camless_mode ) {
#ifdef HAVE_TRICLOPS_SDK
	stereo_params.min_disparity = triclops->disparity_range_min();
	stereo_params.max_disparity = triclops->disparity_range_max();
	stereo_params.min_disparity_mapping = triclops->disparity_mapping_min();
	stereo_params.max_disparity_mapping = triclops->disparity_mapping_max();
	stereo_params.edge_masksize = triclops->edge_masksize();
	stereo_params.stereo_masksize = triclops->stereo_masksize();
	stereo_params.flag_subpixel_interpolation = triclops->subpixel_interpolation();
	stereo_params.flag_disparity_mapping = triclops->disparity_mapping();
	stereo_params.flag_surface_validation = triclops->surface_validation();
	stereo_params.flag_texture_validation = triclops->texture_validation();
	stereo_params.flag_lowpass_filter = triclops->lowpass();
#endif
      } else {
	stereo_params.min_disparity = 10;
	stereo_params.max_disparity = 20;
	stereo_params.min_disparity_mapping = 30;
	stereo_params.max_disparity_mapping = 40;
	stereo_params.edge_masksize = 11;
	stereo_params.stereo_masksize = 23;
	stereo_params.flag_subpixel_interpolation = 1;
	stereo_params.flag_disparity_mapping = 0;
	stereo_params.flag_surface_validation = 1;
	stereo_params.flag_texture_validation = 0;
	stereo_params.flag_lowpass_filter = 1;
      }
      if ( generate_output ) {
	cout << "SENDING stereo parameters:" << endl
	     << "  Disparity Range:          [" << (unsigned int)stereo_params.min_disparity << ".." << (unsigned int)stereo_params.min_disparity << "]" << endl
	     << "  Disparity Mapping Range:  [" << (unsigned int)stereo_params.min_disparity_mapping << ".." << (unsigned int)stereo_params.min_disparity_mapping << "]" << endl
	     << "  Edge Mask Size:           " << (unsigned int)stereo_params.edge_masksize << endl
	     << "  Stereo Mask Size:         " << (unsigned int)stereo_params.stereo_masksize << endl
	     << "  Disparity mapping:        " << stereo_params.flag_disparity_mapping << endl
	     << "  Subpixel Interpolation:   " << stereo_params.flag_subpixel_interpolation << endl
	     << "  Lowpass filtering:        " << stereo_params.flag_lowpass_filter << endl
	     << "  Surface Validation:       " << stereo_params.flag_surface_validation << endl
	     << "  Texture Validation:       " << stereo_params.flag_texture_validation << endl;
      }
      msgq->send((IPCMessageQueue::MessageStruct *)&stereo_params, sizeof(stereo_params));
    }
  } catch (Exception &e) {
    cout << "IPC messaging failed" << endl;
    e.print_trace();
  }
}


void
CannikinPipeline::loop()
{

  cup_visible = false;

  ipc_messaging();
  
  if ( state != last_state ) {
    cout << msg_prefix << "State changed to: ";
  }

  switch (state) {
  case CANNIKIN_STATE_TEST_MODE:
    if (state != last_state)  cout << "CANNIKIN_STATE_TEST_MODE" << endl;
    if ( ! camless_mode )  test_mode();
    break;

  case CANNIKIN_STATE_STEREO_TEST_MODE:
    if (state != last_state)  cout << "CANNIKIN_STATE_STEREO_TEST_MODE" << endl;
    if ( ! camless_mode )  stereo_test_mode();
    break;

  case CANNIKIN_STATE_REINITIALIZE_COLORMAP:
    if (state != last_state)  cout << "CANNIKIN_STATE_REINITIALIZE_COLORMAP" << endl;
    if ( ! camless_mode )  reinitialize_colormap();
    state = CANNIKIN_STATE_DETECTION;
    break;

  case CANNIKIN_STATE_DETECTION:
    if (state != last_state)  cout << "CANNIKIN_STATE_DETECTION" << endl;
    if ( ! camless_mode )  detect_cup();
    break;

  case CANNIKIN_STATE_DETERMINE_CUP_COLOR:
    if (state != last_state)  cout << "CANNIKIN_STATE_DETERMINE_CUP_COLOR" << endl;
    if ( ! camless_mode )  determine_cup_color();
    break;

  default: return;
  }

  last_state = state;
}


void
CannikinPipeline::test_mode()
{
  cam->capture();
#ifdef HAVE_TRICLOPS_SDK
  triclops->preprocess_stereo();
  triclops->calculate_yuv(/* both */ true);
  //cam->set_image_number(Bumblebee2Camera::); // left
  //convert(cspace_from, cspace_to, cam->buffer(), buffer_src, width, height);
  //memcpy(buffer_src, cam->buffer(), buffer_size);
  //cam->set_image_number(1); // right
  //memcpy(buffer, cam->buffer(), buffer_size);
  memcpy(buffer_src, triclops->yuv_buffer(), buffer_size);
  memcpy(buffer, triclops->auxiliary_yuv_buffer(), buffer_size);
#endif
  cam->dispose_buffer();
}


void
CannikinPipeline::stereo_test_mode()
{
  cam->capture();
#ifdef HAVE_TRICLOPS_SDK
  triclops->preprocess_stereo();
  triclops->calculate_yuv(/* both */ false);
  triclops->calculate_disparity();
  memcpy(buffer_src, triclops->yuv_buffer(), buffer_size);
  memset(buffer, 128, buffer_size);
  memcpy(buffer, triclops->disparity_buffer(), triclops->disparity_buffer_size());
  disparity_scanlines->set_center(width/2, height/2);
  disparity_scanlines->set_radius(3, 100);
  while ( ! disparity_scanlines->finished() ) {
    unsigned int dx = (*disparity_scanlines)->x;
    unsigned int dy = (*disparity_scanlines)->y;
    drawer->drawPoint(dx, dy);
    ++(*disparity_scanlines);
  }
#endif
  cam->dispose_buffer();
}


void
CannikinPipeline::reinitialize_colormap()
{
  if ( colormaps.find(_cup_color) != colormaps.end() ) {
    cout << "Loading colormap " << colormaps[_cup_color] << endl;
    cm->load(colormaps[_cup_color]);
  } else {
    cout << "Cannot load colormap for requested color!" << endl;
    cm->reset();
  }
}


void
CannikinPipeline::detect_cup()
{
#ifdef HAVE_TRICLOPS_SDK
  cam->capture();

  triclops->preprocess_stereo();
  triclops->calculate_yuv();

  gettimeofday(&data_taken_time, NULL);

  // Convert buffer (re-order bytes) and set classifier buffer
  //convert(cspace_from, cspace_to, cam->buffer(), buffer_src, width, height);
  //memcpy(buffer, buffer_src, buffer_size);
  memcpy(buffer_src, triclops->yuv_buffer(), buffer_size);

  // Classify image, find ROIs by color
  classifier->set_src_buffer( buffer_src, width, height );
  rois = classifier->classify();

  if (rois->empty()) {
    if ( generate_output ) {
      cout << msg_prefix << cred << "No ROIs!" << cnormal << endl;
    }
    // No box
    shm_buffer->set_circle_found(false);
    shm_buffer->set_roi(0, 0, 0, 0);
    //box_rel->reset();
  }

  // Go through all ROIs, filter and recognize shapes
  for (r = rois->begin(); r != rois->end(); ++r) {

    if ( generate_output ) {
      cout << msg_prefix << cgreen << "ROI:     " << cnormal
  	   << "start: (" << (*r).start.x << "," << (*r).start.y << ")"
    	   << "   width: " << (*r).width
    	   << "   height: " << (*r).height
           << endl;
    }

    
    // Try to detect box shape
    if ((*r).hint == H_BALL) {
      cup_visible = true;

      shm_buffer->set_roi(r->start.x, r->start.y, r->width, r->height);

      triclops->calculate_disparity(&(*r));

      if ( (r->width > 10) && (r->height > 10) ) {
	// Take five points and calculate some distances...
	std::vector<DisparityPoint> points;
	std::vector<DisparityPoint> wpoints;
	unsigned int half_width = r->width / 2;
	unsigned int half_height = r->height / 2;
	unsigned int center_x = r->start.x + half_width;
	unsigned int center_y = r->start.y + half_height;

	shm_buffer->set_circle( center_x, center_y, 5 );
	shm_buffer->set_circle_found( true );

	memcpy(buffer, triclops->disparity_buffer(), cam->pixel_width() * cam->pixel_height());
	memset(buffer + width * height, 128, width * height);

	points.clear();
	wpoints.clear();

	disparity_scanlines->set_center(center_x, center_y);
	disparity_scanlines->set_radius(3, 0);
	cout << msg_prefix << "Setting center_x=" << center_x << "  center_y=" << center_y << "  dead_radius=" << 3 << "   max_radius=" << min(r->width, r->height) << endl;
	//disparity_scanlines->setDimensions(center_x - min((unsigned int)10, half_width),
	//				   center_y - ((half_height > 10) ? half_height  - 10 : half_height));

	while ( ! disparity_scanlines->finished() ) {
	  unsigned int dx = (*disparity_scanlines)->x;
	  unsigned int dy = (*disparity_scanlines)->y;
          if ((dx >= r->start.x) && (dx <= r->start.x + r->width) &&
              (dy >= r->start.y) && (dy <= r->start.y + r->height) ) {
  	    if ( (dx > (center_x - min(center_x, (unsigned int)8))) &&
  	         (dx < center_x + 8) ) {
	      // we only take points close to the x center to get a good bearing
              //cout << "checking  ";
	      if ( triclops->get_xyz(dx, dy, &x, &y, &z) ) {
	        points.push_back(DisparityPoint(dx, dy, x, y, z));
	      }
	      if ( triclops->get_world_xyz(dx, dy, &wx, &wy, &wz) ) {
	        wpoints.push_back(DisparityPoint(dx, dy, wx, wy, wz));
                //cout << "using_wp  ";
	      }

              //cout << "OK" << endl;
	      drawer->setColor(128, 0, 255); // orange
	      drawer->drawPoint(dx, dy);
	    } else {
              //cout << "IGNORING" << endl;
	      drawer->setColor(128, 255, 0); // blue
	      drawer->drawPoint(dx, dy);
	    }

          }
	  ++(*disparity_scanlines);
	}

	if ( points.empty() || wpoints.empty() ) {
	  cout << "No valid disparity for any points. Doh!" << endl;
	    cup_visible = false;
	} else {
	  sort( points.begin(), points.end() );
	  int elem = (points.size() + 1) / 2;
	  x = points[elem].x;
	  y = points[elem].y;
	  z = points[elem].z;

	  sort( wpoints.begin(), wpoints.end() );
	  int welem = (wpoints.size() + 1) / 2;
	  wx = wpoints[welem].x;
	  wy = wpoints[welem].y;
	  wz = wpoints[welem].z;
	}
      }

      if ( generate_output ) {
	// find distance for ROI center pixel
	if ( cup_visible ) {
	  cout << msg_prefix << cgreen << " (x,y,z) = ("
		 << x << "," << y << "," << z << ")"
                 << "    (wx,wy,wz) = (" << wx << "," << wy << "," << wz << ")"
                 << cnormal << endl;
	} else {
	  cout << msg_prefix << cred << "Cup not visible" << cnormal << endl;
	}
      }

      break;
    } // end is box
  } // end for rois

  rois->clear();
  delete rois;

  cam->dispose_buffer();

#endif /* HAVE_TRICLOPS_SDK */
}


void
CannikinPipeline::determine_cup_color()
{
#ifdef HAVE_TRICLOPS_SDK
  if ( state != last_state ) {
    // First call
    determined_valid_frames = 0;
    determine_cycle_num = 0;
    cup_color_determination_done = false;
    _cup_color = CC_RED;
    reinitialize_colormap();
  }

  if ( cup_color_determination_done ) {
    return;
    //     determined_valid_frames = 0;
    //     determine_cycle_num = 0;
    //     cup_color_determination_done = false;
    //     _cup_color = CC_YELLOW;
    //     reinitialize_colormap();
  }

  cam->capture();
  triclops->preprocess_stereo();
  triclops->calculate_yuv(/* both */ true);
  memcpy(buffer_src, triclops->yuv_buffer(), buffer_size);
  memcpy(buffer, triclops->auxiliary_yuv_buffer(), buffer_size);

  // Classify image, find ROIs by color
  classifier->set_src_buffer( buffer_src, width, height );
  rois = classifier->classify();

  if (rois->empty()) {
    if ( generate_output ) {
      cout << msg_prefix << cred << "No ROIs!" << cnormal << endl;
    }
    // No box
    shm_buffer->set_circle_found(false);
    shm_buffer->set_roi(0, 0, 0, 0);
    //box_rel->reset();
  }

  cout << "Testing ROIs" << endl;

  // Go through all ROIs, filter and recognize shapes
  for (r = rois->begin(); r != rois->end(); ++r) {

    if ( generate_output ) {
      cout << msg_prefix << cgreen << "ROI:     " << cnormal
  	   << "start: (" << (*r).start.x << "," << (*r).start.y << ")"
    	   << "   width: " << (*r).width
    	   << "   height: " << (*r).height
           << endl;
    }
    
    // Try to detect box shape
    if ((*r).hint == H_BALL) {

      if ( /* (*r).contains(width / 2, height / 2) && */
	   ((*r).width > 100) && ((*r).height > 100) ) {
	// we have a possible ROI
	if ( determine_cycle_num > 0 ) {
	  --determine_cycle_num;
	}
	++determined_valid_frames;
	if ( determined_valid_frames > 4 ) {
	  cup_color_determination_done = true;
	  _determined_cup_color = _cup_color;
	  if ( _cup_color == CC_ORANGE ) {
	    cout << "Determined color: ORANGE" << endl;
	  } else if (_cup_color == CC_GREEN ) {
	    cout << "Determined color: GREEN" << endl;
	  } else if (_cup_color == CC_BLUE ) {
	    cout << "Determined color: BLUE" << endl;
	  } else if (_cup_color == CC_RED ) {
	    cout << "Determined color: RED" << endl;
	  } else if (_cup_color == CC_YELLOW ) {
	    cout << "Determined color: YELLOW" << endl;
	  }
	}
      } else {
	cout << "ROI does not have minimum requested size, only "
	     << (*r).width << " x " << (*r).height << endl;
      }

    } // end is box
  } // end for rois

  rois->clear();
  delete rois;

  ++determine_cycle_num;
  if ( determine_cycle_num > 10 ) {
    // we tried for 10 frames, but did not get the needed valid frames, switch
    // to next color
    determine_cycle_num = 0;
    determined_valid_frames = 0;
    cmit = colormaps.begin();
    while (cmit != colormaps.end()) {
      if ( (*cmit).first == _cup_color ) {
        ++cmit;
        if ( cmit == colormaps.end() ) {
          cmit = colormaps.begin();
        }
        _cup_color = ((*cmit).first);
        break;
      }
      ++cmit;
    }
    reinitialize_colormap();
  }

  cam->dispose_buffer();
#endif /* HAVE_TRICLOPS_SDK */
}


bool
CannikinPipeline::get_xyz(float *x, float *y, float *z)
{
  if ( cup_visible ) {
    *x = this->x;
    *y = this->y;
    *z = this->z;
    return true;
  } else {
    return false;
  }
}

bool
CannikinPipeline::get_world_xyz(float *x, float *y, float *z)
{
  if ( cup_visible ) {
    *x = this->wx;
    *y = this->wy;
    *z = this->wz;
    return true;
  } else {
    return false;
  }
}

/// @endcond
