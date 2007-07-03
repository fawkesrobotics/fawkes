
/***************************************************************************
 *  firewire.cpp - Implementation to access FW cam using libdc1394
 *
 *  Generated: Tue Feb 22 13:28:08 2005
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

#include <core/exception.h>
#include <utils/system/console_colors.h>

#include <cstdlib>
#include <unistd.h>

#include <string>

#include <cams/firewire.h>
#include <cams/cam_exceptions.h>
#include <fvutils/system/camargp.h>

#include <dc1394/utils.h>

using namespace std;

/** @class FirewireCamera <cams/firewire.h>
 * Firewire camera.
 * This camera implementation allows for access to IEEE1394 cameras via
 * libdc1394.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param framerate desired framerate
 * @param mode desired mode
 * @param speed IEEE 1394 speed
 * @param num_buffers number of DMA buffers
 */
FirewireCamera::FirewireCamera(dc1394framerate_t framerate,
			       dc1394video_mode_t mode,
			       dc1394speed_t speed,
			       int num_buffers)
{
  started = opened = false;
  valid_frame_received = false;
  _auto_focus = true; // assume auto_focus, checked in open()
  this->speed = speed;
  this->num_buffers = num_buffers;
  this->mode = mode;
  this->framerate = framerate;

  camera = NULL;

  if ((mode = DC1394_VIDEO_MODE_640x480_YUV422) && (framerate == DC1394_FRAMERATE_30)) {
    // cerr  << "When in mode YUV422 @ 640x480 with more than 15 fps. Setting framerate to 15fps." << endl;
    this->framerate = DC1394_FRAMERATE_15;
  }

  model = NULL;
}


/** Empty destructor. */
FirewireCamera::~FirewireCamera()
{
  if ( model != NULL ) {
    free(model);
  }
}


void
FirewireCamera::open()
{
  if (opened) return;

  unsigned int i = 0;
  dc1394camera_t       **cameras;
  unsigned int num_cameras = 0;
  unsigned int cam = 0;
  bool found = false;

  opened = false;

  if ( dc1394_find_cameras(&cameras, &num_cameras) != DC1394_SUCCESS ) {
    /*
    cout  << cyellow << "Failure while finding cameras." << cnormal << endl
	  << "Please check that" << endl
	  << "  - the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded " << endl
	  << "  - you have read/write access to /dev/raw1394" << endl;
    */
    throw Exception("Could not find any cameras");
  }

  if (num_cameras > 0) {
    if ( strcmp(model, "any") == 0 ) {
      /* use the first camera found */
      cam=0;
      found = true;

      camera = cameras[cam];
      for (i = 1; i < num_cameras; ++i) {
	dc1394_free_camera(cameras[i]);
      }
      free(cameras);
      cameras = NULL;
    } else {
      camera = NULL;
      for (i = 0; i < num_cameras; ++i) {
	if ( !found && strcmp(model, cameras[i]->model) == 0) {
	  // found desired camera
	  camera = cameras[i];
	  found = true;
	} else {
	  dc1394_free_camera(cameras[i]);
	}
      }
      free(cameras);
      cameras = NULL;
      if ( camera == NULL ) {
	Exception e("Could not find camera");
	e.append("Camera of model '%s' not found!");
	throw e;
      }
    }
    if (camera->bmode_capable > 0) {
      // set b-mode and reprobe modes,...
      // (higher fps formats might not be reported as available in legacy mode)
      dc1394_video_set_operation_mode(camera, DC1394_OPERATION_MODE_1394B);
    }
    dc1394_cleanup_iso_channels_and_bandwidth(camera);
    
    dc1394_video_set_iso_speed(camera, speed);
    dc1394_video_set_mode(camera, mode);
    dc1394_video_set_framerate(camera, framerate);

  } else {
    // cout  << cred << "Could not find any camera." << cnormal << endl;
    throw Exception("No cameras connected");;
  }

  opened = found;
}


void
FirewireCamera::start()
{
  if (started) return;

  if (!opened) {
    // cout  << cred << "Tried to start closed cam. Giving up." << cnormal << endl;
    return;
  }


  //cout  << "Starting camera" << endl;

  if ( dc1394_capture_setup( camera, num_buffers ) != DC1394_SUCCESS ) {
    // cout  << cred << "Could not start capturing" << endl;
    dc1394_capture_stop(camera);
    return;
  }

  if ( dc1394_video_set_transmission(camera, DC1394_ON) != DC1394_SUCCESS) {
    // cout  << cred << "Could not start video transmission" << cnormal << endl;
    dc1394_capture_stop(camera);
    return;
  }
				
  // Give it some time to be ready
  usleep(500000);

  started = true;
}


void
FirewireCamera::stop()
{
  dc1394_capture_stop(camera);
  dc1394_video_set_transmission(camera, DC1394_OFF);
  started = false;
}


/** Check if ISO mode is enabled.
 * @return true if isochronous transfer is running, false otherwise.
 */
bool
FirewireCamera::iso_mode_enabled()
{
  dc1394switch_t status;
  if ( dc1394_video_get_transmission(camera, &status) != DC1394_SUCCESS) {
    // cout  << cred << "Could not get transmission status" << cnormal << endl;
    return false;
  } else {
    return (status == DC1394_ON);
  }
}


void
FirewireCamera::print_info()
{
  if (opened) {
    dc1394_print_camera_info( camera );
  }
}

void
FirewireCamera::capture()
{

  if (! opened) return;
  if (! started) return;

  if (! iso_mode_enabled()) {
    //cout  << cred << "ISO mode enabled while trying to capture" << cnormal << endl;
    return;
  }

  dc1394error_t err;
  if (DC1394_SUCCESS != (err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame))) {
    valid_frame_received = false;
    throw CaptureException(dc1394_error_strings[err]);
  } else {
    valid_frame_received = true;
  }
}


void
FirewireCamera::flush()
{
  capture();
  // HACK, needed or we will get kernel NULL pointer exception *urgh*
  usleep(100000);
  dispose_buffer();
}


unsigned char*
FirewireCamera::buffer()
{
  if ( valid_frame_received ) {
    return frame->image;
  } else {
    return NULL;
  }
}


unsigned int
FirewireCamera::buffer_size()
{
  if ( valid_frame_received ) {
    return frame->total_bytes;
  } else {
    return 0;
  }
}

void
FirewireCamera::close()
{
  if ( started ) stop();
  if ( opened ) {
    dc1394_free_camera( camera );
    camera = NULL;
    opened = false;
  }
}


void
FirewireCamera::dispose_buffer()
{
  ////cout  << "Finishing move on buffer" << endl;
  dc1394_capture_enqueue( camera, frame );
}


unsigned int
FirewireCamera::pixel_width()
{
  if (opened) {
    if ( valid_frame_received ) {
      return frame->size[0];
    } else {
      unsigned int width, height;
      dc1394_get_image_size_from_video_mode(camera, mode, &width, &height);
      return width;
    }
  } else {
    throw Exception("Camera not opened");
  }
}


unsigned int
FirewireCamera::pixel_height()
{
  if (opened) {
    if ( valid_frame_received ) {
      return frame->size[1];
    } else {
      unsigned int width, height;
      dc1394_get_image_size_from_video_mode(camera, mode, &width, &height);
      return height;
    }
  } else {
    throw Exception("Camera not opened");
  }
}


colorspace_t
FirewireCamera::colorspace()
{
  // this needs to be changed for different modes
  return YUV422_PACKED;
}


bool
FirewireCamera::ready()
{
  return started;
}


void
FirewireCamera::set_image_number(unsigned int n)
{
}


/* CAMERA CONTROL STUFF */

bool
FirewireCamera::supports_focus()
{
  return true;
}


void
FirewireCamera::set_auto_focus(bool enabled)
{
  /* 0 == auto off */
  if (dc1394_feature_set_mode(camera, DC1394_FEATURE_FOCUS,
			      enabled ? DC1394_FEATURE_MODE_AUTO : DC1394_FEATURE_MODE_MANUAL)
      == DC1394_SUCCESS) {
    _auto_focus = enabled;
  }
}


bool
FirewireCamera::auto_focus()
{
  return _auto_focus;
}


unsigned int
FirewireCamera::focus()
{
  unsigned int focus = 0;
  if (dc1394_feature_get_value(camera, DC1394_FEATURE_FOCUS, &focus) == DC1394_SUCCESS) {
    return focus;
  } else {
    return 0;
  }
  
}

void
FirewireCamera::set_focus(unsigned int focus)
{
  dc1394_feature_set_value(camera, DC1394_FEATURE_FOCUS, focus);
}

unsigned int
FirewireCamera::focus_min()
{
  unsigned int min = 0;
  unsigned int max = 0;
  if (dc1394_feature_get_boundaries(camera, DC1394_FEATURE_FOCUS, &min, &max) == DC1394_SUCCESS) {
    return min;
  } else {
    return 0;
  }
}

unsigned int
FirewireCamera::focus_max()
{
  unsigned int max = 0;
  unsigned int min = 0;
  if (dc1394_feature_get_boundaries(camera, DC1394_FEATURE_FOCUS, &min, &max) == DC1394_SUCCESS) {
    return max;
  } else {
    return 0;
  }
}



/** Constructor.
 * Initialize and take parameters from camera argument parser. The following
 * arguments are supported:
 * - mode=MODE where MODE is one of
 *   - 640x480_YUV422
 *   - 640x480_MONO16
 *   - FORMAT7_0
 *   - FORMAT7_1
 *   - FORMAT7_2
 *   - FORMAT7_3
 *   - FORMAT7_4
 *   - FORMAT7_5
 *   - FORMAT7_6
 *   - FORMAT7_7
 * - coding=CODING, color coding for Format7, CODING is one of:
 *   - YUV422
 *   - MONO16
 *   - RAW16
 * - isospeed=SPEED, ISO speed, SPEED is one of:
 *   - 400
 *   - 800
 * - framerate=FPS, desired rate in frames per second, FPS is one of:
 *   - 15
 *   - 30
 *   - 60
 *   - 120
 * - nbufs=NBUFS, number of DMA buffers, integer, 0 < n <= 32
 * - width=WIDTH, width in pixels of Format7 ROI
 * - height=HEIGHT, height in pixels of Format7 ROI
 * - startx=STARTX, X start of Format7 ROI
 * - starty=STARTY, Y start of Format7 ROI
 * @param cap camera argument parser
 */
FirewireCamera::FirewireCamera(const CameraArgumentParser *cap)
{
  started = opened = false;
  valid_frame_received = false;
  _auto_focus = true; // assume auto_focus, checked in open()

  // Defaults
  mode = DC1394_VIDEO_MODE_640x480_YUV422;
  speed = DC1394_ISO_SPEED_400;
  framerate = DC1394_FRAMERATE_30;
  camera = NULL;
  format7_width = format7_height = format7_startx = format7_starty = 0;
  model = strdup(cap->cam_id().c_str());

  if ( cap->has("mode") ) {
    string m = cap->get("mode");
    if ( m == "640x480_MONO16" ) {
      mode = DC1394_VIDEO_MODE_640x480_MONO16;
    } else if ( m == "FORMAT7_0" ) {
      mode = DC1394_VIDEO_MODE_FORMAT7_0;
    } else if ( m == "FORMAT7_1" ) {
      mode = DC1394_VIDEO_MODE_FORMAT7_1;
    } else if ( m == "FORMAT7_2" ) {
      mode = DC1394_VIDEO_MODE_FORMAT7_2;
    } else if ( m == "FORMAT7_3" ) {
      mode = DC1394_VIDEO_MODE_FORMAT7_3;
    } else if ( m == "FORMAT7_4" ) {
      mode = DC1394_VIDEO_MODE_FORMAT7_4;
    } else if ( m == "FORMAT7_5" ) {
      mode = DC1394_VIDEO_MODE_FORMAT7_5;
    } else if ( m == "FORMAT7_6" ) {
      mode = DC1394_VIDEO_MODE_FORMAT7_6;
    } else if ( m == "FORMAT7_7" ) {
      mode = DC1394_VIDEO_MODE_FORMAT7_7;
    }
  }
  if ( cap->has("coding") ) {
    string c = cap->get("coding");
    if ( c == "YUV422" ) {
      format7_coding = DC1394_COLOR_CODING_YUV422;
    } else if ( c == "MONO16" ) {
      format7_coding = DC1394_COLOR_CODING_MONO16;
    } else if ( c == "RAW16" ) {
      format7_coding = DC1394_COLOR_CODING_RAW16;
    }
  }
  if ( cap->has("isospeed") ) {
    string s = cap->get("isospeed");
    if ( s == "400" ) {
      speed = DC1394_ISO_SPEED_400;
    } else if ( s == "800" ) {
      speed = DC1394_ISO_SPEED_800;
    }
  }
  if ( cap->has("framerate") ) {
    string f = cap->get("framerate");
    if ( f == "15" ) {
      framerate = DC1394_FRAMERATE_15;
    } else if ( f == "30" ) {
      framerate = DC1394_FRAMERATE_30;
    } else if ( f == "60" ) {
      framerate = DC1394_FRAMERATE_60;
    } else if ( f == "120" ) {
      framerate = DC1394_FRAMERATE_120;
    }
  }
  if ( cap->has("nbufs") ) {
    num_buffers = atoi(cap->get("nbufs").c_str());
  }
  if ( cap->has("width") ) {
    format7_width = atoi(cap->get("width").c_str());
  }
  if ( cap->has("height") ) {
    format7_height = atoi(cap->get("height").c_str());
  }
  if ( cap->has("startx") ) {
    format7_startx = atoi(cap->get("startx").c_str());
  }
  if ( cap->has("starty") ) {
    format7_starty = atoi(cap->get("starty").c_str());
  }
}


/** Print list of cameras.
 * Prints a list of available cameras to stdout.
 */
void
FirewireCamera::print_available_fwcams()
{
  
  dc1394camera_t       **cameras;
  unsigned int num_cameras = 0;

  if ( dc1394_find_cameras(&cameras, &num_cameras) != DC1394_SUCCESS ) {
    printf("Finding cameras failed\n");
    return;
  }

  if (num_cameras > 0) {
    // print cameras
    for (unsigned int i = 0; i < num_cameras; ++i) {
      printf("Vendor: %30s (%8.0x)  Model: %30s (%8.0x)\n",
	     cameras[i]->vendor, cameras[i]->vendor_id,
	     cameras[i]->model, cameras[i]->model_id);
      dc1394_print_camera_info(cameras[i]);
      dc1394_free_camera(cameras[i]);
    }
  } else {
    printf("Could not find any cameras\n");
  }
}
