
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

#include <stdlib.h>
#include <unistd.h>

#include <string>

#include <cams/firewire.h>
#include <fvutils/system/camargp.h>


namespace dc1394 {
#include <dc1394/utils.h>
}

using namespace dc1394;
using namespace std;

/** Constructor.
 * @param framerate desired framerate
 * @param mode desired mode
 * @param speed IEEE 1394 speed
 * @param num_buffers number of DMA buffers
 */
FirewireCamera::FirewireCamera(dc1394::dc1394framerate_t framerate,
			       dc1394::dc1394video_mode_t mode,
			       dc1394::dc1394speed_t speed,
			       int num_buffers)
{
  started = opened = false;
  valid_frame_received = false;
  drop_frames = 1;
  _auto_focus = true; // assume auto_focus, checked in open()
  this->speed = speed;
  this->num_buffers = num_buffers;
  this->mode = mode;
  this->framerate = framerate;

  camera = NULL;
  cameras = NULL;

  if ((mode = DC1394_VIDEO_MODE_640x480_YUV422) && (framerate == DC1394_FRAMERATE_30)) {
    // cerr << msg_prefix << "When in mode YUV422 @ 640x480 with more than 15 fps. Setting framerate to 15fps." << endl;
    this->framerate = DC1394_FRAMERATE_15;
  }
}


/** Empty destructor. */
FirewireCamera::~FirewireCamera()
{
}


void
FirewireCamera::open()
{

  if (opened) return;

  unsigned int i = 0;
  unsigned int num_cameras = 0;
  unsigned int cam = 0;
  int found = 0;

  opened = false;

  if ( dc1394_find_cameras(&cameras, &num_cameras) != DC1394_SUCCESS ) {
    /*
    cout << msg_prefix << cyellow << "Failure while finding cameras." << cnormal << endl
	 << msg_prefix << "Please check that" << endl
	 << msg_prefix << "  - the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded " << endl
	 << msg_prefix << "  - you have read/write access to /dev/raw1394" << endl;
    */
    throw Exception("Could not find any cameras");
  }

  if (num_cameras > 0) {
    /* use the first camera found */
    cam=0;
    found = 1;

    camera = cameras[cam];
    for (i = 1; i < num_cameras; ++i) {
      dc1394_free_camera(cameras[i]);
    }
    free(cameras);
    cameras = NULL;

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
    // cout << msg_prefix << cred << "Could not find any camera." << cnormal << endl;
    throw Exception("No cameras connected");;
  }

  opened = (found != 0);
}


void
FirewireCamera::start()
{
  if (started) return;

  if (!opened) {
    // cout << msg_prefix << cred << "Tried to start closed cam. Giving up." << cnormal << endl;
    return;
  }


  //cout << msg_prefix << "Starting camera" << endl;

  if ( dc1394_capture_setup( camera, num_buffers ) != DC1394_SUCCESS ) {
    //cout << msg_prefix << cred << "Could not start capturing" << endl;
    dc1394_capture_stop(camera);
    dc1394_free_camera(camera);
    return;
  }

  if ( dc1394_video_set_transmission(camera, DC1394_ON) != DC1394_SUCCESS) {
    //cout << msg_prefix << cred << "Could not start video transmission" << cnormal << endl;
    dc1394_capture_stop(camera);
    dc1394_free_camera(camera);
    return;
  }
				
  // Give it some time to be ready
  usleep(500000);

  started = true;
}


/** Check if ISO mode is enabled.
 * @return true if isochronous transfer is running, false otherwise.
 */
bool
FirewireCamera::iso_mode_enabled()
{
  dc1394switch_t status;
  if ( dc1394_video_get_transmission(camera, &status) != DC1394_SUCCESS) {
    //cout << msg_prefix << cred << "Could not get transmission status" << cnormal << endl;
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
    //cout << msg_prefix << cred << "ISO mode enabled while trying to capture" << cnormal << endl;
    return;
  }

  if (dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame) != DC1394_SUCCESS) {
    //cout << msg_prefix << cred << "Could not capture frame" << cnormal << endl;
    valid_frame_received = false;
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
  if (started) {
    if ( dc1394_video_set_transmission(camera, DC1394_OFF) != DC1394_SUCCESS) {
      //cout << msg_prefix << cred << "Could not stop video transmission" << cnormal << endl;
    } else {
      usleep(50000);
      if (iso_mode_enabled()) {
	//cout << msg_prefix << cred << "ISO mode still enabled after trying to shut it down. Will remain on" << cnormal << endl;
      }
    }
    dc1394_capture_stop( camera );
    dc1394_free_camera( camera );
  }
}


void
FirewireCamera::dispose_buffer()
{
  ////cout << msg_prefix << "Finishing move on buffer" << endl;
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
 * - mode=<mode> where <mode> is one of
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
 * - coding=<coding>, color coding for Format7, <coding> is one of:
 *   - YUV422
 *   - MONO16
 *   - RAW16
 * - isospeed=<speed>, ISO speed, <speed> is one of:
 *   - 400
 *   - 800
 * - framerate=<fps>, desired rate in frames per second, <fps> is one of:
 *   - 15
 *   - 30
 *   - 60
 *   - 120
 * - nbufs=<nbufs>, number of DMA buffers, integer, 0 < n <= 32
 * - width=<width>, width in pixels of Format7 ROI
 * - height=<height>, height in pixels of Format7 ROI
 * - startx=<startx>, X start of Format7 ROI
 * - starty=<starty>, Y start of Format7 ROI
 * @param cap camera argument parser
 */
FirewireCamera::FirewireCamera(CameraArgumentParser *cap)
{
  // Defaults
  mode = DC1394_VIDEO_MODE_640x480_YUV422;
  speed = DC1394_ISO_SPEED_400;
  framerate = DC1394_FRAMERATE_30;
  camera = NULL;
  cameras = NULL;
  format7_width = format7_height = format7_startx = format7_starty = 0;

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
