 
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
#include <core/exceptions/software.h>
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
  _started = _opened = false;
  _valid_frame_received = false;
  _auto_focus = true; // assume auto_focus, checked in open()
  _auto_shutter = false;
  _auto_white_balance = false;
  _speed = speed;
  _num_buffers = num_buffers;
  _mode = mode;
  _framerate = framerate;
  _white_balance_ub = 0xFFFFFFFF;
  _white_balance_vr = 0xFFFFFFFF;

  _dc1394 = NULL;
  _camera = NULL;

  if ((mode == DC1394_VIDEO_MODE_640x480_YUV422) && (framerate == DC1394_FRAMERATE_30)) {
    // cerr  << "When in mode YUV422 @ 640x480 with more than 15 fps. Setting framerate to 15fps." << endl;
    _framerate = DC1394_FRAMERATE_15;
  }

  _model = NULL;
}


/** Empty destructor. */
FirewireCamera::~FirewireCamera()
{
  close();
  
  if ( _model != NULL ) {
    free(_model);
  }
}


void
FirewireCamera::open()
{
  if (_opened) return;

  _dc1394 = dc1394_new();
  dc1394camera_list_t *list;
  dc1394error_t        err;

  if ( dc1394_camera_enumerate(_dc1394, &list) != DC1394_SUCCESS ) {
    throw Exception("Could not enumerate cameras");
  }

  if (list->num > 0) {
    if ( strcmp(_model, "any") == 0 ) {
      /* use the first camera found */
      _camera = dc1394_camera_new(_dc1394, list->ids[0].guid);
      if (! _camera) {
        dc1394_free(_dc1394);
        _dc1394 = NULL;
        throw Exception("Could not create camera for first foiund camera");
      }
    } else {
      _camera = NULL;
      for (unsigned int i = 0; i < list->num; ++i) {
        dc1394camera_t *tmpcam = dc1394_camera_new(_dc1394, list->ids[i].guid);
	if ( strcmp(_model, tmpcam->model) == 0) {
	  // found desired camera
	  _camera = tmpcam;
          break;
	} else {
	  dc1394_camera_free(tmpcam);
	}
      }
      if ( _camera == NULL ) {
	throw Exception("Could not find camera with model %s", _model);
      }
    }

    if ( iso_mode_enabled() ) {
      dc1394_video_set_transmission(_camera, DC1394_OFF);
    }
    // These methods would cleanup the mess left behind by other processes,
    // but as of now (libdc1394 2.0.0 rc9) this is not supported for the Juju stack
    dc1394_iso_release_bandwidth(_camera, INT_MAX);
    for (int channel = 0; channel < 64; ++channel) {
      dc1394_iso_release_channel(_camera, channel);
    }
    // This is rude, but for now needed (Juju)...
    //dc1394_reset_bus(_camera);

    if (_camera->bmode_capable > 0) {
      dc1394_video_set_operation_mode(_camera, DC1394_OPERATION_MODE_1394B);
    }
    if ( //((err = dc1394_cleanup_iso_channels_and_bandwidth(camera)) != DC1394_SUCCESS) ||
         ((err = dc1394_video_set_iso_speed(_camera, _speed)) != DC1394_SUCCESS) ||
         ((err = dc1394_video_set_mode(_camera, _mode)) != DC1394_SUCCESS) ||
         ((err = dc1394_video_set_framerate(_camera, _framerate)) != DC1394_SUCCESS) ) {
      throw Exception("Setting up the camera failed: %s", dc1394_error_get_string(err));
    }

    if (_format7_mode_enabled) {
      if ( ((err = dc1394_format7_set_image_size(_camera, _mode, _format7_width, _format7_height)) != DC1394_SUCCESS) ||
           ((err = dc1394_format7_set_image_position(_camera, _mode, _format7_startx, _format7_starty)) != DC1394_SUCCESS) ||
           ((err = dc1394_format7_set_color_coding(_camera, _mode, _format7_coding)) != DC1394_SUCCESS) ||
           ((err = dc1394_format7_set_packet_size(_camera, _mode, _format7_bpp)) != DC1394_SUCCESS) ) {
        throw Exception("Could not setup Format7 parameters: %s", dc1394_error_get_string(err));
      }
    }

    set_auto_focus(_auto_focus);
    set_auto_shutter(_auto_shutter);
    set_auto_white_balance(_auto_white_balance);
    if ( ! _auto_white_balance &&
	 (_white_balance_ub != 0xFFFFFFFF) &&
	 (_white_balance_vr != 0xFFFFFFFF)) {
      set_white_balance(_white_balance_ub, _white_balance_vr);
    }

  } else {
    throw Exception("No cameras connected");
  }

  _opened = true;
}


void
FirewireCamera::start()
{
  if (_started) return;

  if (! _opened) {
    throw Exception("FirewireCamera: Cannot start closed camera");
  }

  dc1394error_t err;
  if ( (err = dc1394_capture_setup(_camera, _num_buffers, DC1394_CAPTURE_FLAGS_DEFAULT )) != DC1394_SUCCESS ) {
    dc1394_capture_stop(_camera);
    throw Exception("FirewireCamera: Could not setup capture (%s)", dc1394_error_get_string(err));
  }

  if ( (err = dc1394_video_set_transmission(_camera, DC1394_ON)) != DC1394_SUCCESS) {
    // cout  << cred << "Could not start video transmission" << cnormal << endl;
    dc1394_capture_stop(_camera);
    throw Exception("FirewireCamera: Could not start ISO transmission (%s)", dc1394_error_get_string(err));
  }
				
  // Give it some time to be ready
  usleep(500000);

  _started = true;
}


void
FirewireCamera::stop()
{
  dc1394_video_set_transmission(_camera, DC1394_OFF);
  dc1394_capture_stop(_camera);
  _started = false;
}


/** Check if ISO mode is enabled.
 * @return true if isochronous transfer is running, false otherwise.
 * @exception Exception thrown if the transmission status could not be determined
 */
bool
FirewireCamera::iso_mode_enabled()
{
  dc1394switch_t status;
  if ( dc1394_video_get_transmission(_camera, &status) != DC1394_SUCCESS) {
    throw Exception("Could not get transmission status");
  } else {
    return (status == DC1394_ON);
  }
}


void
FirewireCamera::print_info()
{
  if (_opened) {
    dc1394_camera_print_info( _camera, stdout );
  }
}


/** Get Firewire GUID of camera.
 * @return IEEE1394 GUID
 */
uint64_t
FirewireCamera::guid() const
{
  if ( ! _opened ) {
    throw Exception("Camera not opened");
  }

  return _camera->guid;
}


/** Get camera model.
 * @return string with the camera model name
 */
const char *
FirewireCamera::model() const
{
  if ( ! _opened ) {
    throw Exception("Camera not opened");
  }

  return _camera->model;
}


void
FirewireCamera::capture()
{

  if (! _opened) {
    throw CaptureException("FirewireCamera(%s): cannot capture on closed camera", _model);
  }
  if (! _started) {
    throw CaptureException("FirewireCamera(%s): cannot capture on stopped camera", _model);
  }

  if (! iso_mode_enabled()) {
    throw CaptureException("FirewireCamera(%s): isochronous transfer not active", _model);
  }

  dc1394error_t err;
  if (DC1394_SUCCESS != (err = dc1394_capture_dequeue(_camera, DC1394_CAPTURE_POLICY_WAIT, &_frame))) {
    _valid_frame_received = false;
    throw CaptureException("FireWireCamera(%s): capture failed (%s)",
			   _model, dc1394_error_get_string(err));
  } else {
    _valid_frame_received = true;
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
  if ( _valid_frame_received ) {
    return _frame->image;
  } else {
    return NULL;
  }
}


unsigned int
FirewireCamera::buffer_size()
{
  if ( _valid_frame_received ) {
    return _frame->total_bytes;
  } else {
    return 0;
  }
}

void
FirewireCamera::close()
{
  if ( _started ) stop();
  if ( _opened ) {
    dc1394_camera_free( _camera );
    dc1394_free(_dc1394);
    _camera = NULL;
    _dc1394 = NULL;
    _opened = false;
  }
}


void
FirewireCamera::dispose_buffer()
{
  if ( _valid_frame_received ) {
    dc1394_capture_enqueue( _camera, _frame );
  }
}


unsigned int
FirewireCamera::pixel_width()
{
  if (_opened) {
    if ( _valid_frame_received ) {
      return _frame->size[0];
    } else {
      unsigned int width, height;
      dc1394error_t err;
      if ((err = dc1394_get_image_size_from_video_mode(_camera, _mode, &width, &height)) != DC1394_SUCCESS) {
	throw Exception("FirewireCamera(%s): cannot get width (%s)", _model,
			dc1394_error_get_string(err));
      }
      return width;
    }
  } else {
    throw Exception("Camera not opened");
  }
}


unsigned int
FirewireCamera::pixel_height()
{
  if (_opened) {
    if ( _valid_frame_received ) {
      return _frame->size[1];
    } else {
      unsigned int width, height;
      dc1394error_t err;
      if ((err = dc1394_get_image_size_from_video_mode(_camera, _mode, &width, &height)) != DC1394_SUCCESS) {
	throw Exception("FirewireCamera(%s): cannot get width (%s)", _model,
			dc1394_error_get_string(err));
      }
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
  switch (_mode) {
  case DC1394_VIDEO_MODE_320x240_YUV422:
  case DC1394_VIDEO_MODE_640x480_YUV422:
  case DC1394_VIDEO_MODE_800x600_YUV422:
  case DC1394_VIDEO_MODE_1024x768_YUV422:
  case DC1394_VIDEO_MODE_1280x960_YUV422:
  case DC1394_VIDEO_MODE_1600x1200_YUV422:
    return YUV422_PACKED;

  case DC1394_VIDEO_MODE_640x480_YUV411:
    return YUV411_PACKED;


  case DC1394_VIDEO_MODE_640x480_RGB8:
  case DC1394_VIDEO_MODE_800x600_RGB8:
  case DC1394_VIDEO_MODE_1024x768_RGB8:
  case DC1394_VIDEO_MODE_1280x960_RGB8:
  case DC1394_VIDEO_MODE_1600x1200_RGB8:
    return RGB;

  case DC1394_VIDEO_MODE_640x480_MONO8:
  case DC1394_VIDEO_MODE_800x600_MONO8:
  case DC1394_VIDEO_MODE_1024x768_MONO8:
  case DC1394_VIDEO_MODE_1280x960_MONO8:
  case DC1394_VIDEO_MODE_1600x1200_MONO8:
    return MONO8;

  case DC1394_VIDEO_MODE_640x480_MONO16:
  case DC1394_VIDEO_MODE_800x600_MONO16:
  case DC1394_VIDEO_MODE_1024x768_MONO16:
  case DC1394_VIDEO_MODE_1280x960_MONO16:
  case DC1394_VIDEO_MODE_1600x1200_MONO16:
    return MONO16;

  case DC1394_VIDEO_MODE_FORMAT7_0:
  case DC1394_VIDEO_MODE_FORMAT7_1:
  case DC1394_VIDEO_MODE_FORMAT7_2:
  case DC1394_VIDEO_MODE_FORMAT7_3:
  case DC1394_VIDEO_MODE_FORMAT7_4:
  case DC1394_VIDEO_MODE_FORMAT7_5:
  case DC1394_VIDEO_MODE_FORMAT7_6:
  case DC1394_VIDEO_MODE_FORMAT7_7:
    switch (_format7_coding) {
    case DC1394_COLOR_CODING_MONO8:
      return MONO8;
    case DC1394_COLOR_CODING_YUV411:
      return YUV411_PACKED;
    case DC1394_COLOR_CODING_YUV422:
      return YUV422_PACKED;
    case DC1394_COLOR_CODING_RGB8:
      return RGB;
    case DC1394_COLOR_CODING_MONO16:
      return MONO16;
    case DC1394_COLOR_CODING_RAW8:
      return RAW8;
    case DC1394_COLOR_CODING_RAW16:
      return RAW16;
    default:
      return CS_UNKNOWN;
    }
    break;

  default:
    return CS_UNKNOWN;
  }
}


bool
FirewireCamera::ready()
{
  return _started;
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
  dc1394error_t err;
  if ((err = dc1394_feature_set_mode(_camera, DC1394_FEATURE_FOCUS,
				     enabled ? DC1394_FEATURE_MODE_AUTO : DC1394_FEATURE_MODE_MANUAL))
      == DC1394_SUCCESS) {
    _auto_focus = enabled;
  } else {
    throw Exception("FirewireCamera(%s): Setting auto focus failed (%s)", _model,
		    dc1394_error_get_string(err));
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
  if (dc1394_feature_get_value(_camera, DC1394_FEATURE_FOCUS, &focus) == DC1394_SUCCESS) {
    return focus;
  } else {
    return 0;
  }
  
}


void
FirewireCamera::set_focus(unsigned int focus)
{
  dc1394_feature_set_value(_camera, DC1394_FEATURE_FOCUS, focus);
}


unsigned int
FirewireCamera::focus_min()
{
  unsigned int min = 0;
  unsigned int max = 0;
  if (dc1394_feature_get_boundaries(_camera, DC1394_FEATURE_FOCUS, &min, &max) == DC1394_SUCCESS) {
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
  if (dc1394_feature_get_boundaries(_camera, DC1394_FEATURE_FOCUS, &min, &max) == DC1394_SUCCESS) {
    return max;
  } else {
    return 0;
  }
}


/** Set status of auto shutter.
 * @param enabled true to enable auto shutter, false to disable.
 */
void
FirewireCamera::set_auto_shutter(bool enabled)
{
  if (dc1394_feature_set_mode(_camera, DC1394_FEATURE_SHUTTER,
			      enabled ? DC1394_FEATURE_MODE_AUTO : DC1394_FEATURE_MODE_MANUAL)
      == DC1394_SUCCESS) {
    _auto_shutter = enabled;
  }
}


/** Get status of auto shutter.
 * @return true if auto shutter is enabled, false otherwise
 */
bool
FirewireCamera::auto_shutter()
{
  return _auto_shutter;
}


/** Set status of auto white balance.
 * @param enabled true to enable auto white balance, false to disable.
 */
void
FirewireCamera::set_auto_white_balance(bool enabled)
{
  if (dc1394_feature_set_mode(_camera, DC1394_FEATURE_WHITE_BALANCE,
			      enabled ? DC1394_FEATURE_MODE_AUTO : DC1394_FEATURE_MODE_MANUAL)
      == DC1394_SUCCESS) {
    _auto_white_balance = enabled;
  }
}


/** Get status of auto white balance.
 * @return true if white balance is enabled, false otherwise
 */
bool
FirewireCamera::auto_white_balance()
{
  return _auto_white_balance;
}


/** Get white balance values.
 * @param ub contains U/B value upon return
 * @param vr contains V/R value upon return
 */
void
FirewireCamera::white_balance(unsigned int *ub, unsigned int *vr)
{
  if ( dc1394_feature_whitebalance_get_value(_camera, ub, vr) != DC1394_SUCCESS ) {
    throw Exception("Failed to retrieve white balance values");
  }
}


/** Set white balance values.
 * @param ub U/B value
 * @param vr V/R value
 */
void
FirewireCamera::set_white_balance(unsigned int ub, unsigned int vr)
{
  if ( dc1394_feature_whitebalance_set_value(_camera, ub, vr) != DC1394_SUCCESS ) {
    throw Exception("Failed to retrieve white balance values");
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
 * - packetsize=BYTES, packet size in BYTES
 * - white_balance=(auto|U,V), white balance value, either auto for auto white balance
 *                             or U/B and V/R values for adjustment
 * - shutter=auto, determine the shutter time automatically
 * @param cap camera argument parser
 */
FirewireCamera::FirewireCamera(const CameraArgumentParser *cap)
{
  _started = _opened = false;
  _valid_frame_received = false;
  _auto_focus = true; // assume auto_focus, checked in open()
  _auto_shutter = false;
  _auto_white_balance = false;
  _white_balance_ub = 0xFFFFFFFF;
  _white_balance_vr = 0xFFFFFFFF;

  // Defaults
  _mode = DC1394_VIDEO_MODE_640x480_YUV422;
  _speed = DC1394_ISO_SPEED_400;
  _framerate = DC1394_FRAMERATE_30;
  _camera = NULL;
  _dc1394 = NULL;
  _format7_mode_enabled = false;
  _format7_width = _format7_height = _format7_startx = _format7_starty = 0;
  _format7_bpp = 4096;
  _model = strdup(cap->cam_id().c_str());

  if ( cap->has("mode") ) {
    string m = cap->get("mode");
    if ( m == "640x480_MONO16" ) {
      _mode = DC1394_VIDEO_MODE_640x480_MONO16;
    } else if ( m == "FORMAT7_0" ) {
      _mode = DC1394_VIDEO_MODE_FORMAT7_0;
      _format7_mode_enabled = true;
    } else if ( m == "FORMAT7_1" ) {
      _mode = DC1394_VIDEO_MODE_FORMAT7_1;
      _format7_mode_enabled = true;
    } else if ( m == "FORMAT7_2" ) {
      _mode = DC1394_VIDEO_MODE_FORMAT7_2;
      _format7_mode_enabled = true;
    } else if ( m == "FORMAT7_3" ) {
      _mode = DC1394_VIDEO_MODE_FORMAT7_3;
      _format7_mode_enabled = true;
    } else if ( m == "FORMAT7_4" ) {
      _mode = DC1394_VIDEO_MODE_FORMAT7_4;
      _format7_mode_enabled = true;
    } else if ( m == "FORMAT7_5" ) {
      _mode = DC1394_VIDEO_MODE_FORMAT7_5;
      _format7_mode_enabled = true;
    } else if ( m == "FORMAT7_6" ) {
      _mode = DC1394_VIDEO_MODE_FORMAT7_6;
      _format7_mode_enabled = true;
    } else if ( m == "FORMAT7_7" ) {
      _mode = DC1394_VIDEO_MODE_FORMAT7_7;
      _format7_mode_enabled = true;
    }
  }
  if ( cap->has("coding") ) {
    string c = cap->get("coding");
    if ( c == "YUV422" ) {
      _format7_coding = DC1394_COLOR_CODING_YUV422;
    } else if ( c == "MONO16" ) {
      _format7_coding = DC1394_COLOR_CODING_MONO16;
    } else if ( c == "RAW16" ) {
      _format7_coding = DC1394_COLOR_CODING_RAW16;
    }
  }
  if ( cap->has("isospeed") ) {
    string s = cap->get("isospeed");
    if ( s == "400" ) {
      _speed = DC1394_ISO_SPEED_400;
    } else if ( s == "800" ) {
      _speed = DC1394_ISO_SPEED_800;
    }
  }
  if ( cap->has("framerate") ) {
    string f = cap->get("framerate");
    if ( f == "15" ) {
      _framerate = DC1394_FRAMERATE_15;
    } else if ( f == "30" ) {
      _framerate = DC1394_FRAMERATE_30;
    } else if ( f == "60" ) {
      _framerate = DC1394_FRAMERATE_60;
    } else if ( f == "120" ) {
      _framerate = DC1394_FRAMERATE_120;
    }
  }
  if ( cap->has("nbufs") ) {
    _num_buffers = atoi(cap->get("nbufs").c_str());
  } else {
    _num_buffers = 4;
  }
  if ( cap->has("width") ) {
    _format7_width = atoi(cap->get("width").c_str());
  }
  if ( cap->has("height") ) {
    _format7_height = atoi(cap->get("height").c_str());
  }
  if ( cap->has("startx") ) {
    _format7_startx = atoi(cap->get("startx").c_str());
  }
  if ( cap->has("starty") ) {
    _format7_starty = atoi(cap->get("starty").c_str());
  }
  if ( cap->has("packetsize") ) {
    _format7_bpp = atoi(cap->get("packetsize").c_str());
  }
  if ( cap->has("white_balance") ) {
    string w = cap->get("white_balance");
    if ( w == "auto" ) {
      _auto_white_balance = true;
    } else {
      // try to parse U/V values
      string::size_type commapos = w.find(",", 0);
      if ( commapos == string::npos ) {
	throw Exception("Illegal white balance value, neither auto and no comma found");
      }
      string ub = w.substr(0, commapos);
      string vr = w.substr(commapos + 1);
      char *endptr;
      long int ub_i = strtol(ub.c_str(), &endptr, 10);
      if ( endptr[0] != 0 ) {
	throw TypeMismatchException("White balance value for U/B is invalid. "
				    "String to int conversion failed");
      } else if ( ub_i < 0 ) {
	throw OutOfBoundsException("White balance value for U/B < 0", ub_i, 0, 0xFFFFFFFF);
      }
      long int vr_i = strtol(vr.c_str(), &endptr, 10);
      if ( endptr[0] != 0 ) {
	throw TypeMismatchException("White balance value for V/R is invalid. "
				    "String to int conversion failed");
      } else if ( vr_i < 0 ) {
	throw OutOfBoundsException("White balance value for V/R < 0", vr_i, 0, 0xFFFFFFFF);
      }

      _auto_white_balance = false;
      _white_balance_ub = ub_i;
      _white_balance_vr = vr_i;
    }
  }
  if ( cap->has("shutter") ) {
    string s = cap->get("shutter");
    if ( s == "auto" ) {
      _auto_shutter = true;
    }
  }
}


/** Print list of cameras.
 * Prints a list of available cameras to stdout.
 */
void
FirewireCamera::print_available_fwcams()
{

  dc1394_t *dc1394 = dc1394_new();
  dc1394camera_list_t *list;
  dc1394error_t        err;
  if ( (err = dc1394_camera_enumerate(dc1394, &list)) != DC1394_SUCCESS ) {
    throw Exception("Could not enumerate cameras: %s", dc1394_error_get_string(err));
  }

  if (list->num > 0) {
    for (unsigned int i = 0; i < list->num; ++i) {
      dc1394camera_t *tmpcam = dc1394_camera_new(dc1394, list->ids[i].guid);
      dc1394_camera_print_info(tmpcam, stdout);
      dc1394_camera_free(tmpcam);
    }
  } else {
    printf("Could not find any cameras\n");
  }
}

