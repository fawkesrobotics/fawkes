 
/***************************************************************************
 *  swissranger.cpp - Implementation to access SwissRanger SR4000 camera
 *
 *  Created: Wed Jan 13 17:02:39 2010
 *  Copyright  2005-2010  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <utils/system/console_colors.h>

#include <fvcams/swissranger.h>
#include <fvcams/cam_exceptions.h>
#include <fvutils/system/camargp.h>

#include <cstdlib>
#include <unistd.h>
#include <climits>
#include <string>
#include <cstring>
#include <cstdio>
#include <regex.h>

#include <usb.h>
#include <libMesaSR.h>

using namespace std;
using namespace fawkes;

namespace firevision {

/** @class SwissRangerCamera <fvcams/swissranger.h>
 * SwissRanger camera.
 * This camera implementation provides access to the SwissRanger SR4000 camera
 * (and probably other models supported by libmesasr, but cannot test).
 * @author Tim Niemueller
 */


/** Empty destructor. */
SwissRangerCamera::~SwissRangerCamera()
{
  close();
  
  if ( model_ != NULL ) {
    free(model_);
  }
}


void
SwissRangerCamera::open()
{
  if (_opened) return;

  usb_set_debug(0);

  int num_cams = SR_OpenUSB(&cam_, 0);
  if (num_cams <= 0) {
    throw Exception("Cannot find SwissRanger camera");
  }

  // turn off debugging after open, it already sucks during open...
  usb_set_debug(0);

  char devstr[1024];
  SR_GetDeviceString(cam_, devstr, 1024);

  regmatch_t m[5];
  regex_t re;
  if ( (regcomp(&re, "VendorID:0x([a-fA-F0-9]{4}), ProductID:0x([a-fA-F0-9]{4}), "
		"Manufacturer:'([^' ]+) *', Product:'([^' ]+) *'",
		REG_EXTENDED) != 0) ||
       (regexec(&re, devstr, 5, m, 0) != 0) ) {
    SR_Close(cam_);
    throw Exception("Could not parse device string");
  }
  char *tmp;
  tmp = strndup(&(devstr[m[1].rm_so]), m[1].rm_eo - m[3].rm_so);
  vendor_id_  = strtol(tmp, NULL, 16);
  free(tmp);
  tmp = strndup(&(devstr[m[2].rm_so]), m[2].rm_eo - m[3].rm_so);
  product_id_  = strtol(tmp, NULL, 16);
  free(tmp);
  vendor_     = strndup(&(devstr[m[3].rm_so]), m[3].rm_eo - m[3].rm_so);
  model_      = strndup(&(devstr[m[4].rm_so]), m[4].rm_eo - m[4].rm_so);
  regfree(&re);

  serial_ = SR_ReadSerial(cam_);

  width_  = SR_GetCols(cam_);
  height_ = SR_GetRows(cam_);


  int acqm = AM_COR_FIX_PTRN;
  if ( (mode_ == AMPLITUDE_GRAY) || (mode_ == AMPLITUDE_GRAY_8) ) {
    acqm |= AM_CONV_GRAY;
  } else if (mode_ == CONF_MAP) {
    acqm |= AM_CONF_MAP;
  }
  if (use_median_) {
    acqm |= AM_MEDIAN;
  }
  if (use_denoise_) {
    acqm |= AM_DENOISE_ANF;
  }
  SR_SetMode(cam_, acqm);

  if (integration_time_ > 0) {
    SR_SetIntegrationTime(cam_, integration_time_);
  }

  SR_SetAmplitudeThreshold(cam_, amplitude_threshold_);

  if (set_modfreq_) {
    SR_SetModulationFrequency(cam_, modulation_freq_);
  }

  buffer_size_      = width_ * height_;
  gray_buffer_      = NULL;
  coord_uint16_buf_ = NULL;
  coord_float_buf_  = NULL;
  coord_double_buf_ = NULL;
  if ( (mode_ == AMPLITUDE_GRAY_8) || (mode_ == DISTANCE_GRAY_8) ) {
    gray_buffer_ = (unsigned char *)malloc(width_ * height_);
    buffer_ = gray_buffer_;
  } else if (mode_ == CARTESIAN_UINT16) {
    buffer_size_ = 3 * width_ * height_ * sizeof(unsigned short);
    coord_uint16_buf_ = (unsigned short *)malloc(buffer_size_);
    xu_ = (short *)coord_uint16_buf_;
    yu_ = &(xu_[width_ * height_]);
    zu_ = (unsigned short *)&(yu_[width_ * height_]);
    buffer_ = (unsigned char *)coord_uint16_buf_;
  } else if (mode_ == CARTESIAN_FLOAT) {
    buffer_size_ = 3 * width_ * height_ * sizeof(float);
    coord_float_buf_ = (float *)malloc(buffer_size_);
    xf_ = coord_float_buf_;
    yf_ = &(coord_float_buf_[    width_ * height_]);
    zf_ = &(coord_float_buf_[2 * width_ * height_]);
    buffer_ = (unsigned char *)coord_float_buf_;
  } else if (mode_ == CARTESIAN_FLOAT) {
    buffer_size_ = 3 * width_ * height_ * sizeof(double);
    coord_double_buf_ = (double *)malloc(buffer_size_);
    xd_ = coord_double_buf_;
    yd_ = &(coord_double_buf_[    width_ * height_]);
    zd_ = &(coord_double_buf_[2 * width_ * height_]);
    buffer_ = (unsigned char *)coord_double_buf_;
  }

  _opened = true;
}


void
SwissRangerCamera::start()
{
  if (_started) return;

  if (! _opened) {
    throw Exception("SwissRangerCamera: Cannot start closed camera");
  }

  _started = true;
}


void
SwissRangerCamera::stop()
{
  _started = false;
}


void
SwissRangerCamera::print_info()
{
  printf("Vendor: %-20s (0x%04x)\n"
	 "Model:  %-20s (0x%04x)\n"
	 "Serial: %x\n",
	 vendor_, vendor_id_, model_, product_id_, serial_);
}


void
SwissRangerCamera::close()
{
  if ( _started ) stop();
  if ( _opened ) {
    SR_Close(cam_);
    if (gray_buffer_) {
      free(gray_buffer_);
      gray_buffer_ = NULL;
    }
    if (coord_uint16_buf_) {
      free(coord_uint16_buf_);
      coord_uint16_buf_ = NULL;
    }
    if (coord_float_buf_) {
      free(coord_float_buf_);
      coord_float_buf_ = NULL;
    }
    if (coord_double_buf_) {
      free(coord_double_buf_);
      coord_double_buf_ = NULL;
    }
    _opened = false;
  }
}


/** Get camera model.
 * @return string with the camera model name
 */
const char *
SwissRangerCamera::model() const
{
  if ( ! _opened ) {
    throw Exception("Camera not opened");
  }

  return model_;
}


void
SwissRangerCamera::capture()
{

  if (! _opened) {
    throw CaptureException("SwissRangerCamera(%s): cannot capture on closed camera", model_);
  }
  if (! _started) {
    throw CaptureException("SwissRangerCamera(%s): cannot capture on stopped camera", model_);
  }

  _valid_frame_received = (SR_Acquire(cam_) > 0);
  if (!_valid_frame_received) {
    throw CaptureException("SwissRangerCamera(%s): failed to acquire image", model_);
  }

  if (mode_ == DISTANCE) {
    buffer_ = (unsigned char *)SR_GetImage(cam_, 0);
  } else if ( (mode_ == AMPLITUDE) || (mode_ == AMPLITUDE_GRAY) ) {
    buffer_ = (unsigned char *)SR_GetImage(cam_, 1);
  } else if ( (mode_ == DISTANCE_GRAY_8) || (mode_ == AMPLITUDE_GRAY_8) ) {
    unsigned int image_num = (mode_ == DISTANCE_GRAY_8) ? 0 : 1;
    unsigned short *buf = (unsigned short *)SR_GetImage(cam_, image_num);
    // convert image
    for (unsigned int h = 0; h < height_; ++h) {
      for (unsigned int w = 0; w < width_; ++w) {
	gray_buffer_[h * width_ + w] = buf[h * width_ + w] / 2;
      }
    }
  } else if (mode_ == CONF_MAP) {
    buffer_ = (unsigned char *)SR_GetImage(cam_, 2);
  } else if (mode_ == CARTESIAN_UINT16) {
    SR_CoordTrfUint16(cam_, xu_, yu_, zu_, 2, 2, 2);
  } else if (mode_ == CARTESIAN_FLOAT) {
    SR_CoordTrfFlt(cam_, xf_, yf_, zf_,
		   sizeof(float), sizeof(float), sizeof(float));
  } else if (mode_ == CARTESIAN_DOUBLE) {
    SR_CoordTrfDbl(cam_, xd_, yd_, zd_,
		   sizeof(double), sizeof(double), sizeof(double));
  }
}


void
SwissRangerCamera::flush()
{
}


unsigned char*
SwissRangerCamera::buffer()
{
  if ( _valid_frame_received ) {
    return buffer_;
  } else {
    return NULL;
  }
}


unsigned int
SwissRangerCamera::buffer_size()
{
  if ( _valid_frame_received ) {
    return buffer_size_;
  } else {
    return 0;
  }
}

void
SwissRangerCamera::dispose_buffer()
{
  _valid_frame_received = false;
}


unsigned int
SwissRangerCamera::pixel_width()
{
  if (_opened) {
    return width_;
  } else {
    throw Exception("Camera not opened");
  }
}


unsigned int
SwissRangerCamera::pixel_height()
{
  if (_opened) {
    return height_;
  } else {
    throw Exception("Camera not opened");
  }
}


colorspace_t
SwissRangerCamera::colorspace()
{
  switch (mode_) {
  case DISTANCE:
  case AMPLITUDE:
  case CONF_MAP:
  case CARTESIAN_UINT16:
    return RAW16;
  case AMPLITUDE_GRAY:
    return MONO16;
  case DISTANCE_GRAY_8:
  case AMPLITUDE_GRAY_8:
    return GRAY8;
  case CARTESIAN_FLOAT:
    return CARTESIAN_3D_FLOAT;
  case CARTESIAN_DOUBLE:
    return CARTESIAN_3D_DOUBLE;
  }

  return RAW16;
}


bool
SwissRangerCamera::ready()
{
  return _started;
}


void
SwissRangerCamera::set_image_number(unsigned int n)
{
}

/** Constructor.
 * Initialize and take parameters from camera argument parser. The following
 * arguments are supported:
 * - mode=MODE where MODE is one of
 * - DISTANCE
 * - DISTANCE_GRAY_8
 * - AMPLITUDE
 * - AMPLITUDE_GRAY
 * - AMPLITUDE_GRAY_8
 * - CONF_MAP
 * - CARTESIAN_UINT16
 * - CARTESIAN_FLOAT
 * - CARTESIAN_DOUBLE
 * - median=on (enable median filter)
 * - denoise=on (enable denoise filter)
 * - modfreq=MODFREQ where MODFREQ (modulation frequency) is one of
 *   - 40MHz
 *   - 30MHz
 *   - 21MHz
 *   - 20MHz
 *   - 19MHz
 *   - 60MHz
 *   - 15MHz
 *   - 10MHz
 *   - 29MHz
 *   - 31MHz
 *   - 14.5MHz
 *   - 15.5MHz
 * - integration_time=NUM
 *   integration time, confer camera's API documentation
 * - amplitude_threshold=NUM
 *   amplitude threshold, must be unsigned 16 bit value
 * @param cap camera argument parser
 */
SwissRangerCamera::SwissRangerCamera(const CameraArgumentParser *cap)
{
  _started = _opened = false;
  _valid_frame_received = false;

  model_ = vendor_ = NULL;
  vendor_id_ = product_id_ = 0;

  buffer_ = NULL;

  mode_ = AMPLITUDE_GRAY_8;
  if (cap->has("mode")) {
    string m = cap->get("mode");
    if (m == "DISTANCE") {
      mode_ = DISTANCE;
    } else if (m == "DISTANCE_GRAY_8") {
      mode_ = DISTANCE_GRAY_8;
    } else if (m == "AMPLITUDE") {
      mode_ = AMPLITUDE;
    } else if (m == "AMPLITUDE_GRAY") {
      mode_ = AMPLITUDE_GRAY;
    } else if (m == "AMPLITUDE_GRAY_8") {
      mode_ = AMPLITUDE_GRAY_8;
    } else if (m == "CONF_MAP") {
      mode_ = CONF_MAP;
    } else if (m == "CARTESIAN_UINT16") {
      mode_ = CARTESIAN_UINT16;
    } else if (m == "CARTESIAN_FLOAT") {
      mode_ = CARTESIAN_FLOAT;
    } else if (m == "CARTESIAN_DOUBLE") {
      mode_ = CARTESIAN_DOUBLE;
    } else {
      throw Exception("Unknown mode %s given", m.c_str());
    }
  }

  use_median_ = false;
  if (cap->get("median") == "on") {
    use_median_=true;
  }

  use_denoise_ = false;
  if (cap->get("denoise") == "on") {
    use_denoise_=true;
  }

  integration_time_ = 0; // do not set
  if (cap->has("integration_time")) {
    integration_time_ = cap->get_int("integration_time");
  }

  amplitude_threshold_ = 0;
  if (cap->has("amplitude_threshold")) {
    amplitude_threshold_ = cap->get_int("amplitude_threshold");
  }

  set_modfreq_ = false;
  modulation_freq_ = MF_40MHz;
  if (cap->has("modfreq")) {
    string m = cap->get("modfreq");
    set_modfreq_ = true;
    if (m == "40MHz") {
      modulation_freq_ = MF_40MHz;
    } else if (m == "30MHz") {
      modulation_freq_ = MF_30MHz;
    } else if (m == "21MHz") {
      modulation_freq_ = MF_21MHz;
    } else if (m == "20MHz") {
      modulation_freq_ = MF_20MHz;
    } else if (m == "19MHz") {
      modulation_freq_ = MF_19MHz;
    } else if (m == "60MHz") {
      modulation_freq_ = MF_60MHz;
    } else if (m == "15MHz") {
      modulation_freq_ = MF_15MHz;
    } else if (m == "10MHz") {
      modulation_freq_ = MF_10MHz;
    } else if (m == "29MHz") {
      modulation_freq_ = MF_29MHz;
    } else if (m == "31MHz") {
      modulation_freq_ = MF_31MHz;
    } else if (m == "14.5MHz") {
      modulation_freq_ = MF_14_5MHz;
    } else if (m == "15.5MHz") {
      modulation_freq_ = MF_15_5MHz;
    } else {
      throw Exception("Unknown modulation frequency %s given", m.c_str());
    }
  }

}


/** Print list of cameras.
 * Prints a list of available cameras to stdout.
 */
void
SwissRangerCamera::print_available_cams()
{
  SRCAM cams[16];
  //SR_SetCallback(sr_callback);
  int num_cams = SR_OpenAll(cams, 16, 0, 0xFFFFFFFF);
  if (num_cams < 0) {
    printf("Error opening SwissRanger cameras\n");
  } else if (num_cams == 0) {
    printf("No SwissRanger camera found\n");
  } else {
    for (int i = 0; i < 1; ++i) {
      char devstr[1024];
      SR_GetDeviceString(cams[i], devstr, 1024);
      unsigned int serial = SR_ReadSerial(cams[i]);
      printf("%s, Serial:'%x'\n", devstr, serial);
      SR_Close(cams[i]);
    }
  }
}

} // end namespace firevision

