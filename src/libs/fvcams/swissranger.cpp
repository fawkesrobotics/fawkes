 
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
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

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
  
  if ( __model != NULL ) {
    free(__model);
  }
}


void
SwissRangerCamera::open()
{
  if (_opened) return;

  usb_set_debug(0);

  int num_cams = SR_OpenUSB(&__cam, 0);
  if (num_cams <= 0) {
    throw Exception("Cannot find SwissRanger camera");
  }

  // turn off debugging after open, it already sucks during open...
  usb_set_debug(0);

  char devstr[1024];
  SR_GetDeviceString(__cam, devstr, 1024);

  regmatch_t m[5];
  regex_t re;
  if ( (regcomp(&re, "VendorID:0x([a-fA-F0-9]{4}), ProductID:0x([a-fA-F0-9]{4}), "
		"Manufacturer:'([^' ]+) *', Product:'([^' ]+) *'",
		REG_EXTENDED) != 0) ||
       (regexec(&re, devstr, 5, m, 0) != 0) ) {
    SR_Close(__cam);
    throw Exception("Could not parse device string");
  }
  char *tmp;
  tmp = strndup(&(devstr[m[1].rm_so]), m[1].rm_eo - m[3].rm_so);
  __vendor_id  = strtol(tmp, NULL, 16);
  free(tmp);
  tmp = strndup(&(devstr[m[2].rm_so]), m[2].rm_eo - m[3].rm_so);
  __product_id  = strtol(tmp, NULL, 16);
  free(tmp);
  __vendor     = strndup(&(devstr[m[3].rm_so]), m[3].rm_eo - m[3].rm_so);
  __model      = strndup(&(devstr[m[4].rm_so]), m[4].rm_eo - m[4].rm_so);
  regfree(&re);

  __serial = SR_ReadSerial(__cam);

  __width  = SR_GetCols(__cam);
  __height = SR_GetRows(__cam);


  int acqm = AM_COR_FIX_PTRN;
  if ( (__mode == AMPLITUDE_GRAY) || (__mode == AMPLITUDE_GRAY_8) ) {
    acqm |= AM_CONV_GRAY;
  } else if (__mode == CONF_MAP) {
    acqm |= AM_CONF_MAP;
  }
  if (__use_median) {
    acqm |= AM_MEDIAN;
  }
  if (__use_denoise) {
    acqm |= AM_DENOISE_ANF;
  }
  SR_SetMode(__cam, acqm);

  if (__integration_time > 0) {
    SR_SetIntegrationTime(__cam, __integration_time);
  }

  SR_SetAmplitudeThreshold(__cam, __amplitude_threshold);

  if (__set_modfreq) {
    SR_SetModulationFrequency(__cam, __modulation_freq);
  }

  __buffer_size      = __width * __height;
  __gray_buffer      = NULL;
  __coord_uint16_buf = NULL;
  __coord_float_buf  = NULL;
  __coord_double_buf = NULL;
  if ( (__mode == AMPLITUDE_GRAY_8) || (__mode == DISTANCE_GRAY_8) ) {
    __gray_buffer = (unsigned char *)malloc(__width * __height);
    __buffer = __gray_buffer;
  } else if (__mode == CARTESIAN_UINT16) {
    __buffer_size = 3 * __width * __height * sizeof(unsigned short);
    __coord_uint16_buf = (unsigned short *)malloc(__buffer_size);
    __xu = (short *)__coord_uint16_buf;
    __yu = &(__xu[__width * __height]);
    __zu = (unsigned short *)&(__yu[__width * __height]);
    __buffer = (unsigned char *)__coord_uint16_buf;
  } else if (__mode == CARTESIAN_FLOAT) {
    __buffer_size = 3 * __width * __height * sizeof(float);
    __coord_float_buf = (float *)malloc(__buffer_size);
    __xf = __coord_float_buf;
    __yf = &(__coord_float_buf[    __width * __height]);
    __zf = &(__coord_float_buf[2 * __width * __height]);
    __buffer = (unsigned char *)__coord_float_buf;
  } else if (__mode == CARTESIAN_FLOAT) {
    __buffer_size = 3 * __width * __height * sizeof(double);
    __coord_double_buf = (double *)malloc(__buffer_size);
    __xd = __coord_double_buf;
    __yd = &(__coord_double_buf[    __width * __height]);
    __zd = &(__coord_double_buf[2 * __width * __height]);
    __buffer = (unsigned char *)__coord_double_buf;
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
	 __vendor, __vendor_id, __model, __product_id, __serial);
}


void
SwissRangerCamera::close()
{
  if ( _started ) stop();
  if ( _opened ) {
    SR_Close(__cam);
    if (__gray_buffer) {
      free(__gray_buffer);
      __gray_buffer = NULL;
    }
    if (__coord_uint16_buf) {
      free(__coord_uint16_buf);
      __coord_uint16_buf = NULL;
    }
    if (__coord_float_buf) {
      free(__coord_float_buf);
      __coord_float_buf = NULL;
    }
    if (__coord_double_buf) {
      free(__coord_double_buf);
      __coord_double_buf = NULL;
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

  return __model;
}


void
SwissRangerCamera::capture()
{

  if (! _opened) {
    throw CaptureException("SwissRangerCamera(%s): cannot capture on closed camera", __model);
  }
  if (! _started) {
    throw CaptureException("SwissRangerCamera(%s): cannot capture on stopped camera", __model);
  }

  _valid_frame_received = (SR_Acquire(__cam) > 0);
  if (!_valid_frame_received) {
    throw CaptureException("SwissRangerCamera(%s): failed to acquire image", __model);
  }

  if (__mode == DISTANCE) {
    __buffer = (unsigned char *)SR_GetImage(__cam, 0);
  } else if ( (__mode == AMPLITUDE) || (__mode == AMPLITUDE_GRAY) ) {
    __buffer = (unsigned char *)SR_GetImage(__cam, 1);
  } else if ( (__mode == DISTANCE_GRAY_8) || (__mode == AMPLITUDE_GRAY_8) ) {
    unsigned int image_num = (__mode == DISTANCE_GRAY_8) ? 0 : 1;
    unsigned short *buf = (unsigned short *)SR_GetImage(__cam, image_num);
    // convert image
    for (unsigned int h = 0; h < __height; ++h) {
      for (unsigned int w = 0; w < __width; ++w) {
	__gray_buffer[h * __width + w] = buf[h * __width + w] / 2;
      }
    }
  } else if (__mode == CONF_MAP) {
    __buffer = (unsigned char *)SR_GetImage(__cam, 2);
  } else if (__mode == CARTESIAN_UINT16) {
    SR_CoordTrfUint16(__cam, __xu, __yu, __zu, 2, 2, 2);
  } else if (__mode == CARTESIAN_FLOAT) {
    SR_CoordTrfFlt(__cam, __xf, __yf, __zf,
		   sizeof(float), sizeof(float), sizeof(float));
  } else if (__mode == CARTESIAN_DOUBLE) {
    SR_CoordTrfDbl(__cam, __xd, __yd, __zd,
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
    return __buffer;
  } else {
    return NULL;
  }
}


unsigned int
SwissRangerCamera::buffer_size()
{
  if ( _valid_frame_received ) {
    return __buffer_size;
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
    return __width;
  } else {
    throw Exception("Camera not opened");
  }
}


unsigned int
SwissRangerCamera::pixel_height()
{
  if (_opened) {
    return __height;
  } else {
    throw Exception("Camera not opened");
  }
}


colorspace_t
SwissRangerCamera::colorspace()
{
  switch (__mode) {
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

  __model = __vendor = NULL;
  __vendor_id = __product_id = 0;

  __buffer = NULL;

  __mode = AMPLITUDE_GRAY_8;
  if (cap->has("mode")) {
    string m = cap->get("mode");
    if (m == "DISTANCE") {
      __mode = DISTANCE;
    } else if (m == "DISTANCE_GRAY_8") {
      __mode = DISTANCE_GRAY_8;
    } else if (m == "AMPLITUDE") {
      __mode = AMPLITUDE;
    } else if (m == "AMPLITUDE_GRAY") {
      __mode = AMPLITUDE_GRAY;
    } else if (m == "AMPLITUDE_GRAY_8") {
      __mode = AMPLITUDE_GRAY_8;
    } else if (m == "CONF_MAP") {
      __mode = CONF_MAP;
    } else if (m == "CARTESIAN_UINT16") {
      __mode = CARTESIAN_UINT16;
    } else if (m == "CARTESIAN_FLOAT") {
      __mode = CARTESIAN_FLOAT;
    } else if (m == "CARTESIAN_DOUBLE") {
      __mode = CARTESIAN_DOUBLE;
    } else {
      throw Exception("Unknown mode %s given", m.c_str());
    }
  }

  __use_median = false;
  if (cap->get("median") == "on") {
    __use_median=true;
  }

  __use_denoise = false;
  if (cap->get("denoise") == "on") {
    __use_denoise=true;
  }

  __integration_time = 0; // do not set
  if (cap->has("integration_time")) {
    __integration_time = cap->get_int("integration_time");
  }

  __amplitude_threshold = 0;
  if (cap->has("amplitude_threshold")) {
    __amplitude_threshold = cap->get_int("amplitude_threshold");
  }

  __set_modfreq = false;
  __modulation_freq = MF_40MHz;
  if (cap->has("modfreq")) {
    string m = cap->get("modfreq");
    __set_modfreq = true;
    if (m == "40MHz") {
      __modulation_freq = MF_40MHz;
    } else if (m == "30MHz") {
      __modulation_freq = MF_30MHz;
    } else if (m == "21MHz") {
      __modulation_freq = MF_21MHz;
    } else if (m == "20MHz") {
      __modulation_freq = MF_20MHz;
    } else if (m == "19MHz") {
      __modulation_freq = MF_19MHz;
    } else if (m == "60MHz") {
      __modulation_freq = MF_60MHz;
    } else if (m == "15MHz") {
      __modulation_freq = MF_15MHz;
    } else if (m == "10MHz") {
      __modulation_freq = MF_10MHz;
    } else if (m == "29MHz") {
      __modulation_freq = MF_29MHz;
    } else if (m == "31MHz") {
      __modulation_freq = MF_31MHz;
    } else if (m == "14.5MHz") {
      __modulation_freq = MF_14_5MHz;
    } else if (m == "15.5MHz") {
      __modulation_freq = MF_15_5MHz;
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

