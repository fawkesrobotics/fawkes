 
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

#include <cams/swissranger.h>
#include <cams/cam_exceptions.h>
#include <fvutils/system/camargp.h>

#include <cstdlib>
#include <unistd.h>
#include <climits>
#include <cstring>
#include <cstdio>
#include <regex.h>

#include <usb.h>
#include <libMesaSR.h>

using namespace std;
using namespace fawkes;

/** @class SwissRangerCamera <cams/swissranger.h>
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

  __gray_buffer = (unsigned char *)malloc(__width * __height);

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
    free(__gray_buffer);
    __gray_buffer = NULL;
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

  __buffer = (unsigned short *)SR_GetImage(__cam, 1);
  // convert image
  for (unsigned int h = 0; h < __height; ++h) {
    for (unsigned int w = 0; w < __width; ++w) {
      __gray_buffer[h * __width + w] = __buffer[h * __width + w] / 2;
    }
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
    return __gray_buffer;
  } else {
    return NULL;
  }
}


unsigned int
SwissRangerCamera::buffer_size()
{
  if ( _valid_frame_received ) {
    return 0; // FIXME
  } else {
    return 0;
  }
}

void
SwissRangerCamera::dispose_buffer()
{
  if ( _valid_frame_received ) {
  }
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
  return GRAY8;
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
 * - focus=MODE, MODE is either auto for auto focus, manual for manual focus without
 *               actually setting (for example set from external application) or a
 *               number for the focus.
 * @param cap camera argument parser
 */
SwissRangerCamera::SwissRangerCamera(const CameraArgumentParser *cap)
{
  _started = _opened = false;
  _valid_frame_received = false;

  __model = __vendor = NULL;
  __vendor_id = __product_id = 0;

  __buffer = NULL;
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

