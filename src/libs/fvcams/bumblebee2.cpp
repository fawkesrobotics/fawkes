
/***************************************************************************
 *  bumblebee2.cpp - Point Grey Bumblebee 2 camera
 *
 *  Generated: Sat Apr 14 20:51:19 2007 (watching Ghostbusters)
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvcams/bumblebee2.h>

#include <fvcams/cam_exceptions.h>
#include <core/exception.h>
#include <fvutils/system/camargp.h>
#include <fvutils/color/conversions.h>
// include <fvutils/writers/pnm.h>

#include <stdlib.h>
#include <unistd.h>
#include <string>
#ifdef __FreeBSD__
#  include <sys/endian.h>
#elif defined(__MACH__) && defined(__APPLE__)
#  include <sys/_endian.h>
#else
#  include <endian.h>
#endif

#include <utils/math/angle.h>

#include <cstdio>

#include <dc1394/utils.h>
#include <dc1394/register.h>
#include <dc1394/conversions.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Bumblebee2Camera <fvcams/bumblebee2.h>
 * Bumblebee2 camera.
 * Camera implementation that allows fo access to the PointGrey Research Bumblebee2
 * camera. It uses libdc1394 to access the camera for fast image transfers (as recommended
 * by PTGrey) and can be used in conjunction with the TriclopsStereoProcessor in the
 * stereo utilities library.
 *
 * and the Triclops SDK by PTGrey for calculation of the stereo image.
 * This implementation is based on the Firewire implementation and extends it. The
 * capture() method implicitly does all the stereo processing needed. This cannot
 * be turned off. The video modes is appropriately configured for the camera. You can
 * get access to the left and right images where necessary using the set_image_number()
 * method and the constants LEFT_ORIGINAL and RIGHT_ORIGINAL. The disparity image buffer
 * can be retrieved via buffer_disparity().
 *
 * Currently only the low resolution version (640x480) of the Bumblebee2 is supported,
 * an extension for the hires version may follow if we get one of these cameras.
 *
 * This class also encapsulates a coordinate system transformation that you can use to
 * transform the coordinates from the camera system to another right-handed system like
 * the robot system.
 *
 * The camera coordinate system has the X-axis pointing to the right,
 * Y-axis to the floor and Z-axis forward, if the camera is placed parallel to the ground
 * and you look in the direction of the camera. The origin of the system is in the right
 * lens system of the Bumblebee.
 *
 * @author Tim Niemueller
 */


/** Original image in RAW16 */
const unsigned int Bumblebee2Camera::ORIGINAL = 0;

/** Deinterlaced image */
const unsigned int Bumblebee2Camera::DEINTERLACED = 1;

/** From bayer tile decoded RGB image */
const unsigned int Bumblebee2Camera::RGB_IMAGE = 2;


/// PGR specific registers
/** PTGrey proprietary register: Bayer tile mapping information */
#define PGR_BAYER_TILE_MAPPING_REGISTER (0x1040)

#define PGR_SENSOR_BOARD_INFO_REGISTER  (0x1f28)

/** PTGrey proprietary: config data length */
#define PGR_REG_CONFIG_LENGTH         	(0x1FFC) 

/** PTGrey proprietary register: config register */
#define PGR_REG_CONFIG_DATA           	(0x2000) 

/** PTGrey proprietary register: unit directory offset */
#define PGR_REG_UNIT_DIRECTORY_OFFSET   (0x0424)

/** PTGrey proprietary register: image data format */
#define PGR_REG_IMAGE_DATA_FORMAT       (0x1048)
/** PTGrey image data format: PGR-specific (little endian) mode */
#define PTG_Y16_Data_Format_PGR_specific  (0xFFFFFFFE)

/** PTGrey proprietary register: serial number */
#define PGR_REG_SERIAL_NUMBER           (0x1F20)

/** PTGrey image data format: PGR-specific (little endian) mode */
/** Constructor.
 * Initialize and take parameters from camera argument parser. The following
 * arguments are supported:
 * - nbufs=NBUFS, number of DMA buffers, integer, 0 < n <= 32
 * - width=WIDTH, width in pixels of Format7 ROI
 * - height=HEIGHT, height in pixels of Format7 ROI
 * - startx=STARTX, X start of Format7 ROI
 * - starty=STARTY, Y start of Format7 ROI
 * @param cap camera argument parser
 */
Bumblebee2Camera::Bumblebee2Camera(const CameraArgumentParser *cap)
  : FirewireCamera(DC1394_FRAMERATE_30,
		   DC1394_VIDEO_MODE_FORMAT7_3,
		   DC1394_ISO_SPEED_400,
		   /* num buffers */ 8)
{
  // Defaults

  _supports_color = true;
  _auto_acquire_sensor_info = false;

  _model = strdup(cap->cam_id().c_str());
  // num_buffers set in constructor call
  _format7_coding = DC1394_COLOR_CODING_RAW16;
  _format7_width  = 640;
  _format7_height = 480;
  _format7_startx = _format7_starty = 0;

  if ( cap->has("nbufs") ) {
    _num_buffers = atoi(cap->get("nbufs").c_str());
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
  if ( cap->has("focus") ) {
    parse_set_focus(cap->get("focus").c_str());
  }
  if ( cap->has("white_balance") ) {
    parse_set_white_balance(cap->get("white_balance").c_str());
  }
  if ( cap->has("shutter") ) {
    parse_set_shutter(cap->get("shutter").c_str());
  }

  __buffer_deinterlaced = NULL;
  __buffer_rgb = NULL;
}

/** Constructor.
 * Initialize and use largest possible video mode suitable for stereo
 * processing.
 */
Bumblebee2Camera::Bumblebee2Camera()
  : FirewireCamera(DC1394_FRAMERATE_30,
		   DC1394_VIDEO_MODE_FORMAT7_3,
		   DC1394_ISO_SPEED_400,
		   /* num buffers */ 8)
{
  _auto_acquire_sensor_info = true;

  _model = strdup("Bumblebee2");
  // num_buffers set in constructor call
  _format7_coding = DC1394_COLOR_CODING_RAW16;
  _format7_width  = 640;
  _format7_height = 480;
  _format7_startx = _format7_starty = 0;
}

/** Destructor. */
Bumblebee2Camera::~Bumblebee2Camera()
{
  if (__buffer_deinterlaced != NULL)  free(__buffer_deinterlaced);
  if (__buffer_rgb != NULL)           free(__buffer_rgb);
}


/** Get BB2 serial no.
 * @return BB2 serial number.
 */
uint32_t
Bumblebee2Camera::serial_no() const
{
  if ( ! _opened )  throw Exception("Camera not opened");

  uint32_t value = 0;
  dc1394error_t err = dc1394_get_control_register( _camera, PGR_REG_SERIAL_NUMBER, &value );
  if ( err != DC1394_SUCCESS ) {
    throw Exception("Bumblebee2::serial_no: dc1394_get_control_register(PGR_REG_SERIAL_NUMBER) failed\n");
  }
  return value;
}


/** Verify GUID validity.
 * Compares the given GUID with the GUID of the camera. The GUID may be of two
 * forms. If the first four bytes are all 0xFF then it is assumed that the
 * GUID was created from the BB2-specific serial number. For example if a
 * rectification LUT was generated with the context file only but without
 * access to the real camera. Otherwise the GUID is matched against the
 * Firewire GUID.
 * @param ver_guid GUID to verify
 * @return true if the given GUID matches the current camera, false otherwise
 */
bool
Bumblebee2Camera::verify_guid(uint64_t ver_guid) const
{
  if ( ! _opened )  throw Exception("Camera not opened");

  uint64_t tguid = ver_guid;
  tguid >>= 32;
  tguid &= 0xFFFFFFFF;
  if ( tguid == 0xFFFFFFFF ) {
    // serial number!
    ver_guid &= 0xFFFFFFFF;
    return (serial_no() == ver_guid);
  } else {
    return (guid() == ver_guid);
  }
}


void
Bumblebee2Camera::get_sensor_info()
{
  uint32_t value;
  dc1394error_t err;

  // This register is an advanced PGR register called SENSOR_BOARD_INFO
  err = dc1394_get_control_register(_camera, PGR_SENSOR_BOARD_INFO_REGISTER, &value );
  if ( err != DC1394_SUCCESS )
  {
    throw Exception("Failed to read sensor borad info register");
  }

  unsigned char ucSensorInfo = 0xf & value;

  switch( ucSensorInfo )
  {
  default:
    // unknown sensor!
    throw Exception("Illegal sensor board info detected!");
  case 0xA:	// color 640x480
    _supports_color	= true;
    _format7_height	= 480;
    _format7_width	= 640;
    break;
  case 0xB:	// mono 640x480
    _supports_color	= false;
    _format7_height	= 480;
    _format7_width	= 640;
    break;
  case 0xC:	// color 1024x768
    _supports_color	= true;
    _format7_height	= 768;
    _format7_width	= 1024;
    break;
  case 0xD:	// mono 1024x768
    _supports_color	= false;
    _format7_height	= 768;
    _format7_width	= 1024;
    break;
  case 0xE:	// color 1280x960
    _supports_color	= true;
    _format7_height	= 960;
    _format7_width	= 1280;
    break;
  case 0xF:	// mono 1280x960
    _supports_color	= false;
    _format7_height	= 960;
    _format7_width	= 1280;
    break;
  }
}

void
Bumblebee2Camera::print_info()
{
  FirewireCamera::print_info();

  printf("Serial: %u\n", serial_no());
#if (defined(__WORDSIZE) && __WORDSIZE == 64) || (defined(LONG_BIT) && LONG_BIT == 64)
  printf("GUID:   0x%016lx\n", (long unsigned int)guid());
#else
  printf("GUID:   0x%016llx\n", guid());
#endif
}


void
Bumblebee2Camera::open_device()
{
  _dc1394 = dc1394_new();
  dc1394camera_list_t *list;

  if ( dc1394_camera_enumerate(_dc1394, &list) != DC1394_SUCCESS ) {
    throw Exception("Could not enumerate cameras");
  }

  if (list->num > 0) {
    _camera = NULL;
    for (unsigned int i = 0; i < list->num; ++i) {
      dc1394camera_t *tmpcam = dc1394_camera_new(_dc1394, list->ids[i].guid);
      if ( strncmp("Bumblebee2", tmpcam->model, strlen("Bumblebee2")) == 0) {
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
  } else {
    throw Exception("No cameras connected");
  }

  _device_opened = true;
}

void
Bumblebee2Camera::open()
{
  if (_auto_acquire_sensor_info) {
    open_device();
    get_sensor_info();
  }
  FirewireCamera::open();

  if ( ! _opened ) {
    throw Exception("Bumblebee2Camera::open: FirewireCamera::open dit not suceed");
  }

  __buffer_deinterlaced = (unsigned char *)malloc(pixel_width() * pixel_height() * 2);
  __buffer_rgb = malloc_buffer(RGB, pixel_width(), pixel_height() * 2);
  __buffer = NULL;

#if __BYTE_ORDER == __LITTLE_ENDIAN
  dc1394error_t err;
  typedef union {
    uint32_t value;
    struct {
      uint32_t   presence   :  1;
      uint32_t   reserved1  : 21;
      uint32_t   mirror     :  1;
      uint32_t   bayer_mono :  1;
      uint32_t   reserved2  :  7;
      uint32_t   data_format:  1;
    } idf;
  } idf_u;
  idf_u value;
  err = dc1394_get_control_register( _camera, PGR_REG_IMAGE_DATA_FORMAT, &(value.value) );
  if ( err != DC1394_SUCCESS ) {
    throw Exception("Bumblebee2::open: dc1394_get_control_register(PGR_REG_DATA_FORMAT) failed\n");
  }
  value.value &= PTG_Y16_Data_Format_PGR_specific;
  value.idf.data_format = 0;
  err = dc1394_set_control_register( _camera, PGR_REG_IMAGE_DATA_FORMAT, value.value );
  if ( err != DC1394_SUCCESS ) {
    throw Exception("Bumblebee2::open: Setting PGR-specific mode on little-endian system failed\n");
  }
#endif

  get_bayer_tile();
}


void
Bumblebee2Camera::close()
{
  if ( _opened ) {
    FirewireCamera::close();  
    if (__buffer_deinterlaced != NULL) {
      free(__buffer_deinterlaced);
      __buffer_deinterlaced = NULL;
    }
    if (__buffer_rgb != NULL) {
      free(__buffer_rgb);
      __buffer_rgb = NULL;
    }
  }
}

void
Bumblebee2Camera::capture()
{
  try {
    FirewireCamera::capture();
  } catch (CaptureException &e) {
    e.append("Bumblebee2Camera::capture: failed to retrieve image");
    if ( ORIGINAL == __image_num )  __buffer = NULL;
    throw;
  }
  if ( ORIGINAL == __image_num ) {
    __buffer = _frame->image;
  }
}


unsigned char *
Bumblebee2Camera::buffer()
{
  return __buffer;
}


void
Bumblebee2Camera::set_image_number(unsigned int image_num)
{
  __image_num = image_num;
  switch ( image_num ) {
  case DEINTERLACED: __buffer = __buffer_deinterlaced; break;
  case RGB_IMAGE: __buffer = __buffer_rgb; break;
  default:  __buffer = NULL; break;
  }
}


/** Check if connected camera is a Bumblebee2.
 * @return true, if the connected camera is a Bumblebee2, false otherwise
 */
bool
Bumblebee2Camera::is_bumblebee2()
{
  if ( ! _opened ) throw CameraNotOpenedException();

  return( strncmp( _camera->model, "Bumblebee2", strlen("Bumblebee2") ) == 0);
}


/** De-interlace the 16 bit data into 2 bayer tile pattern images. */
void
Bumblebee2Camera::deinterlace_stereo()
{
  dc1394_deinterlace_stereo( _frame->image, __buffer_deinterlaced,
			     pixel_width(), 2 * pixel_height() ); 
}


/** Extract RGB color image from the bayer tile image.
 * This will transform the bayer tile image to an RGB image using the
 * nearest neighbour method.
 * Note: this will alias colors on the top and bottom rows
 */
void
Bumblebee2Camera::decode_bayer()
{
  dc1394_bayer_decoding_8bit( __buffer_deinterlaced, __buffer_rgb,
			      pixel_width(), 2 * pixel_height(), 
			      __bayer_pattern, DC1394_BAYER_METHOD_NEAREST ); 
}





/** De-interlace the 16 bit data into 2 bayer tile pattern images.
 * Can be used for offline de-interlacing.
 * @param raw16 In-buffer RAW16-encoded
 * @param deinterlaced upon return contains the deinterlaced image
 * @param width width of image in pixels
 * @param height height of image in pixels
 */
void
Bumblebee2Camera::deinterlace_stereo(unsigned char *raw16, unsigned char *deinterlaced,
				     unsigned int width, unsigned int height)
{
  dc1394_deinterlace_stereo( raw16, deinterlaced, width, 2 * height ); 
}


/** Extract RGB color image from the bayer tile image.
 * This will transform the bayer tile image to an RGB image using the
 * nearest neighbour method.
 * Note: this will alias colors on the top and bottom rows
 * @param deinterlaced in-buffer with deinterlaced image
 * @param rgb upon return contains RGB image
 * @param width width of image in pixels
 * @param height height of image in pixels
 * @param bayer_pattern bayer pattern, one of
 *  - 0x59595959 (YYYY, no pattern)
 *  - 0x52474742 (RGGB)
 *  - 0x47524247 (GRBG)
 *  - 0x42474752 (BGGR)
 * This depends on the used camera.
 */
void
Bumblebee2Camera::decode_bayer(unsigned char *deinterlaced, unsigned char *rgb,
			       unsigned int width, unsigned int height,
			       bayer_pattern_t bayer_pattern)
{
  dc1394color_filter_t dc_bayer_pattern;

  switch (bayer_pattern) {
  default:
  case BAYER_PATTERN_YYYY:
    dc_bayer_pattern = (dc1394color_filter_t) 0;
    break;
  case BAYER_PATTERN_RGGB:
    dc_bayer_pattern = DC1394_COLOR_FILTER_RGGB;
    break;
  case BAYER_PATTERN_GBRG:
    dc_bayer_pattern = DC1394_COLOR_FILTER_GBRG;
    break;
  case BAYER_PATTERN_GRBG:
    dc_bayer_pattern = DC1394_COLOR_FILTER_GRBG;
    break;
  case BAYER_PATTERN_BGGR:
    dc_bayer_pattern = DC1394_COLOR_FILTER_BGGR;
    break;
  }

  dc1394_bayer_decoding_8bit( deinterlaced, rgb, width, 2 * height, 
			      dc_bayer_pattern, DC1394_BAYER_METHOD_NEAREST ); 
}


/** Retrieve bayer tile.
 * This is an internal method that access a special PTGrey register in the camera to
 * determine the bayer tile mode.
 */
void
Bumblebee2Camera::get_bayer_tile()
{
  uint32_t value;
  if (dc1394_get_control_register( _camera, PGR_BAYER_TILE_MAPPING_REGISTER, &value) != DC1394_SUCCESS ) {
    throw Exception("Could not query bayer tile register");
  }

  // Magic numbers are specific to PTGrey cameras
  switch (value) {
  default:
  case 0x59595959:	// YYYY
    // no bayer
    __bayer_pattern = (dc1394color_filter_t) 0;
    break;
  case 0x52474742:	// RGGB
    __bayer_pattern = DC1394_COLOR_FILTER_RGGB;
    break;
  case 0x47425247:	// GBRG
    __bayer_pattern = DC1394_COLOR_FILTER_GBRG;
    break;
  case 0x47524247:	// GRBG
    __bayer_pattern = DC1394_COLOR_FILTER_GRBG;
    break;
  case 0x42474752:	// BGGR
    __bayer_pattern = DC1394_COLOR_FILTER_BGGR;
    break;
  }
}


/** Retrieve config from camera.
 * This method retrieves the config from the camera and writes it to a file such that
 * the Triclops SDK can use it for context initialization.
 * @param filename filename to write the config to
 * @exception Exception thrown if there is an error when trying to retrieve the config
 * or writing it to a file.
 */
void
Bumblebee2Camera::write_triclops_config_from_camera_to_file(const char *filename)
{
  dc1394error_t err;
  uint32_t value;
  
  err = dc1394_get_control_register( _camera, PGR_REG_CONFIG_LENGTH, &value );
  if ( err != DC1394_SUCCESS ) {
    throw Exception("dc1394_get_control_register(PGR_REG_CONFIG_LENGTH) failed\n");
  }
   
  // the length of the config file
  unsigned long file_size_bytes = value;
  if( file_size_bytes == 0 ) {
    throw Exception("File size == 0!\n" );
  }
   
  FILE* file = fopen( filename, "w" );
  if ( !file ) {
    throw Exception("Can't open temporary file\n" );
  }

  // Read the config file, and save it to the output file,
  // while fixing endianness.
  for( unsigned long offset = 0 ; offset < file_size_bytes; offset += 4 ) {
    err = dc1394_get_control_register( _camera,
				       PGR_REG_CONFIG_DATA + offset, 
				       &value );
     
    if( err != DC1394_SUCCESS ) {
      Exception e("Failed to get control register");
      e.append("Can't get control register 0x%x\n",
	       (int) (PGR_REG_CONFIG_DATA+offset) );
      fclose( file );
      throw e;
    }
    
    for( int i = 24; i >= 0; i -= 8 ) {
      fputc( ( (value>>i) & 0xFF ), file );
    }
  }
  fclose( file );  
}

} // end namespace firevision
