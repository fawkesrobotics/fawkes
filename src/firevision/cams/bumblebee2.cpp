
/***************************************************************************
 *  bumblebee2.cpp - Point Grey Bumblebee 2 camera
 *
 *  Generated: Sat Apr 14 20:51:19 2007 (watching Ghostbusters)
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

#include <cams/bumblebee2.h>

#include <cams/cam_exceptions.h>
#include <core/exception.h>
#include <fvutils/system/camargp.h>
#include <fvutils/color/conversions.h>
// include <fvutils/writers/pnm.h>

#include <stdlib.h>
#include <unistd.h>
#include <string>

#include <utils/math/angle.h>

#include <iostream>

#include <dc1394/utils.h>
#include <dc1394/register.h>
#include <dc1394/conversions.h>

using namespace std;


/** @class Bumblebee2Camera <cams/bumblebee2.h>
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

/** PTGrey proprietary: config data length */
#define PGR_REG_CONFIG_LENGTH         	(0x1FFC) 

/** PTGrey proprietary register: config register */
#define PGR_REG_CONFIG_DATA           	(0x2000) 

/** PTGrey proprietary register: unit directory offset */
#define PGR_REG_UNIT_DIRECTORY_OFFSET   (0x0424)

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
  : FirewireCamera(cap)
{
  // Defaults
  // num_buffers set in constructor call
  format7_coding = DC1394_COLOR_CODING_RAW16;
  format7_width  = 640;
  format7_height = 480;
  format7_startx = format7_starty = 0;

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

  _buffer_deinterlaced = (unsigned char *)malloc(pixel_width() * pixel_height() * 2);
  _buffer_rgb = malloc_buffer(RGB, pixel_width(), pixel_height() * 2);
  _buffer = NULL;
}


/** Destructor. */
Bumblebee2Camera::~Bumblebee2Camera()
{
  free(_buffer_deinterlaced);
  free(_buffer_rgb);
}


void
Bumblebee2Camera::open()
{
  try {
    FirewireCamera::open();
  } catch (Exception &e) {
    e.printTrace();
    throw;
  }

  if ( ! opened ) {
    throw Exception("Bumblebee2Camera::open: FirewireCamera::open dit not suceed");
  }

  get_bayer_tile();
}


colorspace_t
Bumblebee2Camera::colorspace()
{
  return YUV422_PLANAR;
}


void
Bumblebee2Camera::capture()
{
  try {
    FirewireCamera::capture();
  } catch (CaptureException &e) {
    e.append("Bumblebee2Camera::capture: failed to retrieve image");
    if ( ORIGINAL == _image_num )  _buffer = NULL;
    throw;
  }
  if ( ORIGINAL == _image_num ) {
    _buffer = frame->image;
  }
}


unsigned char *
Bumblebee2Camera::buffer()
{
  return _buffer;
}


void
Bumblebee2Camera::set_image_number(unsigned int image_num)
{
  _image_num = image_num;
  switch ( image_num ) {
  case DEINTERLACED: _buffer = _buffer_deinterlaced; break;
  case RGB_IMAGE: _buffer = _buffer_rgb;
  default:  _buffer = NULL; break;
  }
}


/** Check if connected camera is a Bumblebee2.
 * @return true, if the connected camera is a Bumblebee2, false otherwise
 */
bool
Bumblebee2Camera::is_bumblebee2()
{
  if ( ! opened ) throw CameraNotOpenedException();

  return( strncmp( camera->model, "Bumblebee2", strlen("Bumblebee2") ) == 0);
}


/** De-interlace the 16 bit data into 2 bayer tile pattern images. */
void
Bumblebee2Camera::deinterlace_stereo()
{
  dc1394_deinterlace_stereo( frame->image, _buffer_deinterlaced,
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
  dc1394_bayer_decoding_8bit( _buffer_deinterlaced, _buffer_rgb,
			      pixel_width(), 2 * pixel_height(), 
			      bayer_pattern, DC1394_BAYER_METHOD_NEAREST ); 
}


/** Retrieve bayer tile.
 * This is an internal method that access a special PTGrey register in the camera to
 * determine the bayer tile mode.
 */
void
Bumblebee2Camera::get_bayer_tile()
{
  uint32_t value;
  if (GetCameraControlRegister( camera, PGR_BAYER_TILE_MAPPING_REGISTER, &value) != DC1394_SUCCESS ) {
    throw Exception("Could not query bayer tile register");
  }

  // Magic numbers are specific to PTGrey cameras
  switch (value) {
  default:
  case 0x59595959:	// YYYY
    // no bayer
    bayer_pattern = (dc1394color_filter_t) 0;
    break;
  case 0x52474742:	// RGGB
    bayer_pattern = DC1394_COLOR_FILTER_RGGB;
    break;
  case 0x47425247:	// GBRG
    bayer_pattern = DC1394_COLOR_FILTER_GBRG;
    break;
  case 0x47524247:	// GRBG
    bayer_pattern = DC1394_COLOR_FILTER_GRBG;
    break;
  case 0x42474752:	// BGGR
    bayer_pattern = DC1394_COLOR_FILTER_BGGR;
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
  
  err = GetCameraControlRegister( camera, PGR_REG_CONFIG_LENGTH, &value );
  if ( err != DC1394_SUCCESS ) {
    throw Exception("GetCameraControlRegister(PGR_REG_CONFIG_LENGTH) failed\n");
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
    err = GetCameraControlRegister( camera,
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

