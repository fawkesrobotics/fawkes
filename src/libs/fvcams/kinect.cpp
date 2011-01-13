
/***************************************************************************
 *  kinect.cpp - Microsoft Kinect 3D Camera using the freenect driver
 *
 *  Created: Fri Nov 26 11:03:24 2010
 *  Copyright  2010  Daniel Beck
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

#include "kinect.h"

#include <cstdlib>
#include <cstring>
#include <cmath>

#include <cstdio>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Color image */
const unsigned int KinectCamera::RGB_IMAGE = 0;

/** False color depth image */
const unsigned int KinectCamera::FALSE_COLOR_DEPTH_IMAGE = 1;

/** @class KinectCamera <fvcams/kinect.h>
 * Access the Microsoft Kinect camera using the freenect driver.
 * @author Daniel Beck
 */

/** @class FvFreenectDevice <fvcams/kinect.h>
 * Implementation of the FreenectDevice interface of the driver.
 * @author Daniel Beck
 */

/** Constructor.
 * @param ctx the freenet context
 * @param index the index of the new device
 */
FvFreenectDevice::FvFreenectDevice( freenect_context* ctx, int index )
  : FreenectDevice( ctx, index )
{
  m_rgb_buffer   = (unsigned char *) malloc( FREENECT_RGB_SIZE ); 
  m_depth_buffer = (uint16_t *)      malloc( FREENECT_DEPTH_SIZE );
}

/** Destructor. */
FvFreenectDevice::~FvFreenectDevice()
{
  free( m_rgb_buffer );
  free( m_depth_buffer );
}

/** Callback function for the freenect driver.
 * This function is called with a pointer to the RGB image and the
 * timestamp of the frame.
 * @param rgb pointer to the RGB image
 * @param timestamp timestamp of the image
 */
void
FvFreenectDevice::RGBCallback( freenect_pixel* rgb, uint32_t timestamp )
{
  memcpy( (void *) m_rgb_buffer, (void *) rgb, FREENECT_RGB_SIZE );
  m_depth_timestamp = timestamp;
}

/** Callback function for the freenect driver.
 * This function is called with a pointer to the depth image and the
 * timestamp of the frame.
 * @param depth pointer to the depth image
 * @param timestamp timestamp of the image
 */
void
FvFreenectDevice::DepthCallback( void* depth, uint32_t timestamp )
{
  memcpy( (void *) m_depth_buffer, (void *) depth, FREENECT_DEPTH_SIZE );
  m_depth_timestamp = timestamp;
}

/** Access the RGB buffer.
 * @return pointer to the RGB buffer
 */
unsigned char*
FvFreenectDevice::rgb_buffer()
{
  return m_rgb_buffer;
}

/** Access the depth buffer.
 * @return pointer to the depth buffer
 */
uint16_t*
FvFreenectDevice::depth_buffer()
{
  return m_depth_buffer;
}

/** Constructor.
 * @param cap camera argument parser
 */
KinectCamera::KinectCamera( const CameraArgumentParser* cap )
  : m_freenect_dev( 0 ),
    m_opened( false ),
    m_started( false ),
    m_image_num( FALSE_COLOR_DEPTH_IMAGE ),
    m_buffer( 0 ),
    m_false_color_depth_buffer( 0 )
{
  // init freenect context
  m_freenect_ctx = new Freenect::Freenect< FvFreenectDevice >();

  for ( unsigned int i = 0; i < 2048; ++i )
  {
    float v = i / 2048.0;
    v = powf(v, 3) * 6;
    m_gamma[i] = v * 6 * 256;
  }
}

/** Destructor. */
KinectCamera::~KinectCamera()
{
  delete m_freenect_ctx;
}

void
KinectCamera::open()
{
  try
  {
    m_freenect_dev = &( m_freenect_ctx->createDevice( 0 ) );
    m_opened = true;
  }
  catch( std::runtime_error& e )
  {
    m_opened = false;
  }

  m_false_color_depth_buffer = (unsigned char*) malloc( FREENECT_RGB_SIZE );
  set_image_number( m_image_num );
}

void
KinectCamera::start()
{
  if ( !m_started )
  {
    try
    {
      m_freenect_dev->startRGB();
      m_freenect_dev->startDepth();
      m_started = true;
    }
    catch( std::runtime_error& e )
    {
      m_started = false;
    } 
  }
}

void
KinectCamera::stop()
{
  if ( m_started )
  {
    try
    {
      m_freenect_dev->stopRGB();
      m_freenect_dev->stopDepth();
      m_started = false;
    }
    catch( std::runtime_error& e )
    {
      m_started = true;
    } 
  }      
}

void
KinectCamera::close()
{
  m_freenect_ctx->deleteDevice( 0 );
  free( m_false_color_depth_buffer );
}

void
KinectCamera::capture()
{
  if ( !m_started || !m_opened ) { return; }

  if ( FALSE_COLOR_DEPTH_IMAGE == m_image_num )
  {
    for ( unsigned int i = 0; i < FREENECT_FRAME_PIX; ++i )
    {
      freenect_depth* depth = m_freenect_dev->depth_buffer();
      int pval = m_gamma[ depth[i] ];
      int lb = pval & 0xff;
      switch (pval>>8) {
      case 0:
	m_false_color_depth_buffer[3*i+0] = 255;
	m_false_color_depth_buffer[3*i+1] = 255-lb;
	m_false_color_depth_buffer[3*i+2] = 255-lb;
	break;

      case 1:
	m_false_color_depth_buffer[3*i+0] = 255;
	m_false_color_depth_buffer[3*i+1] = lb;
	m_false_color_depth_buffer[3*i+2] = 0;
	break;

      case 2:
	m_false_color_depth_buffer[3*i+0] = 255-lb;
	m_false_color_depth_buffer[3*i+1] = 255;
	m_false_color_depth_buffer[3*i+2] = 0;
	break;

      case 3:
	m_false_color_depth_buffer[3*i+0] = 0;
	m_false_color_depth_buffer[3*i+1] = 255;
	m_false_color_depth_buffer[3*i+2] = lb;
	break;

      case 4:
	m_false_color_depth_buffer[3*i+0] = 0;
	m_false_color_depth_buffer[3*i+1] = 255-lb;
	m_false_color_depth_buffer[3*i+2] = 255;
	break;

      case 5:
	m_false_color_depth_buffer[3*i+0] = 0;
	m_false_color_depth_buffer[3*i+1] = 0;
	m_false_color_depth_buffer[3*i+2] = 255-lb;
	break;

      default:
	m_false_color_depth_buffer[3*i+0] = 0;
	m_false_color_depth_buffer[3*i+1] = 0;
	m_false_color_depth_buffer[3*i+2] = 0;
	break;
      }
    }
  }
}

void
KinectCamera::flush()
{
}

bool
KinectCamera::ready()
{
  return m_started;
}

void
KinectCamera::print_info()
{
}

unsigned char*
KinectCamera::buffer()
{
  return m_buffer;
}

unsigned int
KinectCamera::buffer_size()
{
  return FREENECT_RGB_SIZE;
}

void
KinectCamera::dispose_buffer()
{
}

unsigned int
KinectCamera::pixel_width()
{
  return FREENECT_FRAME_W;
}

unsigned int
KinectCamera::pixel_height()
{
  return FREENECT_FRAME_H;
}

colorspace_t
KinectCamera::colorspace()
{
  return RGB;
}

void
KinectCamera::set_image_number( unsigned int n )
{
  m_image_num = n;
  switch ( m_image_num )
  {
  case RGB_IMAGE:
    m_buffer = m_freenect_dev->rgb_buffer();
    printf( "Selected RGB buffer\n" );
    break;

  case FALSE_COLOR_DEPTH_IMAGE:
    m_buffer = m_false_color_depth_buffer;
    printf( "Selected false color depth buffer\n" );
    break;

  default:
    m_buffer = m_freenect_dev->rgb_buffer();
  }
}

} // end namespace firevision
