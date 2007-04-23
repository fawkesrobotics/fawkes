
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

// PGR Triclops SDK
#include <triclops.h>

using namespace std;

/// @cond INTERNALS
/** Data internal to Bumblebee2Camera.
 * This class exists to be able to hide the triclops stuff from the camera
 * user and not to expose the Triclops SDK headers.
 */
class Bumblebee2CameraData
{
 public:
  TriclopsContext triclops;
  TriclopsInput   input;
  TriclopsError   err;
  TriclopsImage   rectified_image;
  TriclopsImage16 disparity_image_hires;
  TriclopsImage   disparity_image_lores;
  bool            enable_subpixel_interpolation;
};
/// @endcond


/** Left image already converted to YUV422_PLANAR */
const unsigned int Bumblebee2Camera::LEFT_ORIGINAL = 0;

/** Right image already converted to YUV422_PLANAR */
const unsigned int Bumblebee2Camera::RIGHT_ORIGINAL = 0;

/** @class Bumblebee2Camera <cams/bumblebee2.h>
 * Bumblebee2 stereo camera.
 * This camera implementation allows for access to the IEEE1394 based stereo
 * camera Bumblebee 2 by Point Grey Research.
 * @author Tim Niemueller
 */


/// PGR specific registers
/** Bayer Tile mapping information */
#define PGR_BAYER_TILE_MAPPING_REGISTER (0x1040)
#define PGR_REG_CONFIG_LENGTH         	(0x1FFC) 
#define PGR_REG_CONFIG_DATA           	(0x2000) 
/* unit directory offset */
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
Bumblebee2Camera::Bumblebee2Camera(CameraArgumentParser *cap)
  : FirewireCamera(DC1394_FRAMERATE_30,
		   DC1394_VIDEO_MODE_FORMAT7_3,
		   DC1394_ISO_SPEED_400,
		   /* num_buffers */ 8)
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

  buffer_deinterlaced = NULL;
  buffer_green = NULL;
  buffer_rgb = NULL;
  buffer_yuv_right = NULL;
  buffer_yuv_left = NULL;

  done = false;
}


/** Destructor. */
Bumblebee2Camera::~Bumblebee2Camera()
{
  delete data;
  if ( buffer_deinterlaced != NULL )  free(buffer_deinterlaced);
  if ( buffer_green != NULL )         free(buffer_green);
  if ( buffer_rgb != NULL )           free(buffer_rgb);
  if ( buffer_yuv_right != NULL )     free(buffer_yuv_right);
  if ( buffer_yuv_left != NULL )      free(buffer_yuv_left);
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

  /* Working buffers.
   * Note that in general cameras should _not_ have internal buffers and do extensive
   * calculations. But until we have the hardware ready for disparity calculation
   * we will use the triclops SDK and we want to hide that from the rest of the
   * software.
   *
   * buffer size calculated as: we have RAW16 format, which means two bytes per
   * pixel, thus total buffer size must be w * h * 2
   */
  buffer_deinterlaced = (unsigned char *)malloc(pixel_width() * pixel_height() * 2);
  buffer_green        = (unsigned char *)malloc(pixel_width() * pixel_height() * 2);
  buffer_rgb          = malloc_buffer(RGB, pixel_width(), pixel_height() * 2);
  buffer_yuv_right    = malloc_buffer(YUV422_PLANAR, pixel_width(), pixel_height());
  buffer_yuv_left     = malloc_buffer(YUV422_PLANAR, pixel_width(), pixel_height());
  _buffer = buffer_yuv_right;

  // Internal data
  data = new Bumblebee2CameraData();
  // Always the same
  data->input.inputType   = TriInp_RGB;
  data->input.nrows       = pixel_height();
  data->input.ncols       = pixel_width();
  data->input.rowinc      = data->input.ncols;
  /*
  data->input.u.rgb.red   = buffer_yuv_right;
  data->input.u.rgb.green = buffer_yuv_left;
  data->input.u.rgb.blue  = buffer_yuv_left;
  */
  data->input.u.rgb.red   = buffer_green;
  data->input.u.rgb.green = buffer_green + pixel_width() * pixel_height();
  data->input.u.rgb.blue  = data->input.u.rgb.green;
  get_triclops_context_from_camera();

  data->enable_subpixel_interpolation = false;

  TriclopsBool on;
  int masksize;
  float focal_length;
  triclopsGetSubpixelInterpolation( data->triclops, &on );
  cout << "Triclops Subpixel Interpolation: " << on << endl;
  triclopsGetEdgeCorrelation( data->triclops, &on );
  cout << "Edge correlation: " << on << endl;
  triclopsGetEdgeMask( data->triclops, &masksize );
  cout << "Edge mask: " << masksize << endl;
  triclopsGetLowpass( data->triclops, &on );
  cout << "Low pass filter: " << on << endl;
  triclopsGetRectify( data->triclops, &on );
  cout << "Rectification: " << on << endl;
  triclopsGetFocalLength( data->triclops, &focal_length );
  cout << "Focal length: " << focal_length << endl;

  triclopsSetResolutionAndPrepare( data->triclops,
				   pixel_height(), pixel_width(),
				   pixel_height(), pixel_width());

  triclopsSetSubpixelInterpolation( data->triclops, data->enable_subpixel_interpolation ? 1 : 0 );

  triclopsSetEdgeCorrelation( data->triclops, 1 );
  triclopsSetLowpass( data->triclops, 1 );
  triclopsSetDisparity( data->triclops, 5, 100);
  triclopsSetEdgeMask(data->triclops, 11);
  triclopsSetStereoMask(data->triclops, 23);
  triclopsSetSurfaceValidation(data->triclops, 1);
  triclopsSetTextureValidation(data->triclops, 0);

  triclopsSetDisparityMapping( data->triclops, 10, 85 );
  triclopsSetDisparityMappingOn( data->triclops, 1 );

  /*
  TriclopsTransform tr;
  / first index row, second index column
  tr.matrix[0][0] =  1;  tr.matrix[0][1] =  1;     tr.matrix[0][2] =  0;     tr.matrix[0][3] =  0;
  tr.matrix[1][0] =  0;  tr.matrix[1][1] =  0;  tr.matrix[1][2] = 0;  tr.matrix[1][3] =  0;
  tr.matrix[2][0] =  0;  tr.matrix[2][1] =  0;  tr.matrix[2][2] =  0;  tr.matrix[2][3] =  0;
  tr.matrix[3][0] =  0;  tr.matrix[3][1] =  0;     tr.matrix[3][2] =  0;     tr.matrix[3][3] =  0;
  
  * first index col, second index row
  tr.matrix[0][0] =  1;  tr.matrix[1][0] = 0;     tr.matrix[2][0] =  0;     tr.matrix[3][0] = 0;
  tr.matrix[0][1] =  0;  tr.matrix[1][1] = 0.34;  tr.matrix[2][1] = -0.94;  tr.matrix[3][1] = 0;
  tr.matrix[0][2] =  0;  tr.matrix[1][2] = 0.94;  tr.matrix[2][2] =  0.34;  tr.matrix[3][2] = 0;
  tr.matrix[0][3] =  0;  tr.matrix[1][3] = 0;     tr.matrix[2][3] =  0;     tr.matrix[3][3] = 1;

  triclopsSetTriclopsToWorldTransform(data->triclops, tr);
  */

  int nrows, ncols;
  triclopsGetResolution(data->triclops, &nrows, &ncols);
  cout << "Resolution: " << ncols << "x" << nrows << endl;

}


void Bumblebee2Camera::close()
{
  FirewireCamera::close();

  if ( buffer_deinterlaced != NULL )  free(buffer_deinterlaced);
  if ( buffer_green != NULL )         free(buffer_green);
  if ( buffer_rgb != NULL )           free(buffer_rgb);
  if ( buffer_yuv_right != NULL )     free(buffer_yuv_right);
  if ( buffer_yuv_left != NULL )      free(buffer_yuv_left);

  buffer_deinterlaced = NULL;
  buffer_green = NULL;
  buffer_rgb = NULL;
  buffer_yuv_right = NULL;
  buffer_yuv_left = NULL;
  _buffer = NULL;

  // segfaults :-/
  // triclopsDestroyContext( data->triclops );
}


void
Bumblebee2Camera::capture()
{

  if ( ! opened  ) throw CameraNotOpenedException();
  if ( ! started ) throw CameraNotStartedException();

  if (! iso_mode_enabled()) {
    //cout << msg_prefix << cred << "ISO mode enabled while trying to capture" << cnormal << endl;
    throw CameraNotStartedException();
  }

  if (dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame) != DC1394_SUCCESS) {
    //cout << msg_prefix << cred << "Could not capture frame" << cnormal << endl;
    valid_frame_received = false;
    return;
  } else {
    valid_frame_received = true;
  }

  unsigned int width = pixel_width();
  unsigned int height = pixel_height();
  
  // de-interlace the 16 bit data into 2 bayer tile pattern images
  dc1394_deinterlace_stereo( frame->image, buffer_deinterlaced, width, 2 * height );

  // extract color from the bayer tile image
  // note: this will alias colors on the top and bottom rows
  dc1394_bayer_decoding_8bit( buffer_deinterlaced, buffer_rgb, width, 2 * height,
			      bayer_pattern, DC1394_BAYER_METHOD_NEAREST );
  
  // now deinterlace the RGB Buffer
  deinterlace_green( buffer_rgb, buffer_green, width, 6 * height );

  buffer_rgb_right  = buffer_rgb;
  buffer_rgb_left   = buffer_rgb + colorspace_buffer_size(RGB, width, height);
  buffer_rgb_center = buffer_rgb_left; // wtf? Done so in pgr code

  // RGB -> YUV
  convert(RGB, YUV422_PLANAR, buffer_rgb_right, buffer_yuv_right, width, height);
  convert(RGB, YUV422_PLANAR, buffer_rgb_left, buffer_yuv_left, width, height);
  buffer_yuv_center = buffer_yuv_left;

  // rectify
  if ( (data->err = triclopsRectify( data->triclops, &(data->input))) != TriclopsErrorOk ) {
    throw Exception("Rectifying the image failed");
  }

  // disparity
  if ( (data->err = triclopsStereo( data->triclops )) != TriclopsErrorOk ) {
    throw Exception("Calculating the disparity image failed");
  }

  triclopsGetImage(data->triclops, TriImg_RECTIFIED, TriCam_REFERENCE, &(data->rectified_image));

  if ( data->enable_subpixel_interpolation ) {
    triclopsGetImage16( data->triclops, TriImg16_DISPARITY, TriCam_REFERENCE, &(data->disparity_image_hires) );
  } else {
    triclopsGetImage( data->triclops, TriImg_DISPARITY, TriCam_REFERENCE, &(data->disparity_image_lores) );
  }
  /*
  if ( data->enable_subpixel_interpolation ) {
    if ( data->disparity_image_hires.data[640 * 240 + 320] < 0xFF00 ) {
      float x, y, z;
      triclopsRCD16ToXYZ(data->triclops, pixel_height(), pixel_width(), data->disparity_image_hires.data[640 * 240 + 320], &x, &y, &z);
      cout << "Pixel coordinate: (x, y, z) = (" << x << "," << y << "," << z << ")" << endl;
    } else {
      cout << "Pixel has invalid disparity value" << endl;
    }
    if ( ! done ) {
      triclopsSaveImage16( &(data->disparity_image_hires), "disparity.pgm" );
      triclopsSetDisparityMappingOn( data->triclops, 0 );
      done = true;
    }
  } else {
    if ( (data->disparity_image_lores.data + data->disparity_image_lores.rowinc * 240 + 320) != 0 ) {
      float x, y, z;
      triclopsRCD8ToXYZ(data->triclops, 240, 320, *(data->disparity_image_lores.data + data->disparity_image_lores.rowinc * 240 + 320), &x, &y, &z);
      cout << "Pixel coordinate: (x, y, z) = (" << x << "," << y << "," << z << ")" << endl;
    } else {
      cout << "Pixel has invalid disparity value" << endl;
    }
    if ( ! done ) {
      PNMWriter pnm(PNM_PGM, "disparity_own.pgm", pixel_width(), pixel_height());
      pnm.set_buffer(YUV422_PLANAR, data->disparity_image_lores.data);
      pnm.write();
      triclopsSaveImage( &(data->disparity_image_lores), "disparity_lores.pgm" );
      triclopsSetDisparityMappingOn( data->triclops, 0 );
      done = true;
    }
  }
  */

}


bool
Bumblebee2Camera::get_xyz(unsigned int px, unsigned int py, float *x, float *y, float *z)
{
  if ( data->enable_subpixel_interpolation ) {
    unsigned short disparity = data->disparity_image_hires.data[pixel_width() * py + px];
    if ( disparity < 0xFF00 ) {
      triclopsRCD16ToXYZ(data->triclops, py, px, disparity, x, y, z);
      return true;
    }
  } else {
    unsigned char disparity = data->disparity_image_lores.data[pixel_width() * py + px];
    if ( disparity != 0 ) {
      triclopsRCD8ToXYZ(data->triclops, py, px, disparity, x, y, z);
      return true;
    }
  }

  return false;
}


bool
Bumblebee2Camera::get_world_xyz(unsigned int px, unsigned int py, float *x, float *y, float *z)
{
  float cam_angle = deg2rad(60);
  float trans_x = -0.1;
  float trans_y =  0.05;
  float trans_z = -0.78;
  float tx, ty, tz;
  if ( get_xyz(px, py, &tx, &ty, &tz) ) {
    /* transform around x axis
    *x = tx;
    *y = cos(cam_angle) * ty  + sin(cam_angle) * tz;
    *z = -sin(cam_angle) * ty + cos(cam_angle) * tz;
    */
    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    x1 = tx;
    y1 = cos(cam_angle) * ty  + sin(cam_angle) * tz;
    z1 = -sin(cam_angle) * ty + cos(cam_angle) * tz;
    // cout << "Transform 1: (" << tx << "," << ty << "," << tz << ") -> (" << x1 << "," << y1 << "," << z1 << ")" << endl;
    // *x = y1;
    // *y =  cos(cam_angle) * x1 + sin(cam_angle) * z1;
    // *z = -sin(cam_angle) * x1 + cos(cam_angle) * z1;
    x2 = y1;
    y2 = x1;
    z2 = z1;
    // cout << "Transform 2: (" << x1 << "," << y1 << "," << z1 << ") -> (" << x2 << "," << y2 << "," << z2 << ")" << endl;

    x3 = z2;
    y3 = y2;
    z3 = x2;
    // cout << "Transform 3: (" << x2 << "," << y2 << "," << z2 << ") -> (" << x3 << "," << y3 << "," << z3 << ")" << endl;

    *x = x3 + trans_x;
    *y = y3 + trans_y;
    *z = z3 + trans_z; 

    // cout << "Transform 4: (" << x3 << "," << y3 << "," << z3 << ") -> (" << *x << "," << *y << "," << *z << ")" << endl;

    /*
    *x = ty + trans_x;
    *y = -sin(cam_angle) * tx + cos(cam_angle) * tz + trans_y;
    *z = cos(cam_angle) * tx + sin(cam_angle) * tz + trans_z;
    */
    return true;
  } else {
    return false;
  }
}

void
Bumblebee2Camera::flush()
{
  capture();
  // HACK, needed or we will get kernel NULL pointer exception *urgh*
  usleep(100000);
  dispose_buffer();
}


void
Bumblebee2Camera::set_image_number(unsigned int n)
{
  if ( n == LEFT_ORIGINAL ) {
    _buffer = buffer_yuv_left;
  } else {
    _buffer = buffer_yuv_right;
  }
}


unsigned char*
Bumblebee2Camera::buffer()
{
  if ( valid_frame_received ) {
    return _buffer;
  } else {
    return NULL;
  }
}


unsigned char *
Bumblebee2Camera::buffer_disparity()
{
  if (valid_frame_received) {
    if ( data->enable_subpixel_interpolation ) {
      return NULL; // data->disparity_image_hires.data;
    } else {
      return data->disparity_image_lores.data;
    }
  } else {
    return NULL;
  }
}


unsigned int
Bumblebee2Camera::buffer_size()
{
  if ( valid_frame_received ) {
    return frame->total_bytes;
  } else {
    return 0;
  }
}


colorspace_t
Bumblebee2Camera::colorspace()
{
  return YUV422_PLANAR;
}


bool
Bumblebee2Camera::is_bumblebee2()
{
  if ( ! opened ) throw CameraNotOpenedException();

  return( strncmp( camera->model, "Bumblebee2", strlen("Bumblebee2") ) == 0);
}


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


void
Bumblebee2Camera::get_triclops_context_from_camera()
{
  char *tmpname = (char *)malloc(strlen("triclops_cal_XXXXXX") + 1);
  strcpy(tmpname, "triclops_cal_XXXXXX");
  char *tmpfile = mktemp(tmpname);
  write_triclops_config_from_camera_to_file(tmpfile);

  data->err = triclopsGetDefaultContextFromFile(&(data->triclops), tmpfile);
  if ( data->err != TriclopsErrorOk ) {
    free(tmpfile);
    throw Exception("Fetching Triclops context from camera failed");
  }
  unlink(tmpfile);
  free(tmpfile);
}


void
Bumblebee2Camera::deinterlace_green( unsigned char* src, 
				     unsigned char* dest, 
				     unsigned int width, 
				     unsigned int height)
{
  register int i = (width*height)-2;
  register int g = ((width*height)/3)-1;

  while (i >= 0) {
    dest[g--] = src[i-=3];
  }
}
