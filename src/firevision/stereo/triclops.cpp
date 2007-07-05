
/***************************************************************************
 *  triclops.cpp - Stereo processor using the TriclopsSDK
 *
 *  Created: Fri May 18 17:20:31 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <stereo/triclops.h>

#include <core/exceptions/software.h>
#include <cams/bumblebee2.h>
#include <fvutils/base/roi.h>
#include <utils/math/angle.h>
#include <fvutils/color/conversions.h>

// PGR Triclops SDK
#include <triclops.h>

/// @cond INTERNALS
/** Data internal to Triclops stereo processor
 * This class exists to be able to hide the triclops stuff from the camera
 * user and not to expose the Triclops SDK headers.
 */
class TriclopsStereoProcessorData
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


/** Constructor.
 * @param camera Must be of type Bumblebee2Camera
 */
TriclopsStereoProcessor::TriclopsStereoProcessor(Camera *camera)
{
  bb2 = dynamic_cast<Bumblebee2Camera *>(camera);
  if ( ! bb2 ) {
    throw TypeMismatchException("Camera is not of type Bumblebee2Camera");
  }
  if ( ! bb2->is_bumblebee2() ) {
    throw TypeMismatchException("Camera is not a Bumblebee 2");
  }

  bb2->set_image_number(Bumblebee2Camera::RGB_IMAGE);

  _width      = bb2->pixel_width();
  _height     = bb2->pixel_height();

  /* Working buffers.
   * Note that in general cameras should _not_ have internal buffers and do extensive
   * calculations.
   *
   * buffer size calculated as: we have RAW16 format, which means two bytes per
   * pixel, thus total buffer size must be w * h * 2
   */
  buffer_rgb          = bb2->buffer();
  buffer_green        = (unsigned char *)malloc(_width * _height * 2);
  buffer_yuv_right    = malloc_buffer(YUV422_PLANAR, _width, _height);
  buffer_yuv_left     = malloc_buffer(YUV422_PLANAR, _width, _height);
  buffer_rgb_right    = buffer_rgb; 
  buffer_rgb_left     = buffer_rgb + colorspace_buffer_size(RGB, _width, _height); 
  buffer_rgb_center   = buffer_rgb_left; // wtf? Done so in pgr code

  // Internal data
  data = new TriclopsStereoProcessorData();
  // Always the same
  data->input.inputType   = TriInp_RGB;
  data->input.nrows       = _height;
  data->input.ncols       = _width;
  data->input.rowinc      = data->input.ncols;
  /*
  data->input.u.rgb.red   = buffer_yuv_right;
  data->input.u.rgb.green = buffer_yuv_left;
  data->input.u.rgb.blue  = buffer_yuv_left;
  */
  data->input.u.rgb.red   = buffer_green;
  data->input.u.rgb.green = buffer_green + _width * _height;
  data->input.u.rgb.blue  = data->input.u.rgb.green;
  get_triclops_context_from_camera();

  data->enable_subpixel_interpolation = false;

  triclopsSetSubpixelInterpolation( data->triclops, 0);

  triclopsSetResolutionAndPrepare( data->triclops,
				   _height, _width,
				   _height, _width);


  // Set defaults
  triclopsSetEdgeCorrelation( data->triclops, 1 );
  triclopsSetLowpass( data->triclops, 1 );
  triclopsSetDisparity( data->triclops, 5, 100);
  triclopsSetEdgeMask(data->triclops, 11);
  triclopsSetStereoMask(data->triclops, 23);
  triclopsSetSurfaceValidation(data->triclops, 1);
  triclopsSetTextureValidation(data->triclops, 0);

  triclopsSetDisparityMapping( data->triclops, 10, 85 );
  triclopsSetDisparityMappingOn( data->triclops, 1 );
}


/** Destructor. */
TriclopsStereoProcessor::~TriclopsStereoProcessor()
{
  // segfaults :-/
  // triclopsDestroyContext( data->triclops );

  if ( buffer_green != NULL )         free(buffer_green);
  if ( buffer_yuv_right != NULL )     free(buffer_yuv_right);
  if ( buffer_yuv_left != NULL )      free(buffer_yuv_left);

  buffer_green = NULL;
  buffer_rgb = NULL;
  buffer_yuv_right = NULL;
  buffer_yuv_left = NULL;
  _buffer = NULL;

  delete data;
}


/** Enable or disable subpixel interpolation
 * @param enabled true to enable, false to disable
 */
void
TriclopsStereoProcessor::set_subpixel_interpolation(bool enabled)
{
  data->enable_subpixel_interpolation = enabled;
  triclopsSetSubpixelInterpolation(data->triclops, enabled);
}


/** Enable or disable edge correlation.
 * @param enabled true to enable, false to disable
 */
void
TriclopsStereoProcessor::set_edge_correlation(bool enabled)
{
  triclopsSetEdgeCorrelation(data->triclops, enabled);
}


/** Enable or disable lowpass filtering before rectification.
 * @param enabled true to enable, false to disable
 */
void
TriclopsStereoProcessor::set_lowpass(bool enabled)
{
  triclopsSetLowpass(data->triclops, enabled);
}


/** Set disparity range.
 * @param min minimum disparity
 * @param max maximum disparity
 */
void
TriclopsStereoProcessor::set_disparity_range(int min, int max)
{
  triclopsSetDisparity(data->triclops, min, max);
}


/** Set edge mask.
 * Size of the kernel used for edge detection.
 * This value must be in the range [3..13].
 * @param mask_size mask size
 */
void
TriclopsStereoProcessor::set_edge_masksize(unsigned int mask_size)
{
  triclopsSetEdgeMask(data->triclops, mask_size);
}


/** Set stereo mask.
 * Size of the mask used for stereo correlation.
 * @param mask_size mask size
 */
void
TriclopsStereoProcessor::set_stereo_masksize(unsigned int mask_size)
{
  triclopsSetStereoMask(data->triclops, mask_size);
}


/** Enable or disable surface validation.
 * @param enabled true to enable, false to disable
 */
void
TriclopsStereoProcessor::set_surface_validation(bool enabled)
{
  triclopsSetSurfaceValidation(data->triclops, enabled);
}


/** Enable or disable texture validation.
 * @param enabled true to enable, false to disable
 */
void
TriclopsStereoProcessor::set_texture_validation(bool enabled)
{
  triclopsSetTextureValidation(data->triclops, enabled);  
}


/** Set disparity mapping range.
 * @param min minimum disparity
 * @param max maximum disparity
 */
void
TriclopsStereoProcessor::set_disparity_mapping_range(unsigned char min, unsigned char max)
{
  triclopsSetDisparityMapping(data->triclops, min, max);
}


/** Enable or disable disparity mapping.
 * @param enabled true to enable, false to disable
 */
void
TriclopsStereoProcessor::set_disparity_mapping(bool enabled)
{
  triclopsSetDisparityMappingOn(data->triclops, enabled);
}


/** Check state of subpixel interpolation
 * @return true if enabled, false otherwise
 */
bool
TriclopsStereoProcessor::subpixel_interpolation()
{
  TriclopsBool on;
  triclopsGetSubpixelInterpolation(data->triclops, &on);
  return on;
}


/** Check state of edge correlation.
 * @return true if enabled, false otherwise
 */
bool
TriclopsStereoProcessor::edge_correlation()
{
  TriclopsBool on;
  triclopsGetEdgeCorrelation(data->triclops, &on);
  return on;
}


/** Check state of lowpass filtering.
 * @return true if enabled, false otherwise
 */
bool
TriclopsStereoProcessor::lowpass()
{
  TriclopsBool on;
  triclopsGetLowpass(data->triclops, &on);
  return on;
}


/** Get disparity range min value.
 * @return disparity range min value
 */
int
TriclopsStereoProcessor::disparity_range_min()
{
  int min, max;
  triclopsGetDisparity( data->triclops, &min, &max );
  return min;
}


/** Get disparity range max value.
 * @return disparity range max value
 */
int
TriclopsStereoProcessor::disparity_range_max()
{
  int min, max;
  triclopsGetDisparity( data->triclops, &min, &max );
  return max;
}


/** Get edge mask size.
 * @return size of the edge mask
 */
unsigned int
TriclopsStereoProcessor::edge_masksize()
{
  int mask_size = 0;
  triclopsGetEdgeMask( data->triclops, &mask_size );
  return mask_size;
}


/** Get stereo mask size.
 * @return size of the stereo mask
 */
unsigned int
TriclopsStereoProcessor::stereo_masksize()
{
  int mask_size = 0;
  triclopsGetStereoMask( data->triclops, &mask_size );
  return mask_size;
}


/** Check state of surface validation.
 * @return true if enabled, false otherwise
 */
bool
TriclopsStereoProcessor::surface_validation()
{
  TriclopsBool on;
  triclopsGetSurfaceValidation(data->triclops, &on);
  return on;
}


/** Check state of texture validation.
 * @return true if enabled, false otherwise
 */
bool
TriclopsStereoProcessor::texture_validation()
{
  TriclopsBool on;
  triclopsGetTextureValidation(data->triclops, &on);
  return on;
}


/** Get disparity mapping min value.
 * @return min value for disparity mapping
 */
unsigned char
TriclopsStereoProcessor::disparity_mapping_min()
{
  unsigned char min, max;
  triclopsGetDisparityMapping( data->triclops, &min, &max );
  return min;
}


/** Get disparity mapping max value.
 * @return max value for disparity mapping
 */
unsigned char
TriclopsStereoProcessor::disparity_mapping_max()
{
  unsigned char min, max;
  triclopsGetDisparityMapping( data->triclops, &min, &max );
  return max;
}


/** Check state of disparity mapping
 * @return true if enabled, false otherwise
 */
bool
TriclopsStereoProcessor::disparity_mapping()
{
  TriclopsBool on;
  triclopsGetDisparityMappingOn(data->triclops, &on);
  return on;
}


void
TriclopsStereoProcessor::preprocess_stereo()
{
  bb2->deinterlace_stereo();
  bb2->decode_bayer();
}

void
TriclopsStereoProcessor::calculate_yuv(bool both)
{
  // RGB -> YUV
  convert(RGB, YUV422_PLANAR, buffer_rgb_right, buffer_yuv_right, _width, _height);
  if ( both ) {
    convert(RGB, YUV422_PLANAR, buffer_rgb_left, buffer_yuv_left, _width, _height);
  }
}


void
TriclopsStereoProcessor::calculate_disparity(ROI *roi)
{
  TriclopsROI *rois;
  int          max_rois;

  if ( NULL != roi ) {
    if ( TriclopsErrorOk == triclopsGetROIs( data->triclops, &rois, &max_rois ) ) {
      // assume there is always at least one ROI
      rois[0].col = roi->start.x;
      rois[0].row = roi->start.y;
      rois[0].ncols = roi->width;
      rois[0].nrows = roi->height;

      triclopsSetNumberOfROIs( data->triclops, 1 );
    } else {
      triclopsSetNumberOfROIs( data->triclops, 0 );
    }
  } else {
    triclopsSetNumberOfROIs( data->triclops, 0 );
  }

  // now deinterlace the RGB Buffer
  deinterlace_green( buffer_rgb, buffer_green, _width, 6 * _height );

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
      triclopsRCD16ToXYZ(data->triclops, _height, _width, data->disparity_image_hires.data[640 * 240 + 320], &x, &y, &z);
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
      PNMWriter pnm(PNM_PGM, "disparity_own.pgm", _width, _height);
      pnm.set_buffer(YUV422_PLANAR, data->disparity_image_lores.data);
      pnm.write();
      triclopsSaveImage( &(data->disparity_image_lores), "disparity_lores.pgm" );
      triclopsSetDisparityMappingOn( data->triclops, 0 );
      done = true;
    }
  }
  */
}


/** Get the disparity image buffer.
 * Access method to the disparity image buffer.
 * @return pointer to the internal disparity image buffer
 */
unsigned char *
TriclopsStereoProcessor::disparity_buffer()
{
  if ( data->enable_subpixel_interpolation ) {
    return (unsigned char *)data->disparity_image_hires.data;
  } else {
    return data->disparity_image_lores.data;
  }
}


size_t
TriclopsStereoProcessor::disparity_buffer_size() const
{
  if ( data->enable_subpixel_interpolation ) {
    return _width * _height * 2;
  } else {
    return _width * _height;
  }
}


unsigned char *
TriclopsStereoProcessor::yuv_buffer()
{
  return buffer_yuv_right;
}


unsigned char *
TriclopsStereoProcessor::auxiliary_yuv_buffer()
{
  return buffer_yuv_left;
}


/** Get Triclops context.
 * This retrieves calibration information from the camera and stores it into a
 * temporary file. With this file the Triclops context is initialized. Afterwards
 * the temporary file is removed.
 */
void
TriclopsStereoProcessor::get_triclops_context_from_camera()
{
  char *tmpname = (char *)malloc(strlen("triclops_cal_XXXXXX") + 1);
  strcpy(tmpname, "triclops_cal_XXXXXX");
  char *tmpfile = mktemp(tmpname);
  bb2->write_triclops_config_from_camera_to_file(tmpfile);

  data->err = triclopsGetDefaultContextFromFile(&(data->triclops), tmpfile);
  if ( data->err != TriclopsErrorOk ) {
    free(tmpfile);
    throw Exception("Fetching Triclops context from camera failed");
  }
  unlink(tmpfile);
  free(tmpfile);
}


/** Deinterlace green buffer.
 * Method used in stereo processing. Following the PTGrey example, seems useless
 * if we have YUV planar and thus grey images anyway.
 * @param src source buffer
 * @param dest destination buffer
 * @param width width of the image
 * @param height height of the image
 */
void
TriclopsStereoProcessor::deinterlace_green( unsigned char* src, 
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




/** Get camera-relative coordinates of a point.
 * Use this method to get the coordinates in the camera coordinate system of a given
 * point in the image. It may not be possible to provide such a coordinate if no valid
 * disparity information could be calculated for the given point.
 * @param px x coordinate in image
 * @param py y coordinate in image
 * @param x contains the x coordinate in the camera-relative coordinate system upon
 * successful return
 * @param y contains the y coordinate in the camera-relative coordinate system upon
 * successful return
 * @param z contains the z coordinate in the camera-relative coordinate system upon
 * successful return
 * @return true, if valid information could be retrieved. In that case (x, y, z) is filled
 * with the coordinates, false otherwise (x, y, and z are not modified).
 */
bool
TriclopsStereoProcessor::get_xyz(unsigned int px, unsigned int py, float *x, float *y, float *z)
{
  if ( data->enable_subpixel_interpolation ) {
    unsigned short disparity = data->disparity_image_hires.data[_width * py + px];
    if ( disparity < 0xFF00 ) {
      triclopsRCD16ToXYZ(data->triclops, py, px, disparity, x, y, z);
      return true;
    }
  } else {
    unsigned char disparity = data->disparity_image_lores.data[_width * py + px];
    if ( disparity != 0 ) {
      triclopsRCD8ToXYZ(data->triclops, py, px, disparity, x, y, z);
      return true;
    }
  }

  return false;
}


/** Get transformed coordinates of a point.
 * Use this method to get the coordinates in the transformed coordinate system of a given
 * point in the image. It may not be possible to provide such a coordinate if no valid
 * disparity information could be calculated for the given point.
 * @param px x coordinate in image
 * @param py y coordinate in image
 * @param x contains the x coordinate in the camera-relative coordinate system upon
 * successful return
 * @param y contains the y coordinate in the camera-relative coordinate system upon
 * successful return
 * @param z contains the z coordinate in the camera-relative coordinate system upon
 * successful return
 * @return true, if valid information could be retrieved. In that case (x, y, z) is filled
 * with the coordinates, false otherwise (x, y, and z are not modified).
 */
bool
TriclopsStereoProcessor::get_world_xyz(unsigned int px, unsigned int py, float *x, float *y, float *z)
{
  float cam_angle = deg2rad(57);
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
