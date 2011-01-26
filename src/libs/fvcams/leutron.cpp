
/***************************************************************************
 *  leutron.cpp - Leutron camera
 *
 *  Generated: Thu Mar 24 22:36:05 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#include <fvcams/leutron.h>
#include <fvutils/color/colorspaces.h>

#include <lvdef.h>
#include <dsylib.h>
#include <grabber.h>
#include <cstdlib>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LeutronCamera <fvcams/leutron.h>
 * Cameras accessed through Leutron framegrabber.
 */

/** Constructor. */
LeutronCamera::LeutronCamera()
{
  started = opened = false;
  autodetect = false;

  cspace = YUV422_PACKED;

  camera_name = "PAL_S_CCIR";
}


/** Destructor. */
LeutronCamera::~LeutronCamera()
{
}


void
LeutronCamera::open()
{
  opened = false;
  camera_handle = HANDLE_INVALID;

  // cout << "LeutronCam: Calling DsyInit(). This may take a while..." << std::flush;
  DsyInit();
  // cout << "done" << endl;

  if (DsyRecheckConnector() != I_NoError) {
    throw Exception("LeutronCam: DsyRecheckConnector() failed");
  }

  LvCameraConnDlgInfo *info;
  HANDLE conn_info_handle = NULL;
  HGRABBER grabber_handle = HANDLE_INVALID;

  if (DsyDetectCamera(&conn_info_handle) > 0) {
    // We found at least one camera
    info = (LvCameraConnDlgInfo *)GlobalLock(conn_info_handle);
    if (info) {
      
      grabber = info[0].Grabber;

      int camera_id = -1;
      if (autodetect) {
	// Take the first one detected
	camera_id = info[0].CameraType;
      } else {
	// Find given camera
	LvCameraInfo cam_info;
	for (int i = 0; DsyEnumCameraType(i, &cam_info) == I_NoError; ++i) {
	  if ( strcmp(camera_name, cam_info.Name) == 0 ) {
	    // We found the camera
	    camera_id = cam_info.Id;
	    break;
	  }
	}
	if (camera_id == -1) {
	  // throw Exception("LeutronCam: Could not find the requested camera. Trying default.");
	  camera_id = info[0].CameraType;
	}
      }

      if (grabber == NULL) {
	throw Exception("LeutronCam: grabber == NULL");
      }
      grabber_handle = info[0].hGrabber;
      if (grabber_handle == HANDLE_INVALID) {
	throw Exception("LeutronCam: grabber handle is invalid.");
      }
      if (info[0].hConn == HANDLE_INVALID) {
	throw Exception("LeutronCam: connection handle is invalid.");
      }
      camera_handle = grabber->ConnectCamera(camera_id,
					     info[0].hConn,
					     info[0].SyncNr);
      if ( camera_handle == HANDLE_INVALID ) {
	throw Exception("LeutronCam: Could not connect the camera");
      }

      char tmp[128];
      camera = grabber->GetCameraPtr(camera_handle);
      camera->GetDescription(tmp, sizeof(tmp));
      // cout << "LeutronCam: Camera '" << tmp << "' is connected to '" << grabber->GetName() << "'" << endl;

      // No effect: GlobalUnlock(conn_info_handle);
      GlobalFree(conn_info_handle);

      if (grabber->ActivateCamera( camera_handle ) != DSY_I_NoError) {
	throw Exception("LeutronCam: Could not activate camera");
      }

      LvSourceInfo src_info;
      camera->GetSourceROI(&src_info);
      src_info.StartX = 0;
      src_info.StartY = 0;

      if (camera->SetSourceROI( &src_info ) != DSY_I_NoError) {
	// cout << "LeutronCam: WARNING. Cannot set source info ROI" << endl;
      }

      width  = src_info.MaxWidth;
      height = src_info.MaxHeight;
      scaled_width = width;
      scaled_height = height;
      /*
      cout << "LeutronCam:  Width:        " << width << "   Height:        " << height << endl;
      cout << "LeutronCam:  Scaled Width: " << scaled_width
	   << "   Scaled Height: " << scaled_height << endl;
      */

      if ( (scaled_width != width) || (scaled_height != height) ) {
	// scaled image
	scaled_buffer = (unsigned char*)malloc(colorspace_buffer_size(YUV422_PACKED, scaled_width, scaled_height));
      }

      LvROI  roi;
      grabber->GetConnectionInfo( camera_handle, &roi );
      roi.SetTargetBuffer( TgtBuffer_CPU );
      roi.SetDIBMode( TRUE );
      if (cspace != YUV422_PACKED) {
	// cout << "LeutronCam: WARNING! Cannot capture in requested color space, defaulting to packed  YUV422" << endl;
      }
      roi.SetColorFormat( ColF_YUV_422 );
      roi.SetStartPosition( 0, 0 );
      roi.SetDimension( scaled_width, scaled_height );
      roi.SetMemoryWidth( width );

      /*
      cout << "LeutronCam(Memory Info): PixelIncrement: " << roi.GetPixelIncrement() << endl
	   << "LeutronCam(Memory Info): LineIncrement:  " << roi.GetLineIncrement()  << endl;
      printf( "LeutronCam(Memory Info): BaseAddress:    %x\n", (unsigned int)roi.MemoryInfo.BaseAddress);
      printf( "LeutronCam(Memory Info): StartAddress    %x\n", (unsigned int)roi.StartAddress);
      */

      if (grabber->ActivateROI(camera_handle, &roi) != DSY_I_NoError) {
	throw Exception("LeutronCam: Cannot activate ROI");
      }

      camera->Live( SY_None );

    } else {
      throw Exception("LeutronCam: Could not get lock on connection info.");
    }
    opened = true;
  } else {
    throw Exception("LeutronCam: Could not find any camera.");
  }

}


void
LeutronCamera::start()
{
  if ( started ) return;
  if (!opened) {
    throw Exception("LeutronCam: Trying to start closed cam!");
  }

  started = true;
}


void
LeutronCamera::stop()
{
  started = false;
}

void
LeutronCamera::print_info()
{
}

void
LeutronCamera::capture()
{
}

void
LeutronCamera::flush()
{
}

unsigned char*
LeutronCamera::buffer()
{
  LvROI roi;
  grabber->GetConnectionInfo(camera_handle, &roi);

  if ( (scaled_width != width) || (scaled_height != height) ) {
    unsigned char *r, *buf;
    r   = (unsigned char*)roi.MemoryInfo.BaseAddress+roi.StartAddress;
    buf = scaled_buffer;
    for (unsigned int i = 0; i < height; ++i) {
      memcpy(buf, r, roi.GetPixelIncrement() * scaled_width);
      buf += roi.GetPixelIncrement() * scaled_width;
      r   += roi.GetLineIncrement();
    }
    return scaled_buffer;
  } else {
    return (unsigned char*) roi.MemoryInfo.BaseAddress+roi.StartAddress;
  }

}

unsigned int
LeutronCamera::buffer_size()
{
  return colorspace_buffer_size(YUV422_PACKED, 0, 0);
}

void
LeutronCamera::close()
{
  if (opened) {
    if ( (scaled_width != width) || (scaled_height != height) ) {
      free(scaled_buffer);
    }
  }
  //cout << "LeutronCam: Calling DsyClose().." << std::flush;
  DsyClose();
  //cout << "done" << endl;
}

void
LeutronCamera::dispose_buffer()
{
}

unsigned int
LeutronCamera::pixel_width()
{
  if (opened) {
    return scaled_width;
  } else {
    throw Exception("LeutronCam: Camera not opened");
  }
}

unsigned int
LeutronCamera::pixel_height()
{
  if (opened) {
    return scaled_height;
  } else {
    throw Exception("LeutronCam: Camera not opened");
  }
}


colorspace_t
LeutronCamera::colorspace()
{
  return cspace;
}


bool
LeutronCamera::ready()
{
  return started;
}


void
LeutronCamera::set_image_number(unsigned int n)
{
}

} // end namespace firevision
