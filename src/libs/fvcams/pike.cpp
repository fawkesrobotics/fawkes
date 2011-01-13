
/***************************************************************************
 *  pike.cpp - Allied Vision Technologies Pike camera
 *
 *  Generated: Tue Mar 16 15:27:32 2010
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

#include <fvcams/pike.h>
#include <fvcams/cam_exceptions.h>

#include <fvutils/system/camargp.h>

#include <cstring>
#include <cstdlib>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class PikeCamera <fvcams/pike.h>
 * Pike camera.
 * Allows to access some special features of the Pike camera made by
 * Allied Vision Technologies.
 */

// AVT specific registers
/** Register for white balance settings */
#define AVT_WHITE_BALANCE_REGISTER     (0x0F0080C)

/** Registers for area of interest settings */
#define AVT_AUTOFNC_AOI_REGISTER       (0x0390)
#define AVT_AF_AREA_POSITION_REGISTER  (0x0394)
#define AVT_AF_AREA_SIZE_REGISTER      (0x0398)

/** Extended version information registerst */
#define AVT_VERSION_INFO1_REGISTER     (0x1000010)
#define AVT_VERSION_INFO3_REGISTER     (0x1000018)

// AVT specific data structures
// /** White balance settings data structure */
// typedef struct {
//   uint32_t abs_control  :  1;
//   uint32_t reserved     :  3;
//   uint32_t one_push     :  1;
//   uint32_t on_off       :  1;
//   uint32_t a_m_mode     :  1;
//   uint32_t ub_value     : 12;
//   uint32_t vr_value     : 12;
//   uint32_t presence_inq :  1;
// } avt_white_balance_t;

/** Datastructure for the autofunction AOI */
typedef struct {
  uint32_t xuints         : 12; /**< X units of work area/pos. beginning with 0 (read only) */
  uint32_t yuints         : 12; /**< Y units of work area/pos. beginning with 0 (read only) */
  uint32_t reserved3      :  1; /**< Reserved. */
  uint32_t on_off         :  1; /**< Enable/disable AOI (see note above). */
  uint32_t reserved2      :  1; /**< Reserved. */
  uint32_t show_work_area :  1; /**< Show work area. */
  uint32_t reserved1      :  3; /**< Reserved. */
  uint32_t presence_inq   :  1; /**< Indicates presence of this feature (read only). */
} avt_autofnc_aoi_t;

/** Datastructure for the position of the autofunction AOI */
typedef struct {
  uint32_t top  : 16; /**< Work area position (top coordinate). */
  uint32_t left : 16; /**< Work area position (left coordinate). */
} avt_af_area_position_t;

/** Datastructure for the size of the autofunction AOI */
typedef struct {
  uint32_t height : 16; /**< Height of work area size. */
  uint32_t width  : 16; /**< Width of work area size. */
} avt_af_area_size_t;

/** Datastructure for version information of the uC */
typedef struct {
  uint32_t uc_version : 16; /**< Bcd-coded version number. */
  uint32_t uc_type_id : 16; /**< Always 0. */
} avt_version_info1_t;

/** Datastructure for version information of the FGPA */
typedef struct {
  uint32_t fpga_version   : 16; /**< Bcd-coded version number. */
  uint32_t camera_type_id : 16; /**< See Table 122: Camera type ID list on page 267 in the technical manual (v 4.3.0). */
} avt_version_info3_t;


/** Constructor.
 * @param cap Camera argument parser.
 */
PikeCamera::PikeCamera(const CameraArgumentParser* cap)
  : FirewireCamera( cap )
{
  __aoi_left   = 0;
  __aoi_top    = 0;
  __aoi_width  = 0;
  __aoi_height = 0;
  __aoi_show_work_area = false;

  __set_autofnc_aoi = false;

  if ( cap->has( "autofnc_aoi" ) )
  {
    __set_autofnc_aoi = true;
    parse_set_autofnc_aoi( cap->get( "autofnc_aoi" ).c_str() );
  }
}

/** Destructor. */
PikeCamera::~PikeCamera()
{
}

void
PikeCamera::open()
{
  try
  {
    FirewireCamera::open();
  } catch ( Exception &e )
  { throw; }

  if ( !_opened )
  { throw Exception( "PikeCamera::open: FirewireCamera::open dit not succed" ); }

  if ( !set_autofunction_aoi( __aoi_left, __aoi_top, __aoi_width, __aoi_height,
			      __aoi_show_work_area ) )
  {
    throw Exception( "PikeCamera::PikeCamera: setting autofnc AOI failed." );
  }
}

void
PikeCamera::print_info()
{
  FirewireCamera::print_info();

  uint32_t value;
  dc1394error_t err = dc1394_get_register( _camera,
					   AVT_VERSION_INFO1_REGISTER,
					   &value );

  if ( err != DC1394_SUCCESS )
  {
    throw Exception( "Pike::print_info; dc1394_get_register(AVT_VERSION_INFO1_REGISTER) failed\n" );
  }

  avt_version_info1_t version1;
  memcpy( (void*) &version1, (void*) &value, sizeof(uint32_t) );

  err = dc1394_get_register( _camera,
			     AVT_VERSION_INFO3_REGISTER,
			     &value );

  if ( err != DC1394_SUCCESS )
  {
    throw Exception( "Pike::print_info; dc1394_get_register(AVT_VERSION_INFO3_REGISTER) failed\n" );
  }

  avt_version_info3_t version3;
  memcpy( (void*) &version3, (void*) &value, sizeof(uint32_t) );

  printf( "uC type ID: %d  uC version: %x  camera type id: %d   FPGA version: %x\n",
	  version1.uc_type_id, version1.uc_version, version3.camera_type_id, version3.fpga_version );
}

/** Set the area of interest (AOI) for the auto functions.
 * @param left offset form the left image border
 * @param top offset form the top image border
 * @param width width of the AOI
 * @param height height of the AOI
 * @param show_work_area highlight the work area in the image
 * @return true on success, false otherwise
 */
bool
PikeCamera::set_autofunction_aoi( unsigned int left,
				  unsigned int top,
				  unsigned int width,
				  unsigned int height,
				  bool show_work_area )
{
  if ( !_opened )
  { return false; }

  if ( !__set_autofnc_aoi )
  { return true; }

  avt_autofnc_aoi_t aoi;
  avt_af_area_position_t position;
  avt_af_area_size_t size;

  aoi.show_work_area = show_work_area;
  aoi.on_off         = true;

  position.left = left;
  position.top  = top;

  size.width  = width;
  size.height = height;

  dc1394error_t err;

  uint32_t value = 0;
  memcpy( (void*) &value, (void*) &aoi, sizeof( value ) );
  
  err = dc1394_set_adv_control_register( _camera,
					 AVT_AUTOFNC_AOI_REGISTER,
					 value );

  if ( err != DC1394_SUCCESS )
  {
    throw Exception( "Pike::set_autofunction_aoi; dc1394_set_register(AVT_AUTOFNC_AOI_REGISTER) failed\n" );
  }
  
  memcpy( (void*) &value, (void*) &position, sizeof( value ) );
  err = dc1394_set_adv_control_register( _camera,
					 AVT_AF_AREA_POSITION_REGISTER,
					 value );

  if ( err != DC1394_SUCCESS )
  {
    throw Exception( "Pike::set_autofunction_aoi; dc1394_set_register(AVT_AF_AREA_POSITION_REGISTER) failed\n" );
  }

  memcpy( (void*) &value, (void*) &size, sizeof( value ) );
  err = dc1394_set_adv_control_register( _camera,
					 AVT_AF_AREA_SIZE_REGISTER,
					 value );

  if ( err != DC1394_SUCCESS )
  {
    throw Exception( "Pike::set_autofunction_aoi; dc1394_set_register(AVT_AF_AREA_SIZE_REGISTER) failed\n" );
  }

  err = dc1394_get_adv_control_register( _camera,
					 AVT_AUTOFNC_AOI_REGISTER,
					 &value );
  if ( err != DC1394_SUCCESS )
  {
    throw Exception( "Pike::set_autofunction_aoi; dc1394_get_register(AVT_AUTOFNC_AOI_REGISTER) failed\n" );
  }

  memcpy ( (void*) &aoi, (void*) &value, sizeof( value ) );

  return aoi.on_off;
}

/** Parse the autofnc_aoi parameter in the camera argument string.
 * The format ist \<left\>x\<top\>+\<width\>x\<height\>-\<show\>. "-\<show\>" is
 * optional.
 * @param aoi the parameter string of the autofnc_aoi parameter
 */
void
PikeCamera::parse_set_autofnc_aoi( const char* aoi )
{
  // format: left x top + width x height - show

  string a = aoi;
  
  string::size_type pos;

  pos = a.find( "x", 0 );
  if ( pos == string::npos )
  { throw Exception( "Illegal autofnc AOI parameter" ); }
  string left = a.substr( 0, pos );
  a = a.substr( pos + 1 );

  pos = a.find( "+", 0 );
  if ( pos == string::npos )
  { throw Exception( "Illegal autofnc AOI parameter" ); }
  string top = a.substr( 0, pos );
  a = a.substr( pos + 1 );

  pos = a.find( "x", 0 );
  if ( pos == string::npos )
  { throw Exception( "Illegal autofnc AOI parameter" ); }
  string width = a.substr( 0, pos );
  a = a.substr( pos + 1 );

  string height;
  string show;
  pos = a.find( "-", 0 );
  if ( pos == string::npos )
  {
    height = a;
    __aoi_show_work_area = false;
  }
  else
  {
    height = a.substr( 0, pos );
    show = a.substr( pos + 1 );

    __aoi_show_work_area = ( show == "show" ) ? true : false;
  }

  __aoi_left   = atoi( left.c_str() );
  __aoi_top    = atoi( top.c_str() );
  __aoi_width  = atoi( width.c_str() );
  __aoi_height = atoi( height.c_str() );
}

} // end namespace firevision
