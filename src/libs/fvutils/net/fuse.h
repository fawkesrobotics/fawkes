
/***************************************************************************
 *  fuse.h - FireVision Remote Control Protocol
 *
 *  Generated: Mon Jan 09 15:47:58 2006
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_H_
#define __FIREVISION_FVUTILS_NET_FUSE_H_

#include <fvutils/color/colorspaces.h>
#include <stdint.h>
#include <fvutils/ipc/defs.h>
#include <netcomm/utils/dynamic_buffer.h>

/* Present this e-a-s-t-e-r e-g-g to Tim and get one package of Maoam! */

#pragma pack(push,4)

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** FUSE version enum. */
typedef enum {
  FUSE_VERSION_1 = 1,	/**< Version 1 */
  FUSE_VERSION_2 = 2,	/**< Version 2 */
  FUSE_VERSION_3 = 3	/**< Version 3 - current */
} FUSE_version_t;

/** Current FUSE version */
#define FUSE_CURRENT_VERSION FUSE_VERSION_3

/** FUSE packet types */
typedef enum {
  /* bi-directional packages,  1-999 and 0xFFFFFFFE */
  FUSE_MT_GREETING            = 0xFFFFFFFE,	/**< version */

  /* server to client, 1000-1999 */
  FUSE_MT_IMAGE               = 1000,		/**< image */
  FUSE_MT_LUT                 = 1001,		/**< lookup table */
  FUSE_MT_IMAGE_LIST          = 1002,		/**< image list */
  FUSE_MT_LUT_LIST            = 1003,		/**< lut list */
  FUSE_MT_GET_IMAGE_FAILED    = 1004,		/**< Fetching an image failed */
  FUSE_MT_GET_LUT_FAILED      = 1005,		/**< Fetching a LUT failed */
  FUSE_MT_SET_LUT_SUCCEEDED   = 1006,		/**< Setting a LUT succeeded */
  FUSE_MT_SET_LUT_FAILED      = 1007,		/**< Setting a LUT failed */
  FUSE_MT_IMAGE_INFO          = 1008,		/**< image info */
  FUSE_MT_IMAGE_INFO_FAILED   = 1009,		/**< Retrieval of image info failed */

  /* client to server, 2000-2999 */
  FUSE_MT_GET_IMAGE           = 2000,		/**< request image */
  FUSE_MT_GET_LUT             = 2001,		/**< request lookup table */
  FUSE_MT_SET_LUT             = 2002,		/**< set lookup table */
  FUSE_MT_GET_IMAGE_LIST      = 2003,		/**< get image list */
  FUSE_MT_GET_LUT_LIST        = 2004,		/**< get LUT list */
  FUSE_MT_GET_IMAGE_INFO      = 2005,		/**< get image info */

} FUSE_message_type_t;

/** Image format. */
typedef enum {
  FUSE_IF_RAW      = 1,	/**< Raw image */
  FUSE_IF_JPEG     = 2	/**< JPEG image */
} FUSE_image_format_t;


/** general packet header */
typedef struct {
  uint32_t message_type; /**< packet type from FUSE_message_type_t */
  uint32_t payload_size; /**< payload size */
} FUSE_header_t;

/** FUSE message. */
typedef struct {
  FUSE_header_t  header;	/**< header */
  void *         payload;	/**< payload */
} FUSE_message_t;

/** version packet, bi-directional */
typedef struct {
  uint32_t version;	/**< version from FUSE_version_t */
} FUSE_greeting_message_t;

/** Lookup table packet header.
 * server to client: PT_LUT
 * client to server: PT_SETLUT
 */
typedef struct {
  char     lut_id[LUT_ID_MAX_LENGTH];	/**< LUT ID */
  uint32_t width;			/**< width of LUT */
  uint32_t height;			/**< height of LUT */
  uint32_t depth;			/**< depth of LUT */
  uint32_t bytes_per_cell;		/**< bytes per cell */
} FUSE_lut_message_header_t;



//  uint32_t  next_header;		/**< ID of next header. */
/** Image packet header.
 * (server to client)
 */
typedef struct {
  char      image_id[IMAGE_ID_MAX_LENGTH];	/**< image ID */
  uint32_t  format       :  8;			/**< Image format */
  uint32_t  colorspace   : 16;			/**< color space */
  uint32_t  reserved     :  8;			/**< reserved for future use */
  uint32_t  width;				/**< width in pixels */
  uint32_t  height;				/**< height in pixels */
  uint32_t  buffer_size;			/**< size of following image buffer in bytes */
  int64_t   capture_time_sec;			/**< capture time seconds part */
  int64_t   capture_time_usec;			/**< capture time microseconds part */
} FUSE_image_message_header_t;

/*
  uint32_t  roi_x;			*< ROI X coordinate 
  uint32_t  roi_y;			*< ROI Y coordinate 
  uint32_t  roi_width;		*< ROI width 
  uint32_t  roi_height;		*< ROI height 
  // Circle relative to ROI

  int32_t   circle_x;			*< circle x coordinate 
  int32_t   circle_y;			*< circle y coordinate 
  uint32_t  circle_radius;		*< circle radius 
  uint32_t  flag_circle_found :  1;	*< circle found, 1 if found 
  uint32_t  flag_reserved     : 31;	*< reserved for future use 
*/

/** Image request message. */
typedef struct {
  char image_id[IMAGE_ID_MAX_LENGTH];	/**< image ID */
  uint32_t format   : 8;		/**< requested image format, see FUSE_image_format_t */
  uint32_t reserved : 24;		/**< reserved for future use */
} FUSE_imagereq_message_t;


/** Image description message. */
typedef struct {
  char image_id[IMAGE_ID_MAX_LENGTH];	/**< image ID */
} FUSE_imagedesc_message_t;

/** LUT description message. */
typedef struct {
  char lut_id[LUT_ID_MAX_LENGTH];	/**< LUT ID */
} FUSE_lutdesc_message_t;

/** Image info message. */
typedef struct {
  char      image_id[IMAGE_ID_MAX_LENGTH];	/**< image ID */
  uint32_t  colorspace   : 16;			/**< color space */
  uint32_t  reserved     : 16;			/**< reserved for future use */
  uint32_t  width;				/**< width in pixels */
  uint32_t  height;				/**< height in pixels */
  uint32_t  buffer_size;			/**< size of following image buffer in bytes */
} FUSE_imageinfo_t;

/** LUT info message. */
typedef struct {
  char     lut_id[LUT_ID_MAX_LENGTH];	/**< LUT ID */
  uint32_t width;			/**< width of LUT */
  uint32_t height;			/**< height of LUT */
  uint32_t depth;			/**< depth of LUT */
  uint32_t bytes_per_cell;		/**< bytes per cell */
} FUSE_lutinfo_t;

/** Image list message. */
typedef struct {
  fawkes::dynamic_list_t image_list;	/**< DynamicBuffer holding a list of FUSE_imageinfo_t */
} FUSE_imagelist_message_t;

/** LUT list message. */
typedef struct {
  fawkes::dynamic_list_t lut_list;	/**< DynamicBuffer holding a list of FUSE_lutinfo_t */
} FUSE_lutlist_message_t;

} // end namespace firevision

#pragma pack(pop)
#endif
