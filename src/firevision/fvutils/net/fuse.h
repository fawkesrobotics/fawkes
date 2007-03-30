
/***************************************************************************
 *  fuse.h - FireVision Remote Control Protocol
 *
 *  Generated: Mon Jan 09 15:47:58 2006
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_H_
#define __FIREVISION_FVUTILS_NET_FUSE_H_

#include <fvutils/color/colorspaces.h>

#define FUSE_ERROR_MSG_LENGTH 100

/* Present this e-a-s-t-e-r e-g-g to Tim and get one package of Maoam! */

/** FUSE version enum. */
typedef enum {
  FUSE_VERSION_1 = 1,	/**< Version 1 */
  FUSE_VERSION_2 = 2	/**< Version 2 - current */
} FUSE_version_t;

/** FUSE packet types */
typedef enum {
  FUSE_PT_INVALID             = 0xFFFFFFFF,	/**< invalid message, not send over the net, just as internal return code */

  /* bi-directional packages,  1-999 */
  FUSE_PT_VERSION             = 1,		/**< version */
  FUSE_PT_RESULT              = 2,		/**< result */
  FUSE_PT_MESSAGE             = 3,		/**< message */
  FUSE_PT_DISCONNECT          = 4,		/**< disconnect */

  /* server to client, 1000-1999 */
  FUSE_PT_IMAGE               = 1000,		/**< image */
  FUSE_PT_LUT                 = 1001,		/**< lookup table */
  FUSE_PT_IMAGE_INFO          = 1002,		/**< image info */

  /* client to server, 2000-2999 */
  FUSE_PT_REQIMAGE            = 2000,		/**< request image */
  FUSE_PT_GETLUT              = 2001,		/**< get lookup table */
  FUSE_PT_SETLUT              = 2002,		/**< set lookup table */
  FUSE_PT_GET_IMAGE_INFO      = 2003,		/**< get image info */
  FUSE_PT_MESSAGE_SUBSCRIBE   = 2004,		/**< subscribe to message queue */
  FUSE_PT_MESSAGE_UNSUBSCRIBE = 2005,		/**< unsubscribe from message queue */

} FUSE_packet_type_t;

#define FUSE_RESULT_SUCCESS                0
#define FUSE_RESULT_ERROR                  1
#define FUSE_RESULT_ERROR_IMAGE_UNAVAIL    2
#define FUSE_RESULT_ERROR_LUT_UNAVAIL      3
#define FUSE_RESULT_ERROR_ALREADY          4
#define FUSE_RESULT_ERROR_IMAGE_APPL_DEAD  5
#define FUSE_RESULT_ERROR_LUT_APPL_DEAD    6
#define FUSE_RESULT_ERROR_TIMEOUT          7

/** version packet, bi-directional */
typedef struct {
  unsigned int version;		/**< version from FUSE_version_t */
} FUSE_version_packet_t;

/** general packet header */
typedef struct {
  unsigned int packet_type;	/**< packet type from FUSE_packet_type_t */
} FUSE_header_t;

/** error packet, bi-directional */
typedef struct {
  unsigned int packet_type;	/**< packet type from FUSE_packet_type_t */
  char         error_message[FUSE_ERROR_MSG_LENGTH];	/**< error message (text) */
} FUSE_error_packet_t;

/** result for request, bi-directional */
typedef struct {
  unsigned int result;		/**< result code */
  char         extra[64];	/**< extra data specific to request */
} FUSE_result_packet_t;

/** message packet header, bi-directional */
typedef struct {
  unsigned int msgqid;		/**< message queue ID */
  unsigned int data_size;	/**< data size */
} FUSE_message_packet_header_t;


/** Lookup table packet header.
 * server to client: PT_LUT
 * client to server: PT_SETLUT
 */
typedef struct {
  unsigned int lut_id;		/**< LUT ID */
  unsigned int width;		/**< width of LUT */
  unsigned int height;		/**< height of LUT */
  unsigned int bytes_per_cell;	/**< bytes per cell */
} FUSE_lookuptable_packet_header_t;



/** Image packet header.
 * (server to client)
 */
typedef struct {
  unsigned int  image_num;		/**< image number */
  unsigned int  colorspace;		/**< color space */
  unsigned int  width;			/**< width */
  unsigned int  height;			/**< height */
  unsigned int  roi_x;			/**< ROI X coordinate */
  unsigned int  roi_y;			/**< ROI Y coordinate */
  unsigned int  roi_width;		/**< ROI width */
  unsigned int  roi_height;		/**< ROI height */
  // Circle relative to ROI
  int           circle_x;		/**< circle x coordinate */
  int           circle_y;		/**< circle y coordinate */
  unsigned int  circle_radius;		/**< circle radius */
  unsigned int  flag_circle_found :  1;	/**< circle found, 1 if found */
  unsigned int  flag_image_ready  :  1;	/**< image ready, 1 if ready */
  unsigned int  flag_reserved     : 30;	/**< reserved for future use */
} FUSE_image_packet_header_t;


/** Image info packet.
 * (server to client)
 */
typedef struct {
  unsigned int  image_num;		/**< image number */
  unsigned int  colorspace;		/**< color space */
  unsigned int  width;			/**< width */
  unsigned int  height;			/**< height */
} FUSE_image_info_packet_t;


/** Request image.
 * (client to server)
 */
typedef struct {
  unsigned int image_num;	/**< image number */
} FUSE_reqimage_packet_t;

/** Get image info request.
 * (client to server)
 */
typedef FUSE_reqimage_packet_t FUSE_get_image_info_packet_t;

/** Get LUT request.
 * (client to server)
 */
typedef struct {
  unsigned int lut_id;	/**< LUT ID */
} FUSE_getlut_packet_t;

/** Subcribe to message queue.
 * (client to server)
 */
typedef struct {
  unsigned int msgqid;	/**< message queue */
  long         mtype;	/**< message type */
  unsigned int data_size;	/**< maximum data size. */
} FUSE_message_subscribe_packet_t;

/** Unsubscribe from message queue.
 * (client to server)
 */
typedef struct {
  unsigned int msgqid;	/**< message queue ID */
  long         mtype;	/**< message type */
} FUSE_message_unsubscribe_packet_t;


/** Message from message queue. */
typedef struct {
  unsigned int msgqid;		/**< message queue ID */
  unsigned int msg_size;	/**< message size */
  char *msg;			/**< message data */
} FUSE_message_t;



/* helper */

/** Convert unsigned int to colorspace_t.
 * @param cspace unsigned int from packet
 * @return colorspace
 */
inline colorspace_t
fuse_ui2cs(unsigned int cspace)
{
  if (cspace < (unsigned int)COLORSPACE_N) {
    return (colorspace_t)cspace;
  } else {
    return CS_UNKNOWN;
  }
}


/** Convert colorspace_t to unsigned int.
 * @param cspace colorspace
 * @return unsigned int representation
 */
inline unsigned int
fuse_cs2ui(colorspace_t cspace)
{
  return (unsigned int)cspace;
}


#endif
