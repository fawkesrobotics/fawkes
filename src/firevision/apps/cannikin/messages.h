
/***************************************************************************
 *  messages.h - Cannikin messages
 *
 *  Created: Mon Jun 25 16:29:30 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __FIREVISION_APPS_CANNIKIN_MESSAGES_H_
#define __FIREVISION_APPS_CANNIKIN_MESSAGES_H_

#include <stdint.h>

#define CANNIKIN_MTYPE_GET_STEREOPARAMS  1
#define CANNIKIN_MTYPE_STEREOPARAMS      2
#define CANNIKIN_MTYPE_SET_STEREOPARAMS  3

/** Cannikin stereo parameters message struct. */
typedef struct
{
  unsigned int long mtype;			/**< message type, as required by IPCMessageQueue */
  uint8_t  min_disparity;			/**< minimum disparity */
  uint8_t  max_disparity;			/**< maximum disparity */
  uint8_t  min_disparity_mapping;		/**< minimum disparity mapping */
  uint8_t  max_disparity_mapping;		/**< maximum disparity mapping */
  uint8_t  edge_masksize;			/**< edge mask size */
  uint8_t  stereo_masksize;			/**< stereo mask size */
  uint16_t flag_disparity_mapping      :  1;	/**< 1 if disparity mapping is enabled, 0 otherwise */
  uint16_t flag_surface_validation     :  1;	/**< 1 if surface validation is enabled, 0 otherwise */
  uint16_t flag_texture_validation     :  1;	/**< 1 if texture validation is enabled, 0 otherwise */
  uint16_t flag_lowpass_filter         :  1;	/**< 1 if lowpass filtering is enabled, 0 otherwise */
  uint16_t flag_subpixel_interpolation :  1;	/**< 1 if subpixel interpolation is enabled, 0 otherwise */
  uint16_t flags_reserved              : 11;	/**< reserved for future use */
} cannikin_stereo_params_t;

#endif
