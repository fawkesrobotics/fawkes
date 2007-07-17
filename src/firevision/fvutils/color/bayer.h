
/***************************************************************************
 *  bayer.h - Conversion methods for bayer mosaic images
 *
 *  Generated: Fri Aug 11 00:07:41 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_UTILS_BAYER_H_
#define __FIREVISION_UTILS_BAYER_H_

/** Bayer pattern enumeration.
 * This enumeration lists the differen possible bayer patterns.
 */
typedef enum {
  BAYER_PATTERN_YYYY = 0x59595959,	/**< YYYY pattern (no bayer) */
  BAYER_PATTERN_RGGB = 0x52474742,	/**< RGGB */
  BAYER_PATTERN_GBRG = 0x47425247,	/**< GBRG */
  BAYER_PATTERN_GRBG = 0x47524247,	/**< GRBG */
  BAYER_PATTERN_BGGR = 0x42474752	/**< BGGR */
} bayer_pattern_t;

void bayerGBRG_to_yuv422planar_nearest_neighbour(unsigned char *bayer, unsigned char *yuv,
						 unsigned int width, unsigned int height);

void bayerGBRG_to_yuv422planar_bilinear(unsigned char *bayer, unsigned char *yuv,
					unsigned int width, unsigned int height);
void bayerGBRG_to_yuv422planar_bilinear2(unsigned char *bayer, unsigned char *yuv,
					 unsigned int width, unsigned int height);

#endif
