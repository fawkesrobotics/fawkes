
/***************************************************************************
 *  rectinfo.h - Rectification info file format
 *
 *  Created: Tue Oct 30 11:19:35 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_RECTIFICATION_RECTINFO_H_
#define __FIREVISION_FVUTILS_RECTIFICATION_RECTINFO_H_

#pragma pack(push,4)

#ifndef __STDC_LIMIT_MACROS
#define __STDC_LIMIT_MACROS
#endif
#include <stdint.h>

#define FIREVISION_RECTINFO_MAGIC  0xFF03
#define FIREVISION_RECTINFO_CURVER 2

#define FIREVISION_RECTINFO_CAMERA_MODEL_MAXLENGTH  32

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Header for a rectification information file (rectinfo).
 * The header defines the basic parameters needed to correctly interpret the
 * following rectification file data.
 *
 * It defines a content specific header for the FireVision data file format (fvff).
 *
 * The header defines a magic by which a rectinfo can be identified. This is
 * always FF03 (exactly in that order, no matter on the host systems endianess,
 * this has to be stored literally) for FireVision File Format 03. The version
 * is stored as a sequential number. This version has to be changed whenever either
 * the header or the file data format changes. The file defines the endianess of the
 * supplied data, which is important since the mapping in general has to be stored
 * at least to 2-byte-sized data fields. There are several reserved bits that may
 * be used later to store flags.
 *
 * The header also carries a globally unique ID of the camera. This allows for checking
 * if the file is used for the correct camera. This should be an EUI-64 number supplied
 * by the camera, for instance the IEEE1394 GUID. If that is not available for your
 * camera type use another distinguishing criterion like a serial number. If even that
 * cannot be queried from the camera make one up, for instance a checksum of the
 * robot name which carries the camera or even the (shortened) name itself.
 * The camera model is stored as a string and can also be used to discriminate a
 * specific camera. It can also be used for an easier identification of the camera
 * this file belongs to.
 *
 * Directly following this header the first rectification info is stored. Each info
 * has it's own per-info header defining the size of the info which can be read as
 * offset to the next info block (if there is one). This is followed by more reserved
 * bits. All reserved bits have to be set to zero.
 *
 * The general layout of the file is the following:
 * @code
 *   rectinfo_header_t        (file header, at least one block)
 *   rectinfo_block_header_t  (info block header, defining size S)
 *   [rectinfo_TYPE_block_header_t  (type-specific block header)
 *   <data> of size S - sizeof(type-specific-header).
 *   optional:
 *   rectinfo_block_header_t n
 *   <data> of block n
 * @endcode
 *
 * The first version supports only rectification lookup tables (rectlut, rectification LUT).
 * For this the block type is set to FIREVISION_RECTINFO_TYPE_LUT_16x16, because each
 * mapping consists of two uint16_t values.
 */
typedef struct _rectinfo_header_t {
  uint64_t guid;		/**< GUID of camera */
  char     camera_model[FIREVISION_RECTINFO_CAMERA_MODEL_MAXLENGTH]; /**< camera model */
} rectinfo_header_t;


/** The per-image rectification info block header.
 * A type can be given for the the following data. See rectinfo_block_type_t for the
 * possible types. The reserved bits may not be used and have to be set to zero.
 * There is also a total size of this info block in bytes. This has to include any
 * type specific header and all data stored in that block.
 * This maybe used for ignoring info blocks of unknown types and proceeding to the next
 * block (if there is one).
 * This header is usually followed by another block type specific header. This depends
 * on the type of data, see rectinfo_block_type_t.
 * A camera identifier is given to specify the image of the camera system. This is
 * necessary for instance if all rectificion info blocks of a stereo camera are named
 * in one file. The interpretation of this field depends on the used camera. Use the
 * constants defined by rectinfo_camera_t whenever possible. If that does not match your
 * situtation you may as well use custom IDs. The range [200:220] has been reserved
 * for this kind of IDs.
 */
typedef struct _rectinfo_block_header_t {
  uint32_t camera     :  8;	/**< camera, as specified per rectinfo_camera_t */
  uint32_t reserved   : 24;	/**< reserved for future use */
} rectinfo_block_header_t;


/** Block header for rectification LUTs wit 16-bit values.
 * The width and height of the rectification LUT is given. The LUT is assumed to be a
 * mapping of pixel coordinates in an image to coordinates in the unrectified image.
 * So following this header there have to be exactly width * height cells of
 * type rectinfo_lut_16x16_entry_t.
 * The rectification then works by iterating of the resulting image and the LUT at
 * the same time. For each pixel in the resulting image the pixel mentioned by the
 * coordinates in the LUT cell from the original image is copied.
 * The maximum LUT size and pixel coordinate values are 65535 (value that can be stored
 * in a 16 bit unsigned integer).
 */
typedef struct _rectinfo_lut_16x16_block_header_t {
  uint16_t width;		/**< width of the LUT file and image */
  uint16_t height;		/**< height of the LUT file and image */
} rectinfo_lut_16x16_block_header_t;

/** Data type used to build a rectification LUT.
 * The values are stored in the endianess of the host system.
 * The LUT has to be stored in memory line by line (height number of lines), each has
 * width number of reclut_lut_entry_t cells. Each cell represents one pixel in the rectified
 * image. The coordinates point to pixel coordinates in the unrectified image.
 * A simple rectification can thus iterate over the rectified image and the rectification
 * LUT and copy the pixel at the coordinates given by the LUT cell to the current
 * pixel of the rectified image.
 */
typedef struct _rectinfo_lut_16x16_entry_t {
  uint16_t x;	/**< map to x pixel coordinate */
  uint16_t y;	/**< map to y pixel coordinate */
} rectinfo_lut_16x16_entry_t;


/** Rectification info block type.
 * An info block may come in different types, probably mainly depending on the data type
 * but also the data structure may change in future versions.
 */
typedef enum _rectinfo_block_type_t {
  /* supported by file version 1: */
  FIREVISION_RECTINFO_TYPE_INVALID  = 0,	/**< invalid */
  FIREVISION_RECTINFO_TYPE_LUT_16x16  = 1	/**< Rectification LUT with 16 bit values,
						   see rectinfo_lut_16x16_block_header_t */
} rectinfo_block_type_t;


/** Rectification camera.
 * This describes the camera this info block belongs to. This is especially important
 * if rectification information of multiple images is stored for one camera, e.g. for
 * a stereo camera. The interpretation of this information heavily depends on the
 * used camera type. For single-lens cameras use main as the camera identifier.
 */
typedef enum _rectinfo_camera_t {
  /* supported by file version 1: */
  FIREVISION_RECTINFO_CAMERA_MAIN    = 0,	/**< Main image */
  FIREVISION_RECTINFO_CAMERA_LEFT    = 1,	/**< Left image */
  FIREVISION_RECTINFO_CAMERA_RIGHT   = 2,	/**< Right image */
  FIREVISION_RECTINFO_CAMERA_CENTER  = 3,	/**< Center image */
  FIREVISION_RECTINFO_CAMERA_TOP     = 4	/**< Top image */
} rectinfo_camera_t;

/** Rectification camera strings.
 * Follows the index in rectinfo_camera_t and gives a string for each of the
 * cameras.
 */
extern const char* rectinfo_camera_strings[];

extern const char* rectinfo_type_strings[];

} // end namespace firevision

#pragma pack(pop)
#endif
