
/***************************************************************************
 *  rectfile.cpp - Rectification info file
 *
 *  Created: Wed Oct 31 11:48:07 2007
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

#include <fvutils/rectification/rectinfo.h>
#include <fvutils/rectification/rectfile.h>
#include <fvutils/rectification/rectinfo_block.h>
#include <fvutils/rectification/rectinfo_lut_block.h>

#include <core/exceptions/system.h>

#include <cstring>
#include <cstdio>
#include <errno.h>
#include <netinet/in.h>

/** @class RectificationInfoFile <fvutils/rectification/rectfile.h>
 * Rectification Info File.
 * This class provides access files that contain rectification info.
 * Currently it supports writing and reading of such data and supports
 * any number of rectificatoin info blocks (although this is limited
 * by the file format!).
 *
 * It follows the file format as defined in rectinfo.h. Files that are written
 * are always of the current version. The endianess is automatically set to the
 * current's system endianess.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cam_guid Camera globally unique identifier.
 * @param model String with the model name of the camera
 */
RectificationInfoFile::RectificationInfoFile(uint64_t cam_guid, const char *model)
{
  _header = new rectinfo_header_t;

  _cam_guid = cam_guid;
  _model = strdup(model);

  clear();
}


/** Constructor.
 * This constructor may only be used for reading files, as the GUID of the camera
 * is invalid for writing.
 */
RectificationInfoFile::RectificationInfoFile()
{
  _header = new rectinfo_header_t;

  _cam_guid = 0;
  _model = strdup("");

  clear();
}


/** Destructor. */
RectificationInfoFile::~RectificationInfoFile()
{
  for (ibi = info_blocks.begin(); ibi != info_blocks.end(); ++ibi) {
    delete *ibi;
  }
  info_blocks.clear();
  delete _header;  
  if (_model != NULL)  free(_model);
}


/** Get the version of the file.
 * @return version of the file (or the current supported version if no file was loaded)
 */
unsigned int
RectificationInfoFile::version()
{
  return _header->version;
}


/** Check if data is encoded as big endian.
 * @return true if data is encoded as big endian, false otherwise
 */
bool
RectificationInfoFile::is_big_endian()
{
  return (_header->endianess == 1);
}


/** Check if data is encoded as little endian.
 * @return true if data is encoded as little endian, false otherwise
 */
bool
RectificationInfoFile::is_little_endian()
{
  return (_header->endianess == 0);
}


/** Get the number of available info blocks.
 * @return number of available info blocks
 */
unsigned int
RectificationInfoFile::num_blocks()
{
  return _header->num_blocks;
}


/** Get the GUID of camera.
 * @return GUID of the camera this rectification info file belongs to.
 */
uint64_t
RectificationInfoFile::guid()
{
  return _header->guid;
}


/** Get the model of the camera.
 * @return string with the camera's model name
 */
const char *
RectificationInfoFile::model()
{
  return _model;
}


/** Add a rectification info block.
 * This instance takes over ownership of the rectinfo block. This means that the
 * object is automatically deleted if this instance is deleted.
 * @param block block to add
 */
void
RectificationInfoFile::add_rectinfo_block(RectificationInfoBlock *block)
{
  _header->num_blocks++;
  info_blocks.push_back(block);
}


/** Clear internal structures.
 * This resets the internal header. It will also delete all rectification information
 * blocks.
 */
void
RectificationInfoFile::clear()
{
  for (ibi = info_blocks.begin(); ibi != info_blocks.end(); ++ibi) {
    delete *ibi;
  }
  info_blocks.clear();

  memset(_header, 0, sizeof(rectinfo_header_t));

  _header->magic   = htons(FIREVISION_RECTINFO_MAGIC);
  _header->version = FIREVISION_RECTINFO_CURVER;
  _header->num_blocks = 0;
#if __BYTE_ORDER == __BIG_ENDIAN
  _header->endianess = 1;
#else
  _header->endianess = 0;
#endif
  _header->guid = _cam_guid;
}


/** Get all rectification info blocks.
 * @return reference to internal vector of rectinfo blocks.
 */
RectificationInfoFile::RectInfoBlockVector &
RectificationInfoFile::blocks()
{
  return info_blocks;
}


/** Write file.
 * @param file_name file to write to
 */
void
RectificationInfoFile::write(const char *file_name)
{
  FILE *f = fopen(file_name, "w");
  if ( f == NULL ) {
    throw CouldNotOpenFileException(file_name, errno, "Could not open rectlut file "
				                      "for writing");
  }

  // Just to be safe
  _header->num_blocks = info_blocks.size();
  strncpy(_header->camera_model, _model, FIREVISION_RECTINFO_CAMERA_MODEL_MAXLENGTH);

  if ( fwrite(_header, sizeof(rectinfo_header_t), 1, f) != 1 ) {
    fclose(f);
    throw FileWriteException(file_name, errno, "Writing rectlut header failed");
  }

  for (ibi = info_blocks.begin(); ibi != info_blocks.end(); ++ibi) {
    // write this info block
    if ( fwrite((*ibi)->block_memptr(), (*ibi)->block_size(), 1, f) != 1 ) {
      fclose(f);
      throw FileWriteException(file_name, errno, "Failed to write info block");
    }
  }

  fclose(f);
}


/** Read file.
 * @param file_name file to read from
 */
void
RectificationInfoFile::read(const char *file_name)
{
  FILE *f = fopen(file_name, "r");
  if ( f == NULL ) {
    throw CouldNotOpenFileException(file_name, errno, "Could not open rectlut file "
				                      "for reading");
  }

  clear();

  if ( fread(_header, sizeof(rectinfo_header_t), 1, f) != 1) {
    fclose(f);
    throw FileReadException(file_name, errno, "Reading rectlut header failed");
  }

  if ( _header->magic != htons(FIREVISION_RECTINFO_MAGIC) ) {
    fclose(f);
    throw Exception("Unknown magic in rectinfo file");
  }

  if ( _header->version != FIREVISION_RECTINFO_CURVER ) {
    fclose(f);
    throw Exception("Unsupported version of rectinfo file");
  }

  if ( _model != NULL )  free(_model);
  _model = strndup(_header->camera_model, FIREVISION_RECTINFO_CAMERA_MODEL_MAXLENGTH);

  if ( _header->endianess ==
#if __BYTE_ORDER == __BIG_ENDIAN
       0
#else
       1
#endif
       ) {
    fclose(f);
    throw Exception("Rectification information cannot be translated for endiannes by now");
  }

  for (uint8_t b = 0; b < _header->num_blocks && !feof(f); ++b) {
    rectinfo_block_header_t bh;
    if ( fread(&bh, sizeof(bh), 1, f) != 1 ) {
      fclose(f);
      throw FileReadException(file_name, errno,
			      "Could not read block info header while there should be one");
    }
    if ( bh.type == FIREVISION_RECTINFO_TYPE_LUT_16x16 ) {
      // read LUT
      void * chunk = malloc(sizeof(bh) + bh.size);
      memset(chunk, 0, sizeof(bh) + bh.size);
      memcpy(chunk, &bh, sizeof(bh));
      void *block_chunk = (char *)chunk + sizeof(bh);
      if ( fread(block_chunk, bh.size, 1, f) != 1 ) {
	fclose(f);
	free(chunk);
	throw FileReadException(file_name, errno, "RectLUT: short read");
      }
      RectificationLutInfoBlock *rlib = new RectificationLutInfoBlock(chunk, sizeof(bh) + bh.size);
      info_blocks.push_back(rlib);
    } else {
      fclose(f);
      throw Exception("Unsupported file format (unknown block type %u)", bh.type);
    }
  }

  fclose(f);
}
