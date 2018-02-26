
/***************************************************************************
 *  rectfile.cpp - Rectification info file
 *
 *  Created: Wed Oct 31 11:48:07 2007
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

#include <fvutils/rectification/rectinfo.h>
#include <fvutils/rectification/rectfile.h>
#include <fvutils/rectification/rectinfo_block.h>
#include <fvutils/rectification/rectinfo_lut_block.h>

#include <core/exceptions/system.h>
#include <utils/misc/strndup.h>

#include <cstring>
#include <cstdio>
#include <errno.h>
#include <netinet/in.h>
#include <cstdlib>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

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
  : FireVisionDataFile(FIREVISION_RECTINFO_MAGIC, FIREVISION_RECTINFO_CURVER)
{
  _spec_header      = calloc(1, sizeof(rectinfo_header_t));
  _spec_header_size = sizeof(rectinfo_header_t);
  _header = (rectinfo_header_t *)_spec_header;

  _cam_guid = cam_guid;
  _model = strdup(model);

  strncpy(_header->camera_model, _model, FIREVISION_RECTINFO_CAMERA_MODEL_MAXLENGTH-1);
  _header->guid = _cam_guid;
}


/** Constructor.
 * This constructor may only be used for reading files, as the GUID of the camera
 * is invalid for writing.
 */
RectificationInfoFile::RectificationInfoFile()
  : FireVisionDataFile(FIREVISION_RECTINFO_MAGIC, FIREVISION_RECTINFO_CURVER)
{
  _spec_header      = calloc(1, sizeof(rectinfo_header_t));
  _spec_header_size = sizeof(rectinfo_header_t);
  _header = (rectinfo_header_t *)_spec_header;

  _cam_guid = 0;
  _model = strdup("");

  strncpy(_header->camera_model, _model, FIREVISION_RECTINFO_CAMERA_MODEL_MAXLENGTH-1);
  _header->guid = _cam_guid;
}


/** Destructor. */
RectificationInfoFile::~RectificationInfoFile()
{
  free(_model);
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
  add_block(block);
}


/** Get all rectification info blocks.
 * @return reference to internal vector of rectinfo blocks.
 */
RectificationInfoFile::RectInfoBlockVector *
RectificationInfoFile::rectinfo_blocks()
{
  FireVisionDataFile::BlockList &b = blocks();
  printf("Processing blocks: %zu\n", b.size());
  RectInfoBlockVector *rv = new RectInfoBlockVector();
  for (std::list<FireVisionDataFileBlock *>::iterator i = b.begin(); i != b.end(); ++i) {
    printf("Processing block\n");
    if ((*i)->type() == FIREVISION_RECTINFO_TYPE_LUT_16x16) {
      printf("Pushing lut block\n");
      RectificationLutInfoBlock *libl = new RectificationLutInfoBlock(*i);
      rv->push_back(libl);
    }
  }

  return rv;
}


void
RectificationInfoFile::read(const char *filename)
{
  FireVisionDataFile::read(filename);

  _header = (rectinfo_header_t *)_spec_header;

  if (_model) free(_model);
  _model    = strndup(_header->camera_model, FIREVISION_RECTINFO_CAMERA_MODEL_MAXLENGTH);
  _cam_guid = _header->guid;
}


RectificationInfoFile::RectInfoBlockVector::~RectInfoBlockVector()
{
  for (iterator i = begin(); i != end(); ++i) {
    delete *i;
  }
}

} // end namespace firevision
