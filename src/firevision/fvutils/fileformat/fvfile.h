
/***************************************************************************
 *  fvfile.h - FireVision file
 *
 *  Created: Fri Mar 28 11:29:55 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_FILEFORMAT_FVFILE_H_
#define __FIREVISION_FVUTILS_FILEFORMAT_FVFILE_H_

#include <fvutils/fileformat/fvff.h>
#include <fvutils/fileformat/fvfile_block.h>
#include <cstdlib>
#include <list>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FireVisionDataFile
{
 public:
  FireVisionDataFile(unsigned short int magic_token, unsigned short int version);
  virtual ~FireVisionDataFile();

  unsigned int  magic_token();
  unsigned int  version();
  bool          is_big_endian();
  bool          is_little_endian();
  size_t        num_blocks();

  const char *  get_comment() const;
  void          set_comment(const char *comment);

  void          set_owns_blocks(bool owns_blocks);

  virtual void add_block(FireVisionDataFileBlock *block);
  virtual void clear();

  virtual void write(const char *file_name);
  virtual void read(const char *file_name);

  static unsigned short int read_magic_token(const char *filename);
  static bool has_magic_token(const char *filename, unsigned short int magic_token);

  /** List of FireVision data file blocks. */
  typedef std::list<FireVisionDataFileBlock *> BlockList;
  BlockList &  blocks();

 protected:
  void         *_spec_header;
  size_t        _spec_header_size;

 private:
  fvff_header_t       *__header;
  BlockList            __blocks;
  BlockList::iterator  __bi;

  unsigned int  __magic_token;
  unsigned int  __version;

  char *        __comment;

  bool          __owns_blocks;
};

} // end namespace firevision

#endif
