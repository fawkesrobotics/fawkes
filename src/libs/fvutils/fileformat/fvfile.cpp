
/***************************************************************************
 *  fvfile.cpp - FireVision file
 *
 *  Created: Fri Mar 28 11:45:47 2008
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

#include <fvutils/fileformat/fvfile.h>

#include <core/exceptions/system.h>
#include <utils/misc/strndup.h>

#include <cstring>
#include <cstdio>
#include <cerrno>
#include <netinet/in.h>
#include <sys/time.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FireVisionDataFile <fvutils/fileformat/fvff.h>
 * FireVision File Format for data files.
 * The FireVision File Format (FVFF) defines a generic file layout that is used
 * to store large chunks of data on the the disk drive in a byte efficient way.
 *
 * It is meant to serve as a skeleton which is used by subclasses to implement
 * support for a concrete file format like colormaps or rectification information.
 * It allows for arbitrary meta data to be added that is relevant to the format and
 * it provides all the generic meta data that is needed to make the file format
 * work and that is common to all formats.
 *
 * Each format has a two byte magic token. In general it is of the form FFNN, where
 * FF stays literally (FireVision File) and NN is replaced with a number of the format.
 * Currently assigned format numbers include:
 * - FF01: colormaps
 * - FF02: generic lookup tables
 * - FF03: rectification information
 * - FF04: histograms
 *
 * We assume large chunks of data that is saved most efficiently in a proprietary
 * binary format that can be read and written quickly and mimics the layout of the
 * file in the memory.
 *
 * The general layout is:
 * @code
 *  1. General header (file type, version, endianess, number of blocks, etc.)
 *  2. Content type specific header (optional)
 *  3. Data blocks
 * @endcode
 * Each of the data blocks itself is of the following form:
 * @code
 *  1. General block header (type, size)
 *  2. Content type specific block header (optional)
 *  3. Data chunk (raw byte stream, content-specific)
 * @endcode
 *
 * @author Tim Niemueller
 */

/** @var void * FireVisionDataFile::_spec_header
 * Content specific header.
 * Create this buffer and set the size in _spec_header_size to get it written to
 * the file.
 */
/** @var size_t FireVisionDataFile::_spec_header_size
 * Size in bytes of _spec_header.
 */


/** Constructor.
 * @param magic_token magic token for the concrete file type
 * @param version file format version
 */
FireVisionDataFile::FireVisionDataFile(unsigned short int magic_token,
				       unsigned short int version)
{
  __header       = (fvff_header_t *)calloc(1, sizeof(fvff_header_t));

  __magic_token = magic_token;
  __version     = version;
  __comment     = strdup("");

  _spec_header = NULL;
  _spec_header_size = 0;

  __owns_blocks = true;

  clear();
}


/** Destructor. */
FireVisionDataFile::~FireVisionDataFile()
{
  clear();

  free(__header);
  free(__comment);
  if ( _spec_header ) {
    free(_spec_header);
  }
}


/** Clear internal storage.
 * All internal data is deleted.
 */
void
FireVisionDataFile::clear()
{
  if (__owns_blocks) {
    for (__bi = __blocks.begin(); __bi != __blocks.end(); ++__bi) {
      delete *__bi;
    }
  }

  __blocks.clear();
  memset(__header, 0, sizeof(fvff_header_t));

  __header->magic_token = htons(__magic_token);
  __header->version     = __version;
  __header->num_blocks = 0;
#if __BYTE_ORDER == __BIG_ENDIAN
  __header->endianess = 1;
#else
  __header->endianess = 0;
#endif
  free(__comment);
  __comment = strdup("");
}


/** Get the magic token of the file.
 * @return Magic token
 */
unsigned int
FireVisionDataFile::magic_token()
{
  return __header->magic_token;
}


/** Get the version of the file.
 * @return version of the file (or the current supported version if no file was loaded)
 */
unsigned int
FireVisionDataFile::version()
{
  return __header->version;
}


/** Check if data is encoded as big endian.
 * @return true if data is encoded as big endian, false otherwise
 */
bool
FireVisionDataFile::is_big_endian()
{
  return (__header->endianess == 1);
}


/** Check if data is encoded as little endian.
 * @return true if data is encoded as little endian, false otherwise
 */
bool
FireVisionDataFile::is_little_endian()
{
  return (__header->endianess == 0);
}


/** Get comment.
 * @return comment of the file
 */
const char *
FireVisionDataFile::get_comment() const
{
  return __comment;
}


/** Set comment.
 * @param comment new comment to set
 */
void
FireVisionDataFile::set_comment(const char *comment)
{
  free(__comment);
  __comment = strndup(comment, FVFF_COMMENT_SIZE);
  strncpy(__header->comment, comment, FVFF_COMMENT_SIZE-1);
}


/** Lets the file take over the ownership and give up the ownership of the blocks,
 * respectively. By default, the file is the owner of the blocks. If a file owns
 * the blocks they will be deleted in the files destructor.
 * @param owns_blocks if true file owns the blocks
 */
void
FireVisionDataFile::set_owns_blocks(bool owns_blocks)
{
  __owns_blocks = owns_blocks;
}


/** Get the number of available info blocks.
 * @return number of available info blocks
 */
size_t
FireVisionDataFile::num_blocks()
{
  return __blocks.size();
}


/** Add a block.
 * @param block block to add
 */
void
FireVisionDataFile::add_block(FireVisionDataFileBlock *block)
{
  __blocks.push_back(block);
}


/** Get blocks.
 * @return block list
 */
FireVisionDataFile::BlockList &
FireVisionDataFile::blocks()
{
  return __blocks;
}


/** Write file.
 * @param file_name file to write to
 */
void
FireVisionDataFile::write(const char *file_name)
{
  FILE *f = fopen(file_name, "w");
  if ( f == NULL ) {
    throw CouldNotOpenFileException(file_name, errno, "Could not open rectlut file "
				                      "for writing");
  }

  __header->num_blocks = (unsigned int)__blocks.size();
  timeval t;
  gettimeofday(&t, NULL);
  __header->created_sec  = t.tv_sec;
  __header->created_usec = t.tv_usec;
  __header->spec_head_size = _spec_header_size;

  //printf("Writing %zu bytes for header\n", sizeof(fvff_header_t));
  if ( fwrite(__header, sizeof(fvff_header_t), 1, f) != 1 ) {
    fclose(f);
    throw FileWriteException(file_name, errno, "Writing fvff header failed");
  }

  if ( _spec_header_size > 0 ) {
    //printf("Writing %zu bytes for spec header\n", _spec_header_size);
    if ( fwrite(_spec_header, _spec_header_size, 1, f) != 1 ) {
      fclose(f);
      throw FileWriteException(file_name, errno, "Writing content specific header failed");
    }
  }

  for (__bi = __blocks.begin(); __bi != __blocks.end(); ++__bi) {
    // write this info block
    //printf("Writing %zu bytes for block\n", (*__bi)->block_size());
    if ( fwrite((*__bi)->block_memptr(), (*__bi)->block_size(), 1, f) != 1 ) {
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
FireVisionDataFile::read(const char *file_name)
{
  FILE *f = fopen(file_name, "r");
  if ( f == NULL ) {
    throw CouldNotOpenFileException(file_name, errno, "Could not open rectlut file "
				                      "for reading");
  }

  clear();

  //printf("Reading %zu bytes for header\n", sizeof(fvff_header_t));
  if ( fread(__header, sizeof(fvff_header_t), 1, f) != 1) {
    fclose(f);
    throw FileReadException(file_name, errno, "Reading header failed");
  }

  if ( __header->magic_token != htons(__magic_token) ) {
    fclose(f);
    throw Exception("Unknown magic in fvff file (read: 0x%04x req: 0x%04x)",
		    __header->magic_token, __magic_token);
  }

  if ( __header->version != __version ) {
    fclose(f);
    throw Exception("Unsupported version of fvff file (read: %u req: %u)",
		    __header->version, __version);
  }

  if ( __header->endianess ==
#if __BYTE_ORDER == __BIG_ENDIAN
       0
#else
       1
#endif
       ) {
    fclose(f);
    throw Exception("FVFile header cannot be translated for endianess by now");
  }

  free(__comment);
  __comment = strndup(__header->comment, FVFF_COMMENT_SIZE);

  if ( _spec_header ) {
    free(_spec_header);
  }
  _spec_header = calloc(1, __header->spec_head_size);
  if ( ! _spec_header ) {
    throw OutOfMemoryException("Cannot allocate memory for content specific header");
  }

  if ( __header->spec_head_size > 0 ) {
    //printf("Reading %u bytes for spec header\n", __header->spec_head_size);
    if ( fread(_spec_header, __header->spec_head_size, 1, f) != 1) {
      fclose(f);
      throw FileReadException(file_name, errno, "Reading content specific header failed");
    }
  }

  //printf("Reading %u blocks\n", __header->num_blocks);
  for (unsigned int b = 0; b < __header->num_blocks && !feof(f); ++b) {
    fvff_block_header_t bh;
    //printf("Reading %zu bytes for block header\n", sizeof(bh));
    if ( fread(&bh, sizeof(bh), 1, f) != 1 ) {
      fclose(f);
      throw FileReadException(file_name, errno,
			      "Could not read block info header while there should be one");
    }
    void *spec_header = NULL;

    if ( bh.spec_head_size > 0 ) {
      // Read specific header
      spec_header = malloc(bh.spec_head_size);
      if ( ! spec_header ) {
	throw OutOfMemoryException("Could not allocate %u bytes for content specific header",
				   bh.spec_head_size);
      }

      //printf("Reading %u bytes for block spec header\n", bh.spec_head_size);
      if ( fread(spec_header, bh.spec_head_size, 1, f) != 1 ) {
	fclose(f);
	free(spec_header);
	throw FileReadException(file_name, errno,
				"Could not read content specific block header");
      }
    }

    FireVisionDataFileBlock *block = new FireVisionDataFileBlock(bh.type, bh.size,
								 spec_header, bh.spec_head_size);

    free(spec_header);

    //printf("Reading %u bytes for block data\n", bh.size);
    if ( bh.size && fread(block->data_ptr(), bh.size, 1, f) != 1 ) {
      fclose(f);
      delete block;
      throw FileReadException(file_name, errno,
			      "Could not read block data");
    }

    __blocks.push_back(block);
  }

  fclose(f);
}


/** Get magic token from file.
 * @param filename name of file to read the magic token from
 * @return magic token
 */
unsigned short int
FireVisionDataFile::read_magic_token(const char *filename)
{
  uint16_t magic_token = 0;

  FILE *f;
  f = fopen(filename, "r");
  if (f != NULL) {
    if ( fread((char *)&magic_token, sizeof(magic_token), 1, f) != 1 ) {
      fclose(f);
      throw FileReadException(filename, errno, "Could not read magic token from file");
    }
    fclose(f);
  } else {
    throw FileReadException(filename, errno,
                            "Could not read magic token from file");
  }

  return magic_token;
}


/** Check if file has a certain magic token.
 * @param filename name of file to read the magic token from
 * @param magic_token magic token to look for
 * @return true if magic token was  found, false otherwise
 */
bool
FireVisionDataFile::has_magic_token(const char *filename, unsigned short int magic_token)
{
  uint16_t file_magic_token = read_magic_token(filename);
  return (htons(magic_token) == file_magic_token);
}

} // end namespace firevision
