
/***************************************************************************
 *  bblogfile.cpp - BlackBoard log file access convenience class
 *
 *  Created: Sun Feb 21 11:27:41 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include "bblogfile.h"

#include <core/exceptions/system.h>
#include <utils/misc/strndup.h>
#include <blackboard/internal/instance_factory.h>

#include <cstdlib>
#include <cerrno>
#include <cstring>
#ifdef __FreeBSD__
#  include <sys/endian.h>
#elif defined(__MACH__) && defined(__APPLE__)
#  include <sys/_endian.h>
#else
#  include <endian.h>
#endif
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/mman.h>

using namespace fawkes;

/** @class BBLogFile "bblogfile.h"
 * Class to easily access bblogger log files.
 * This class provides an easy way to interact with bblogger log files.
 * @author Tim Niemueller
 */

/** Constructor.
 * Opens the given file and performs basic sanity checks.
 * @param filename log file to open
 * @param interface optional interface instance which must match the data
 * from the log file. Read methods will store read data in this interface
 * instance. If no interface is given an instance is created that is not
 * tied to a blackboard.
 * @param do_sanity_check true to perform a sanity check on the file on
 * opening. Turn this off only if you know what you are doing.
 * @exception CouldNotOpenFileException thrown if file cannot be opened
 * @exception FileReadException some error occured while reading data from
 */
BBLogFile::BBLogFile(const char *filename, fawkes::Interface *interface,
		     bool do_sanity_check)
{
  ctor(filename, do_sanity_check);

  if (interface) {
    __instance_factory = NULL;
    __interface = interface;
    if ((strcmp(__interface->type(), __interface_type) != 0) ||
	(strcmp(__interface->id(), __interface_id) != 0)) {
      fclose(__f);
      free(__filename);
      free(__scenario);
      free(__interface_type);
      free(__interface_id);
      throw Exception("Interface UID %s does not match expected %s:%s",
		      __interface->uid(), __interface_type, __interface_id);
    }
  } else {
    __instance_factory = new BlackBoardInstanceFactory();
    __interface = __instance_factory->new_interface_instance(__interface_type,
							     __interface_id);
  }
}


/** Constructor.
 * Opens the given file and performs basic sanity checks.
 * No internal interface is created. You must take care to set it using
 * set_interface() before any reading is done.
 * @param filename log file to open
 * @param do_sanity_check true to perform a sanity check on the file on
 * opening. Turn this off only if you know what you are doing.
 * @exception CouldNotOpenFileException thrown if file cannot be opened
 * @exception FileReadException some error occured while reading data from
 */
BBLogFile::BBLogFile(const char *filename, bool do_sanity_check)
{
  ctor(filename, do_sanity_check);

  __instance_factory = NULL;
  __interface        = NULL;
}


void
BBLogFile::ctor(const char *filename, bool do_sanity_check)
{
  __f = fopen(filename, "r");
  if (!__f) {
    throw CouldNotOpenFileException(filename, errno);
  }

  __filename = strdup(filename);
  __header   = (bblog_file_header *)malloc(sizeof(bblog_file_header));

  try {
    read_file_header();
    if (do_sanity_check)  sanity_check();
  } catch (Exception &e) {
    free(__filename);
    free(__scenario);
    free(__interface_type);
    free(__interface_id);
    fclose(__f);
    throw;
  }

  __ifdata = malloc(__header->data_size);
}

/** Destructor. */
BBLogFile::~BBLogFile()
{
  if (__instance_factory) {
    __instance_factory->delete_interface_instance(__interface);
    delete __instance_factory;
  }

  fclose(__f);

  free(__filename);
  free(__scenario);
  free(__interface_type);
  free(__interface_id);

  free(__header);
  free(__ifdata);
}


/** Read file header. */
void
BBLogFile::read_file_header()
{
  uint32_t magic;
  uint32_t version;
  if ((fread(&magic, sizeof(uint32_t), 1, __f) == 1) &&
      (fread(&version, sizeof(uint32_t), 1, __f) == 1) ) {
    if ( (ntohl(magic) == BBLOGGER_FILE_MAGIC) &&
	 (ntohl(version) == BBLOGGER_FILE_VERSION) ) {
      ::rewind(__f);
      if (fread(__header, sizeof(bblog_file_header), 1, __f) != 1) {
	throw FileReadException(__filename, errno, "Failed to read file header");
      }
    } else {
      throw Exception("File magic/version %X/%u does not match (expected %X/%u)",
		      ntohl(magic), ntohl(version),
		      BBLOGGER_FILE_VERSION, BBLOGGER_FILE_MAGIC);
    }
  } else {
    throw Exception(__filename, errno, "Failed to read magic/version from file");
  }

  __scenario = strndup(__header->scenario, BBLOG_SCENARIO_SIZE);
  __interface_type = strndup(__header->interface_type, BBLOG_INTERFACE_TYPE_SIZE);
  __interface_id = strndup(__header->interface_id, BBLOG_INTERFACE_ID_SIZE);

  __start_time.set_time(__header->start_time_sec, __header->start_time_usec);
}


/** Perform sanity checks.
 * This methods performs some sanity checks like:
 * - check if number of items is 0
 * - check if number of items and file size match
 * - check endianess of file and system
 * - check if file seems to be truncated
 */
void
BBLogFile::sanity_check()
{
  if (__header->num_data_items == 0) {
    Exception e("File %s does not specify number of data items", __filename);
    e.set_type_id("bblogfile-num-items-zero");
    throw e;
  }

  struct stat fs;
  if (fstat(fileno(__f), &fs) != 0) {
    Exception e(errno, "Failed to stat file %s", __filename);
    e.set_type_id("bblogfile-stat-failed");
    throw e;
  }

  long int expected_size = sizeof(bblog_file_header)
    + __header->num_data_items * __header->data_size
    + __header->num_data_items * sizeof(bblog_entry_header);
  if (expected_size != fs.st_size) {
    Exception e("Size of file %s does not match expectation "
		"(actual: %li, actual: %li)",
		__filename, expected_size, (long int)fs.st_size);
    e.set_type_id("bblogfile-file-size-mismatch");
    throw e;
  }

#if __BYTE_ORDER == __LITTLE_ENDIAN
  if (__header->endianess == 1)
#else
  if (__header->endianess == 0)
#endif
  {
    Exception e("File %s has incompatible endianess", __filename);
    e.set_type_id("bblogfile-endianess-mismatch");
    throw e;
  }
}

/** Read entry at particular index.
 * @param index index of entry, 0-based
 */
void
BBLogFile::read_index(unsigned int index)
{
  long offset = sizeof(bblog_file_header)
    + (sizeof(bblog_entry_header) + __header->data_size) * index;

  if (fseek(__f, offset, SEEK_SET) != 0) {
    throw Exception(errno, "Cannot seek to index %u", index);
  }

  read_next();
}


/** Rewind file to start.
 * This moves the file cursor immediately before the first entry.
 */
void
BBLogFile::rewind()
{
  if (fseek(__f, sizeof(bblog_file_header), SEEK_SET) != 0) {
    throw Exception(errno, "Cannot reset file");
  }
  __entry_offset.set_time(0, 0);
}


/** Check if another entry is available.
 * @return true if a consecutive read_next() will succeed, false otherwise
 */
bool
BBLogFile::has_next()
{
  // we always re-test to support continuous file watching
  clearerr(__f);
  if (getc(__f) == EOF) {
    return false;
  } else {
    fseek(__f, -1, SEEK_CUR);
    return true;
  }
}

/** Read next entry.
 * @exception Exception thrown if reading fails, for example because no more
 * entries are left.
 */
void
BBLogFile::read_next()
{
  bblog_entry_header entryh;

  if ( (fread(&entryh, sizeof(bblog_entry_header), 1, __f) == 1) &&
       (fread(__ifdata, __header->data_size, 1, __f) == 1) ) {
    __entry_offset.set_time(entryh.rel_time_sec, entryh.rel_time_usec);
    __interface->set_from_chunk(__ifdata);
  } else {
    throw Exception("Cannot read interface data");
  }
}


/** Set number of entries.
 * Set the number of entries in the file. Attention, this is only to be used
 * by the repair() method.
 * @param num_entries number of entries
 */
void
BBLogFile::set_num_entries(size_t num_entries)
{
#if _POSIX_MAPPED_FILES
  void *h = mmap(NULL, sizeof(bblog_file_header), PROT_WRITE, MAP_SHARED,
		   fileno(__f), 0);
  if (h == MAP_FAILED) {
    throw Exception(errno, "Failed to mmap log, not updating number of data items");
  } else {
    bblog_file_header *header = (bblog_file_header *)h;
    header->num_data_items = num_entries;
    munmap(h, sizeof(bblog_file_header));
  }
#else
  throw Exception("Cannot set number of entries, mmap not available.");
#endif    
}


/** Repair file.
 * @param filename file to repair
 * @see repair()
 */
void
BBLogFile::repair_file(const char *filename)
{
  BBLogFile file(filename, NULL, false);
  file.repair();
}


/** This tries to fix files which are in an inconsistent state.
 * On success, an exception is thrown with a type_id of "repair-success", which
 * will have an entry for each successful operation.
 */
void
BBLogFile::repair()
{
  FILE *f = freopen(__filename, "r+", __f);
  if (! f) {
    throw Exception("Reopening file %s with new mode failed", __filename);
  }
  __f = f;

  bool repair_done = false;

  Exception success("Successfully repaired file");
  success.set_type_id("repair-success");

#if __BYTE_ORDER == __LITTLE_ENDIAN
  if (__header->endianess == 1)
#else
  if (__header->endianess == 0)
#endif
  {
    throw Exception("File %s has incompatible endianess. Cannot repair.",
		    __filename);
  }

  struct stat fs;
  if (fstat(fileno(__f), &fs) != 0) {
    throw Exception(errno, "Failed to stat file %s", __filename);
  }

  size_t entry_size = sizeof(bblog_entry_header) + __header->data_size;
  size_t all_entries_size = fs.st_size - sizeof(bblog_file_header);
  size_t num_entries = all_entries_size / entry_size;
  size_t extra_bytes = all_entries_size % entry_size;

  if (extra_bytes != 0) {
    success.append("FIXING: errorneous bytes at end of file, "
		   "truncating by %zu b", extra_bytes);
    if (ftruncate(fileno(__f), fs.st_size - extra_bytes) == -1) {
      throw Exception(errno, "Failed to truncate file %s", __filename);
    }
    all_entries_size -= extra_bytes;
    extra_bytes = 0;
    if (fstat(fileno(__f), &fs) != 0) {
      throw Exception(errno, "Failed to update information of file %s "
		      "after truncate", __filename);
    }
    repair_done = true;
  }
  if (__header->num_data_items == 0) {
    success.append("FIXING: header of file %s has 0 data items, setting to %zu.",
		   __filename, num_entries);
    set_num_entries(num_entries);
    repair_done = true;
  } else if (__header->num_data_items != num_entries) {
    success.append("FIXING: header has %u data items, but expecting %zu, setting",
		   __header->num_data_items, num_entries);
    set_num_entries(num_entries);
    repair_done = true;
  }

  f = freopen(__filename, "r", __f);
  if (! f) {
    throw Exception("Reopening file %s with read-only mode failed", __filename);
  }
  __f = f;

  if (repair_done) {
    throw success;
  }
}


/** Print file meta info.
 * @param line_prefix a prefix printed before each line
 * @param outf file handle to print to
 */
void
BBLogFile::print_info(const char *line_prefix, FILE *outf)
{
  char interface_hash[BBLOG_INTERFACE_HASH_SIZE * 2 + 1];

  for (unsigned int i = 0; i < BBLOG_INTERFACE_HASH_SIZE; ++i) {
    snprintf(&interface_hash[i*2], 3, "%02X", __header->interface_hash[i]);
  }

  struct stat fs;
  if (fstat(fileno(__f), &fs) != 0) {
    throw Exception(errno, "Failed to get stat file");
  }

  fprintf(outf,
	  "%sFile version: %-10u  Endianess: %s Endian\n"
	  "%s# data items: %-10u  Data size: %u bytes\n"
	  "%sHeader size:  %zu bytes   File size: %li bytes\n"
	  "%s\n"
	  "%sScenario:   %s\n"
	  "%sInterface:  %s::%s (%s)\n"
	  "%sStart time: %s\n",
	  line_prefix, ntohl(__header->file_version),
	  (__header->endianess == 1) ? "Big" : "Little",
	  line_prefix, __header->num_data_items, __header->data_size,
	  line_prefix, sizeof(bblog_file_header), (long int)fs.st_size,
	  line_prefix,
	  line_prefix, __scenario,
	  line_prefix, __interface_type, __interface_id, interface_hash,
	  line_prefix, __start_time.str());

}


/** Print an entry.
 * Verbose print of a single entry.
 * @param outf file handle to print to
 */
void
BBLogFile::print_entry(FILE *outf)
{
  fprintf(outf, "Time Offset: %f\n", __entry_offset.in_sec());

  InterfaceFieldIterator i;
  for (i = __interface->fields(); i != __interface->fields_end(); ++i) {
    char *typesize;
    if (i.get_length() > 1) {
      if (asprintf(&typesize, "%s[%zu]", i.get_typename(), i.get_length()) == -1) {
	throw Exception("Out of memory");
      }
    } else {
      if (asprintf(&typesize, "%s", i.get_typename()) == -1) {
	throw Exception("Out of memory");
      }
    }
    fprintf(outf, "%-16s %-18s: %s\n",
	    i.get_name(), typesize, i.get_value_string());
    free(typesize);
  }
}


/** Get interface instance.
 * @return internally used interface
 */
fawkes::Interface *
BBLogFile::interface()
{
  return __interface;
}


/** Set the internal interface.
 * @param interface an interface matching the type and ID given in the
 * log file.
 */
void
BBLogFile::set_interface(fawkes::Interface *interface)
{
  if ( (strcmp(interface->type(), __interface_type) == 0) &&
       (strcmp(interface->id(), __interface_id) == 0) &&
       (memcmp(interface->hash(), __header->interface_hash,
	       __INTERFACE_HASH_SIZE) == 0) ) {
    if (__instance_factory) {
      __instance_factory->delete_interface_instance(__interface);
      delete __instance_factory;
      __instance_factory = NULL;
    }
    __interface = interface;
  } else {
    throw TypeMismatchException("Interfaces incompatible");
  }
}


/** Get current entry offset.
 * @return offset from start time of current entry (may be 0 if no entry has
 * been read, yet, or after rewind()).
 */
const fawkes::Time &
BBLogFile::entry_offset() const
{
  return __entry_offset;
}


/** Get file version.
 * @return file version
 */
uint32_t
BBLogFile::file_version() const
{
  return ntohl(__header->file_version);
}


/** Check if file is big endian.
 * @return true if file is big endian, false otherwise
 */
bool
BBLogFile::is_big_endian() const
{
  return (__header->endianess == 1);
}

/** Get number of data items in file.
 * @return number of data items
 */
uint32_t
BBLogFile::num_data_items() const
{
  return __header->num_data_items;
}


/** Get scenario identifier.
 * @return scenario identifier
 */
const char *
BBLogFile::scenario() const
{
  return __scenario;
}


/** Get interface type.
 * @return type of logged interface
 */
const char *
BBLogFile::interface_type() const
{
  return __interface_type;
}


/** Get interface ID.
 * @return ID of logged interface
 */
const char *
BBLogFile::interface_id() const
{
  return __interface_id;
}


/** Get interface hash.
 * Hash of logged interface.
 * @return interface hash
 */
unsigned char *
BBLogFile::interface_hash() const
{
  return __header->interface_hash;
}


/** Get data size.
 * @return size of the pure data part of the log entries
 */
uint32_t
BBLogFile::data_size()
{
  return __header->data_size;
}


/** Get start time.
 * @return starting time of log
 */
fawkes::Time &
BBLogFile::start_time()
{
  return __start_time;
}


/** Get number of remaining entries.
 * @return number of remaining entries
 */
unsigned int
BBLogFile::remaining_entries()
{
  // we make this so "complicated" to be able to use it from a FAM handler
  size_t entry_size = sizeof(bblog_entry_header) + __header->data_size;
  long   curpos     = ftell(__f);
  size_t fsize      = file_size();
  ssize_t sizediff  = fsize - curpos;

  if (sizediff < 0) {
    throw Exception("File %s shrank while reading it", __filename);
  }

  return sizediff / entry_size;
}

/** Get file size.
 * @return total size of log file including all headers
 */
size_t
BBLogFile::file_size() const
{
  struct stat fs;
  if (fstat(fileno(__f), &fs) != 0) {
    Exception e(errno, "Failed to stat file %s", __filename);
    e.set_type_id("bblogfile-stat-failed");
    throw e;
  }
  return fs.st_size;
}
