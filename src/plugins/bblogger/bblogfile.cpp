
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
    interface_ = interface;
    if ((strcmp(interface_->type(), interface_type_) != 0) ||
        (strcmp(interface_->id(), interface_id_) != 0))
    {
      fclose(f_);
      free(filename_);
      free(scenario_);
      std::string interface_type(interface_type_);
      std::string interface_id(interface_id_);
      free(interface_type_);
      free(interface_id_);
      throw Exception("Interface UID %s does not match expected %s:%s",
                      interface_->uid(), interface_type.c_str(), interface_id.c_str());
    }
  } else {
	  instance_factory_.reset(new BlackBoardInstanceFactory());
    interface_ = instance_factory_->new_interface_instance(interface_type_,
                                                           interface_id_);
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

  interface_        = NULL;
}


void
BBLogFile::ctor(const char *filename, bool do_sanity_check)
{
  f_ = fopen(filename, "r");
  if (!f_) {
    throw CouldNotOpenFileException(filename, errno);
  }

  filename_ = strdup(filename);
  header_   = (bblog_file_header *)malloc(sizeof(bblog_file_header));

  try {
    read_file_header();
    if (do_sanity_check)  sanity_check();
  } catch (Exception &e) {
    free(filename_);
    free(scenario_);
    free(interface_type_);
    free(interface_id_);
    fclose(f_);
    throw;
  }

  ifdata_ = malloc(header_->data_size);
}

/** Destructor. */
BBLogFile::~BBLogFile()
{
  if (instance_factory_) {
    instance_factory_->delete_interface_instance(interface_);
    instance_factory_.reset();
  }

  fclose(f_);

  free(filename_);
  free(scenario_);
  free(interface_type_);
  free(interface_id_);

  free(header_);
  free(ifdata_);
}


/** Read file header. */
void
BBLogFile::read_file_header()
{
  uint32_t magic;
  uint32_t version;
  if ((fread(&magic, sizeof(uint32_t), 1, f_) == 1) &&
      (fread(&version, sizeof(uint32_t), 1, f_) == 1) ) {
    if ( (ntohl(magic) == BBLOGGER_FILE_MAGIC) &&
	 (ntohl(version) == BBLOGGER_FILE_VERSION) ) {
      ::rewind(f_);
      if (fread(header_, sizeof(bblog_file_header), 1, f_) != 1) {
	throw FileReadException(filename_, errno, "Failed to read file header");
      }
    } else {
      throw Exception("File magic/version %X/%u does not match (expected %X/%u)",
		      ntohl(magic), ntohl(version),
		      BBLOGGER_FILE_VERSION, BBLOGGER_FILE_MAGIC);
    }
  } else {
    throw Exception(filename_, errno, "Failed to read magic/version from file");
  }

  scenario_ = strndup(header_->scenario, BBLOG_SCENARIO_SIZE);
  interface_type_ = strndup(header_->interface_type, BBLOG_INTERFACE_TYPE_SIZE);
  interface_id_ = strndup(header_->interface_id, BBLOG_INTERFACE_ID_SIZE);

  start_time_.set_time(header_->start_time_sec, header_->start_time_usec);
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
  if (header_->num_data_items == 0) {
    Exception e("File %s does not specify number of data items", filename_);
    e.set_type_id("bblogfile-num-items-zero");
    throw e;
  }

  struct stat fs;
  if (fstat(fileno(f_), &fs) != 0) {
    Exception e(errno, "Failed to stat file %s", filename_);
    e.set_type_id("bblogfile-stat-failed");
    throw e;
  }

  long int expected_size = sizeof(bblog_file_header)
    + (size_t)header_->num_data_items * header_->data_size
    + (size_t)header_->num_data_items * sizeof(bblog_entry_header);
  if (expected_size != fs.st_size) {
    Exception e("Size of file %s does not match expectation "
		"(actual: %li, actual: %li)",
		filename_, expected_size, (long int)fs.st_size);
    e.set_type_id("bblogfile-file-size-mismatch");
    throw e;
  }

#if BYTE_ORDER_ == LITTLE_ENDIAN_
  if (header_->endianess == 1)
#else
  if (header_->endianess == 0)
#endif
  {
    Exception e("File %s has incompatible endianess", filename_);
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
    + (sizeof(bblog_entry_header) + header_->data_size) * index;

  if (fseek(f_, offset, SEEK_SET) != 0) {
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
  if (fseek(f_, sizeof(bblog_file_header), SEEK_SET) != 0) {
    throw Exception(errno, "Cannot reset file");
  }
  entry_offset_.set_time(0, 0);
}


/** Check if another entry is available.
 * @return true if a consecutive read_next() will succeed, false otherwise
 */
bool
BBLogFile::has_next()
{
  // we always re-test to support continuous file watching
  clearerr(f_);
  if (getc(f_) == EOF) {
    return false;
  } else {
    fseek(f_, -1, SEEK_CUR);
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

  if ( (fread(&entryh, sizeof(bblog_entry_header), 1, f_) == 1) &&
       (fread(ifdata_, header_->data_size, 1, f_) == 1) ) {
    entry_offset_.set_time(entryh.rel_time_sec, entryh.rel_time_usec);
    interface_->set_from_chunk(ifdata_);
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
		   fileno(f_), 0);
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
  FILE *f = freopen(filename_, "r+", f_);
  if (! f) {
    throw Exception("Reopening file %s with new mode failed", filename_);
  }
  f_ = f;

  bool repair_done = false;

  Exception success("Successfully repaired file");
  success.set_type_id("repair-success");

#if BYTE_ORDER_ == LITTLE_ENDIAN_
  if (header_->endianess == 1)
#else
  if (header_->endianess == 0)
#endif
  {
    throw Exception("File %s has incompatible endianess. Cannot repair.",
		    filename_);
  }

  struct stat fs;
  if (fstat(fileno(f_), &fs) != 0) {
    throw Exception(errno, "Failed to stat file %s", filename_);
  }

  size_t entry_size = sizeof(bblog_entry_header) + header_->data_size;
  size_t all_entries_size = fs.st_size - sizeof(bblog_file_header);
  size_t num_entries = all_entries_size / entry_size;
  size_t extra_bytes = all_entries_size % entry_size;

  if (extra_bytes != 0) {
    success.append("FIXING: errorneous bytes at end of file, "
		   "truncating by %zu b", extra_bytes);
    if (ftruncate(fileno(f_), fs.st_size - extra_bytes) == -1) {
      throw Exception(errno, "Failed to truncate file %s", filename_);
    }
    all_entries_size -= extra_bytes;
    extra_bytes = 0;
    if (fstat(fileno(f_), &fs) != 0) {
      throw Exception(errno, "Failed to update information of file %s "
		      "after truncate", filename_);
    }
    repair_done = true;
  }
  if (header_->num_data_items == 0) {
    success.append("FIXING: header of file %s has 0 data items, setting to %zu.",
		   filename_, num_entries);
    set_num_entries(num_entries);
    repair_done = true;
  } else if (header_->num_data_items != num_entries) {
    success.append("FIXING: header has %u data items, but expecting %zu, setting",
		   header_->num_data_items, num_entries);
    set_num_entries(num_entries);
    repair_done = true;
  }

  f = freopen(filename_, "r", f_);
  if (! f) {
    throw Exception("Reopening file %s with read-only mode failed", filename_);
  }
  f_ = f;

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
    snprintf(&interface_hash[i*2], 3, "%02X", header_->interface_hash[i]);
  }

  struct stat fs;
  if (fstat(fileno(f_), &fs) != 0) {
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
	  line_prefix, ntohl(header_->file_version),
	  (header_->endianess == 1) ? "Big" : "Little",
	  line_prefix, header_->num_data_items, header_->data_size,
	  line_prefix, sizeof(bblog_file_header), (long int)fs.st_size,
	  line_prefix,
	  line_prefix, scenario_,
	  line_prefix, interface_type_, interface_id_, interface_hash,
	  line_prefix, start_time_.str());

}


/** Print an entry.
 * Verbose print of a single entry.
 * @param outf file handle to print to
 */
void
BBLogFile::print_entry(FILE *outf)
{
  fprintf(outf, "Time Offset: %f\n", entry_offset_.in_sec());

  InterfaceFieldIterator i;
  for (i = interface_->fields(); i != interface_->fields_end(); ++i) {
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
  return interface_;
}


/** Set the internal interface.
 * @param interface an interface matching the type and ID given in the
 * log file.
 */
void
BBLogFile::set_interface(fawkes::Interface *interface)
{
  if ( (strcmp(interface->type(), interface_type_) == 0) &&
       (strcmp(interface->id(), interface_id_) == 0) &&
       (memcmp(interface->hash(), header_->interface_hash,
	       INTERFACE_HASH_SIZE_) == 0) ) {
    if (instance_factory_) {
      instance_factory_->delete_interface_instance(interface_);
      instance_factory_.reset();
    }
    interface_ = interface;
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
  return entry_offset_;
}


/** Get file version.
 * @return file version
 */
uint32_t
BBLogFile::file_version() const
{
  return ntohl(header_->file_version);
}


/** Check if file is big endian.
 * @return true if file is big endian, false otherwise
 */
bool
BBLogFile::is_big_endian() const
{
  return (header_->endianess == 1);
}

/** Get number of data items in file.
 * @return number of data items
 */
uint32_t
BBLogFile::num_data_items() const
{
  return header_->num_data_items;
}


/** Get scenario identifier.
 * @return scenario identifier
 */
const char *
BBLogFile::scenario() const
{
  return scenario_;
}


/** Get interface type.
 * @return type of logged interface
 */
const char *
BBLogFile::interface_type() const
{
  return interface_type_;
}


/** Get interface ID.
 * @return ID of logged interface
 */
const char *
BBLogFile::interface_id() const
{
  return interface_id_;
}


/** Get interface hash.
 * Hash of logged interface.
 * @return interface hash
 */
unsigned char *
BBLogFile::interface_hash() const
{
  return header_->interface_hash;
}


/** Get data size.
 * @return size of the pure data part of the log entries
 */
uint32_t
BBLogFile::data_size()
{
  return header_->data_size;
}


/** Get start time.
 * @return starting time of log
 */
fawkes::Time &
BBLogFile::start_time()
{
  return start_time_;
}


/** Get number of remaining entries.
 * @return number of remaining entries
 */
unsigned int
BBLogFile::remaining_entries()
{
  // we make this so "complicated" to be able to use it from a FAM handler
  size_t entry_size = sizeof(bblog_entry_header) + header_->data_size;
  long   curpos     = ftell(f_);
  size_t fsize      = file_size();
  ssize_t sizediff  = fsize - curpos;

  if (sizediff < 0) {
    throw Exception("File %s shrank while reading it", filename_);
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
  if (fstat(fileno(f_), &fs) != 0) {
    Exception e(errno, "Failed to stat file %s", filename_);
    e.set_type_id("bblogfile-stat-failed");
    throw e;
  }
  return fs.st_size;
}
