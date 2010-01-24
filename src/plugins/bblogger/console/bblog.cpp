
/***************************************************************************
 *  bblog.cpp - BBLogger console tool
 *
 *  Created: Thu Jan 21 01:33:45 2010
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

#include "../file.h"

#include <utils/system/argparser.h>
#include <utils/system/signal.h>
#include <utils/time/time.h>
#include <utils/system/fam.h>

#include <blackboard/internal/instance_factory.h>

#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/mman.h>

using namespace fawkes;

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-h] <watch|info|replay|repair> <logfile>\n"
	 "       %s print <logfile> <index> [index ...]\n"
	 "       %s convert <infile> <outfile> <format>\n"
	 "\n"
         " -h  Print this usage information\n"
         "COMMANDS:\n"
         " watch     Continuously watch a log file (like tail)\n"
         " info      Print meta information of log file\n"
         " print     Print specific data index\n"
	 "           <index> [index ...] is a list of indices to print\n"
         " replay    Replay log file in real-time to console\n"
	 " repair    Repair file, i.e. properly set number of entries\n"
	 " convert   Convert logfile to different format\n"
	 "           <infile>  input log file\n"
	 "           <outfile> converted output file\n"
	 "           <format>  format to convert to, currently supported:\n"
	 "             - csv  Comma-separated values\n",
	 program_name, program_name, program_name);
}


void
read_file_header(FILE *f, bblog_file_header *header)
{
  uint32_t version;
  if (fread(&version, sizeof(uint32_t), 1, f) == 1) {
    if (ntohl(version) == BBLOGGER_FILE_VERSION) {
      rewind(f);
      if (fread(header, sizeof(bblog_file_header), 1, f) != 1) {
	printf("Failed to read file header\n");
	throw Exception(errno, "Failed to read file header");
      }
    } else {
      printf("File version %u does not match supported version %u",
	     version, BBLOGGER_FILE_VERSION);
      throw Exception("File version %u does not match supported version %u",
		      version, BBLOGGER_FILE_VERSION);
    }
  } else {
    perror("Failed to read version number from file");
    throw Exception(errno, "Failed to read version number from file");
  }
}


void
print_header(FILE *f, bblog_file_header *header, const char *line_prefix = "", 
	     FILE *outf = stdout)
{
  char scenario[BBLOG_SCENARIO_SIZE + 1];
  char interface_type[BBLOG_INTERFACE_TYPE_SIZE + 1];
  char interface_id[BBLOG_INTERFACE_ID_SIZE + 1];
  char interface_hash[BBLOG_INTERFACE_HASH_SIZE * 2 + 1];

  strncpy(scenario, header->scenario, BBLOG_SCENARIO_SIZE);
  strncpy(interface_type, header->interface_type, BBLOG_INTERFACE_TYPE_SIZE);
  strncpy(interface_id, header->interface_id, BBLOG_INTERFACE_ID_SIZE);

  for (unsigned int i = 0; i < BBLOG_INTERFACE_HASH_SIZE; ++i) {
    snprintf(&interface_hash[i*2], 3, "%02X", header->interface_hash[i]);
  }

  Time t(header->start_time_sec, header->start_time_usec);

  struct stat fs;
  if (fstat(fileno(f), &fs) != 0) {
    throw Exception(errno, "Failed to get stat file");
  }

  fprintf(outf,
	  "%sFile version: %-10u  Endianess: %s Endian\n"
	  "%s# data items: %-10u  Data size: %u bytes\n"
	  "%sHeader size:  %zu bytes   File size: %zu bytes\n%s\n"
	  "%sScenario:   %s\n"
	  "%sInterface:  %s::%s (%s)\n"
	  "%sStart time: %s\n",
	  line_prefix, htonl(header->file_version),
	  (header->endianess == 1) ? "Big" : "Little",
	  line_prefix, header->num_data_items, header->data_size,
	  line_prefix, sizeof(bblog_file_header), fs.st_size, line_prefix,
	  line_prefix, scenario,
	  line_prefix, interface_type, interface_id, interface_hash,
	  line_prefix, t.str());

}

void
sanity_check(FILE *f, bblog_file_header *header)
{
  if (header->num_data_items == 0) {
    printf("\nWARNING: file does not specify number of data items. This usually\n"
	   "         happens if the logger was stopped unexpectedly or is still\n"
	   "         running. Either stop the logger, use repair command to fix\n"
	   "         the file.\n");
    throw Exception("file does not specify number of data items");
  }

  struct stat fs;
  if (fstat(fileno(f), &fs) != 0) {
    throw Exception(errno, "Failed to get stat file");
  }

  off_t expected_size = sizeof(bblog_file_header)
    + header->num_data_items * header->data_size
    + header->num_data_items * sizeof(bblog_entry_header);
  if (expected_size != fs.st_size) {
    printf("\nWARNING: file size does not match expectation. Expected %zu b,\n"
	   "         but file has %zu b. The logger might still be running.\n"
	   "         Otherwise use repair command to fix the file.\n",
	   expected_size, fs.st_size);
    throw Exception("file size does not match expectation");
  }

#if __BYTE_ORDER == __LITTLE_ENDIAN
  if (header->endianess == 1)
#else
  if (header->endianess == 0)
#endif
  {
    printf("\nWARNING: file has incompatible endianess.\n");
    throw Exception("file size does not match expectation");
  }
  
}

void
set_num_entries(FILE *f, size_t num_entries)
{
#if _POSIX_MAPPED_FILES
  void *h = mmap(NULL, sizeof(bblog_file_header), PROT_WRITE, MAP_SHARED,
		   fileno(f), 0);
  if (h == MAP_FAILED) {
    printf("ERROR: Failed to mmap log (%s), not updating number of data items",
	   strerror(errno));
    throw Exception(errno, "Failed to mmap log");
  } else {
    bblog_file_header *header = (bblog_file_header *)h;
    header->num_data_items = num_entries;
    munmap(h, sizeof(bblog_file_header));
  }
#else
  printf("ERROR: header set to 0 data items, cannot fix, mmap not available.\n");
#endif    
}


void
read_entry(FILE *f, bblog_file_header *header, bblog_entry_header *entryh,
	   Interface *iface, unsigned int index, bool do_seek = true)
{
  if (do_seek) {
    long offset = sizeof(bblog_file_header)
      + (sizeof(bblog_entry_header) + header->data_size) * index;

    if (fseek(f, offset, SEEK_SET) != 0) {
      throw Exception(errno, "Cannot seek to index %u", index);
    }
  }

  void *data = malloc(header->data_size);
  if ( (fread(entryh, sizeof(bblog_entry_header), 1, f) == 1) &&
       (fread(data, header->data_size, 1, f) == 1) ) {
    iface->set_from_chunk(data);
  } else {
    free(data);
    throw Exception("Cannot read interface data");
  }
  free(data);
}


void
print_entry(bblog_entry_header *entryh, Interface *iface)
{
  printf("Time Offset: %u.%u\n", entryh->rel_time_sec, entryh->rel_time_usec);

  for (InterfaceFieldIterator i = iface->fields(); i != iface->fields_end(); ++i) {
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
    printf("%-16s %-18s: %s\n",
	   i.get_name(), typesize, i.get_value_string());
    free(typesize);
  }
}


int
print_info(std::string &filename)
{
  FILE *f = fopen(filename.c_str(), "r");
  if (!f) {
    perror("Failed to open log file");
    return 3;
  }

  try {
    bblog_file_header header;
    read_file_header(f, &header);
    print_header(f, &header);
    sanity_check(f, &header);
  } catch (Exception &e) {
    //printf("Failed to read log file: %s\n", e.what());
    fclose(f);
    return 4;
  }

  fclose(f);

  return 0;
}


int
repair_file(std::string &filename)
{
  bool repair_done = false;

  FILE *f = fopen(filename.c_str(), "r+");
  if (!f) {
    perror("Failed to open log file");
    return 3;
  }

  bblog_file_header header;
  read_file_header(f, &header);

#if __BYTE_ORDER == __LITTLE_ENDIAN
  if (header.endianess == 1)
#else
  if (header.endianess == 0)
#endif
  {
    printf("Cannot repair files of incompatible endianess.\n");
    return 5;
  }

  struct stat fs;
  if (fstat(fileno(f), &fs) != 0) {
    throw Exception(errno, "Failed to get stat file");
  }

  size_t entry_size = sizeof(bblog_entry_header) + header.data_size;
  size_t all_entries_size = fs.st_size - sizeof(bblog_file_header);
  size_t num_entries = all_entries_size / entry_size;
  size_t extra_bytes = all_entries_size % entry_size;

  if (extra_bytes != 0) {
    printf("FIXING: errorneous bytes at end of file, truncating by %zu b\n",
	   extra_bytes);
    ftruncate(fileno(f), fs.st_size - extra_bytes);
    all_entries_size -= extra_bytes;
    extra_bytes = 0;
    if (fstat(fileno(f), &fs) != 0) {
      throw Exception(errno, "Failed to update file information after truncate");
    }
    repair_done = true;
  }
  if (header.num_data_items == 0) {
    printf("FIXING: header has 0 data items, setting to %zu.\n", num_entries);
    set_num_entries(f, num_entries);
    repair_done = true;
  } else if (header.num_data_items != num_entries) {
    printf("FIXING: header has %u data items, but expecting %zu, setting\n",
	   header.num_data_items, num_entries);
    set_num_entries(f, num_entries);
    repair_done = true;
  }

  fclose(f);

  if (!repair_done) {
    printf("File is fine, no repair needed\n");
  }

  return 0;
}


int
print_indexes(std::string &filename, std::vector<unsigned int> &indexes)
{
  FILE *f = fopen(filename.c_str(), "r");
  if (!f) {
    perror("Failed to open log file");
    return 3;
  }

  try {
    bblog_file_header header;
    read_file_header(f, &header);
    sanity_check(f, &header);

    char interface_type[BBLOG_INTERFACE_TYPE_SIZE + 1];
    char interface_id[BBLOG_INTERFACE_ID_SIZE + 1];

    strncpy(interface_type, header.interface_type, BBLOG_INTERFACE_TYPE_SIZE);
    strncpy(interface_id, header.interface_id, BBLOG_INTERFACE_ID_SIZE);

    BlackBoardInstanceFactory bbif;
    Interface *iface = bbif.new_interface_instance(interface_type, interface_id);

    if (memcmp(header.interface_hash, iface->hash(), BBLOG_INTERFACE_HASH_SIZE) != 0) {
      printf("Cannot read data. Hash mismatch between local interface and\n"
	     "log data.\n\n");
      bbif.delete_interface_instance(iface);
      throw Exception("Interface hash mismatch\n");
    }

    bblog_entry_header entryh;
    for (unsigned int i = 0; i < indexes.size(); ++i) {
      read_entry(f, &header, &entryh, iface, indexes[i]);
      print_entry(&entryh, iface);
    }

    bbif.delete_interface_instance(iface);

  } catch (Exception &e) {
    printf("Failed to read log file: %s\n", e.what());
    fclose(f);
    return 4;
  }

  fclose(f);

  return 0;  
}


int
replay_file(std::string &filename)
{
  FILE *f = fopen(filename.c_str(), "r");
  if (!f) {
    perror("Failed to open log file");
    return 3;
  }

  try {
    bblog_file_header header;
    read_file_header(f, &header);
    sanity_check(f, &header);

    char interface_type[BBLOG_INTERFACE_TYPE_SIZE + 1];
    char interface_id[BBLOG_INTERFACE_ID_SIZE + 1];

    strncpy(interface_type, header.interface_type, BBLOG_INTERFACE_TYPE_SIZE);
    strncpy(interface_id, header.interface_id, BBLOG_INTERFACE_ID_SIZE);

    BlackBoardInstanceFactory bbif;
    Interface *iface = bbif.new_interface_instance(interface_type, interface_id);

    if (memcmp(header.interface_hash, iface->hash(), BBLOG_INTERFACE_HASH_SIZE) != 0) {
      printf("Cannot read data. Hash mismatch between local interface and\n"
	     "log data.\n\n");
      bbif.delete_interface_instance(iface);
      throw Exception("Interface hash mismatch\n");
    }

    bblog_entry_header entryh;
    timeval last = {0, 0};
    for (unsigned int i = 0; i < header.num_data_items; ++i) {
      read_entry(f, &header, &entryh, iface, i);
      timeval next = {entryh.rel_time_sec, entryh.rel_time_usec};
      usleep(time_diff_usec(next, last));
      last = next;
      print_entry(&entryh, iface);
      printf("\n");
    }

    bbif.delete_interface_instance(iface);

  } catch (Exception &e) {
    printf("Failed to read log file: %s\n", e.what());
    fclose(f);
    return 4;
  }

  fclose(f);

  return 0;  
}

/// @cond INTERNAL

class BBLogWatcher
  : public FamListener,
    public SignalHandler
{
 public:
  BBLogWatcher(const char *filename, FILE *f,
	       bblog_file_header *header, Interface *iface)
  {
    __quit = false;
    __f = f;
    __header = header;
    __iface = iface;
    __fam = new FileAlterationMonitor();
    __fam->add_listener(this);
    __fam->watch_file(filename);
    fstat(fileno(f), &__stat);
    SignalManager::register_handler(SIGINT, this);
  }

  ~BBLogWatcher() {
    SignalManager::unregister_handler(this);
    __fam->remove_listener(this);
    delete __fam;
  }

  virtual void fam_event(const char *filename, unsigned int mask)
  {
    if (mask & FAM_DELETE) {
      __quit = true;
      __fam->interrupt();
    } else {
      struct stat s;
      fstat(fileno(__f), &s);
      size_t sizediff = s.st_size - __stat.st_size;
      if (sizediff < 0) {
	printf("ERROR: file shrank while reading, aborting.");
	__quit = true;
	__fam->interrupt();
      } else {
	size_t entry_size = sizeof(bblog_entry_header) + __header->data_size;
	unsigned int num_new_entries = sizediff / __header->data_size;
	size_t all_entries_size = s.st_size - sizeof(bblog_file_header);
	size_t extra_bytes = all_entries_size % entry_size;
	off_t offset_from_back = extra_bytes + num_new_entries * entry_size;
	if (fseek(__f, s.st_size - offset_from_back, SEEK_SET) == 0) {

	  for (unsigned int i = 0; i < num_new_entries; ++i) {
	    read_entry(__f, __header, &__entryh, __iface, 0, false);
	    print_entry(&__entryh, __iface);
	    printf("\n");
	  }
	  
	} else {
	  perror("Seek failed, not printing new data");
	}

	__stat = s;
      }
    }
  }

  virtual void handle_signal(int signal)
  {
    __quit = true;
    __fam->interrupt();
  }

  void run()
  {
    while (! __quit) {
      __fam->process_events(-1);
    }
  }


 private:
  bool __quit;
  FILE *__f;
  bblog_file_header *__header;
  Interface *__iface;
  FileAlterationMonitor *__fam;
  struct stat __stat;
  bblog_entry_header __entryh;
};

int
watch_file(std::string &filename)
{
  FILE *f = fopen(filename.c_str(), "r");
  if (!f) {
    perror("Failed to open log file");
    return 3;
  }

  try {
    bblog_file_header header;
    read_file_header(f, &header);

    char interface_type[BBLOG_INTERFACE_TYPE_SIZE + 1];
    char interface_id[BBLOG_INTERFACE_ID_SIZE + 1];

    strncpy(interface_type, header.interface_type, BBLOG_INTERFACE_TYPE_SIZE);
    strncpy(interface_id, header.interface_id, BBLOG_INTERFACE_ID_SIZE);

    BlackBoardInstanceFactory bbif;
    Interface *iface = bbif.new_interface_instance(interface_type, interface_id);

    if (memcmp(header.interface_hash, iface->hash(), BBLOG_INTERFACE_HASH_SIZE) != 0) {
      printf("Cannot read data. Hash mismatch between local interface and\n"
	     "log data.\n\n");
      bbif.delete_interface_instance(iface);
      throw Exception("Interface hash mismatch\n");
    }

    BBLogWatcher watcher(filename.c_str(), f, &header, iface);
    watcher.run();

    bbif.delete_interface_instance(iface);

  } catch (Exception &e) {
    printf("Failed to read log file: %s\n", e.what());
    fclose(f);
    return 4;
  }

  fclose(f);

  return 0;  
}

/// @endcond


void
convert_file_csv(FILE *inf, FILE *outf, bblog_file_header *header)
{
  char interface_type[BBLOG_INTERFACE_TYPE_SIZE + 1];
  char interface_id[BBLOG_INTERFACE_ID_SIZE + 1];

  strncpy(interface_type, header->interface_type, BBLOG_INTERFACE_TYPE_SIZE);
  strncpy(interface_id, header->interface_id, BBLOG_INTERFACE_ID_SIZE);

  BlackBoardInstanceFactory bbif;
  Interface *iface = bbif.new_interface_instance(interface_type, interface_id);

  if (memcmp(header->interface_hash, iface->hash(), BBLOG_INTERFACE_HASH_SIZE) != 0) {
    printf("Cannot read data. Hash mismatch between local interface and\n"
	   "log data.\n\n");
    bbif.delete_interface_instance(iface);
    throw Exception("Interface hash mismatch\n");
  }

  // print header row
  fprintf(outf, "# Time relative to beginning");
  InterfaceFieldIterator i;
  for (i = iface->fields(); i != iface->fields_end(); ++i) {
    fprintf(outf, ";%s (%s[%zu])",
	    i.get_name(), i.get_typename(), i.get_length());
  }
  fprintf(outf, "\n");

  bblog_entry_header entryh;
  for (unsigned int i = 0; i < header->num_data_items; ++i) {
    read_entry(inf, header, &entryh, iface, i);

    fprintf(outf, "%u.%u", entryh.rel_time_sec, entryh.rel_time_usec);

    InterfaceFieldIterator i;
    for (i = iface->fields(); i != iface->fields_end(); ++i) {
      fprintf(outf, ";%s", i.get_value_string());
    }
    fprintf(outf, "\n");
  }

  bbif.delete_interface_instance(iface);
}


int
convert_file(std::string &infile, std::string &outfile, std::string &format)
{
  if (format != "csv") {
    printf("Unsupported output format '%s'\n", format.c_str());
    return 8;
  }

  FILE *inf = fopen(infile.c_str(), "r");
  if (!inf) {
    perror("Failed to open log file");
    return 3;
  }
  FILE *outf = fopen(outfile.c_str(), "wx");
  if (!outf) {
    perror("Failed to open output file");
    return 3;
  }

  try {
    bblog_file_header header;
    read_file_header(inf, &header);
    sanity_check(inf, &header);
    print_header(inf, &header, "# ", outf);

    // Do the conversion!
    if (format == "csv") {
      convert_file_csv(inf, outf, &header);
    }

  } catch (Exception &e) {
    //printf("Failed to read log file: %s\n", e.what());
    fclose(inf);
    fclose(outf);
    return 4;
  }

  fclose(inf);
  fclose(outf);

  return 0;
}


/** BBLogger tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "h");

  if ( argp.has_arg("h") ) {
    print_usage(argv[0]);
    exit(0);
  }

  if (argp.num_items() < 2) {
    printf("Invalid number of arguments\n");
    print_usage(argv[0]);
    exit(1);
  }

  std::string command = argp.items()[0];
  std::string file    = argp.items()[1];

  if (command == "watch") {
    return watch_file(file);

  } else if (command == "info") {
    return print_info(file);

  } else if (command == "print") {
    std::vector<const char *> index_strings = argp.items();
    index_strings.erase(index_strings.begin(), index_strings.begin() + 2);

    std::vector<unsigned int> indexes(index_strings.size());
    for (unsigned int i = 0; i < index_strings.size(); ++i) {
      long l = atol(index_strings[i]);
      if (l < 0) throw Exception("Invalid index %li", l);

      indexes[i] = l;
    }

    if (indexes.size() == 0) {
      printf("No indexes given.\n\n");
      print_usage(argv[0]);
      exit(6);
    }

    return print_indexes(file, indexes);

  } else if (command == "replay") {
    return replay_file(file);

  } else if (command == "repair") {
    return repair_file(file);

  } else if (command == "convert") {
    if (argp.num_items() != 4) {
      printf("Invalid number of arguments\n");
      print_usage(argv[0]);
      exit(7);
    }
    std::string outfile = argp.items()[2];
    std::string format = argp.items()[3];
    return convert_file(file, outfile, format);

  } else {
    printf("Invalid command '%s'\n", command.c_str());
    print_usage(argv[0]);
    exit(2);
  }

  return 0;
}
