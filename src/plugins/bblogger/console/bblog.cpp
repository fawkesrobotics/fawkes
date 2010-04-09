
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

#include "../bblogfile.h"

#include <utils/system/argparser.h>
#include <utils/system/signal.h>
#include <utils/time/time.h>
#include <utils/system/fam.h>

#include <blackboard/remote.h>
#include <blackboard/internal/instance_factory.h>
#include <interfaces/SwitchInterface.h>

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
  printf("Usage: %s [-h] [-r host:port] <COMMAND> <logfile>\n"
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
	 " enable    Enable logging on a remotely running bblogger\n"
	 " disable   Disable logging on a remotely running bblogger\n"
	 " convert   Convert logfile to different format\n"
	 "           <infile>  input log file\n"
	 "           <outfile> converted output file\n"
	 "           <format>  format to convert to, currently supported:\n"
	 "             - csv  Comma-separated values\n",
	 program_name, program_name, program_name);
}


int
print_info(std::string &filename)
{
  try {
    BBLogFile bf(filename.c_str());
    bf.print_info();
    return 0;
  } catch (Exception &e) {
    printf("Failed to print info, exception follows\n");
    e.print_trace();
    return -1;
  }
}

int
repair_file(std::string &filename)
{
  try {
    BBLogFile::repair_file(filename.c_str());
    printf("Nothing to repair, files are fine\n");
    return 0;
  } catch (Exception &e) {
    if (strcmp(e.type_id(), "repair-success") == 0) {
      printf("Repair successful, actions done follow.\n");
      e.print_trace();
      return 0;
    } else {
      printf("Repair failed, exception follows.\n");
      e.print_trace();
      return -1;
    }
  }
}

int
print_indexes(std::string &filename, std::vector<unsigned int> &indexes)
{
  try {
    BBLogFile bf(filename.c_str());
    for (unsigned int i = 0; i < indexes.size(); ++i) {
      bf.read_index(indexes[i]);
      bf.print_entry();
    }
    return 0;
  } catch (Exception &e) {
    printf("Failed to print info, exception follows\n");
    e.print_trace();
    return -1;
  }

  return 0;  
}

int
replay_file(std::string &filename)
{
  try {
    BBLogFile bf(filename.c_str());

    Time last_offset((long)0);

    if (! bf.has_next()) {
      printf("File does not have any entries, aborting.\n");
      return -1;
    }

    // print out first immediately, the first offset, usually is a waiting
    // period until everything was started during logging
    bf.read_next();
    bf.print_entry();
    last_offset = bf.entry_offset();

    Time diff;
    while (bf.has_next()) {
      bf.read_next();
      diff = bf.entry_offset() - last_offset;
      diff.wait();
      last_offset = bf.entry_offset();
      bf.print_entry();
    }
    return 0;
  } catch (Exception &e) {
    printf("Failed to print info, exception follows\n");
    e.print_trace();
    return -1;
  }

  return 0;  
}
/// @cond INTERNAL

class BBLogWatcher
  : public FamListener,
    public SignalHandler
{
 public:
  BBLogWatcher(const char *filename, BBLogFile &file)
    : __file(file)
  {
    __quit = false;
    __fam = new FileAlterationMonitor();
    __fam->add_listener(this);
    __fam->watch_file(filename);
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
      unsigned int remaining = __file.remaining_entries();
      for (unsigned int i = 0; i < remaining; ++i) {
	__file.read_next();
	__file.print_entry();
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
  FileAlterationMonitor *__fam;
  BBLogFile &__file;
};

int
watch_file(std::string &filename)
{
  BBLogFile file(filename.c_str(), NULL, false);
  if (file.remaining_entries() > 0) {
    // jump to end of file
    file.read_index(file.remaining_entries() - 1);
  }
  BBLogWatcher watcher(filename.c_str(), file);
  watcher.run();

  return 0;  
}


int
set_enabled(const char *hostname, unsigned short int port, bool enabled)
{
  bool rv = 0;

  BlackBoard *bb = new RemoteBlackBoard(hostname, port);
  SwitchInterface *si = bb->open_for_reading<SwitchInterface>("BBLogger");
  if ( ! si->has_writer() ) {
    printf("No writer exists, BBLogger not loaded?\n");
    rv = -1;
  } else {
    if (enabled) {
      si->msgq_enqueue(new SwitchInterface::EnableSwitchMessage());
    } else {
      si->msgq_enqueue(new SwitchInterface::DisableSwitchMessage());
    }
  }

  bb->close(si);
  delete bb;
  return rv;
}

/// @endcond

void
convert_file_csv(BBLogFile &bf, FILE *outf)
{
  fawkes::Interface *iface = bf.interface();

  // print header row
  fprintf(outf, "# Time relative to beginning (in sec)");
  InterfaceFieldIterator i;
  for (i = iface->fields(); i != iface->fields_end(); ++i) {
    fprintf(outf, ";%s (%s[%zu])",
	    i.get_name(), i.get_typename(), i.get_length());
  }
  fprintf(outf, "\n");

  while (bf.has_next()) {
    bf.read_next();
    fprintf(outf, "%f", bf.entry_offset().in_sec());

    InterfaceFieldIterator i;
    for (i = iface->fields(); i != iface->fields_end(); ++i) {
      fprintf(outf, ";%s", i.get_value_string());
    }
    fprintf(outf, "\n");
  }
}


int
convert_file(std::string &infile, std::string &outfile, std::string &format)
{
  if (format != "csv") {
    printf("Unsupported output format '%s'\n", format.c_str());
    return 8;
  }

  FILE *outf = fopen(outfile.c_str(), "wx");
  if (!outf) {
    perror("Failed to open output file");
    return 3;
  }

  try {
    BBLogFile bf(infile.c_str());

    // Do the conversion!
    if (format == "csv") {
      convert_file_csv(bf, outf);
    }

  } catch (Exception &e) {
    printf("Failed to convert log file: %s\n", e.what());
    e.print_trace();
    fclose(outf);
    return 4;
  }

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

  std::string command, file;
  if (argp.num_items() < 1) {
    printf("Invalid number of arguments\n");
    print_usage(argv[0]);
    exit(1);
  } else if (argp.num_items() >= 2) {
    file    = argp.items()[1];
  }

  command = argp.items()[0];

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

  } else if ( (command == "enable") || (command == "disable")) {
    char *host = strdup("localhost");
    unsigned short int port = 1910;
    if (argp.has_arg("r")) {
      argp.parse_hostport("r", &host, &port);
    }
    int rv = set_enabled(host, port, (command == "enable"));
    free(host);
    return rv;

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
