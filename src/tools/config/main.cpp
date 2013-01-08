
/***************************************************************************
 *  main.cpp - Fawkes config tool
 *
 *  Created: Mon Jan 08 16:43:45 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/fawkes/client.h>
#include <config/netconf.h>
#include <config/change_handler.h>
#include <utils/system/argparser.h>
#include <utils/system/signal.h>

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cstdio>

using namespace fawkes;

/** Tool to watch and output config changes.
 */
class ConfigChangeWatcherTool
  : public ConfigurationChangeHandler, public SignalHandler
{
 public:

  /** Constructor.
   * @param config Configuration to watch
   * @param c network client, thread is cancelled on signal
   */
  ConfigChangeWatcherTool(Configuration *config, FawkesNetworkClient *c)
    : ConfigurationChangeHandler("")
  {
    this->c = c;
    this->config = config;
    quit = false;
    config->add_change_handler(this);
  }

  virtual void handle_signal(int signal)
  {
    config->rem_change_handler(this);
    quit = true;
  }

  virtual void config_tag_changed(const char *new_tag)
  {
    printf("--> New tag loaded: %s\n", new_tag);
  }

  virtual void config_value_changed(const Configuration::ValueIterator *v)
  {
    printf("%s %-55s| %-8s| %-14s\n", v->is_default() ? "*" : " ",
	   v->path(), v->type(), v->get_as_string().c_str());
  }

  virtual void config_comment_changed(const Configuration::ValueIterator *v)
  {
    printf("%s %s: %s\n", v->is_default() ? "C" : "c",
	   v->path(), v->get_comment().c_str());
  }

  virtual void config_value_erased(const char *path)
  {
    printf("  %-55s| %-8s| %-14s\n", path, "", "ERASED");
  }


  /** Run.
   * This joins the network thread.
   */
  void
  run()
  {
    while ( ! quit ) {
      c->wait(FAWKES_CID_CONFIGMANAGER);
    }
  }

 private:
  FawkesNetworkClient *c;
  Configuration *config;
  bool quit;

};


/** Print header. */
void
print_header()
{
  printf("D %-55s| %-8s| %-14s\n", "Path", "Type", "Value");
  printf("--------------------------------------------------------------------------------------\n");
}

/** Print a single value.
 * @param i config item to print.
 */
void
print_line(Configuration::ValueIterator *i, bool show_comment = false)
{
  if (i->is_list()) {
    printf("%s %-55s| %-8s| LIST (values below)\n", (i->is_default() ? "*" : " "),
	   i->path(), i->type());
    if ( i->is_uint() ) {
      std::vector<unsigned int> values = i->get_uints();
      for (size_t j = 0; j < values.size(); ++j) {
	printf("  %-67s%-14u\n", "", values[j]);
      }
    } else if ( i->is_int() ) {
      std::vector<int> values = i->get_ints();
      for (size_t j = 0; j < values.size(); ++j) {
	printf("  %-67s%-14i\n", "", values[j]);
      }
    } else if ( i->is_bool() ) {
      std::vector<bool> values = i->get_bools();
      for (size_t j = 0; j < values.size(); ++j) {
	printf("  %-67s%-14s\n", "", values[j] ? "true" : "false");
      }
    } else if ( i->is_float() ) {
      std::vector<float> values = i->get_floats();
      for (size_t j = 0; j < values.size(); ++j) {
	printf("  %-67s%-14f\n", "", values[j]);
      }
    } else if ( i->is_string() ) {
      std::vector<std::string> values = i->get_strings();
      for (size_t j = 0; j < values.size(); ++j) {
	printf("  %-67s%-14s\n", "", values[j].c_str());
      }
    } else {
      printf("%s %-55s| UNKNOWN LIST TYPE\n", (i->is_default() ? "*" : " "), i->path());
    }
  } else {
    if ( i->is_uint() ) {
      printf("%s %-55s| %-8s| %-14u\n", (i->is_default() ? "*" : " "), i->path(), "uint", i->get_uint());
    } else if ( i->is_int() ) {
      printf("%s %-55s| %-8s| %-14i\n", (i->is_default() ? "*" : " "), i->path(), i->type(), i->get_int());
    } else if ( i->is_bool() ) {
      printf("%s %-55s| %-8s| %-14s\n", (i->is_default() ? "*" : " "), i->path(), i->type(), (i->get_bool() ? "true" : "false"));
    } else if ( i->is_float() ) {
      printf("%s %-55s| %-8s| %-14f\n", (i->is_default() ? "*" : " "), i->path(), i->type(), i->get_float());
    } else if ( i->is_string() ) {
      printf("%s %-55s| %-8s| %-14s\n", (i->is_default() ? "*" : " "), i->path(), i->type(), i->get_string().c_str());
    } else {
      printf("%s %-55s| UNKNOWN TYPE\n", (i->is_default() ? "*" : " "), i->path());
    }

    if (show_comment) {
      try {
	std::string comment = i->get_comment();
	if (comment != "") {
	  printf("C %-55s: %s\n", i->path(), comment.c_str());
	}
      } catch (Exception &e) {
	// maybe there is no comment, ignore it...
      }
    }
  }
}

/** Print a line of output.
 * @param i config item to print.
 */
void
print_value(Configuration::ValueIterator *i, bool show_comment = false)
{
  if ( i->is_list()) {
    printf("%-14s\n", "LIST");
  } else {
    if ( i->is_uint() ) {
      printf("%-14u\n", i->get_uint());
    } else if ( i->is_int() ) {
      printf("%-14i\n", i->get_int());
    } else if ( i->is_bool() ) {
      printf("%-14s\n", (i->get_bool() ? "true" : "false"));
    } else if ( i->is_float() ) {
      printf("%-14f\n", i->get_float());
    } else if ( i->is_string() ) {
      printf("%-14s\n", i->get_string().c_str());
    }
  }
}


void
print_usage(const char *program_name)
{
  std::cout << "Usage: " << program_name << " [options] <cmd>" << std::endl
	    << "where cmd is one of the following:" << std::endl << std::endl
	    << "  list" << std::endl
	    << "    List all configuration items" << std::endl << std::endl
	    << "  watch" << std::endl
	    << "    Watch configuration changes" << std::endl << std::endl
	    << "  get <path>" << std::endl
	    << "    Get value for the given path" << std::endl << std::endl
	    << "  set <path> <value> [type]" << std::endl
	    << "    Set value for the given path to the given type and value" << std::endl
	    << "    where type is one of float/uint/int/bool/string. The type" << std::endl
	    << "    is only necessary if you are creating a new value" << std::endl << std::endl
	    << "  set_default <path> <value> [type]" << std::endl
	    << "    Set default value for the given path to the given type and value" << std::endl
	    << "    where type is one of float/uint/int/bool/string. The type" << std::endl
	    << "    is only necessary if you are creating a new value" << std::endl << std::endl
	    << "  set_comment <path> <comment>" << std::endl
	    << "    Set comment for the given path to the given value. The value at" << std::endl
	    << "    the given path must already exist in the host-specific configuration." << std::endl << std::endl
	    << "  set_default_comment <path> <comment>" << std::endl
	    << "    Set default comment for the given path to the given value. The value at" << std::endl
	    << "    the given path must already exist in the default configuration." << std::endl << std::endl
	    << "  erase <path>" << std::endl
	    << "    Erase value for given path from config" << std::endl
	    << "  erase_default <path>" << std::endl
	    << "    Erase default value for given path from config" << std::endl << std::endl
	    << "and options is none, one or more of the following:" << std::endl << std::endl
	    << "  -c   Show comments (only available with list and watch cmd)" << std::endl
	    << "  -a   Show all values, even double if default and host-specific " << std::endl
	    << "       values exist (only available with list)" << std::endl
	    << "  -q   Quiet. Only show important output, suitable for parsing. " << std::endl
	    << "       (not supported for all commands yet) " << std::endl
	    << "  -r host[:port]  Remote host (and optionally port) to connect to\n" << std::endl
	    << std::endl;
}

/** Config tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "+hcar:q");

  if ( argp.has_arg("h") ) {
    print_usage(argv[0]);
    exit(0);
  }

  std::string host = "localhost";
  unsigned short int port = 1910;
  if ( argp.has_arg("r") ) {
    argp.parse_hostport("r", host, port);
  }

  bool quiet;
  if ( argp.has_arg("q") ) {
    quiet = true;
  } else {
    quiet = false;
  }

  FawkesNetworkClient *c = new FawkesNetworkClient(host.c_str(), port);
  try {
    c->connect();
  } catch( Exception &e ) {
    printf("Could not connect to host: %s\n", host.c_str());
    exit(1);
  }

  NetworkConfiguration *netconf = new NetworkConfiguration(c);

  const std::vector< const char* > & args = argp.items();

  if ( args.size() == 0) {
    // show usage
    printf("Not enough args\n\n");
    print_usage(argv[0]);
  } else if (strcmp("get", args[0]) == 0) {
    if (args.size() == 2) {
      if( ! quiet ) {
	printf("Requesting value %s\n", args[1]);
      }
      Configuration::ValueIterator *i = netconf->get_value(args[1]);
      if ( i->next() ) {
	if( quiet ) {
	  print_value(i);
	} else {
	  print_header();
	  print_line(i);
	}
      } else {
	printf("No such value found!\n");
      }
      delete i;
    } else {
      // Error!
      printf("You must supply path argument\n");
    }
  } else if ((strcmp("set", args[0]) == 0) || (strcmp("set_default", args[0]) == 0)) {
    bool set_def = (strcmp("set_default", args[0]) == 0);
    if (args.size() >= 3) {
      // we have at least "set path value"
      printf("Requesting old value for %s\n", args[1]);
      Configuration::ValueIterator *i = netconf->get_value(args[1]);
      print_header();
      printf("OLD:\n");
      if ( i->next() ) {
	print_line(i);
      } else {
	printf("Value does not currently exist in configuration.\n");
      }

      std::string desired_type = "";
      if (args.size() == 4) {
	// we have "set path value type"
	desired_type = args[3];
      }

      if ( (desired_type == "") && ! i->valid()) {
	printf("Please specify type\n");
	delete i;
      } else if ( (desired_type != "") && (i->valid() && (desired_type != i->type())) ) {
	printf("The given type '%s' contradicts with type '%s' in config. "
	       "Erase before setting with new type.\n", desired_type.c_str(), i->type());
	delete i;
      } else {
	if ( i->valid() ) desired_type = i->type();

	if ( desired_type == "float" ) {
	  char *endptr;
	  float f = strtod(args[2], &endptr);
	  if ( endptr[0] != 0 ) {
	    printf("ERROR: '%s' is not a float\n", args[2]);
	  } else {
	    if ( ! set_def ) {
	      netconf->set_float(args[1], f);
	    } else {
	      netconf->set_default_float(args[1], f);
	    }
	  }
	} else if ( (desired_type == "unsigned int") || (desired_type == "uint") ) {
	  char *endptr;
	  long int li = strtol(args[2], &endptr, 10);
	  if ( (endptr[0] != 0) || (li < 0) ) {
	    printf("ERROR: '%s' is not an unsigned int\n", args[2]);
	  } else {
	    if ( ! set_def ) {
	      netconf->set_uint(args[1], li);
	    } else {
	      netconf->set_default_uint(args[1], li);
	    }
	  }
	} else if ( desired_type == "int" ) {
	  char *endptr;
	  long int li = strtol(args[2], &endptr, 10);
	  if ( endptr[0] != 0 ) {
	    printf("ERROR: '%s' is not an int\n", args[2]);
	  } else {
	    if ( ! set_def ) {
	      netconf->set_int(args[1], li);
	    } else {
	      netconf->set_default_int(args[1], li);
	    }
	  }
	} else if ( desired_type == "bool" ) {
	  bool valid = false;
	  bool b;
	  if ( strcasecmp("true", args[2]) == 0 ) {
	    b = true;
	    valid = true;
	  } else if ( strcasecmp("false", args[2]) == 0 ) {
	    b = false;
	    valid = true;
	  } else {
	    printf("ERROR: '%s' is not a boolean.\n", args[2]);
	  }
	  if (valid) {
	    if ( ! set_def ) {
	      netconf->set_bool(args[1], b);
	    } else {
	      netconf->set_default_bool(args[1], b);
	    }
	  }
	} else if ( desired_type == "string" ) {
	  if ( ! set_def ) {
	    netconf->set_string(args[1], args[2]);
	  } else {
	    netconf->set_default_string(args[1], args[2]);
	  }
	} else {
	  printf("Invalid type: %s\n", desired_type.c_str());
	}

	delete i;

	printf("NEW:\n");
	i = netconf->get_value(args[1]);
	if ( i->next() ) {
	  print_line(i);
	} else {
	  printf("ERROR: value does not exist\n");
	}
	delete i;

      }
    } else {
      printf("Usage: %s set <path> <value> [type]\n", argp.program_name());
    }
  } else if ((strcmp("set_comment", args[0]) == 0) ||
	     (strcmp("set_default_comment", args[0]) == 0)) {
    bool set_def = (strcmp("set_default_comment", args[0]) == 0);
    if (args.size() >= 3) {
      // we have at least "set_comment path value"

      if ( ! set_def ) {
	netconf->set_comment(args[1], args[2]);
      } else {
	netconf->set_default_comment(args[1], args[2]);
      }

    } else {
      printf("Usage: %s set_(default_)comment <path> <value>\n", argp.program_name());
    }
  } else if ((strcmp("erase", args[0]) == 0) || (strcmp("erase_default", args[0]) == 0)) {
    bool erase_def = (strcmp("erase_default", args[0]) == 0);
    if (args.size() == 2) {
      printf("Erasing %svalue %s\n", (erase_def ? "default " : ""), args[1]);
      bool found = false;
      Configuration::ValueIterator *i = netconf->get_value(args[1]);
      if ( i->next() ) {
	print_header();
	print_line(i);
	found = true;
      } else {
	printf("No such value found!\n");
      }
      delete i;
      if ( found ) {
	if ( erase_def ) {
	  netconf->erase_default(args[1]);
	} else {
	  netconf->erase(args[1]);
	}
	i = netconf->get_value(args[1]);
	if ( i->next() ) {
	  printf("Failed to erase %s (default vs. non-default?)\n", args[1]);
	} else {
	  printf("Successfully erased %s\n", args[1]);
	}
	delete i;
      }
    } else {
      // Error!
      printf("You must supply path argument\n");
    }
  } else if (strcmp("watch", args[0]) == 0) {
    try {
      netconf->set_mirror_mode(true);
    } catch (Exception &e) {
      e.print_trace();
      return -1;
    }
    print_header();
    netconf->lock();
    Configuration::ValueIterator *i = netconf->iterator();
    while ( i->next() ) {
      print_line(i, argp.has_arg("c"));
    }
    delete i;
    netconf->unlock();
    printf("------------------------------------------------------------------------------------\n");
    printf("Modifications since watching:\n");
    printf("------------------------------------------------------------------------------------\n");
    ConfigChangeWatcherTool ccwt(netconf, c);
    ccwt.run();
  } else if (strcmp("list", args[0]) == 0) {
    printf("Transmitting config from host... ");
    fflush(stdout);
    try {
      netconf->set_mirror_mode(true);
    } catch (Exception &e) {
      e.print_trace();
      return -1;
    }
    netconf->lock();
    printf("done\n");
    print_header();
    bool show_comments = argp.has_arg("c");
    if (argp.has_arg("a")) {
      printf("DEFAULT ENTRIES\n");
      Configuration::ValueIterator *i = netconf->iterator_default();
      while ( i->next() ) {
	print_line(i, show_comments);
      }
      delete i;
      printf("HOST-SPECIFIC ENTRIES\n");
      i = netconf->iterator_hostspecific();
      while ( i->next() ) {
	print_line(i, show_comments);
      }
      delete i;
    } else {
      Configuration::ValueIterator *i = netconf->iterator();
      while ( i->next() ) {
	print_line(i, show_comments);
      }
      delete i;
    }
    netconf->unlock();
  }

  if( ! quiet ) {
    printf("Cleaning up... ");
  }
  fflush(stdout);
  delete netconf;
  c->disconnect();

  delete c;
  if( ! quiet ) {
    printf("done\n");
  }

  return 0;
}
