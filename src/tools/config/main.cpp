
/***************************************************************************
 *  main.cpp - Fawkes config tool
 *
 *  Created: Mon Jan 08 16:43:45 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <netcomm/fawkes/client.h>
#include <config/netconf.h>
#include <utils/system/argparser.h>
#include <utils/system/signal.h>

#include <iostream>

/** Tool to watch and output config changes.
 */
class ConfigChangeWatcherTool : public ConfigurationChangeHandler, public SignalHandler
{
 public:

  /** Constructor.
   * @param config Configuration to watch
   * @param c network client, thread is cancelled on signal
   */
  ConfigChangeWatcherTool(Configuration *config, FawkesNetworkClient *c)
  {
    this->c = c;
    this->config = config;
    config->add_change_handler(this);
  }

  /** Handle signal.
   * @param signal signal
   */
  virtual void handle_signal(int signal)
  {
    config->rem_change_handler(this);
    c->cancel();
  }

  /** Configuration tag changed.
   * @param new_tag new tag
   */
  virtual void configTagChanged(const char *new_tag)
  {
    printf("--> New tag loaded: %s\n", new_tag);
  }

  /** Configuration value changed.
   * @param component component of value
   * @param path path of value
   * @param value new value
   */
  virtual void configValueChanged(const char *component, const char *path,
				  int value)
  {
    printf("%-12s| %-30s| %-8s| %-23i\n", component, path, "int", value);
  }

  /** Configuration value changed.
   * @param component component of value
   * @param path path of value
   * @param value new value
   */
  virtual void configValueChanged(const char *component, const char *path,
				  unsigned int value)
  {
    printf("%-12s| %-30s| %-8s| %-23u\n", component, path, "uint", value);
  }

  /** Configuration value changed.
   * @param component component of value
   * @param path path of value
   * @param value new value
   */
  virtual void configValueChanged(const char *component, const char *path,
				  float value)
  {
    printf("%-12s| %-30s| %-8s| %-23f\n", component, path, "float", value);
  }

  /** Configuration value changed.
   * @param component component of value
   * @param path path of value
   * @param value new value
   */
  virtual void configValueChanged(const char *component, const char *path,
				  bool value)
  {
    printf("%-12s| %-30s| %-8s| %-23s\n", component, path, "bool", (value ? "true" : "false"));
  }

  /** Configuration value changed.
   * @param component component of value
   * @param path path of value
   * @param value new value
   */
  virtual void configValueChanged(const char *component, const char *path,
				  std::string value)
  {
    printf("%-12s| %-30s| %-8s| %-23s\n", component, path, "string", value.c_str());
  }

  /** Configuration value erased.
   * @param component component of value
   * @param path path of value
   */
  virtual void configValueErased(const char *component, const char *path)
  {
    printf("%-12s| %-30s| %-8s| %-23s\n", component, path, "", "ERASED");
  }


  /** Component to monitor.
   * @return NULL, monitor all components.
   */
  virtual const char *  configMonitorComponent()
  {
    return NULL;
  }


  /** Run.
   * This joins the network thread.
   */
  void
  run()
  {
    c->join();
  }

 private:
  FawkesNetworkClient *c;
  Configuration *config;

};


/** Print header. */
void
print_header()
{
  printf("%-12s| %-30s| %-8s| %-23s\n", "Component", "Path", "Type", "Value");
  printf("------------------------------------------------------------------------------------\n");
}


/** Print a line of output.
 * @param i config item to print.
 */
void
print_line(Configuration::ValueIterator *i)
{
  if ( i->is_float() ) {
    printf("%-12s| %-30s| %-8s| %-23f\n", i->component(), i->path(), i->type(), i->get_float());
  } else if ( i->is_uint() ) {
    printf("%-12s| %-30s| %-8s| %-23u\n", i->component(), i->path(), i->type(), i->get_uint());
  } else if ( i->is_int() ) {
    printf("%-12s| %-30s| %-8s| %-23i\n", i->component(), i->path(), i->type(), i->get_int());
  } else if ( i->is_bool() ) {
    printf("%-12s| %-30s| %-8s| %-23s\n", i->component(), i->path(), i->type(), (i->get_bool() ? "true" : "false"));
  } else if ( i->is_string() ) {
    printf("%-12s| %-30s| %-8s| %-23s\n", i->component(), i->path(), i->type(), i->get_string().c_str());
  }
}


void
print_usage(const char *program_name)
{
  std::cout << "Usage: " << program_name << " <cmd>" << std::endl
	    << "where cmd is one of the following:" << std::endl << std::endl
	    << "  list" << std::endl
	    << "    List all configuration items" << std::endl << std::endl
	    << "  watch" << std::endl
	    << "    Watch configuration changes" << std::endl << std::endl
	    << "  get <comp> <path>" << std::endl
	    << "    Get value for the given component and path" << std::endl << std::endl
	    << "  set <comp> <path> <value> [type]" << std::endl
	    << "    Set value for the given component and path to the given type and value" << std::endl
	    << "    where type is one of float/uint/int/bool/string. The type" << std::endl
	    << "    is only necessary if you are creating a new value" << std::endl << std::endl
	    << "  set_default <comp> <path> <value> [type]" << std::endl
	    << "    Set default value for the given component and path to the given type and value" << std::endl
	    << "    where type is one of float/uint/int/bool/string. The type" << std::endl
	    << "    is only necessary if you are creating a new value" << std::endl << std::endl
	    << "  erase <comp> <path>" << std::endl
	    << "    Erase value for given component and path from config" << std::endl
	    << "  erase_default <comp> <path>" << std::endl
	    << "    Erase default value for given component and path from config" << std::endl
	    << std::endl;
}

/** Config tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "H");

  if ( argp.hasArgument("H") ) {
    print_usage(argv[0]);
    exit(0);
  }

  FawkesNetworkClient *c = new FawkesNetworkClient("localhost", 1910);
  c->connect();
  // Not needed, will actually harm the performance, especially on slow network
  // c->setNoDelay(true);
  c->start();

  NetworkConfiguration *netconf = new NetworkConfiguration(c);

  std::vector< char* > & args = argp.getItems();

  if ( args.size() == 0) {
    // show usage
    printf("Not enough args\n\n");
    print_usage(argv[0]);
  } else if (strcmp("get", args[0]) == 0) {
    if (args.size() == 3) {
      printf("Requesting value %s::%s\n", args[1], args[2]);
      Configuration::ValueIterator *i = netconf->get_value(args[1], args[2]);
      if ( i->next() ) {
	print_header();
	print_line(i);
      } else {
	printf("No such value found!\n");
      }
      delete i;
    } else {
      // Error!
      printf("You must supply component and path arguments\n");
    }
  } else if ((strcmp("set", args[0]) == 0) || (strcmp("set_default", args[0]) == 0)) {
    bool set_def = (strcmp("set_default", args[0]) == 0);
    if (args.size() >= 4) {
      // we have at least "set component path value"
      printf("Requesting old value for %s::%s\n", args[1], args[2]);
      Configuration::ValueIterator *i = netconf->get_value(args[1], args[2]);
      print_header();
      printf("OLD:\n");
      if ( i->next() ) {
	print_line(i);
      } else {
	printf("Value does not currently exist in configuration.\n");
      }

      std::string desired_type = "";
      if (args.size() == 5) {
	// we have "set component path value type"
	desired_type = args[4];
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
	  float f = strtod(args[3], &endptr);
	  if ( endptr[0] != 0 ) {
	    printf("ERROR: '%s' is not a float\n", args[3]);
	  } else {
	    if ( set_def ) {
	      netconf->set_float(args[1], args[2], f);
	    } else {
	      netconf->set_default_float(args[1], args[2], f);
	    }
	  }
	} else if ( desired_type == "uint" ) {
	  char *endptr;
	  long int i = strtol(args[3], &endptr, 10);
	  if ( (endptr[0] != 0) || (i < 0) ) {
	    printf("ERROR: '%s' is not an unsigned int\n", args[3]);
	  } else {
	    if ( set_def ) {
	      netconf->set_uint(args[1], args[2], i);
	    } else {
	      netconf->set_default_uint(args[1], args[2], i);
	    }
	  }
	} else if ( desired_type == "int" ) {
	  char *endptr;
	  long int i = strtol(args[3], &endptr, 10);
	  if ( endptr[0] != 0 ) {
	    printf("ERROR: '%s' is not an int\n", args[3]);
	  } else {
	    if ( set_def ) {
	      netconf->set_int(args[1], args[2], i);
	    } else {
	      netconf->set_default_int(args[1], args[2], i);
	    }
	  }
	} else if ( desired_type == "bool" ) {
	  bool valid = false;
	  bool b;
	  if ( strcasecmp("true", args[3]) == 0 ) {
	    b = true;
	    valid = true;
	  } else if ( strcasecmp("false", args[3]) == 0 ) {
	    b = false;
	    valid = true;
	  } else {
	    printf("ERROR: '%s' is not a boolean.\n", args[3]);
	  }
	  if (valid) {
	    if ( set_def ) {
	      netconf->set_bool(args[1], args[2], b);
	    } else {
	      netconf->set_default_bool(args[1], args[2], b);
	    }
	  }
	} else if ( desired_type == "string" ) {
	  if ( set_def ) {
	    netconf->set_string(args[1], args[2], args[3]);
	  } else {
	    netconf->set_default_string(args[1], args[2], args[3]);
	  }
	} else {
	  printf("Invalid type: %s\n", desired_type.c_str());
	}

	delete i;

	printf("NEW:\n");
	i = netconf->get_value(args[1], args[2]);
	if ( i->next() ) {
	  print_line(i);
	} else {
	  printf("ERROR: value does not exist\n");
	}
	delete i;

      }
    } else {
      printf("Usage: %s set <component> <path> <value> [type]\n", argp.getProgramName().c_str());
    }
  } else if ((strcmp("erase", args[0]) == 0) || (strcmp("erase_default", args[0]) == 0)) {
    bool erase_def = (strcmp("erase_default", args[0]) == 0);
    if (args.size() == 3) {
      printf("Erasing value %s::%s\n", args[1], args[2]);
      bool found = false;
      Configuration::ValueIterator *i = netconf->get_value(args[1], args[2]);
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
	  netconf->erase(args[1], args[2]);
	} else {
	  netconf->erase_default(args[1], args[2]);
	}
	i = netconf->get_value(args[1], args[2]);
	if ( i->next() ) {
	  printf("Failed to erase %s::%s (default vs. non-default?)\n", args[1], args[2]);
	} else {
	  printf("Successfully erased %s::%s\n", args[1], args[2]);
	}
	delete i;
      }
    } else {
      // Error!
      printf("You must supply component and path arguments\n");
    }
  } else if (strcmp("watch", args[0]) == 0) {
    try {
      netconf->setMirrorMode(true);
    } catch (Exception &e) {
      e.print_trace();
      return -1;
    }
    print_header();
    netconf->lock();
    Configuration::ValueIterator *i = netconf->iterator();
    while ( i->next() ) {
      print_line(i);
    }
    delete i;
    netconf->unlock();
    printf("------------------------------------------------------------------------------------\n");
    printf("Modifications since watching:\n");
    printf("------------------------------------------------------------------------------------\n");
    ConfigChangeWatcherTool ccwt(netconf, c);
    ccwt.run();
  } else if (strcmp("list", args[0]) == 0) {
    try {
      netconf->setMirrorMode(true);
    } catch (Exception &e) {
      e.print_trace();
      return -1;
    }
    print_header();
    netconf->lock();
    Configuration::ValueIterator *i = netconf->iterator();
    while ( i->next() ) {
      print_line(i);
    }
    delete i;
    netconf->unlock();
  }


  delete netconf;
  c->cancel();
  c->join();

  delete c;

  return 0;
}
