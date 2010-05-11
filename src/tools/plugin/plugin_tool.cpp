
/***************************************************************************
 *  plugin_tool.cpp - Fawkes plugin tool
 *
 *  Created: Mon Dec 04 14:43:23 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <tools/plugin/plugin_tool.h>

#include <netcomm/fawkes/client.h>
#include <plugin/net/messages.h>
#include <plugin/net/list_message.h>
#include <utils/system/argparser.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace fawkes;

/** @class PluginTool tools/plugin/plugin_tool.h
 * Program to communicate with plugin manager via Fawkes network.
 */

/** Constructor.
 * @param argp argument parser, three arguments are handled:
 * - -l plugin_name load plugin named plugin_name
 * - -u plugin_name unload plugin named plugin_name
 * - -w watch for changes
 * @param c FawkesNetworkClient with established connection
 */
PluginTool::PluginTool(ArgumentParser *argp, FawkesNetworkClient *c)
{
  this->c     = c;
  plugin_name = NULL;
  quit        = false;
  
  if ( argp->has_arg("l") ) {
    opmode = M_LOAD;
    plugin_name = argp->arg("l");
  } else if ( argp->has_arg("u") ) {
    opmode = M_UNLOAD;
    plugin_name = argp->arg("u");
  } else if ( argp->has_arg("R") ) {
    opmode = M_RELOAD;
    plugin_name = argp->arg("R");
  } else if ( argp->has_arg("w") ) {
    opmode = M_WATCH;
  } else if ( argp->has_arg("a") ) {
    opmode = M_LIST_AVAIL;
  } else {
    opmode = M_LIST_LOADED;
  }

  __program_name = argp->program_name();

  list_found = false;
}


/** Constructor.
 * This constructor just set the Fawkes network client. A run() call will
 * fail if not one of set_load_plugin(), set_unload_plugin(), set_watch_mode()
 * or set_list_mode() has been called before.
 * @param c Fawkes network client with established connection
 */
PluginTool::PluginTool(FawkesNetworkClient *c)
{
  this->c     = c;
  plugin_name = NULL;
  quit        = false;
  opmode      = M_UNKNOWN;
  list_found  = false;
}

/** Destructor */
PluginTool::~PluginTool()
{
}


/** Print usage.
 * @param program_name program name
 */
void
PluginTool::print_usage(const char *program_name)
{
    printf("Usage: %s [-l plugin|-u plugin|-R plugin|-w|-a|-L] [-r host[:port]]\n"
	   "  -l plugin      Load plugin with given name\n"
	   "  -u plugin      Unload plugin with given name\n"
	   "  -R plugin      Reload plugin with given name\n"
	   "  -w             Watch all load/unload operations\n"
	   "  -a             List available plugins\n"
	   "  -L             List loaded plugins (default)\n\n"
	   "  -r host[:port] Remote host (and optionally port) to connect to\n\n"
	    "  If called without any option list currently loaded plugins\n\n",
	   program_name);
}

/** Load plugin on next run.
 * The next time run is called a LOAD_PLUGIN message is sent for the
 * given plugin name.
 * @param plugin_name name of the plugin to load
 */
void
PluginTool::set_load_plugin(const char *plugin_name)
{
  this->plugin_name = plugin_name;
  opmode = M_LOAD;
}


/** Unload plugin on next run.
 * The next time run is called a UNLOAD_PLUGIN message is sent for the
 * given plugin name.
 * @param plugin_name name of the plugin to unload
 */
void
PluginTool::set_unload_plugin(const char *plugin_name)
{
  this->plugin_name = plugin_name;
  opmode = M_UNLOAD;
}


/** Set watch mode.
 * On next run() call the client will watch for new events.
 */
void
PluginTool::set_watch_mode()
{
  opmode = M_WATCH;
}


/** Set list mode.
 * On next run() call the client will list all loaded plugins once.
 */
void
PluginTool::set_list_mode()
{
  opmode = M_LIST_LOADED;
}


/** Handle signals.
 * @param signum signal number of received signal
 */
void
PluginTool::handle_signal(int signum)
{ 
  c->wake(FAWKES_CID_PLUGINMANAGER);
  quit = true;
}


/** Execute load operation. */
void
PluginTool::load()
{
  printf("Requesting loading of plugin %s\n", plugin_name);
  plugin_load_msg_t *l = (plugin_load_msg_t *)calloc(1, sizeof(plugin_load_msg_t));
  strncpy(l->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);

  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
						       MSG_PLUGIN_LOAD,
						       l, sizeof(plugin_load_msg_t));
  c->enqueue(msg);

  while ( ! quit ) {
    c->wait(FAWKES_CID_PLUGINMANAGER);
  }
}


/** Execute unload operation. */
void
PluginTool::unload()
{
  printf("Requesting unloading of plugin %s\n", plugin_name);
  plugin_unload_msg_t *m = (plugin_unload_msg_t *)calloc(1, sizeof(plugin_unload_msg_t));
  strncpy(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);

  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
						       MSG_PLUGIN_UNLOAD,
						       m, sizeof(plugin_unload_msg_t));
  c->enqueue(msg);

  while ( ! quit ) {
    c->wait(FAWKES_CID_PLUGINMANAGER);
  }
}


/** Execute list available operation. */
void
PluginTool::list_avail()
{
  printf("Request the list of all available plugins\n");
  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
						       MSG_PLUGIN_LIST_AVAIL);
  c->enqueue(msg);

  while ( ! quit ) {
    c->wait(FAWKES_CID_PLUGINMANAGER);
  }
}


/** Execute list operation. */
void
PluginTool::list_loaded()
{
  // we got a list of loaded messages during startup, show them
  printf("Request the list of all loaded plugins\n");
  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
						       MSG_PLUGIN_LIST_LOADED);
  c->enqueue(msg);

  while ( ! quit ) {
    c->wait(FAWKES_CID_PLUGINMANAGER);
  }
}


/** Watch for plugin manager events. */
void
PluginTool::watch()
{
  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
						       MSG_PLUGIN_SUBSCRIBE_WATCH);
  c->enqueue(msg);
  printf("Watching for plugin events\n");
  printf("%-10s   %-40s\n", "Event", "Plugin Name/ID");
  while ( ! quit ) {
    c->wait(FAWKES_CID_PLUGINMANAGER);
  }

  // unsubscribe
  msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNSUBSCRIBE_WATCH);
  c->enqueue(msg);
}


/** Handler has been deregistered.
 */
void
PluginTool::deregistered(unsigned int id) throw()
{
  quit = true;
}


/** Inbound message received.
 * @param msg message.
 */
void
PluginTool::inbound_received(FawkesNetworkMessage *msg,
			     unsigned int id) throw()
{
  if (msg->cid() != FAWKES_CID_PLUGINMANAGER)  return;

  if ( msg->msgid() == MSG_PLUGIN_LOADED ) {
    if ( msg->payload_size() != sizeof(plugin_loaded_msg_t) ) {
      printf("Invalid message size (load succeeded)\n");
    } else {
      plugin_loaded_msg_t *m = (plugin_loaded_msg_t *)msg->payload();
      if ( opmode == M_WATCH ) {
	printf("%-10s   %s\n", "loaded", m->name);
      } else {
	if ( strncmp(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH) == 0 ) {
	  printf("Loading of %s succeeded\n", plugin_name);
	  quit = true;
	}
      }
    }
  } else if ( msg->msgid() == MSG_PLUGIN_LOAD_FAILED) {
    if ( msg->payload_size() != sizeof(plugin_load_failed_msg_t) ) {
      printf("Invalid message size (load failed)\n");
    } else {
      plugin_load_failed_msg_t *m = (plugin_load_failed_msg_t *)msg->payload();
      if ( opmode == M_WATCH ) {
	printf("%-10s   %s\n", "loadfail", m->name);
      } else {
	if ( strncmp(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH) == 0 ) {
	  printf("Loading of %s failed, see log for reason\n", plugin_name);
	  quit = true;
	}
      }
    }
  } else if ( msg->msgid() == MSG_PLUGIN_UNLOADED ) {
    if ( msg->payload_size() != sizeof(plugin_unloaded_msg_t) ) {
      printf("Invalid message size (unload succeeded)\n");
    } else {
      plugin_unloaded_msg_t *m = (plugin_unloaded_msg_t *)msg->payload();
      if ( opmode == M_WATCH ) {
	printf("%-10s   %s\n", "unloaded", m->name);
      } else {
	if ( strncmp(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH) == 0 ) {
	  printf("Unloading of %s succeeded\n", plugin_name);
	  quit = true;
	}
      }
    }
  } else if ( msg->msgid() == MSG_PLUGIN_UNLOAD_FAILED) {
    if ( msg->payload_size() != sizeof(plugin_unload_failed_msg_t) ) {
      printf("Invalid message size (unload failed)\n");
    } else {
      plugin_unload_failed_msg_t *m = (plugin_unload_failed_msg_t *)msg->payload();
      if ( opmode == M_WATCH ) {
	printf("%-10s   %s\n", "unloadfail", m->name);
      } else {
	if ( strncmp(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH) == 0 ) {
	  printf("Unloading of %s failed, see log for reason\n", plugin_name);
	  quit = true;
	}
      }
    }
  } else if (msg->msgid() == MSG_PLUGIN_AVAIL_LIST ) {
    PluginListMessage *plm = msg->msgc<PluginListMessage>();
    if ( plm->has_next() ) {
      printf("Available plugins:\n");
      while ( plm->has_next() ) {
	char *plugin_name = plm->next();
	char *plugin_desc = NULL;
	if ( plm->has_next() ) {
	  plugin_desc = plm->next();
	} else {
	  throw Exception("Invalid plugin list received");
	}
	printf(" %-16s (%s)\n", plugin_name, plugin_desc);
	free(plugin_name);
	free(plugin_desc);
      }
    } else {
      printf("No plugins available\n");
    }
    quit = true;
    delete plm;
  } else if (msg->msgid() == MSG_PLUGIN_LOADED_LIST ) {
    PluginListMessage *plm = msg->msgc<PluginListMessage>();
    if ( plm->has_next() ) {
      printf("Loaded plugins:\n");
      while ( plm->has_next() ) {
	char *p = plm->next();
	printf("  %s\n", p);
	free(p);
      }
    } else {
      printf("No plugins loaded\n");
    }
    quit = true;
    delete plm;
  } else if ( msg->msgid() == MSG_PLUGIN_AVAIL_LIST_FAILED) {
    printf("Obtaining list of available plugins failed\n");
  } else if ( msg->msgid() == MSG_PLUGIN_LOADED_LIST_FAILED) {
    printf("Obtaining list of loaded plugins failed\n");
  }
}


void
PluginTool::connection_established(unsigned int id) throw()
{
  // ignored, client has to be connected already
}


void
PluginTool::connection_died(unsigned int id) throw()
{
  printf("Connection died, exiting\n");
  quit = true;
}

/** Run opmode as requested determined by the arguments. */
void
PluginTool:: run()
{
  c->register_handler(this, FAWKES_CID_PLUGINMANAGER);

  switch (opmode) {
  case M_LOAD:
    load();
    break;

  case M_UNLOAD:
    unload();
    break;

  case M_RELOAD:
    unload();
    quit = false;
    load();
    break;

  case M_LIST_AVAIL:
    list_avail();
    break;

  case M_LIST_LOADED:
    list_loaded();
    break;

  case M_WATCH:
    watch();
    break;

  default:
    print_usage(__program_name);
  }

  c->deregister_handler(FAWKES_CID_PLUGINMANAGER);
}
