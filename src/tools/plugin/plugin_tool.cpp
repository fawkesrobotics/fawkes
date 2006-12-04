
/***************************************************************************
 *  plugin_tool.cpp - Fawkes plugin tool
 *
 *  Created: Mon Dec 04 14:43:23 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <tools/plugin/plugin_tool.h>

#include <netcomm/fawkes/client.h>
#include <mainapp/plugin_messages.h>
#include <utils/system/argparser.h>


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
  this->argp  = argp;
  this->c     = c;
  plugin_name = NULL;
  quit        = false;
  
  if ( argp->hasArgument("l") ) {
    opmode = M_LOAD;
    plugin_name = argp->getArgument("l");
  } else if ( argp->hasArgument("u") ) {
    opmode = M_UNLOAD;
    plugin_name = argp->getArgument("u");
  } else if ( argp->hasArgument("w") ) {
    opmode = M_WATCH;
  } else {
    opmode = M_LIST;
  }
}

/** Destructor */
PluginTool::~PluginTool()
{
}

/** Handle signals.
 * @param signum signal number of received signal
 */
void
PluginTool::handle_signal(int signum)
{
  quit = true;
}


/** Execute load operation. */
void
PluginTool::load()
{
  c->wait();
  c->recv();
  c->inbound_queue()->clear();
  
  printf("Requesting loading of plugin %s\n", plugin_name);
  plugin_load_msg_t *l = (plugin_load_msg_t *)calloc(1, sizeof(plugin_load_msg_t));
  strncpy(l->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);

  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
						       MSG_PLUGIN_LOAD,
						       l, sizeof(plugin_load_msg_t));
  c->enqueue(msg);
  msg->unref();
  c->send();

  bool plugin_msg_received = false;
  do {
    c->wait();
    c->recv();
    FawkesNetworkMessageQueue *q = c->inbound_queue();
    q->lock();
    while ( ! q->empty() ) {
      FawkesNetworkMessage *msg = q->front();
      if (msg->cid() == FAWKES_CID_PLUGINMANAGER) {
	if ( msg->msgid() == MSG_PLUGIN_LOADED ) {
	  if ( msg->payload_size() != sizeof(plugin_loaded_msg_t) ) {
	    printf("Invalid message size (load succeeded)\n");
	  } else {
	    plugin_loaded_msg_t *m = (plugin_loaded_msg_t *)msg->payload();
	    if ( strncmp(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH) == 0 ) {
	      plugin_msg_received = true;
	      printf("Loading of %s succeeded\n", plugin_name);
	    }
	  }
	} else if ( msg->msgid() == MSG_PLUGIN_LOAD_FAILED) {
	  if ( msg->payload_size() != sizeof(plugin_load_failed_msg_t) ) {
	    printf("Invalid message size (load failed)\n");
	  } else {
	    plugin_load_failed_msg_t *m = (plugin_load_failed_msg_t *)msg->payload();
	    if ( strncmp(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH) == 0 ) {
	      plugin_msg_received = true;
	      printf("Loading of %s failed, see log for reason\n", plugin_name);
	    }
	  }
	} else {
	  printf("Ignoring unrelated message\n");
	}
      }
      msg->unref();
      q->pop();
    }
    q->unlock();
    usleep(0);
  } while ( ! plugin_msg_received );
}


/** Execute unload operation. */
void
PluginTool::unload()
{
  c->wait();
  c->recv();
  c->inbound_queue()->clear();

  printf("Requesting unloading of plugin %s\n", plugin_name);
  plugin_unload_msg_t *m = (plugin_unload_msg_t *)calloc(1, sizeof(plugin_unload_msg_t));
  strncpy(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);

  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
						       MSG_PLUGIN_UNLOAD,
						       m, sizeof(plugin_unload_msg_t));
  c->enqueue(msg);
  msg->unref();
  c->send();

  bool plugin_msg_received = false;
  do {
    c->wait();
    c->recv();
    FawkesNetworkMessageQueue *q = c->inbound_queue();
    q->lock();
    while ( ! q->empty() ) {
      FawkesNetworkMessage *msg = q->front();
      if (msg->cid() == FAWKES_CID_PLUGINMANAGER) {
	if ( msg->msgid() == MSG_PLUGIN_UNLOADED ) {
	  if ( msg->payload_size() != sizeof(plugin_unloaded_msg_t) ) {
	    printf("Invalid message size (unload succeeded)\n");
	  } else {
	    plugin_unloaded_msg_t *m = (plugin_unloaded_msg_t *)msg->payload();
	    if ( strncmp(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH) == 0 ) {
	      plugin_msg_received = true;
	      printf("Unloading of %s succeeded\n", plugin_name);
	    }
	  }
	} else if ( msg->msgid() == MSG_PLUGIN_UNLOAD_FAILED) {
	  if ( msg->payload_size() != sizeof(plugin_unload_failed_msg_t) ) {
	    printf("Invalid message size (unload failed)\n");
	  } else {
	    plugin_unload_failed_msg_t *m = (plugin_unload_failed_msg_t *)msg->payload();
	    if ( strncmp(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH) == 0 ) {
	      plugin_msg_received = true;
	      printf("Unloading of %s failed, see log for reason\n", plugin_name);
	    }
	  }
	} else {
	  printf("Ignoring unrelated message\n");
	}
      }
      msg->unref();
      q->pop();
    }
    q->unlock();
  } while ( ! plugin_msg_received );
}


/** Execute list operation. */
void
PluginTool::list()
{
  // we got a list of loaded messages during startup, show them
  c->wait();
  c->recv();
  FawkesNetworkMessageQueue *q = c->inbound_queue();
  q->lock();
  printf("%10s   %-40s\n", "Plugin ID", "Plugin Name");
  while ( ! q->empty() ) {
    FawkesNetworkMessage *msg = q->front();
    if (msg->cid() == FAWKES_CID_PLUGINMANAGER) {
      if ( msg->msgid() == MSG_PLUGIN_LOADED ) {
	if ( msg->payload_size() != sizeof(plugin_loaded_msg_t) ) {
	  printf("Invalid message size (load succeeded)\n");
	} else {
	  plugin_loaded_msg_t *m = (plugin_loaded_msg_t *)msg->payload();
	  printf("%10u   %-40s\n", m->plugin_id, m->name);
	}
      } else if ( msg->msgid() == MSG_PLUGIN_NONE_LOADED ) {
	printf("No plugins loaded\n");
      }
    }
    q->pop();
  }
  q->unlock();
}


/** Watch for plugin manager events. */
void
PluginTool::watch()
{
  printf("Watching for plugin events\n");
  printf("%-10s   %-40s\n", "Event", "Plugin Name/ID");
  while ( ! quit ) {
    c->wait();
    c->recv();
    FawkesNetworkMessageQueue *q = c->inbound_queue();
    q->lock();
    while ( ! q->empty() ) {
      FawkesNetworkMessage *msg = q->front();
      if (msg->cid() == FAWKES_CID_PLUGINMANAGER) {
	if ( msg->msgid() == MSG_PLUGIN_LOADED ) {
	  if ( msg->payload_size() != sizeof(plugin_loaded_msg_t) ) {
	    printf("Invalid message size (load succeeded)\n");
	  } else {
	    plugin_loaded_msg_t *m = (plugin_loaded_msg_t *)msg->payload();
	    printf("%-10s   %s (%u)\n", "loaded", m->name, m->plugin_id);
	  }
	} else if ( msg->msgid() == MSG_PLUGIN_UNLOADED ) {
	  if ( msg->payload_size() != sizeof(plugin_unloaded_msg_t) ) {
	    printf("Invalid message size (unload succeeded)\n");
	  } else {
	    plugin_unloaded_msg_t *m = (plugin_unloaded_msg_t *)msg->payload();
	    printf("%-10s   %s\n", "unloaded", m->name);
	  }
	}
      }
      msg->unref();
      q->pop();
    }
    q->unlock();
    usleep(0);
  }
}


/** Run opmode as requested determined by the arguments. */
void
PluginTool:: run()
{
  switch (opmode) {
  case M_LOAD:
    load();
    break;

  case M_UNLOAD:
    unload();
    break;

  case M_LIST:
    list();
    break;

  case M_WATCH:
    watch();
    break;
  }
}
