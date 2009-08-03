
/***************************************************************************
 *  plugin_messages.h - Fawkes Plugin Messages
 *
 *  Created: Wed Nov 22 17:24:16 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGIN_NET_MESSAGES_H_
#define __PLUGIN_NET_MESSAGES_H_

#include <netcomm/utils/dynamic_buffer.h>

namespace fawkes {

/** Plugin message type. */
typedef enum {
  MSG_PLUGIN_LOAD               =   1,	/**< request plugin load (plugin_load_msg_t) */
  MSG_PLUGIN_LOADED             =   2,	/**< plugin loaded (plugin_loaded_msg_t) */
  MSG_PLUGIN_LOAD_FAILED        =   3,	/**< plugin load failed (plugin_load_failed_msg_t) */
  MSG_PLUGIN_UNLOAD             =   4,	/**< request plugin unload (plugin_unload_msg_t) */
  MSG_PLUGIN_UNLOADED           =   5,	/**< plugin unloaded (plugin_unloaded_msg_t) */
  MSG_PLUGIN_UNLOAD_FAILED      =   6,	/**< plugin unload failed (plugin_unload_failed_msg_t) */
  MSG_PLUGIN_LIST_AVAIL         =   7,	/**< request list of available plugins */
  MSG_PLUGIN_AVAIL_LIST         =   8,	/**< list of available plugins (plugin_list_msg_t) */
  MSG_PLUGIN_AVAIL_LIST_FAILED  =   9,	/**< listing available plugins failed */
  MSG_PLUGIN_LIST_LOADED        =  10,	/**< request lif of loaded plugins */
  MSG_PLUGIN_LOADED_LIST        =  11,	/**< list of loaded plugins (plugin_list_msg_t) */
  MSG_PLUGIN_LOADED_LIST_FAILED =  12,	/**< listing loaded plugins failed */
  MSG_PLUGIN_SUBSCRIBE_WATCH    =  13,	/**< Subscribe for watching load/unload events */
  MSG_PLUGIN_UNSUBSCRIBE_WATCH  =  14	/**< Unsubscribe from watching load/unload events */
} plugin_message_type_t;


/** Maximum length of the plugin name field. */
#define PLUGIN_MSG_NAME_LENGTH 32

/** Load plugin message.
 * Message type Id is MSG_PLUGIN_LOAD.
 */
typedef struct {
  char name[PLUGIN_MSG_NAME_LENGTH];	/**< name of the plugin to load. */
} plugin_load_msg_t;

/** Unload plugin message.
 * Message type Id is MSG_PLUGIN_UNLOAD.
 */
typedef struct {
  char name[PLUGIN_MSG_NAME_LENGTH];	/**< name of te plugin to unload. */
} plugin_unload_msg_t;

/** Plugin loaded message.
 * Message type ID is MSG_PLUGIN_LOADED.
 */
typedef struct {
  char         name[PLUGIN_MSG_NAME_LENGTH];	/**< name of the plugin that has been loaded */
} plugin_loaded_msg_t;

/** Plugin load failed. */
typedef struct {
  char         name[PLUGIN_MSG_NAME_LENGTH];	/**< name of plugin that could not be unloaded */
} plugin_load_failed_msg_t;

/** Plugin unload failed. */
typedef struct {
  char         name[PLUGIN_MSG_NAME_LENGTH];	/**< name of plugin that could not be unloaded */
} plugin_unload_failed_msg_t;

/** Plugin unloaded message.
 * Message type ID is MSG_PLUGIN_UNLOADED.
 */
typedef struct {
  char         name[PLUGIN_MSG_NAME_LENGTH];	/**< name of the plugin that has been unloaded */
} plugin_unloaded_msg_t;

/** Plugin list message.
 * Message type ID is MSG_PLUGIN_LIST.
 */
typedef struct {
  dynamic_list_t plugin_list;	/**< dynamically growing list of plugin names. */
} plugin_list_msg_t;


} // end namespace fawkes

#endif
