
/***************************************************************************
 *  plugin_messages.h - Fawkes Plugin Messages
 *
 *  Created: Wed Nov 22 17:24:16 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FAWKES_PLUGIN_MESSAGES_H_
#define __FAWKES_PLUGIN_MESSAGES_H_

/** PLUGIN_LOAD message type ID */
#define MSG_PLUGIN_LOAD            1

/** PLUGIN_LOADED message type ID */
#define MSG_PLUGIN_LOADED          2

/** PLUGIN_LOAD_FAILED message type ID */
#define MSG_PLUGIN_LOAD_FAILED     3

/** PLUGIN_UNLOAD message type ID */
#define MSG_PLUGIN_UNLOAD          4

/** PLUGIN_UNLOADED message type ID */
#define MSG_PLUGIN_UNLOADED        5

/** PLUGIN_UNLOAD_FAILED message type ID */
#define MSG_PLUGIN_UNLOAD_FAILED   6

/** PLUGIN_NONE_LOADED message type ID */
#define MSG_PLUGIN_NONE_LOADED     7

/** PLUGIN_ALIVE message type ID */
#define MSG_PLUGIN_ALIVE           10

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

/** Alive message.
 * Message type Id is MSG_PLUGIN_ALIVE.
 */
typedef struct {
  unsigned int plugin_id;	/**< plugin ID of the alive plugin */
} plugin_alive_msg_t;


/** Plugin loaded message.
 * Message type ID is MSG_PLUGIN_LOADED.
 */
typedef struct {
  char         name[PLUGIN_MSG_NAME_LENGTH];	/**< name of the plugin that has been loaded */
  unsigned int plugin_id;			/**< plugin id of the loaded plugin */
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


#endif
