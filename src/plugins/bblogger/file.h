
/***************************************************************************
 *  file.h - Fawkes BlackBoard Logger data file definitions
 *
 *  Created: Sat Nov 07 23:20:51 2009 (from earlier edits elsewhere)
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_BBLOGGER_FILE_H_
#define __PLUGINS_BBLOGGER_FILE_H_

#include <interface/interface.h>

#include <stdint.h>

#define BBLOGGER_FILE_MAGIC 0xffbbffbb
#define BBLOGGER_FILE_VERSION 1

#pragma pack(push,4)

#define BBLOG_BIG_ENDIAN 1
#define BBLOG_LITTLE_ENDIAN 0

#define BBLOG_INTERFACE_TYPE_SIZE  __INTERFACE_TYPE_SIZE
#define BBLOG_INTERFACE_ID_SIZE    __INTERFACE_ID_SIZE
#define BBLOG_INTERFACE_HASH_SIZE  __INTERFACE_HASH_SIZE
#define BBLOG_SCENARIO_SIZE 32


/** BBLogger file header definition.
 * To identify log files created for different interfaces but belonging to a
 * single run files must be
 * - created at the exact same timestamp (filename and start_time_* fields
 * - have the same scenario id
 * The file_version is stored in network byte order. Anything beyond this is
 * stored in the native system format, read the endianess field to check whether
 * you must do data conversion.
 */
typedef struct {
  uint32_t file_magic;		/**< Magic value to identify file,
				 * must be 0xFFBBFFBB (big endian) */
  uint32_t file_version;	/**< File version, set to BBLOGGER_FILE_VERSION on
				 * write and verify on read (big endian) */
  uint32_t endianess :  1;	/**< Endianess, 0 little endian, 1 big endian */
  uint32_t reserved  : 31;	/**< Reserved for future use */
  uint32_t num_data_items;	/**< Number of data items in file, if set to zero
				 * reader must scan the file for this number */
  char     scenario[BBLOG_SCENARIO_SIZE];	/**< Scenario as defined in
						 * config */
  char     interface_type[BBLOG_INTERFACE_TYPE_SIZE];	/**< Interface type */
  char     interface_id[BBLOG_INTERFACE_ID_SIZE];	/**< Interface ID */
  unsigned char interface_hash[BBLOG_INTERFACE_HASH_SIZE];	/**< Interface Hash */
  uint32_t data_size;		/**< size of one interface data block */
  uint64_t start_time_sec;	/**< Start time, timestamp seconds */
  uint64_t start_time_usec;	/**< Start time, timestamp microseconds */
} bblog_file_header;

/** BBLogger entry header.
 * This header is written before every data block.
 */
typedef struct {
  uint32_t rel_time_sec;	/**< time since start time, seconds */
  uint32_t rel_time_usec;	/**< time since start time, microseconds */
} bblog_entry_header;

#pragma pack(pop)

#endif
