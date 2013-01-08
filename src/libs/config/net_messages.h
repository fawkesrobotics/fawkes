
/***************************************************************************
 *  config_messages.h - Fawkes Configuration Messages
 *
 *  Created: Sat Jan 06 23:14:59 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FAWKES_CONFIG_MESSAGES_H_
#define __FAWKES_CONFIG_MESSAGES_H_

#include <stdint.h>
#include <netcomm/utils/dynamic_buffer.h>

#pragma pack(push,4)

namespace fawkes {

#define MSG_CONFIG_GET_FLOAT             1
#define MSG_CONFIG_GET_UINT              2
#define MSG_CONFIG_GET_INT               3
#define MSG_CONFIG_GET_BOOL              4
#define MSG_CONFIG_GET_STRING            5
#define MSG_CONFIG_GET_VALUE             6
#define MSG_CONFIG_GET_COMMENT           7
#define MSG_CONFIG_GET_DEFAULT_COMMENT   8
#define MSG_CONFIG_GET_BEGIN             MSG_CONFIG_GET_FLOAT
#define MSG_CONFIG_GET_END               MSG_CONFIG_GET_DEFAULT_COMMENT
#define MSG_CONFIG_GET_ALL               9

#define MSG_CONFIG_SET_FLOAT            10
#define MSG_CONFIG_SET_UINT             11
#define MSG_CONFIG_SET_INT              12
#define MSG_CONFIG_SET_BOOL             13
#define MSG_CONFIG_SET_STRING           14
#define MSG_CONFIG_SET_COMMENT          15
#define MSG_CONFIG_SET_DEFAULT_FLOAT    16
#define MSG_CONFIG_SET_DEFAULT_UINT     17
#define MSG_CONFIG_SET_DEFAULT_INT      18
#define MSG_CONFIG_SET_DEFAULT_BOOL     19
#define MSG_CONFIG_SET_DEFAULT_STRING   20
#define MSG_CONFIG_SET_DEFAULT_COMMENT  21
#define MSG_CONFIG_SET_BEGIN            MSG_CONFIG_SET_FLOAT
#define MSG_CONFIG_SET_END              MSG_CONFIG_SET_DEFAULT_COMMENT
#define MSG_CONFIG_ERASE_VALUE          22

#define MSG_CONFIG_GET_TAGS             25
#define MSG_CONFIG_LOAD_TAG             26
#define MSG_CONFIG_SAVE_TAG             27
#define MSG_CONFIG_INV_TAG              28
#define MSG_CONFIG_DEL_TAG              29

#define MSG_CONFIG_FLOAT_VALUE          30
#define MSG_CONFIG_UINT_VALUE           31
#define MSG_CONFIG_INT_VALUE            32
#define MSG_CONFIG_BOOL_VALUE           33
#define MSG_CONFIG_STRING_VALUE         34
#define MSG_CONFIG_COMMENT_VALUE        35
#define MSG_CONFIG_VALUE_BEGIN          MSG_CONFIG_FLOAT_VALUE
#define MSG_CONFIG_VALUE_END            MSG_CONFIG_COMMENT_VALUE
#define MSG_CONFIG_INV_VALUE            36
#define MSG_CONFIG_VALUE_ERASED         37
#define MSG_CONFIG_LIST                 38

#define MSG_CONFIG_SUBSCRIBE            50
#define MSG_CONFIG_UNSUBSCRIBE          51


/* Length definitions */
#define CONFIG_MSG_PATH_LENGTH         128
#define CONFIG_MSG_MAX_TAG_LENGTH       64

/** Basic config descriptor.
 * Path that defines a unique element in the configuration.
 * It is part of most messages.
 */
typedef struct {
  char path[CONFIG_MSG_PATH_LENGTH];		/**< path to config value. */
  uint16_t   is_default :  1;			/**< 1 if value is a default value, 0
						 * otherwise, only for get response */
  uint16_t   reserved   : 15;			/**< Reserved for future use. */
  uint16_t   num_values;			/**< Number of valus in list. */
} config_descriptor_t;

/** Get value message. */
typedef struct {
  config_descriptor_t  cp;	/**< value descriptor */
} config_getval_msg_t;

/** Invalid value request message. */
typedef struct {
  config_descriptor_t  cp;	/**< value descriptor */
} config_invval_msg_t;

/** Erase value request. */
typedef struct {
  config_descriptor_t  cp;	/**< value descriptor */
} config_erase_value_msg_t;

/** Value erased message. */
typedef struct {
  config_descriptor_t  cp;	/**< value descriptor */
} config_value_erased_msg_t;

/** String value header indicating the string length. */
typedef struct {
  uint16_t s_length;	/**< Length of following string */
  uint16_t reserved;	/**< Reserved for future use */
} config_string_value_t;


/** Comment message */
typedef struct {
  config_descriptor_t  cp;			/**< value descriptor */
  uint16_t s_length;	/**< Length of following string */
  char s[2];		/**< comment, 0-terminated */
} config_comment_msg_t;

/** Tag message. */
typedef struct {
  config_descriptor_t  cp;		/**< value descriptor */
  char tag[CONFIG_MSG_MAX_TAG_LENGTH];	/**< tag */
} config_tag_msg_t;


/** Config list message. */
typedef struct {
  dynamic_list_t config_list;	/**< DynamicBuffer for list */
} config_list_msg_t;

/** Config list entity header. */
typedef struct {
  config_descriptor_t cp;	/**< Config descriptor. */
  uint32_t   type       : 8;	/**< type of entity, uses MSG_CONFIG_*_VALUE message IDs */
  uint32_t   reserved   : 24;	/**< reserved for future use */
} config_list_entity_header_t;

} // end namespace fawkes

#pragma pack(pop)

#endif
