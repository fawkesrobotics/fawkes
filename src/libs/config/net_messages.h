
/***************************************************************************
 *  config_messages.h - Fawkes Configuration Messages
 *
 *  Created: Sat Jan 06 23:14:59 2007
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FAWKES_CONFIG_MESSAGES_H_
#define __FAWKES_CONFIG_MESSAGES_H_

#include <stdint.h>
#include <netcomm/utils/dynamic_buffer.h>

#pragma pack(push,4)

#define MSG_CONFIG_GET_FLOAT             1
#define MSG_CONFIG_GET_UINT              2
#define MSG_CONFIG_GET_INT               3
#define MSG_CONFIG_GET_BOOL              4
#define MSG_CONFIG_GET_STRING            5
#define MSG_CONFIG_GET_VALUE             6
#define MSG_CONFIG_GET_BEGIN             MSG_CONFIG_GET_FLOAT
#define MSG_CONFIG_GET_END               MSG_CONFIG_GET_VALUE
#define MSG_CONFIG_GET_ALL               7

#define MSG_CONFIG_SET_FLOAT            10
#define MSG_CONFIG_SET_UINT             11
#define MSG_CONFIG_SET_INT              12
#define MSG_CONFIG_SET_BOOL             13
#define MSG_CONFIG_SET_STRING           14
#define MSG_CONFIG_SET_DEFAULT_FLOAT    15
#define MSG_CONFIG_SET_DEFAULT_UINT     16
#define MSG_CONFIG_SET_DEFAULT_INT      17
#define MSG_CONFIG_SET_DEFAULT_BOOL     18
#define MSG_CONFIG_SET_DEFAULT_STRING   19
#define MSG_CONFIG_SET_BEGIN            MSG_CONFIG_SET_FLOAT
#define MSG_CONFIG_SET_END              MSG_CONFIG_SET_DEFAULT_STRING
#define MSG_CONFIG_ERASE_VALUE          20
#define MSG_CONFIG_ERASE_DEFAULT_VALUE  21

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
#define MSG_CONFIG_VALUE_BEGIN          MSG_CONFIG_FLOAT_VALUE
#define MSG_CONFIG_VALUE_END            MSG_CONFIG_STRING_VALUE
#define MSG_CONFIG_INV_VALUE            36
#define MSG_CONFIG_VALUE_ERASED         37
#define MSG_CONFIG_DEFAULT_VALUE_ERASED 38
#define MSG_CONFIG_LIST                 39

#define MSG_CONFIG_SUBSCRIBE            50
#define MSG_CONFIG_UNSUBSCRIBE          51


/* Length definitions */
#define CONFIG_MSG_PATH_LENGTH         128
#define CONFIG_MSG_MAX_STRING_LENGTH   256
#define CONFIG_MSG_MAX_TAG_LENGTH       64

/** Basic config descriptor.
 * Path that defines a unique element in the configuration.
 * It is part of most messages.
 */
typedef struct {
  char path[CONFIG_MSG_PATH_LENGTH];		/**< path to config value. */
  uint32_t   is_default :  1;			/**< 1 if value is a default value, 0
						 * otherwise, only for get response */
  uint32_t   reserved   : 31;			/**< Reserved for future use. */
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

/** Float value message */
typedef struct {
  config_descriptor_t  cp;	/**< value descriptor */
  float f;			/**< value */
} config_float_value_msg_t;

/** Unsigned int value message */
typedef struct {
  config_descriptor_t  cp;	/**< value descriptor */
  uint32_t u;			/**< value */
} config_uint_value_msg_t;

/** Integer value message */
typedef struct {
  config_descriptor_t  cp;	/**< value descriptor */
  int32_t i;			/**< value */
} config_int_value_msg_t;

/** Boolean value message */
typedef struct {
  config_descriptor_t  cp;	/**< value descriptor */
  uint32_t b;			/**< value */
} config_bool_value_msg_t;

/** String value message */
typedef struct {
  config_descriptor_t  cp;		/**< value descriptor */
  char s[CONFIG_MSG_MAX_STRING_LENGTH];	/**< value */
} config_string_value_msg_t;

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

/** Config list float entity. */
typedef struct {
  config_list_entity_header_t  header;	/**< config entity header */
  float    f;				/**< float value */
} config_list_float_entity_t;

/** Config list unsigned int entity. */
typedef struct {
  config_list_entity_header_t  header;	/**< config entity header */
  uint32_t u;				/**< uint value */
} config_list_uint_entity_t;

/** Config list int entity. */
typedef struct {
  config_list_entity_header_t  header;	/**< config entity header */
  int32_t i;				/**< float value */
} config_list_int_entity_t;

/** Config list bool entity. */
typedef struct {
  config_list_entity_header_t  header;	/**< config entity header */
  int32_t b;				/**< 0 is false, everything else is true */
} config_list_bool_entity_t;

/** Config list string entity. */
typedef struct {
  config_list_entity_header_t  header;		/**< config entity header */
  char     s[CONFIG_MSG_MAX_STRING_LENGTH];	/**< string value */
} config_list_string_entity_t;

#pragma pack(pop)

#endif
