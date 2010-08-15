
/***************************************************************************
 *  types.h - Field types used in the intefaces and the messages
 *
 *  Created: Fri Jul 16 17:35:43 2009
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *             2009  Daniel Beck
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

#ifndef __INTERFACE_TYPES_H__
#define __INTERFACE_TYPES_H__

#include <sys/types.h>

namespace fawkes {

/** Interface field type*/
typedef enum {
  IFT_BOOL,		/**< boolean field */
  IFT_INT8,		/**< 8 bit integer field */
  IFT_UINT8,		/**< 8 bit unsigned integer field */
  IFT_INT16,		/**< 16 bit integer field */
  IFT_UINT16,		/**< 16 bit unsigned integer field */
  IFT_INT32,		/**< 32 bit integer field */
  IFT_UINT32,		/**< 32 bit unsigned integer field */
  IFT_INT64,		/**< 64 bit integer field */
  IFT_UINT64,		/**< 64 bit unsigned integer field */
  IFT_FLOAT,		/**< float field */
  IFT_STRING,		/**< string field */
  IFT_BYTE,		/**< byte field, alias for uint8 */
  IFT_ENUM		/**< field with interface specific enum type */
} interface_fieldtype_t;

/** Interface field info list */
struct interface_fieldinfo_t {
  interface_fieldtype_t    type;	/**< type of this field */
  const char              *enumtype;	/**< text representation of enum type */
  const char              *name;	/**< Name of this field */
  size_t                   length;	/**< Length of field (array, string) */
  void                    *value;	/**< Current value of this field */
  interface_fieldinfo_t   *next;	/**< next field, NULL if last */
};

}

#endif /* __INTERFACE_TYPES_H__ */
