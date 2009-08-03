
/***************************************************************************
 *  dynamic_buffer.h - A dynamic buffer
 *
 *  Created: Fri Jun 01 13:28:02 2007
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
 *             2007       Daniel Beck
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

#ifndef __NETCOMM_UTILS_DYNAMIC_BUFFER_H_
#define __NETCOMM_UTILS_DYNAMIC_BUFFER_H_

#include <sys/types.h>
#include <stdint.h>

namespace fawkes {

#pragma pack(push,4)

/** Dynamic list type.
 * Use this element in your message struct if you want to add a dynamic list.
 * This is meant to be used in conjunction with DynamicBuffer.
 */
typedef struct {
  uint32_t  size;		/**< total size of list buffer */
  uint32_t  num_elements;	/**< number of elements in list */
} dynamic_list_t;

#pragma pack(pop)

class DynamicBuffer
{
 public:
  DynamicBuffer(dynamic_list_t *db, size_t initial_buffer_size = 1024);
  DynamicBuffer(dynamic_list_t *db, void *buf, size_t size);
  virtual ~DynamicBuffer();

  void   append(const void *data, size_t data_size);
  void * buffer();
  size_t buffer_size();
  unsigned int num_elements();

  size_t real_buffer_size();

  bool   has_next();
  void * next(size_t *size);
  void   reset_iterator();

 private:

  typedef uint16_t element_header_t;

  void   increase();

  bool               _read_only;

  dynamic_list_t    *_db;
  void              *_buffer;
  size_t             _buffer_size;
  element_header_t  *_curhead;
  void              *_curdata;

  uint16_t           _it_curel;
  element_header_t  *_it_curhead;
  void              *_it_curdata;

};

} // end namespace fawkes

#endif
