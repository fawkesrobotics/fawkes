
/***************************************************************************
 *  dynamic_buffer.cpp - A dynamic buffer
 *
 *  Created: Fri Jun 01 13:28:46 2007
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

#include <core/exceptions/system.h>
#include <core/exceptions/software.h>
#include <netcomm/utils/dynamic_buffer.h>

#include <cstdlib>
#include <cstring>
#include <netinet/in.h>

namespace fawkes {

/** @class DynamicBuffer <netcomm/utils/dynamic_buffer.h>
 * Dynamically growing buffer.
 * This class maintains a list or arbitrary data objects stuffed into
 * one consecutive memory. The buffer layout is like the following:
 * A dynamic_list_t element is supplied and can reside anywhere you
 * like (in the case of the Fawkes network protocol in your static
 * struct). It contains information about the size and number of elements
 * of the list. The list itself is formed by concatenated memory regions,
 * each preceeded by a two byte length value.
 *
 * The list may be at most 4 GB in total size (including in-between headers)
 * Each list item by itself can be at most 64 KB in size.
 * The buffer starts with an initial size. If this initial size is exceeded
 * the buffer size is doubled. If the double size would exceed 4 GB it is
 * increased to exactly 4 GB.
 *
 * The numbers in the headers are stored in network byte order and thus are
 * suitable for direct sending over the network.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */


/** Write constructor.
 * Use this constructor to create and write to this dynamic buffer.
 * @param db dynamic list header in your message
 * @param initial_buffer_size initial buffer size to use, by default 1 KB
 */
DynamicBuffer::DynamicBuffer(dynamic_list_t *db, size_t initial_buffer_size)
{
  _read_only   = false;
  _db          = db;
  _buffer_size = initial_buffer_size;
  _buffer      = malloc(_buffer_size);
  _curhead     = (element_header_t *)_buffer;
  _curdata     = (void *)((size_t)_buffer + sizeof(element_header_t));

  _db->size         = htonl(0);
  _db->num_elements = htonl(0);

  _it_curhead = NULL;
  _it_curdata = NULL;
  _it_curel = 0;
}


/** Read constructor.
 * Use this constructor to read from a dynamic buffer.
 * @param db dynamic list header in the incoming message
 * @param buf buffer to parse
 * @param size size of the buffer
 */
DynamicBuffer::DynamicBuffer(dynamic_list_t *db, void *buf, size_t size)
{
  _read_only   = true;
  _db          = db;
  _buffer_size = size;
  _buffer      = buf;

  _curhead     = (element_header_t *)_buffer;
  _curdata     = (void *)((size_t)_buffer + sizeof(element_header_t));

  _it_curhead  = _curhead;
  _it_curdata  = _curdata;
  _it_curel    = 0;
}


/** Destructor.
 * If in writing mode frees up the buffer.
 */
DynamicBuffer::~DynamicBuffer()
{
  if (! _read_only)  free(_buffer);
}


/** Append data.
 * Appends data to the list. Throws an exception if there is not enough memory to
 * hold the data or if in read-only mode.
 * @param data data to append
 * @param data_size size of the data in bytes
 * @exception AccessViolationException thrown if buffer is in read-only mode
 * @exception IllegalArgumentException thrown if data size is bigger than 64 KB - 2 bytes
 * @exception OutOfMemoryException thrown if no memory could be allocated
 */
void
DynamicBuffer::append(const void *data, size_t data_size)
{
  if ( _read_only ) throw AccessViolationException("DynamicBuffer is read-only");

  if ( data_size > (0xFFFF - sizeof(element_header_t)) ) {
    throw IllegalArgumentException("Buffer size too big, max 65535 bytes");
  }

  size_t cur_size = ntohl(_db->size);
  if ( (cur_size + data_size + sizeof(element_header_t)) > _buffer_size ) {
    try {
      increase();
    } catch (OutOfMemoryException &e) {
      throw;
    }
    if ( (cur_size + data_size + sizeof(element_header_t)) > _buffer_size ) {
      throw OutOfMemoryException("Could not increase buffer far enough to hold data");
    }
  }

  *_curhead = htons(data_size);
  memcpy(_curdata, data, data_size);

  _curhead           = (element_header_t *)((size_t)_curhead + data_size
					                     + sizeof(element_header_t));
  _curdata           = (void *)((size_t)_curdata + data_size + sizeof(element_header_t));
  _db->size          = htonl(cur_size + sizeof(element_header_t) + data_size);
  uint16_t tmp = ntohl(_db->num_elements) + 1;
  _db->num_elements  = htonl(tmp);
}


/** Get pointer to buffer.
 * @return packed buffer
 */
void *
DynamicBuffer::buffer()
{
  return _buffer;
}


/** Increase buffer size.
 * Internal usage only.
 */
void
DynamicBuffer::increase()
{
  size_t new_buffer_size;

  if ( (_buffer_size) >= 0xFFFFFFFF / 2 ) {
    if ( _buffer_size == 0xFFFFFFFF ) {
      throw OutOfMemoryException("Dynamic buffer may not be greater than 64KB");
    } else {
      new_buffer_size = 0xFFFFFFFF;
    }
  } else {
    new_buffer_size = _buffer_size * 2;
  }

  void *tmp = realloc(_buffer, new_buffer_size);
  if ( tmp == NULL ) {
    throw OutOfMemoryException();
  } else {
    _buffer_size = new_buffer_size;
    _curhead = (element_header_t *)((size_t)tmp + ((size_t)_curhead - (size_t)_buffer));
    _curdata = (void *)((size_t)tmp + ((size_t)_curdata - (size_t)_buffer));
    _buffer = tmp;
  }
}


/** Get buffer size.
 * Gets the size of the used part of the buffer. The size of the buffer that
 * is really occupied by data.
 * @return size of occupied buffer
 */
size_t
DynamicBuffer::buffer_size()
{
  return ntohl(_db->size);
}


/** Get real buffer size.
 * Gets the real size of the buffer including yet unused parts. Meant to be
 * used for debugging or informational usage.
 * @return real buffer size
 */
size_t
DynamicBuffer::real_buffer_size()
{
  return _buffer_size;
}


/** Get number of elements.
 * @return number of elements in list
 */
unsigned int
DynamicBuffer::num_elements()
{
  return ntohl(_db->num_elements);
}


/** Reset iterator.
 */
void
DynamicBuffer::reset_iterator()
{
  _it_curhead  = _curhead;
  _it_curdata  = _curdata;
  // invalid element
  _it_curel    = 0;
}


/** Check if another element is available.
 * @return true if another element can be fetched with next(), false otherwise
 */
bool
DynamicBuffer::has_next()
{
  return (_read_only && (_it_curel < (ntohl(_db->num_elements))));
}


/** Get next buffer.
 * @param size upon successful return contains size of the current list buffer
 * @return the next buffer.
 * @exception OutOfBoundsException thrown if no further element is available
 * in the list.
 */
void *
DynamicBuffer::next(size_t *size)
{
  // advance
  if ( ! has_next() ) {
    throw OutOfBoundsException("No next element while iterator DynamicBuffer");
  }

  if ( _it_curel > 0 ) {
    size_t offset = ntohs(*_it_curhead) + sizeof(element_header_t);
    _it_curhead = (element_header_t *)((size_t)_it_curhead + offset);
    _it_curdata = (void *)((size_t)_it_curdata + offset);
  }
  ++_it_curel;
  *size = ntohs(*_it_curhead);

  return _it_curdata;
}

} // end namespace fawkes
