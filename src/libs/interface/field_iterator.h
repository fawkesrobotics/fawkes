
/***************************************************************************
 *  field_iterator.h - Iterate over field of an interface or a message
 *
 *  Created: Fri Jul 16 17:12:30 2009
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

#ifndef __INTERFACE_FIELD_ITERATOR_H__
#define __INTERFACE_FIELD_ITERATOR_H__

#include <interface/types.h>

#define __STD_LIMIT_MACROS
#include <stdint.h>

namespace fawkes {
class Interface;
class Message;

class InterfaceFieldIterator
{
  friend class Interface;
  friend class Message;

 public:
  InterfaceFieldIterator();
  InterfaceFieldIterator(const InterfaceFieldIterator &fit);
  ~InterfaceFieldIterator();
  InterfaceFieldIterator &        operator++ ();        // prefix
  InterfaceFieldIterator          operator++ (int inc); // postfix
  InterfaceFieldIterator &        operator+  (unsigned int i);
  InterfaceFieldIterator &        operator+= (unsigned int i);
  bool                   operator== (const InterfaceFieldIterator & fit) const;
  bool                   operator!= (const InterfaceFieldIterator & fit) const;
  const void *           operator*  () const;
  InterfaceFieldIterator &        operator=  (const InterfaceFieldIterator & fit);
  
  interface_fieldtype_t  get_type() const;
  const char *           get_typename() const;
  const char *           get_name() const;
  const void *           get_value() const;
  const char *           get_value_string();
  size_t                 get_length() const;
  bool                   get_bool(unsigned int index = 0) const;
  int8_t                 get_int8(unsigned int index = 0) const;
  uint8_t                get_uint8(unsigned int index = 0) const;
  int16_t                get_int16(unsigned int index = 0) const;
  uint16_t               get_uint16(unsigned int index = 0) const;
  int32_t                get_int32(unsigned int index = 0) const;
  uint32_t               get_uint32(unsigned int index = 0) const;
  int64_t                get_int64(unsigned int index = 0) const;
  uint64_t               get_uint64(unsigned int index = 0) const;
  float                  get_float(unsigned int index = 0) const;
  uint8_t                get_byte(unsigned int index = 0) const;
  bool *                 get_bools() const;
  int8_t *               get_int8s() const;
  uint8_t *              get_uint8s() const;
  int16_t *              get_int16s() const;
  uint16_t *             get_uint16s() const;
  int32_t *              get_int32s() const;
  uint32_t *             get_uint32s() const;
  int64_t *              get_int64s() const;
  uint64_t *             get_uint64s() const;
  float *                get_floats() const;
  uint8_t *              get_bytes() const;
  const char *           get_string() const;

  void                   set_bool(bool b, unsigned int index = 0);
  void                   set_int8(int8_t i, unsigned int index = 0);
  void                   set_uint8(uint8_t i, unsigned int index = 0);
  void                   set_int16(int16_t i, unsigned int index = 0);
  void                   set_uint16(uint16_t i, unsigned int index = 0);
  void                   set_int32(int32_t i, unsigned int index = 0);
  void                   set_uint32(uint32_t i, unsigned int index = 0);
  void                   set_int64(int64_t i, unsigned int index = 0);
  void                   set_uint64(uint64_t i, unsigned int index = 0);
  void                   set_float(float f, unsigned int index = 0);
  void                   set_byte(uint8_t b, unsigned int index = 0);
  void                   set_bools(bool *b);
  void                   set_int8s(int8_t *i);
  void                   set_uint8s(uint8_t *i);
  void                   set_int16s(int16_t *i);
  void                   set_uint16s(uint16_t *i);
  void                   set_int32s(int32_t *i);
  void                   set_uint32s(uint32_t *i);
  void                   set_int64s(int64_t *i);
  void                   set_uint64s(uint64_t *i);
  void                   set_floats(float *f);
  void                   set_bytes(uint8_t* b);
  void                   set_string(const char* s);
  
 protected:
  InterfaceFieldIterator(const Interface *interface,
			 const interface_fieldinfo_t *info_list);
  
 private:
  const interface_fieldinfo_t   *__infol;
  char                          *__value_string;
  const Interface               *__interface;
};

}

#endif /* __INTERFACE_FIELD_ITERATOR_H__ */
