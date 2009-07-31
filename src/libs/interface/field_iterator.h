
/***************************************************************************
 *  field_iterator.h - Iterate over field of an interface or a message
 *
 *  Created: Fri Jul 16 17:12:30 2009
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *             2009  Daniel Beck
 *
 *  $Id$
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
  int                    get_int(unsigned int index = 0) const;
  unsigned int           get_uint(unsigned int index = 0) const;
  long int               get_longint(unsigned int index = 0) const;
  unsigned long int      get_longuint(unsigned int index = 0) const;
  float                  get_float(unsigned int index = 0) const;
  unsigned char          get_byte(unsigned int index = 0) const;
  bool *                 get_bools() const;
  int *                  get_ints() const;
  unsigned int *         get_uints() const;
  long int *             get_longints() const;
  unsigned long int *    get_longuints() const;
  float *                get_floats() const;
  unsigned char *        get_bytes() const;
  const char *           get_string() const;

  void                   set_bool(bool b, unsigned int index = 0);
  void                   set_int(int i, unsigned int index = 0);
  void                   set_uint(unsigned int i, unsigned int index = 0);
  void                   set_longint(long int i, unsigned int index = 0);
  void                   set_longuint(long unsigned int i, unsigned int index = 0);
  void                   set_float(float f, unsigned int index = 0);
  void                   set_byte(unsigned char b, unsigned int index = 0);
  void                   set_bools(bool *b);
  void                   set_ints(int *i);
  void                   set_uints(unsigned int *i);
  void                   set_longints(long int *i);
  void                   set_longuints(long unsigned int *i);
  void                   set_floats(float *f);
  void                   set_bytes(unsigned char* b);
  void                   set_string(const char* s);
  
 protected:
  InterfaceFieldIterator(const interface_fieldinfo_t *info_list);
  
 private:
  const interface_fieldinfo_t   *__infol;
  char                          *__value_string;
};

}

#endif /* __INTERFACE_FIELD_ITERATOR_H__ */
