
/***************************************************************************
 *  Laser360Interface.h - Fawkes BlackBoard Interface - Laser360Interface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
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

#ifndef __INTERFACES_LASER360INTERFACE_H_
#define __INTERFACES_LASER360INTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class Laser360Interface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(Laser360Interface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    float distances[360]; /**< 
      The distances in meter of the beams.
     */
  } Laser360Interface_data_t;

  Laser360Interface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  Laser360Interface();
  ~Laser360Interface();

 public:
  /* Methods */
  float * distances() const;
  float distances(unsigned int index) const;
  void set_distances(unsigned int index, const float new_distances);
  void set_distances(const float * new_distances);
  size_t maxlenof_distances() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);

};

} // end namespace fawkes

#endif
