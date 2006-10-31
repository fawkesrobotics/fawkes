
/***************************************************************************
 *  blackboard.h - Fawkes BlackBoard Interface - BlackBoardInternalsInterface
 *
 *  Interface generated: Mon Oct 30 19:48:49 2006
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2006  Tim Niemueller
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

#ifndef __INTERFACES_BLACKBOARD_H_
#define __INTERFACES_BLACKBOARD_H_

#include <interfaces/interface.h>
#include <interfaces/message.h>

class BlackBoardInternalsInterface : public Interface
{
/// @cond INTERNALS
 friend Interface *  private_newBlackBoardInternalsInterface();
/// @endcond
 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int InstanceSerial; /**< A serial number incremented for each
    interface access instance of any type. This is not the internal memory serial which is
    incremented for every interface allocated in the shared memory! The instance serial
    allows messages to be linked with a specific sending interface instance, and not just
    with the interface block. */
  } BlackBoardInternalsInterface_data_t;

  BlackBoardInternalsInterface_data_t *data;

 public:
  /* constants */

  /* messages */
  class GetInstanceSerialMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
    } GetInstanceSerialMessage_data_t;

    GetInstanceSerialMessage_data_t *data;

   public:
    GetInstanceSerialMessage();
    ~GetInstanceSerialMessage();

    /* Methods */
  };

  class GetMemSerialMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
    } GetMemSerialMessage_data_t;

    GetMemSerialMessage_data_t *data;

   public:
    GetMemSerialMessage();
    ~GetMemSerialMessage();

    /* Methods */
  };

  virtual bool messageValid(const Message *message) const;
 private:
  BlackBoardInternalsInterface();
  ~BlackBoardInternalsInterface();

 public:
  /* Methods */
  unsigned int getInstanceSerial();
  void setInstanceSerial(unsigned int newInstanceSerial);

};

#endif
