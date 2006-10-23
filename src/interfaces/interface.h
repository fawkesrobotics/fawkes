
/***************************************************************************
 *  interface.h - BlackBoard Interface
 *
 *  Generated: Mon Oct 09 18:34:11 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

#ifndef __INTERFACE_H_
#define __INTERFACE_H_

#include <interfaces/message.h>
#include <core/exception.h>

#define __INTERFACE_TYPE_SIZE 32
#define __INTERFACE_ID_SIZE 32

class ReadWriteLock;
class InterfaceMediator;
//class MessageQueue;

class InterfaceWriteDeniedException : public Exception
{
 public:
  InterfaceWriteDeniedException(const char *type, const char *id);
};

class InterfaceInvalidMessageException : public Exception
{
 public:
  InterfaceInvalidMessageException(const char *msg_type, const char *interface_type);
};

class Interface
{
  friend class BlackBoardInterfaceManager;

 public:
  virtual ~Interface();

  bool          oftype(const char *interface_type) const;
  unsigned int  datasize() const;
  const char *  type() const;
  const char *  id() const;
  unsigned int  serial() const;
  bool          operator== (Interface &comp) const;

  void          read();
  void          write();

  bool          hasWriter() const;

  //unsigned int  enqueue(Message *message);

 protected:
  Interface();
  virtual bool  messageValid(const Message *message) const = 0;

  void         *data_ptr;
  unsigned int  data_size;

 private:
  char               _type[__INTERFACE_TYPE_SIZE + 1];
  char               _id[__INTERFACE_ID_SIZE + 1];
  unsigned int       instance_serial;

  void *             mem_data_ptr;
  void *             mem_real_ptr;
  unsigned int       mem_serial;
  bool               write_access;

  ReadWriteLock     *rwlock;

  InterfaceMediator *interface_mediator;
  //MessageQueue      *message_queue;


  typedef struct imsg_list_t {
    imsg_list_t  *next;		/**< pointer to next element in list */
    Message       msg;		/**< pointer to message */
  };

};

typedef Interface *  (* InterfaceFactoryFunc)  (void);
typedef void         (* InterfaceDestroyFunc)  (Interface *);

#endif
