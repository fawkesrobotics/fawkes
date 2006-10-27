
/***************************************************************************
 *  message.h - BlackBoard message
 *
 *  Generated: Sun Oct 08 00:08:10 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __MESSAGE_H_
#define __MESSAGE_H_

#include <core/utils/refcount.h>

class Mutex;

class Message : public RefCount
{
  friend class MessageQueue;
  friend class Interface;
 public:
  Message();
  Message(Message *mesg);
  Message(Message &mesg);
  virtual ~Message();

  Message &         operator=  (const Message & m);

 private:
  virtual void *        data();
  virtual unsigned int  datasize();

  unsigned int  message_id;

  unsigned int  passing_interface_id;  
  unsigned int  originating_interface_id;  

 protected:
  void         *data_ptr;
  unsigned int  data_size;
};

#endif
