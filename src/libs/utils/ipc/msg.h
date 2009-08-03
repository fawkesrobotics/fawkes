
/***************************************************************************
 *  msg.h - IPC message queue
 *
 *  Generated: Mon Mar 13 17:37:49 2006 (from FireVision)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_IPC_MSG_H_
#define __UTILS_IPC_MSG_H_

namespace fawkes {


class IPCMessageQueueData;

class IPCMessageQueue {
 public:

  static const int MaxMessageSize;

  /** This is the struct of the messages that has to be fed to send and
   * receive methods.
   */
  typedef struct {
    long int mtype;      /**< type of the message */
    char     mtext[1];   /**< content of the message, can be of arbitrary
			  * size up to MaxMessageSize
			  */
  } MessageStruct;

  IPCMessageQueue(const char *path, char id,
		  bool create = false,
		  bool destroy_on_delete = false);
  
  IPCMessageQueue(int id,
		  bool create = false,
		  bool destroy_on_delete = false);

  ~IPCMessageQueue();

  bool isValid();
  bool recv(long mtype, MessageStruct *msg, unsigned int data_size);
  bool recvNext(MessageStruct *msg, unsigned int max_data_size, int *data_size);
  bool send(MessageStruct *msg, unsigned int data_size);

  /** Get the message type
   * @param buffer the buffer of the message as returned by getMessage()
   * @return the message type
   */
  static inline long
  mtype(char *buffer)
  {
    return (((MessageStruct *)buffer)->mtype);
  }

 protected:
  bool destroy_on_delete;

 private:
  IPCMessageQueueData *data;

};


} // end namespace fawkes

#endif
