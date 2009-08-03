
/***************************************************************************
 *  msg.cpp - IPC message queue
 *
 *  Generated: Mon Mar 13 17:44:59 2006
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

#include <utils/ipc/msg.h>
#include <utils/ipc/msg_exceptions.h>

#include <errno.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

namespace fawkes {

/// @cond INTERNALS
class IPCMessageQueueData
{
 public:
  key_t   key;
  int     msgqid;
  int     msgflg;

};
/// @endcond

/** @class IPCMessageQueue utils/ipc/msg.h
 * IPC message queue.
 * This class gives access to IPC message queues. You can use this to send
 * messages between different applications running on the same host.
 *
 * @see qa_ipc_msg.cpp
 * @ingroup IPC
 * @author Tim Niemueller
 *
 *
 * @var IPCMessageQueue::destroy_on_delete
 * destroy this message queue on delete?
 *
 */


/** Maximum size of a message.
 */
const int IPCMessageQueue::MaxMessageSize = 8192; // from linux/msg.h


/** Create or open a message queue
 * If a message key with the given identification criteria exists it is
 * opened for sending and receiving. If no such queue exists a new one is
 * create. Use isValid() to check success.
 * @param path path given to ftok to create the message queue identifier
 * @param id id given to ftok to create the message queue identifier
 * @param destroy_on_delete destroy the message queue if the dtor is called?
 * @param create Create the queue if it does not exist, do not create the queue
 *               otherwise, use isValid() to check if queue was opened
 */
IPCMessageQueue::IPCMessageQueue(const char *path, char id,
				 bool create, bool destroy_on_delete)
{
  data = new IPCMessageQueueData();

  this->destroy_on_delete = destroy_on_delete;

  data->msgflg = 0666;
  if (create) {
    data->msgflg |= IPC_CREAT;
  }

  data->key = ftok(path, id);
  data->msgqid = msgget(data->key, data->msgflg);
}


/** Create or open a message queue
 * This is a simplified version of the above function. The path is omitted
 * and . (dot, the current working directory) is used instead.
 * @param id id give to ftok to create the message queue identifier, preferably
 *           use an id from msg_registry.h
 * @param destroy_on_delete set to true to destroy the message queue if the dtor is called
 * @param create if true create the queue if it does not exist, do not create the queue
 *               otherwise, use isValid() to check if queue was opened successfully.
 */
IPCMessageQueue::IPCMessageQueue(int id, bool create, bool destroy_on_delete)
{
  data = new IPCMessageQueueData();

  this->destroy_on_delete = destroy_on_delete;

  data->msgflg = 0666;
  if (create) {
    data->msgflg |= IPC_CREAT;
  }

  data->key = id;
  data->msgqid = msgget(data->key, data->msgflg);
}


/** Destructor */
IPCMessageQueue::~IPCMessageQueue()
{
  if ((data->msgqid != -1) && destroy_on_delete) {
    msgctl(data->msgqid, IPC_RMID, 0);
  }
  delete data;
}


/** Check if the message queue is valid
 * If the queue could not be opened yet (for example if you gave create=false to the
 * constructor) isValid() will try to open the queue.
 * @return This method returns false if the message queue could not be opened
 *         or if it has been closed, it returns true if messages can be sent or received.
 */
bool
IPCMessageQueue::isValid()
{
  if (data->msgqid == -1) {
    data->msgqid = msgget(data->key, data->msgflg);
    if (data->msgqid == -1) {
      return false;
    } else {
      struct msqid_ds m;
      if (msgctl(data->msgqid, IPC_STAT, &m) != -1) {
	return true;
      } else {
	data->msgqid = -1;
	return false;
      }
    }
  } else {
    struct msqid_ds m;
    if (msgctl(data->msgqid, IPC_STAT, &m) != -1) {
      return true;
    } else {
      data->msgqid = -1;
      return false;
    }
  }
}


/** Receive messages from this queue of the given message type
 * @param mtype the message type
 * @param msg The place where the received data will be copied on success.
 * You _must_ have the mtype long field as described for MessageStruct. On recv the
 * struct does not have to be inialized,
 * but the memory has to be allocated already. See the note about the data_size!
 * @param data_size The size of the _whole_ struct, including the mtype field. NOT
 * just the size of the mtext field as for msgrcv!
 * @return returns true, if a message of the appropriate type could be read that fitted
 *         the given memory size. The received data is stored in data.
 * @see MessageStruct
 * @exception MessageTooBigException Message was too big and did not fit into buffer.
 *                                   Message remains on queue and needs to be fetched
 *                                   with a bigger buffer.
 */
bool
IPCMessageQueue::recv(long mtype, MessageStruct *msg, unsigned int data_size)
{
  if (data->msgqid == -1) return false;

  if ( msgrcv(data->msgqid, (struct msgbuf *)msg, data_size - sizeof(long),
	      mtype, IPC_NOWAIT) == -1 ) {
    if ((errno == EIDRM) || (errno == EINVAL)) {
      data->msgqid = -1;
    }
    if (errno == E2BIG) {
      throw MessageTooBigException();
    }
    return false;
  } else {
    return true;
  }
}


/** Receive messages from this queue of any type
 * @param msg a pointer to a message struct of the appropriate size. This is
 *            most likely your own incarnation. It must point to a chunk of memory
 *            which has at least max_data_size bytes.
 * @param max_data_size The maximum size the data may have.
 * @param data_size after successfuly recv will contain the number of bytes actually
 *        copied into data including the size of the mtype field!
 * @return true, if a message could be read that fitted
 *         the given memory size. The received data is stored in data. False, if
 *         no message was in the queue or the queue has been removed.
 * @see MessageStruct
 */
bool
IPCMessageQueue::recvNext(MessageStruct *msg, unsigned int max_data_size,
		       int *data_size)
{
  if (data->msgqid == -1) return false;

  if ( (*data_size = msgrcv(data->msgqid, (struct msgbuf *)msg,
			    max_data_size - sizeof(long), 0, IPC_NOWAIT)) == -1 ) {
    if ((errno == EIDRM) || (errno == EINVAL)) {
      data->msgqid = -1;
    }
    return false;
  } else {
    return true;
  }
}


/** Receive messages from this queue of the given message type
 * @param msg The data to be sent, see note for recv()
 * @param data_size the full data size (sizeof(typeof(data))), NOT just the size of the
 * mtext field (see recv()).
 * @return true, if the message could be sent, false otherwise.
 * @see MessageStruct
 */
bool
IPCMessageQueue::send(MessageStruct *msg, unsigned int data_size)
{
  if (data->msgqid == -1) return false;

  if (msgsnd(data->msgqid, msg, data_size - sizeof(long), IPC_NOWAIT) == -1) {
    if (errno == EIDRM) {
      data->msgqid = -1;
    }
    return false;
  } else {
    return true;
  }
}


} // end namespace fawkes
