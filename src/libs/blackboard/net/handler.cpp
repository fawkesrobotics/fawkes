
/***************************************************************************
 *  network_handler.cpp - BlackBoard Network Handler
 *
 *  Generated: Sat Mar 01 16:00:34 2008
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/net/handler.h>
#include <blackboard/net/messages.h>
#include <blackboard/net/ilist_content.h>
#include <blackboard/blackboard.h>
#include <blackboard/exceptions.h>
#include <blackboard/net/interface_listener.h>
#include <blackboard/net/interface_observer.h>

#include <interface/interface.h>
#include <interface/interface_info.h>

#include <logging/liblogger.h>
#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/hub.h>

#include <cstdlib>
#include <cstring>
#include <arpa/inet.h>

namespace fawkes {

/** @class BlackBoardNetworkHandler <blackboard/net/handler.h>
 * BlackBoard Network Handler.
 * This class provides a network handler that can be registered with the
 * FawkesServerThread to handle client requests to a BlackBoard instance.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param blackboard BlackBoard instance to provide network access to
 * @param hub Fawkes network hub
 */
BlackBoardNetworkHandler::BlackBoardNetworkHandler(BlackBoard *blackboard,
						   FawkesNetworkHub *hub)
  : Thread("BlackBoardNetworkHandler", Thread::OPMODE_WAITFORWAKEUP),
    FawkesNetworkHandler(FAWKES_CID_BLACKBOARD)
{
  __bb   = blackboard;
  __nhub = hub;
  __nhub->add_handler(this);

  __observer = new BlackBoardNetHandlerInterfaceObserver(blackboard, hub);
}


/** Destructor. */
BlackBoardNetworkHandler::~BlackBoardNetworkHandler()
{
  delete __observer;
  __nhub->remove_handler(this);
  __inbound_queue.clear();
  // close all open interfaces
  for (__lit = __listeners.begin(); __lit != __listeners.end(); ++__lit) {
    delete __lit->second;
  }
  for (__iit = __interfaces.begin(); __iit != __interfaces.end(); ++__iit) {
    __bb->close(__iit->second);
  }
}


/** Process all network messages that have been received. */
void
BlackBoardNetworkHandler::loop()
{
  while ( ! __inbound_queue.empty() ) {
    FawkesNetworkMessage *msg = __inbound_queue.front();

    // used often and thus queried _once_
    unsigned int clid = msg->clid();

    switch (msg->msgid()) {
    case MSG_BB_LIST_ALL:
      {
	BlackBoardInterfaceListContent *ilist = new BlackBoardInterfaceListContent();
	InterfaceInfoList *infl = __bb->list_all();
	
	for (InterfaceInfoList::iterator i = infl->begin(); i != infl->end(); ++i) {
	  ilist->append_interface(*i);
	}

	try {
	  __nhub->send(clid, FAWKES_CID_BLACKBOARD, MSG_BB_INTERFACE_LIST, ilist);
	} catch (Exception &e) {
	  LibLogger::log_error("BlackBoardNetworkHandler", "Failed to send interface "
			       "list to %u, exception follows", clid);
	  LibLogger::log_error("BlackBoardNetworkHandler", e);
	}
	delete infl;
      }
      break;

    case MSG_BB_LIST:
      {
	BlackBoardInterfaceListContent *ilist =
	  new BlackBoardInterfaceListContent();

	bb_ilistreq_msg_t *lrm = msg->msg<bb_ilistreq_msg_t>();

	char type_pattern[__INTERFACE_TYPE_SIZE + 1];
	char id_pattern[__INTERFACE_ID_SIZE + 1];
	type_pattern[__INTERFACE_TYPE_SIZE] = 0;
	id_pattern[__INTERFACE_ID_SIZE] = 0;
	strncpy(type_pattern, lrm->type_pattern, __INTERFACE_TYPE_SIZE);
	strncpy(id_pattern, lrm->id_pattern, __INTERFACE_ID_SIZE);

	InterfaceInfoList *infl = __bb->list(type_pattern, id_pattern);
	for (InterfaceInfoList::iterator i = infl->begin(); i != infl->end(); ++i)
	{
	  ilist->append_interface(*i);
	}

	try {
	  __nhub->send(clid, FAWKES_CID_BLACKBOARD, MSG_BB_INTERFACE_LIST, ilist);
	} catch (Exception &e) {
	  LibLogger::log_error("BlackBoardNetworkHandler", "Failed to send "
			       "interface list to %u, exception follows", clid);
	  LibLogger::log_error("BlackBoardNetworkHandler", e);
	}
	delete infl;
      }
      break;

    case MSG_BB_OPEN_FOR_READING:
    case MSG_BB_OPEN_FOR_WRITING:
      {
	bb_iopen_msg_t *om = msg->msg<bb_iopen_msg_t>();

	char type[__INTERFACE_TYPE_SIZE + 1];
	char id[__INTERFACE_ID_SIZE + 1];
	type[__INTERFACE_TYPE_SIZE] = 0;
	id[__INTERFACE_ID_SIZE] = 0;
	strncpy(type, om->type, __INTERFACE_TYPE_SIZE);
	strncpy(id, om->id, __INTERFACE_ID_SIZE);

	LibLogger::log_debug("BlackBoardNetworkHandler", "Remote opens interface %s::%s",
			     type, id);
	try {
	  Interface *iface;

	  if ( msg->msgid() == MSG_BB_OPEN_FOR_READING ) {
	    iface = __bb->open_for_reading(type, id, "remote");
	  } else {
	    iface = __bb->open_for_writing(type, id, "remote");
	  }
	  if ( memcmp(iface->hash(), om->hash, __INTERFACE_HASH_SIZE) != 0 ) {
	    LibLogger::log_warn("BlackBoardNetworkHandler", "Opening interface %s::%s failed, "
				"hash mismatch", type, id);
	    send_openfailure(clid, BB_ERR_HASH_MISMATCH);
	  } else {
	    __interfaces[iface->serial()] = iface;
	    __client_interfaces[clid].push_back(iface);
	    __serial_to_clid[iface->serial()] = clid;
	    __listeners[iface->serial()] = new BlackBoardNetHandlerInterfaceListener(__bb,
										     iface,
										     __nhub,
										     clid);
	    send_opensuccess(clid, iface);
	  }
	} catch (BlackBoardInterfaceNotFoundException &nfe) {
	  LibLogger::log_warn("BlackBoardNetworkHandler", "Opening interface %s::%s failed, "
			      "interface class not found", type, id);
	  send_openfailure(clid, BB_ERR_UNKNOWN_TYPE);
	} catch (BlackBoardWriterActiveException &wae) {
	  LibLogger::log_warn("BlackBoardNetworkHandler", "Opening interface %s::%s failed, "
			      "writer already exists", type, id);
	  send_openfailure(clid, BB_ERR_WRITER_EXISTS);
	} catch (Exception &e) {
	  LibLogger::log_warn("BlackBoardNetworkHandler", "Opening interface %s::%s failed",
			      type, id);
	  LibLogger::log_warn("BlackBoardNetworkHandler", e);
	  send_openfailure(clid, BB_ERR_UNKNOWN_ERR);
	}
	
	//LibLogger::log_debug("BBNH", "interfaces: %zu  s2c: %zu  ci: %zu",
	//		     __interfaces.size(), __serial_to_clid.size(),
	//		     __client_interfaces.size());

      }
      break;

    case MSG_BB_CLOSE:
      {
	bb_iserial_msg_t *sm = msg->msg<bb_iserial_msg_t>();
	unsigned int sm_serial = ntohl(sm->serial);
	if ( __interfaces.find(sm_serial) != __interfaces.end() ) {
	  bool close = false;
	  __client_interfaces.lock();
	  if ( __client_interfaces.find(clid) != __client_interfaces.end()) {
	    // this client has interfaces, check if this one as well
	    for ( __ciit = __client_interfaces[clid].begin(); __ciit != __client_interfaces[clid].end(); ++__ciit) {
	      if ( (*__ciit)->serial() == sm_serial ) {
		close = true;
		__serial_to_clid.erase(sm_serial);
		__client_interfaces[clid].erase(__ciit);
		if ( __client_interfaces[clid].empty() ) {
		  __client_interfaces.erase(clid);
		}
		break;
	      }
	    }
	  }
	  __client_interfaces.unlock();

	  if ( close ) {
	    __interfaces.lock();
	    LibLogger::log_debug("BlackBoardNetworkHandler", "Remote %u closing interface %s",
				 clid, __interfaces[sm_serial]->uid());
	    delete __listeners[sm_serial];
	    __listeners.erase(sm_serial);
	    __bb->close(__interfaces[sm_serial]);
	    __interfaces.erase(sm_serial);
	    __interfaces.unlock();
	  } else {
	    LibLogger::log_warn("BlackBoardNetworkHandler", "Client %u tried to close "
				"interface with serial %u, but opened by other client",
				clid, sm_serial);
	  }
	} else {
	  LibLogger::log_warn("BlackBoardNetworkHandler", "Client %u tried to close "
			      "interface with serial %u which has not been opened",
			      clid, sm_serial);
	}

	//LibLogger::log_debug("BBNH", "C: interfaces: %zu  s2c: %zu  ci: %zu",
	//		     __interfaces.size(), __serial_to_clid.size(),
	//		     __client_interfaces.size());
      }
      break;

    case MSG_BB_DATA_CHANGED:
      {
	void *payload = msg->payload();
	bb_idata_msg_t *dm = (bb_idata_msg_t *)payload;
	unsigned int dm_serial = ntohl(dm->serial);
	if ( __interfaces.find(dm_serial) != __interfaces.end() ) {
	
	  if ( ntohl(dm->data_size) != __interfaces[dm_serial]->datasize() ) {
	    LibLogger::log_error("BlackBoardNetworkHandler", "DATA_CHANGED: Data size mismatch, "
				 "expected %zu, but got %zu, ignoring.",
				 __interfaces[dm_serial]->datasize(), ntohl(dm->data_size));
	  } else {
	    __interfaces[dm_serial]->set_from_chunk((char *)payload + sizeof(bb_idata_msg_t));
	    __interfaces[dm_serial]->write();
	  }
	} else {
	  LibLogger::log_error("BlackBoardNetworkHandler", "DATA_CHANGED: Interface with "
			       "serial %u not found, ignoring.", dm_serial);
	}
      }
      break;

    case MSG_BB_INTERFACE_MESSAGE:
      {
	void *payload = msg->payload();
	bb_imessage_msg_t *mm = (bb_imessage_msg_t *)payload;
	unsigned int mm_serial = ntohl(mm->serial);
	if ( __interfaces.find(mm_serial) != __interfaces.end() ) {

	  if ( ! __interfaces[mm_serial]->is_writer() ) {
	    try {
	      Message *ifm = __interfaces[mm_serial]->create_message(mm->msg_type);
	      ifm->set_id(ntohl(mm->msgid));
	      ifm->set_hops(ntohl(mm->hops));

	      if ( ntohl(mm->data_size) != ifm->datasize() ) {
		LibLogger::log_error("BlackBoardNetworkHandler", "MESSAGE: Data size mismatch, "
				     "expected %zu, but got %zu, ignoring.",
				     ifm->datasize(), ntohl(mm->data_size));
	      } else {
		ifm->set_from_chunk((char *)payload + sizeof(bb_imessage_msg_t));

		__interfaces[mm_serial]->msgq_enqueue(ifm);

	      }
	    } catch (Exception &e) {
	      LibLogger::log_error("BlackBoardNetworkHandler", "MESSAGE: Could not create "
				   "interface message, ignoring.");
	      LibLogger::log_error("BlackBoardNetworkHandler", e);
	    }
	  } else {
	    LibLogger::log_error("BlackBoardNetworkHandler", "MESSAGE: Received message "
				 "notification, but for a writing instance, ignoring.");
	  }
	} else {
	  LibLogger::log_error("BlackBoardNetworkHandler", "DATA_CHANGED: Interface with "
			       "serial %u not found, ignoring.", mm_serial);
	}
      }
      break;

    default:
      LibLogger::log_warn("BlackBoardNetworkHandler", "Unknown message of type %u "
			  "received", msg->msgid());
      break;
    }

    msg->unref();
    __inbound_queue.pop_locked();
  }
}


void
BlackBoardNetworkHandler::send_opensuccess(unsigned int clid, Interface *interface)
{
  void *payload = calloc(1, sizeof(bb_iopensucc_msg_t) + interface->datasize());
  bb_iopensucc_msg_t *osm = (bb_iopensucc_msg_t *)payload;
  osm->serial = htonl(interface->serial());
  osm->writer_readers = htonl(interface->num_readers());
  if (interface->has_writer()) {
    osm->writer_readers |= htonl(0x80000000);
  } else {
    osm->writer_readers &= htonl(0x7FFFFFFF);
  }
  osm->data_size = htonl(interface->datasize());

  if ( ! interface->is_writer() ) {
    interface->read();
  }

  memcpy((char *)payload + sizeof(bb_iopensucc_msg_t),
	 interface->datachunk(), interface->datasize());
  
  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(clid, FAWKES_CID_BLACKBOARD,
							MSG_BB_OPEN_SUCCESS, payload,
							sizeof(bb_iopensucc_msg_t) +
							interface->datasize());
  try {
    __nhub->send(omsg);
  } catch (Exception &e) {
    LibLogger::log_error("BlackBoardNetworkHandler", "Failed to send interface "
			 "open success to %u, exception follows", clid);
    LibLogger::log_error("BlackBoardNetworkHandler", e);
  }
}


void
BlackBoardNetworkHandler::send_openfailure(unsigned int clid, unsigned int error_code)
{
  bb_iopenfail_msg_t *ofm = (bb_iopenfail_msg_t *)malloc(sizeof(bb_iopenfail_msg_t));
  ofm->error_code = htonl(error_code);

  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(clid, FAWKES_CID_BLACKBOARD,
							MSG_BB_OPEN_FAILURE, ofm,
							sizeof(bb_iopenfail_msg_t));
  try {
    __nhub->send(omsg);
  } catch (Exception &e) {
    LibLogger::log_error("BlackBoardNetworkHandler", "Failed to send interface "
			 "open failure to %u, exception follows", clid);
    LibLogger::log_error("BlackBoardNetworkHandler", e);
  }
}


/** Handle network message.
 * The message is put into the inbound queue and processed in processAfterLoop().
 * @param msg message
 */
void
BlackBoardNetworkHandler::handle_network_message(FawkesNetworkMessage *msg)
{
  msg->ref();
  __inbound_queue.push_locked(msg);
  wakeup();
}


/** Client connected. Ignored.
 * @param clid client ID
 */
void
BlackBoardNetworkHandler::client_connected(unsigned int clid)
{
}


/** Client disconnected.
 * If the client had opened any interfaces these are closed.
 * @param clid client ID
 */
void
BlackBoardNetworkHandler::client_disconnected(unsigned int clid)
{
  // close any interface that this client had opened
  __client_interfaces.lock();
  if ( __client_interfaces.find(clid) != __client_interfaces.end() ) {
    // Close all interfaces
    for ( __ciit = __client_interfaces[clid].begin(); __ciit != __client_interfaces[clid].end(); ++__ciit) {
      LibLogger::log_debug("BlackBoardNetworkHandler", "Closing interface %s::%s of remote "
			   "%u (client disconnected)",
			   (*__ciit)->type(), (*__ciit)->id(), clid);

      unsigned int serial = (*__ciit)->serial();
      __serial_to_clid.erase(serial);
      __interfaces.erase_locked(serial);
      delete __listeners[serial];
      __listeners.erase(serial);
      __bb->close(*__ciit);
    }
    __client_interfaces.erase(clid);
  }
  __client_interfaces.unlock();
}

} // end namespace fawkes
