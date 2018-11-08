
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
  bb_   = blackboard;
  nhub_ = hub;
  nhub_->add_handler(this);

  observer_ = new BlackBoardNetHandlerInterfaceObserver(blackboard, hub);
}


/** Destructor. */
BlackBoardNetworkHandler::~BlackBoardNetworkHandler()
{
  delete observer_;
  nhub_->remove_handler(this);
  inbound_queue_.clear();
  // close all open interfaces
  for (lit_ = listeners_.begin(); lit_ != listeners_.end(); ++lit_) {
    delete lit_->second;
  }
  for (iit_ = interfaces_.begin(); iit_ != interfaces_.end(); ++iit_) {
    bb_->close(iit_->second);
  }
}


/** Process all network messages that have been received. */
void
BlackBoardNetworkHandler::loop()
{
  while ( ! inbound_queue_.empty() ) {
    FawkesNetworkMessage *msg = inbound_queue_.front();

    // used often and thus queried _once_
    unsigned int clid = msg->clid();

    switch (msg->msgid()) {
    case MSG_BB_LIST_ALL:
      {
	BlackBoardInterfaceListContent *ilist = new BlackBoardInterfaceListContent();
	InterfaceInfoList *infl = bb_->list_all();
	
	for (InterfaceInfoList::iterator i = infl->begin(); i != infl->end(); ++i) {
	  ilist->append_interface(*i);
	}

	try {
	  nhub_->send(clid, FAWKES_CID_BLACKBOARD, MSG_BB_INTERFACE_LIST, ilist);
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

	char type_pattern[INTERFACE_TYPE_SIZE_ + 1];
	char id_pattern[INTERFACE_ID_SIZE_ + 1];
	type_pattern[INTERFACE_TYPE_SIZE_] = 0;
	id_pattern[INTERFACE_ID_SIZE_] = 0;
	strncpy(type_pattern, lrm->type_pattern, INTERFACE_TYPE_SIZE_);
	strncpy(id_pattern, lrm->id_pattern, INTERFACE_ID_SIZE_);

	InterfaceInfoList *infl = bb_->list(type_pattern, id_pattern);
	for (InterfaceInfoList::iterator i = infl->begin(); i != infl->end(); ++i)
	{
	  ilist->append_interface(*i);
	}

	try {
	  nhub_->send(clid, FAWKES_CID_BLACKBOARD, MSG_BB_INTERFACE_LIST, ilist);
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

	char type[INTERFACE_TYPE_SIZE_ + 1];
	char id[INTERFACE_ID_SIZE_ + 1];
	type[INTERFACE_TYPE_SIZE_] = 0;
	id[INTERFACE_ID_SIZE_] = 0;
	strncpy(type, om->type, INTERFACE_TYPE_SIZE_);
	strncpy(id, om->id, INTERFACE_ID_SIZE_);

	LibLogger::log_debug("BlackBoardNetworkHandler", "Remote opens interface %s::%s",
			     type, id);
	try {
	  Interface *iface;

	  if ( msg->msgid() == MSG_BB_OPEN_FOR_READING ) {
	    iface = bb_->open_for_reading(type, id, "remote");
	  } else {
	    iface = bb_->open_for_writing(type, id, "remote");
	  }
	  if ( memcmp(iface->hash(), om->hash, INTERFACE_HASH_SIZE_) != 0 ) {
	    LibLogger::log_warn("BlackBoardNetworkHandler", "Opening interface %s::%s failed, "
				"hash mismatch", type, id);
	    send_openfailure(clid, BB_ERR_HASH_MISMATCH);
	  } else {
	    interfaces_[iface->serial()] = iface;
	    client_interfaces_[clid].push_back(iface);
	    serial_to_clid_[iface->serial()] = clid;
	    listeners_[iface->serial()] = new BlackBoardNetHandlerInterfaceListener(bb_,
										     iface,
										     nhub_,
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
	//		     interfaces_.size(), serial_to_clid_.size(),
	//		     client_interfaces_.size());

      }
      break;

    case MSG_BB_CLOSE:
      {
	bb_iserial_msg_t *sm = msg->msg<bb_iserial_msg_t>();
	unsigned int sm_serial = ntohl(sm->serial);
	if ( interfaces_.find(sm_serial) != interfaces_.end() ) {
	  bool close = false;
	  client_interfaces_.lock();
	  if ( client_interfaces_.find(clid) != client_interfaces_.end()) {
	    // this client has interfaces, check if this one as well
	    for ( ciit_ = client_interfaces_[clid].begin(); ciit_ != client_interfaces_[clid].end(); ++ciit_) {
	      if ( (*ciit_)->serial() == sm_serial ) {
		close = true;
		serial_to_clid_.erase(sm_serial);
		client_interfaces_[clid].erase(ciit_);
		if ( client_interfaces_[clid].empty() ) {
		  client_interfaces_.erase(clid);
		}
		break;
	      }
	    }
	  }
	  client_interfaces_.unlock();

	  if ( close ) {
	    interfaces_.lock();
	    LibLogger::log_debug("BlackBoardNetworkHandler", "Remote %u closing interface %s",
				 clid, interfaces_[sm_serial]->uid());
	    delete listeners_[sm_serial];
	    listeners_.erase(sm_serial);
	    bb_->close(interfaces_[sm_serial]);
	    interfaces_.erase(sm_serial);
	    interfaces_.unlock();
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
	//		     interfaces_.size(), serial_to_clid_.size(),
	//		     client_interfaces_.size());
      }
      break;

    case MSG_BB_DATA_CHANGED:
      {
	void *payload = msg->payload();
	bb_idata_msg_t *dm = (bb_idata_msg_t *)payload;
	unsigned int dm_serial = ntohl(dm->serial);
	if ( interfaces_.find(dm_serial) != interfaces_.end() ) {
	
	  if ( ntohl(dm->data_size) != interfaces_[dm_serial]->datasize() ) {
	    LibLogger::log_error("BlackBoardNetworkHandler", "DATA_CHANGED: Data size mismatch, "
				 "expected %zu, but got %zu, ignoring.",
				 interfaces_[dm_serial]->datasize(), ntohl(dm->data_size));
	  } else {
	    interfaces_[dm_serial]->set_from_chunk((char *)payload + sizeof(bb_idata_msg_t));
	    interfaces_[dm_serial]->write();
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
	if ( interfaces_.find(mm_serial) != interfaces_.end() ) {

	  if ( ! interfaces_[mm_serial]->is_writer() ) {
	    try {
	      Message *ifm = interfaces_[mm_serial]->create_message(mm->msg_type);
	      ifm->set_id(ntohl(mm->msgid));
	      ifm->set_hops(ntohl(mm->hops));

	      if ( ntohl(mm->data_size) != ifm->datasize() ) {
		LibLogger::log_error("BlackBoardNetworkHandler", "MESSAGE: Data size mismatch, "
				     "expected %zu, but got %zu, ignoring.",
				     ifm->datasize(), ntohl(mm->data_size));
	      } else {
		ifm->set_from_chunk((char *)payload + sizeof(bb_imessage_msg_t));

		interfaces_[mm_serial]->msgq_enqueue(ifm);

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
    inbound_queue_.pop_locked();
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
    nhub_->send(omsg);
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
    nhub_->send(omsg);
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
  inbound_queue_.push_locked(msg);
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
  client_interfaces_.lock();
  if ( client_interfaces_.find(clid) != client_interfaces_.end() ) {
    // Close all interfaces
    for ( ciit_ = client_interfaces_[clid].begin(); ciit_ != client_interfaces_[clid].end(); ++ciit_) {
      LibLogger::log_debug("BlackBoardNetworkHandler", "Closing interface %s::%s of remote "
			   "%u (client disconnected)",
			   (*ciit_)->type(), (*ciit_)->id(), clid);

      unsigned int serial = (*ciit_)->serial();
      serial_to_clid_.erase(serial);
      interfaces_.erase_locked(serial);
      delete listeners_[serial];
      listeners_.erase(serial);
      bb_->close(*ciit_);
    }
    client_interfaces_.erase(clid);
  }
  client_interfaces_.unlock();
}

} // end namespace fawkes
