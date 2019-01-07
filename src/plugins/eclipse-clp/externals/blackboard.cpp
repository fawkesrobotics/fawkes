
/***************************************************************************
 *  blackboard.h - External predicates to remotely access the Fawkes
 *                 blackboard
 *
 *  Created: Wed Mar 09 17:10:54 2011
 *  Copyright  2011  Daniel Beck
 *             2014  Tim Niemueller
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "blackboard.h"


#include <logging/logger.h>
#include <plugins/eclipse-clp/eclipse_thread.h>

#include <cstring>
#include <cstdlib>

#include <plugins/eclipse-clp/blackboard_listener_thread.h>

namespace fawkes{
/** @class fawkes::EclExternalBlackBoard
 * Wrapper class for using the blackboard in the implementation of the external
 * predicates.
 * @author Daniel Beck
 */

BlackBoard* EclExternalBlackBoard::m_blackboard = NULL;
EclExternalBlackBoard*  EclExternalBlackBoard::m_instance = NULL;

  /** Constructor. */
  EclExternalBlackBoard::EclExternalBlackBoard(BlackBoard *blackboard, Logger *logger)
  {
    if (m_instance != NULL) {
      throw Exception("There is already an instance of type "
          "EclExternalBlackBoard instantiated");
    }
    m_blackboard = blackboard;
    m_logger = logger;
  }

  /** Destructor. */
  EclExternalBlackBoard::~EclExternalBlackBoard()
  {}

  /** Creates the initial EclExternalBlackBoard object
   * @param bb pointer to the BlackBoard to be used
   * @param logger pointer to the Logger to be used
   */
  void EclExternalBlackBoard::create_initial_object(BlackBoard *bb, Logger *logger)
  {
      m_instance = new EclExternalBlackBoard(bb, logger);
  }

  /** Delete the current EclExternalBlackBoard instance and set it to NULL */
  void EclExternalBlackBoard::cleanup_instance()
  {
    if (m_instance) {
      delete m_instance;
    }
    m_instance = NULL;
  }


  /** Get the EclExternalBlackBoard instance.
  * @return the instance
  */
  EclExternalBlackBoard* EclExternalBlackBoard::instance()
  {
    return m_instance;
  }


  /** Access the BlackBoard instance.
   * @return the blackboard instance
   */
  BlackBoard* EclExternalBlackBoard::blackboard_instance()
  {
    return m_blackboard;
  }

  /** Obtain the list of opened interfaces.
   * @return list of opened interfaces
   */
  std::map<std::string, Interface *>& EclExternalBlackBoard::interfaces()
  {
    return m_interfaces;
  }

}

using namespace fawkes;


bool process_message_args(Message* msg, EC_word arg_list);


int
p_bb_open_interface()
{
  EC_atom mode;
  char* interface_type;
  char* interface_id;

  if (EC_succeed != EC_arg(1).is_atom(&mode))
  {
    fprintf(stderr, "p_bb_open_interface(): no mode given\n");
    return EC_fail;
  }

  if (EC_succeed != EC_arg(2).is_string(&interface_type))
  {
    fprintf(stderr, "p_bb_open_interface(): no type given\n");
    return EC_fail;
  }

  if (EC_succeed != EC_arg (3).is_string(&interface_id))
  {
    fprintf(stderr, "p_bb_open_interface(): no id given\n");
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  std::string uid = std::string(interface_type) + "::" + interface_id;
  if (interfaces.find(uid) == interfaces.end()) {
    try {
      Interface* iface;

      if (0 == strcmp("w", mode.name()))
      {
	iface = EclExternalBlackBoard::instance()->
	  blackboard_instance()->open_for_writing(interface_type, interface_id);
      } else {
	iface = EclExternalBlackBoard::instance()->
	  blackboard_instance()->open_for_reading(interface_type, interface_id);
      }

      interfaces[iface->uid()] = iface;
    } catch (Exception& e) {
      fprintf(stderr, "p_bb_open_interface() failed: %s\n", e.what_no_backtrace());
      return EC_fail;
    }
  }

  if (interfaces.find(uid) == interfaces.end()) {
    return EC_fail;
  }

  return EC_succeed;
}


int
p_bb_close_interface()
{
  char* uid;

  if (EC_succeed != EC_arg(1).is_string(&uid))
  {
    fprintf(stderr, "p_bb_close_interface(): no id given\n");
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  if (interfaces.find(uid) != interfaces.end()) {
    EclExternalBlackBoard::instance()->blackboard_instance()->close(interfaces[uid]);
    EclExternalBlackBoard::instance()->interfaces().erase(uid);
  }

  return EC_succeed;
}


int
p_bb_has_writer()
{
  char* uid;

  if (EC_succeed != EC_arg(1).is_string(&uid)) {
    fprintf(stderr, "p_bb_has_writer(): no uid given\n");
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  if (interfaces.find(uid) != interfaces.end()) {
    return interfaces[uid]->has_writer() ? EC_succeed : EC_fail;
  }

  return EC_fail;
}


int
p_bb_instance_serial()
{
  char* uid;
  if (EC_succeed != EC_arg(1).is_string(&uid))
  {
    fprintf(stderr, "p_bb_instance_serial(): no interface uid given\n");
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  if (interfaces.find(uid) != interfaces.end()) {

    if (EC_succeed != EC_arg(2).unify(interfaces[uid]->serial()))  {
      fprintf(stderr, "p_bb_instance_serial(): could not bind return value\n");
      return EC_fail;
    } else {
      return EC_succeed;
    }
  }

  return EC_fail;
}


int
p_bb_read_interfaces()
{
  for (std::map<std::string, Interface *>::iterator it = EclExternalBlackBoard::instance()->interfaces().begin();
	it != EclExternalBlackBoard::instance()->interfaces().end();
	++it)
  {
    it->second->read();
  }

  return EC_succeed;
}

int
p_bb_read_interface()
{
  char *uid;
  if (EC_succeed != EC_arg(1).is_string(&uid))
  {
    fprintf(stderr, "p_read_interface(): no interface UID given\n");
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  if (interfaces.find(uid) == interfaces.end()) {
    fprintf(stderr, "p_bb_read_interface: interface %s has not been opened\n", uid);
    return EC_fail;
  }

  interfaces[uid]->read();

  return EC_succeed;
}

int
p_bb_write_interfaces()
{
  for (std::map<std::string, Interface *>::iterator it = EclExternalBlackBoard::instance()->interfaces().begin();
	it != EclExternalBlackBoard::instance()->interfaces().end();
	++it)
  {
    if (it->second->is_writer()) { it->second->write(); }
  }

  return EC_succeed;
}

int
p_bb_write_interface()
{
  char *uid;
  if (EC_succeed != EC_arg(1).is_string(&uid))
  {
    fprintf(stderr, "p_read_interface(): no interface UID given\n");
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  if (interfaces.find(uid) == interfaces.end()) {
    fprintf(stderr, "p_bb_read_interface: interface %s has not been opened\n", uid);
    return EC_fail;
  }

  if (! interfaces[uid]->is_writer()) {
    fprintf(stderr, "p_bb_set(): interface %s not a writer\n", uid);
    return EC_fail;
  }

  interfaces[uid]->write();
  return EC_succeed;
}

int
p_bb_interface_changed()
{
  char *uid;
  if (EC_succeed != EC_arg(1).is_string(&uid))
  {
    fprintf(stderr, "p_interface_changed(): no interface UID given\n");
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  if (interfaces.find(uid) == interfaces.end()) {
    fprintf(stderr, "p_bb_interface_changed: interface %s has not been opened\n", uid);
    return EC_fail;
  }

  return interfaces[uid]->changed() ? EC_succeed : EC_fail;
}


int
p_bb_get()
{
  char* uid;
  char* field;

  if (EC_succeed != EC_arg(1).is_string(&uid))
  {
    fprintf(stderr, "p_bb_get(): no interface uid given\n");
    return EC_fail;
  }

  if (EC_succeed != EC_arg(2).is_string(&field))
  {
    fprintf(stderr, "p_bb_get(): no field given\n");
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  if (interfaces.find(uid) != interfaces.end()) {
    Interface *iface = interfaces[uid];

    InterfaceFieldIterator fit;
    for (fit = iface->fields(); fit != iface->fields_end(); ++fit) {
      if (0 == strcmp(field, fit.get_name())) {
	switch (fit.get_type()) {
	case IFT_BOOL:
	  if (fit.get_bool()) {
	    if (EC_succeed != EC_arg(3).unify(EC_atom((char*) "true"))) {
	      fprintf(stderr, "p_bb_get(): could not bind return value\n");
	      return EC_fail;
	    }
	  } else {
	    if (EC_succeed != EC_arg(3).unify(EC_atom((char*) "false"))) {
	      fprintf(stderr, "p_bb_get(): could not bind return value\n");
	      return EC_fail;
	    }
	  }
	  break;
	    
	case IFT_INT8:
	  if (EC_succeed != EC_arg(3).unify(EC_word((long) fit.get_int8()))) {
	    fprintf(stderr, "p_bb_get(): could not bind return value\n");
	    return EC_fail;
	  }
	  break;
	    
	case IFT_UINT8:
	  if (EC_succeed != EC_arg(3).unify(EC_word((long) fit.get_uint8()))) {
	    fprintf(stderr, "p_bb_get(): could not bind return value\n");
	    return EC_fail;
	  }
	  break;
	    
	case IFT_INT16:
	  if (EC_succeed != EC_arg(3).unify(EC_word((long) fit.get_int16()))) {
	    fprintf(stderr, "p_bb_get(): could not bind return value\n");
	    return EC_fail;
	  }
	  break;
	    
	case IFT_UINT16:
	  if (EC_succeed != EC_arg(3).unify(EC_word((long) fit.get_uint16()))) {
	    fprintf(stderr, "p_bb_get(): could not bind return value\n");
	    return EC_fail;
	  }
	  break;
	    
	case IFT_INT32:
	  if (EC_succeed != EC_arg(3).unify(EC_word((long) fit.get_int32()))) {
	    fprintf(stderr, "p_bb_get: could not bind value\n");
	    return EC_fail;
	  }
	  break;
	    
	case IFT_UINT32:
	  if (EC_succeed != EC_arg(3).unify(EC_word((long) fit.get_uint32()))) {
	    fprintf(stderr, "p_bb_get(): could not bind return value\n");
	    return EC_fail;
	  }
	  break;
	    
	case IFT_INT64:
	  if (EC_succeed != EC_arg(3).unify(EC_word((long) fit.get_int64()))) {
	    fprintf(stderr, "p_bb_get(): could not bind return value\n");
	    return EC_fail;
	  }
	  break;
	    
	case IFT_UINT64:
	  if (EC_succeed != EC_arg(3).unify(EC_word((long) fit.get_uint64()))) {
	    fprintf(stderr, "p_bb_get(): could not bind return value\n");
	    return EC_fail;
	  }
	  break;

	case IFT_FLOAT:
	  if (fit.get_length() > 1) {
	    EC_word res = nil();
	    float* f_array = fit.get_floats();
	    for (int i=fit.get_length() - 1; i >= 0; --i)
              res = ::list(EC_word(f_array[i]), res);
	    if (EC_succeed != EC_arg(3).unify(res)) {
	      fprintf(stderr, "p_bb_get(): could not bind return value\n");
	      return EC_fail;
	    }
	  } else {
	    if (EC_succeed != EC_arg(3).unify(EC_word((double) fit.get_float()))) {
	      fprintf(stderr, "p_bb_get(): could not bind return value\n");
	      return EC_fail;
	    }
	  }
	  break;

	case IFT_DOUBLE:
	  if (fit.get_length() > 1) {
	    EC_word res = nil();
	    double* double_array = fit.get_doubles();
	    for (int i=fit.get_length() - 1; i >= 0; --i)
              res = ::list(EC_word(double_array[i]), res);
	    if (EC_succeed != EC_arg(3).unify(res)) {
	      fprintf(stderr, "p_bb_get(): could not bind return value\n");
	      return EC_fail;
	    }
	  } else {
	    if (EC_succeed != EC_arg(3).unify(EC_word((double) fit.get_double()))) {
	      fprintf(stderr, "p_bb_get(): could not bind return value\n");
	      return EC_fail;
	    }
	  }
	  break;

	case IFT_STRING:
	  if (EC_succeed != EC_arg(3).unify(EC_word(fit.get_string()))) {
	    fprintf(stderr, "p_bb_get(): could not bind return value\n");
	    return EC_fail;
	  }
	  break;
	    
	case IFT_BYTE:
	  if (fit.get_length() > 1)
	  {
	    EC_word res = nil();
	    uint8_t* array = fit.get_bytes();
	    for (int i=fit.get_length()-1; i>= 0; i--)
              res = ::list( EC_word( (long) array[i]), res);
	    if ( EC_succeed != EC_arg( 3 ).unify( res ) )
	    {
	      printf( "p_bb_get(): could not bind return value\n" );
	      return EC_fail;
	    }
	  } else
	  {
	    if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (long) fit.get_byte() ) ) )
	    {
	      printf( "p_bb_get(): could not bind return value\n" );
	      return EC_fail;
	    }
	 }
	 break;

	case IFT_ENUM:
	  if (EC_succeed != EC_arg(3).unify(fit.get_value_string())) {
	    fprintf(stderr, "p_bb_get(): could not bind return value\n");
	    return EC_fail;
	  }
	  break;

	default:
	  fprintf(stderr, "p_bb_get(): could not find type of interface! Type: %s (%d)", fit.get_typename(), fit.get_type());
	  break;
	}
	break;
      }
    }

    if (fit == iface->fields_end()) {
      fprintf(stderr, "p_bb_get(): interface %s has no field %s\n", uid, field);
      return EC_fail;
    }

  } else {
    fprintf(stderr, "p_bb_get(): no interface with id %s found\n", uid);
    return EC_fail;
  }

  return EC_succeed;
}


int
p_bb_set()
{
  char* uid;
  char* field;

  if (EC_succeed != EC_arg(1).is_string(&uid)) {
    fprintf(stderr, "p_bb_set(): no interface id given\n");
    return EC_fail;
  }

  if (EC_succeed != EC_arg(2).is_string(&field)) {
    fprintf(stderr, "p_bb_set(): no field given\n");
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  if (interfaces.find(uid) != interfaces.end()) {
    Interface *iface = interfaces[uid];

    if (!iface->is_writer()) {
      fprintf(stderr, "p_bb_set(): interface %s not a writer\n", uid);
      return EC_fail;
    }

    InterfaceFieldIterator fit;
    for (fit = iface->fields(); fit != iface->fields_end(); ++fit) {
      if (0 == strcmp(field, fit.get_name())) {
	switch (fit.get_type()) {
	case IFT_BOOL:
	  {
	    EC_atom val;
	    if (EC_succeed != EC_arg(3).is_atom(&val)) {
	      fprintf(stderr, "p_bb_set(): no value_given\n");
	      return EC_fail;
	    }

	    if (0 == strcmp("true", val.name()))
	    { fit.set_bool(true); }
	    else if (0 == strcmp("false", val.name()))
	    { fit.set_bool(false); }
	    else
	    {
	      fprintf(stderr, "p_bb_set(): boolean value neither true nor false\n");
	      return EC_fail;
	    }
	  }
	  break;
	    
	case IFT_INT8:
	  {
	    long val;
	    if (EC_succeed != EC_arg(3).is_long(&val))
	    {
	      fprintf(stderr, "p_bb_set(): no value given\n");
	      return EC_fail;
	    }

	    fit.set_int8((int8_t) val);
	  }
	  break;
	    
	case IFT_UINT8:
	  {
	    long val;
	    if (EC_succeed != EC_arg(3).is_long(&val))
	    {
	      fprintf(stderr, "p_bb_set(): no value given\n");
	      return EC_fail;
	    }

	    fit.set_uint8((uint8_t) val);
	  }
	  break;
	    
	case IFT_INT16:
	  {
	    long val;
	    if (EC_succeed != EC_arg(3).is_long(&val))
	    {
	      fprintf(stderr, "p_bb_set(): no value given\n");
	      return EC_fail;
	    }

	    fit.set_int16((int16_t) val);
	  }
	  break;
	    
	case IFT_UINT16:
	  {
	    long val;
	    if (EC_succeed != EC_arg(3).is_long(&val))
	    {
	      fprintf(stderr, "p_bb_set(): no value given\n");
	      return EC_fail;
	    }

	    fit.set_uint16((uint16_t) val);
	  }
	  break;
	    
	case IFT_INT32:
	  {
	    long val;
	    if (EC_succeed != EC_arg(3).is_long(&val))
	    {
	      fprintf(stderr, "p_bb_set(): no value given\n");
	      return EC_fail;
	    }

	    fit.set_int32((int32_t) val);
	  }
	  break;
	    
	case IFT_UINT32:
	  {
	    long val;
	    if (EC_succeed != EC_arg(3).is_long(&val))
	    {
	      fprintf(stderr, "p_bb_set(): no value given\n");
	      return EC_fail;
	    }

	    fit.set_uint32((uint32_t) val);
	  }
	  break;
	    
	case IFT_INT64:
	  {
	    long val;
	    if (EC_succeed != EC_arg(3).is_long(&val))
	    {
	      fprintf(stderr, "p_bb_set(): no value given\n");
	      return EC_fail;
	    }

	    fit.set_int64((int64_t) val);
	  }
	  break;
	    
	case IFT_UINT64:
	  {
	    long val;
	    if (EC_succeed != EC_arg(3).is_long(&val))
	    {
	      fprintf(stderr, "p_bb_set(): no value given\n");
	      return EC_fail;
	    }

	    fit.set_uint64((uint64_t) val);
	  }
	  break;
	    
	case IFT_FLOAT:
	  {
	    double val;
	    if (EC_succeed != EC_arg(3).is_double(&val))
	    {
	      fprintf(stderr, "p_bb_set(): no value given\n");
	      return EC_fail;
	    }

	    fit.set_float((float) val);
	  }
	  break;
	    
	case IFT_STRING:
	  {
	    char* val;
	    if (EC_succeed != EC_arg(3).is_string(&val))
	    {
	      fprintf(stderr, "p_bb_set(): no value given\n");
	      return EC_fail;
	    }

	    fit.set_string(val);
	  }
	  break;
	    
	case IFT_BYTE:
	case IFT_ENUM:
	  fprintf(stderr, "p_bb_set(): NOT YET IMPLEMENTET\n");
	  break;

	default:
	  break;
	}
	break;
      }
    }

    if (fit == iface->fields_end()) {
      fprintf(stderr, "p_bb_set(): interface %s has no field %s\n", uid, field);
      return EC_fail;
    }

  } else {
   fprintf(stderr, "p_bb_set(): no interface with id %s found\n", uid);
    
   return EC_fail;
  }

  return EC_succeed;
}


int
p_bb_send_message()
{
  char* uid;
  char* message_type;

  if (EC_succeed != EC_arg(1).is_string(&uid)) {
    fprintf(stderr, "p_bb_send_message(): no interface id given\n");
    return EC_fail;
  }

  if (EC_succeed != EC_arg(2).is_string(&message_type))
  {
    fprintf(stderr, "p_bb_send_message(): no message type given\n");
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  if (interfaces.find(uid) != interfaces.end()) {
    Interface *iface = interfaces[uid];

    if (iface->is_writer()) {
      fprintf(stderr, "p_bb_send_message(): interface with uid %s is a writer\n", uid);
      return EC_fail;
    }

    try {
      Message* msg = iface->create_message(message_type);

      EC_word head;
      EC_word tail;
      if (EC_succeed == EC_arg(3).is_list(head, tail)) {
	if (!process_message_args(msg, ::list(head, tail))) {
	  return EC_fail;
	};
      }

	msg->ref();
	try {
		(iface)->msgq_enqueue( msg );
		// return the msgID as 4th argument
		EC_arg( 4 ).unify((int)(msg->id()));
		msg->unref();
	} catch (Exception &e) {
		msg->unref();
		e.print_trace();
		return EC_fail;
	}

    } catch (Exception& e) {
      fprintf(stderr, "p_bb_send_message() failed: %s\n", e.what_no_backtrace());
      return EC_fail;
    }
  } else {
    fprintf(stderr, "p_bb_send_message(): no interface with name %s\n", uid);
    return EC_fail;
  }

  return EC_succeed;
}


int
p_bb_recv_messages()
{
  char* uid;

  if (EC_succeed != EC_arg(1).is_string(&uid)) {
    fprintf(stderr, "p_bb_recv_messages(): no interface uid given\n");
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  if (interfaces.find(uid) != interfaces.end()) {
    Interface *iface = interfaces[uid];

    if (! iface->is_writer()) {
      fprintf(stderr, "p_bb_recv_messages(): interface with uid %s is not a writer\n", uid);
      return EC_fail;
    }

    EC_word msg_list = nil();
    
    while (!iface->msgq_empty()) {
      Message* msg = iface->msgq_first();

      // construct list of key-value pairs: [[field1, val1], [field2, val2], ...]
      EC_word args = nil();
      for (InterfaceFieldIterator fit = msg->fields(); fit != msg->fields_end(); ++fit) {
	EC_word value;

	switch (fit.get_type()) {
	case IFT_BOOL:
	  if (fit.get_bool())
	  { value = EC_atom((char*) "true"); }
	  else
	  { value = EC_atom((char*) "false"); }

	  break;

	case IFT_INT8:
	  value = EC_word((long) fit.get_int8());
	  break;

	case IFT_UINT8:
	  value = EC_word((long) fit.get_uint8());
	  break;

	case IFT_INT16:
	  value = EC_word((long) fit.get_int16());
	  break;

	case IFT_UINT16:
	  value = EC_word((long) fit.get_uint16());
	  break;

	case IFT_INT32:
	  value = EC_word((long) fit.get_int32());
	  break;

	case IFT_UINT32:
	  value = EC_word((long) fit.get_uint32());
	  break;

	case IFT_INT64:
	  value = EC_word((long) fit.get_int64());
	  break;

	case IFT_UINT64:
	  value = EC_word((long) fit.get_uint64());
	  break;

	case IFT_FLOAT:
	  value = EC_word((double) fit.get_float());
	  break;

	case IFT_STRING:
	  value = EC_word(fit.get_string());
	  break;

	case IFT_BYTE:
	case IFT_ENUM:
	  fprintf(stderr, "p_bb_recv_messages(): NOT YET IMPLEMENTED\n");
	  break;

	default:
	  fprintf(stderr, "p_bb_recv_messages(): unknown field type\n");
	}

	EC_word field = ::list(EC_word(fit.get_name()),
			       ::list(value, nil()));
	args = ::list(field, args);
      }

      // construct list of messages: [[MsgType, [[Key1, Val1], ...]], ... ]
      msg_list = ::list(::list(EC_word(msg->type()),
			       ::list(args, nil())),
			msg_list);

      iface->msgq_pop();
    }

    if (EC_succeed != EC_arg(2).unify(msg_list)) {
      fprintf(stderr, "p_bb_recv_messages(): could not bind return value\n");
      return EC_fail;
    }

  } else {
    fprintf(stderr, "p_bb_recv_messages(): no interface with id %s found\n", uid);
    return EC_fail;
  }

  return EC_succeed;
}


int
p_bb_observe_pattern()
{
  char *type_pattern, *id_pattern;
  if (EC_succeed != EC_arg(1).is_string(&type_pattern)) {
    EclExternalBlackBoard::logger()->log_error(
          EclExternalBlackBoard::name(), "%s: First argument must be a string.", __func__);
    return EC_fail;
  }
  if (EC_succeed != EC_arg(2).is_string(&id_pattern)) {
    EclExternalBlackBoard::logger()->log_error(
          EclExternalBlackBoard::name(), "%s: Second argument must be a string.", __func__);
    return EC_fail;
  }

  BlackboardListenerThread::instance()->observe_pattern(type_pattern, id_pattern);
  return EC_succeed;
}


int
p_bb_listen_for_change()
{
  char *type, *id;
  if (EC_succeed != EC_arg(1).is_string(&type)) {
    EclExternalBlackBoard::logger()->log_error(
          EclExternalBlackBoard::name(), "%s: First argument must be a string.", __func__);
    return EC_fail;
  }
  if (EC_succeed != EC_arg(2).is_string(&id)) {
    EclExternalBlackBoard::logger()->log_error(
          EclExternalBlackBoard::name(), "%s: Second argument must be a string.", __func__);
    return EC_fail;
  }

  std::map<std::string, Interface *> &interfaces =
    EclExternalBlackBoard::instance()->interfaces();

  std::string uid = std::string(type) + "::" + id;
  std::map<std::string, Interface *>::iterator iface_it = interfaces.find(uid);

  if (iface_it == interfaces.end()) {
    EclExternalBlackBoard::logger()->log_error(
          EclExternalBlackBoard::name(), "%s: Interface %s has not been opened.", __func__, uid.c_str());
    return EC_fail;
  }

  BlackboardListenerThread::instance()->listen_for_change(iface_it->second);
  return EC_succeed;
}


bool
process_message_args(Message* msg, EC_word arg_list)
{
  EC_word head;
  EC_word tail;

  for (; EC_succeed == arg_list.is_list(head, tail) ; arg_list = tail)
  {
    // [field, value]
    EC_word field;
    EC_word value;
    EC_word t1;
    EC_word t2;

    if (EC_succeed != head.is_list(field, t1) ||
	 EC_succeed != t1.is_list(value, t2)       )
    {
      fprintf(stderr, "p_bb_send_messge(): could not parse argument list\n");
      return false;
    }

    char* field_name;
    if (EC_succeed != field.is_string(&field_name))
    {
      fprintf(stderr, "p_bb_send_message(): malformed argument list\n");
      return false;
    }

    InterfaceFieldIterator fit;
    for (fit = msg->fields();
	  fit != msg->fields_end();
	  ++fit)
    {
      if (0 == strcmp(fit.get_name(), field_name))
      {
	switch(fit.get_type())
	{
	case IFT_BOOL:
	  {
	    EC_atom val;
	    if (EC_succeed != value.is_atom(&val))
	    {
	      fprintf(stderr, "p_bb_send_message(): no value_given (bool)\n");
	      return false;
	    }

	    if (0 == strcmp("true", val.name()))
	    { fit.set_bool(true); }
	    else if (0 == strcmp("false", val.name()))
	    { fit.set_bool(false); }
	    else
	    {
	      fprintf(stderr, "p_bb_send_message(): boolean value neither true nor false\n");
	      return false;
	    }
	  }

	  break;

	case IFT_INT8:
	  {
	    long val;
	    if (EC_succeed != value.is_long(&val))
	    {
	      fprintf(stderr, "p_bb_send_message(): no value given (int8)\n");
	      return false;
	    }

	    fit.set_int8((int8_t) val);
	  }

	  break;

	case IFT_UINT8:
	  {
	    long val;
	    if (EC_succeed != value.is_long(&val))
	    {
	      fprintf(stderr, "p_bb_send_message(): no value given (uint8)\n");
	      return false;
	    }

	    fit.set_uint8((uint8_t) val);
	  }

	  break;

	case IFT_INT16:
	  {
	    long val;
	    if (EC_succeed != value.is_long(&val))
	    {
	      fprintf(stderr, "p_bb_send_message(): no value given (int16)\n");
	      return false;
	    }

	    fit.set_int16((int16_t) val);
	  }

	  break;

	case IFT_UINT16:
	  {
	    long val;
	    if (EC_succeed != value.is_long(&val))
	    {
	      fprintf(stderr, "p_bb_send_message(): no value given (uint16)\n");
	      return false;
	    }

	    fit.set_uint16((uint16_t) val);
	  }

	  break;

	case IFT_INT32:
	  {
	    long val;
	    if (EC_succeed != value.is_long(&val))
	    {
	      fprintf(stderr, "p_bb_send_message(): no value given (int32)\n");
	      return false;
	    }

	    fit.set_int32((int32_t) val);
	  }

	  break;

	case IFT_UINT32:
	  {
	    long val;
	    if (EC_succeed != value.is_long(&val))
	    {
	      fprintf(stderr, "p_bb_send_message(): no value given (uint32)\n");
	      return false;
	    }

	    fit.set_uint32((uint32_t) val);
	  }

	  break;

	case IFT_INT64:
	  {
	    long val;
	    if (EC_succeed != value.is_long(&val))
	    {
	      fprintf(stderr, "p_bb_send_message(): no value given (int64)\n");
	      return false;
	    }

	    fit.set_int64((int64_t) val);
	  }

	  break;

	case IFT_UINT64:
	  {
	    long val;
	    if (EC_succeed != value.is_long(&val))
	    {
	      fprintf(stderr, "p_bb_send_message(): no value given (uint64)\n");
	      return false;
	    }

	    fit.set_uint64((uint64_t) val);
	  }

	  break;

	case IFT_FLOAT:
	  {
	    double val;
	    if (EC_succeed != value.is_double(&val))
	    {
	      fprintf(stderr, "p_bb_send_message(): no value given (float)\n");
	      return false;
	    }

	    fit.set_float((float) val);
	  }

	  break;

	case IFT_STRING:
	  {
	    char* val;
	    if (EC_succeed != value.is_string(&val))
	    {
	      fprintf(stderr, "p_bb_send_message(): no value given (string)\n");
	      return false;
	    }

	    fit.set_string(val);
	  }

	  break;

	case IFT_BYTE:
	case IFT_ENUM:
	  fprintf(stderr, "p_bb_send_message(): NOT YET IMPLEMENTET\n");
	  break;

	default:
	  break;
	}

	break;
      }
    }

    if (fit == msg->fields_end())
    {
      fprintf(stderr, "p_bb_send_message(): message has no field with name %s\n",
	      field_name);
      return false;
    }
  }

  return true;
}
