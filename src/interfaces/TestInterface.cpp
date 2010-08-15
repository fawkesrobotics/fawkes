
/***************************************************************************
 *  TestInterface.cpp - Fawkes BlackBoard Interface - TestInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2006-2007  Tim Niemueller
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

#include <interfaces/TestInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class TestInterface <interfaces/TestInterface.h>
 * TestInterface Fawkes BlackBoard Interface.
 * Test interface. Use this to play around. Do NOT remove any fields, as this
      interface is used by BlackBoard QA.
 * @ingroup FawkesInterfaces
 */


/** TEST_CONSTANT constant */
const int32_t TestInterface::TEST_CONSTANT = 5;
/** TEST_FLOAT_CONSTANT constant */
const float TestInterface::TEST_FLOAT_CONSTANT = 1.2;

/** Constructor */
TestInterface::TestInterface() : Interface()
{
  data_size = sizeof(TestInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (TestInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_BOOL, "test_bool", 1, &data->test_bool);
  add_fieldinfo(IFT_INT32, "test_int", 1, &data->test_int);
  add_fieldinfo(IFT_BYTE, "flags", 1, &data->flags);
  add_fieldinfo(IFT_STRING, "test_string", 30, data->test_string);
  add_fieldinfo(IFT_INT32, "result", 1, &data->result);
  add_fieldinfo(IFT_UINT32, "test_uint", 1, &data->test_uint);
  add_fieldinfo(IFT_UINT64, "test_ulint", 1, &data->test_ulint);
  add_fieldinfo(IFT_INT64, "test_lint", 1, &data->test_lint);
  add_messageinfo("SetTestIntMessage");
  add_messageinfo("SetTestStringMessage");
  add_messageinfo("CalculateMessage");
  unsigned char tmp_hash[] = {0x76, 0x67, 0x99, 0x23, 0x8, 0xa7, 0xba, 0xe0, 0x34, 0x70, 0x34, 0xf8, 0xe6, 0x99, 0x7d, 0x99};
  set_hash(tmp_hash);
}

/** Destructor */
TestInterface::~TestInterface()
{
  free(data_ptr);
}
/** Convert TestEnum constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
TestInterface::tostring_TestEnum(TestEnum value) const
{
  switch (value) {
  case TEST_ENUM_1: return "TEST_ENUM_1";
  case TEST_ENUM_2: return "TEST_ENUM_2";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get test_bool value.
 * Test Bool
 * @return test_bool value
 */
bool
TestInterface::is_test_bool() const
{
  return data->test_bool;
}

/** Get maximum length of test_bool value.
 * @return length of test_bool value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::maxlenof_test_bool() const
{
  return 1;
}

/** Set test_bool value.
 * Test Bool
 * @param new_test_bool new test_bool value
 */
void
TestInterface::set_test_bool(const bool new_test_bool)
{
  data->test_bool = new_test_bool;
  data_changed = true;
}

/** Get test_int value.
 * Test integer
 * @return test_int value
 */
int32_t
TestInterface::test_int() const
{
  return data->test_int;
}

/** Get maximum length of test_int value.
 * @return length of test_int value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::maxlenof_test_int() const
{
  return 1;
}

/** Set test_int value.
 * Test integer
 * @param new_test_int new test_int value
 */
void
TestInterface::set_test_int(const int32_t new_test_int)
{
  data->test_int = new_test_int;
  data_changed = true;
}

/** Get flags value.
 * Flags spit down by the writer
 * @return flags value
 */
uint8_t
TestInterface::flags() const
{
  return data->flags;
}

/** Get maximum length of flags value.
 * @return length of flags value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::maxlenof_flags() const
{
  return 1;
}

/** Set flags value.
 * Flags spit down by the writer
 * @param new_flags new flags value
 */
void
TestInterface::set_flags(const uint8_t new_flags)
{
  data->flags = new_flags;
  data_changed = true;
}

/** Get test_string value.
 * A test sring
 * @return test_string value
 */
char *
TestInterface::test_string() const
{
  return data->test_string;
}

/** Get maximum length of test_string value.
 * @return length of test_string value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::maxlenof_test_string() const
{
  return 30;
}

/** Set test_string value.
 * A test sring
 * @param new_test_string new test_string value
 */
void
TestInterface::set_test_string(const char * new_test_string)
{
  strncpy(data->test_string, new_test_string, sizeof(data->test_string));
  data_changed = true;
}

/** Get result value.
 * Result of operation add operation from Calculate message.
 * @return result value
 */
int32_t
TestInterface::result() const
{
  return data->result;
}

/** Get maximum length of result value.
 * @return length of result value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::maxlenof_result() const
{
  return 1;
}

/** Set result value.
 * Result of operation add operation from Calculate message.
 * @param new_result new result value
 */
void
TestInterface::set_result(const int32_t new_result)
{
  data->result = new_result;
  data_changed = true;
}

/** Get test_uint value.
 * Test uint32
 * @return test_uint value
 */
uint32_t
TestInterface::test_uint() const
{
  return data->test_uint;
}

/** Get maximum length of test_uint value.
 * @return length of test_uint value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::maxlenof_test_uint() const
{
  return 1;
}

/** Set test_uint value.
 * Test uint32
 * @param new_test_uint new test_uint value
 */
void
TestInterface::set_test_uint(const uint32_t new_test_uint)
{
  data->test_uint = new_test_uint;
  data_changed = true;
}

/** Get test_ulint value.
 * Test unsigned long int
 * @return test_ulint value
 */
uint64_t
TestInterface::test_ulint() const
{
  return data->test_ulint;
}

/** Get maximum length of test_ulint value.
 * @return length of test_ulint value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::maxlenof_test_ulint() const
{
  return 1;
}

/** Set test_ulint value.
 * Test unsigned long int
 * @param new_test_ulint new test_ulint value
 */
void
TestInterface::set_test_ulint(const uint64_t new_test_ulint)
{
  data->test_ulint = new_test_ulint;
  data_changed = true;
}

/** Get test_lint value.
 * Test long int
 * @return test_lint value
 */
int64_t
TestInterface::test_lint() const
{
  return data->test_lint;
}

/** Get maximum length of test_lint value.
 * @return length of test_lint value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::maxlenof_test_lint() const
{
  return 1;
}

/** Set test_lint value.
 * Test long int
 * @param new_test_lint new test_lint value
 */
void
TestInterface::set_test_lint(const int64_t new_test_lint)
{
  data->test_lint = new_test_lint;
  data_changed = true;
}

/* =========== message create =========== */
Message *
TestInterface::create_message(const char *type) const
{
  if ( strncmp("SetTestIntMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetTestIntMessage();
  } else if ( strncmp("SetTestStringMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetTestStringMessage();
  } else if ( strncmp("CalculateMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CalculateMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
TestInterface::copy_values(const Interface *other)
{
  const TestInterface *oi = dynamic_cast<const TestInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(TestInterface_data_t));
}

const char *
TestInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "TestEnum") == 0) {
    return tostring_TestEnum((TestEnum)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class TestInterface::SetTestIntMessage <interfaces/TestInterface.h>
 * SetTestIntMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_test_int initial value for test_int
 */
TestInterface::SetTestIntMessage::SetTestIntMessage(const int32_t ini_test_int) : Message("SetTestIntMessage")
{
  data_size = sizeof(SetTestIntMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTestIntMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->test_int = ini_test_int;
  add_fieldinfo(IFT_INT32, "test_int", 1, &data->test_int);
}
/** Constructor */
TestInterface::SetTestIntMessage::SetTestIntMessage() : Message("SetTestIntMessage")
{
  data_size = sizeof(SetTestIntMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTestIntMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_INT32, "test_int", 1, &data->test_int);
}

/** Destructor */
TestInterface::SetTestIntMessage::~SetTestIntMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
TestInterface::SetTestIntMessage::SetTestIntMessage(const SetTestIntMessage *m) : Message("SetTestIntMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetTestIntMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get test_int value.
 * Test integer
 * @return test_int value
 */
int32_t
TestInterface::SetTestIntMessage::test_int() const
{
  return data->test_int;
}

/** Get maximum length of test_int value.
 * @return length of test_int value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::SetTestIntMessage::maxlenof_test_int() const
{
  return 1;
}

/** Set test_int value.
 * Test integer
 * @param new_test_int new test_int value
 */
void
TestInterface::SetTestIntMessage::set_test_int(const int32_t new_test_int)
{
  data->test_int = new_test_int;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
TestInterface::SetTestIntMessage::clone() const
{
  return new TestInterface::SetTestIntMessage(this);
}
/** @class TestInterface::SetTestStringMessage <interfaces/TestInterface.h>
 * SetTestStringMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_test_string initial value for test_string
 */
TestInterface::SetTestStringMessage::SetTestStringMessage(const char * ini_test_string) : Message("SetTestStringMessage")
{
  data_size = sizeof(SetTestStringMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTestStringMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->test_string, ini_test_string, 30);
  add_fieldinfo(IFT_STRING, "test_string", 30, data->test_string);
}
/** Constructor */
TestInterface::SetTestStringMessage::SetTestStringMessage() : Message("SetTestStringMessage")
{
  data_size = sizeof(SetTestStringMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTestStringMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "test_string", 30, data->test_string);
}

/** Destructor */
TestInterface::SetTestStringMessage::~SetTestStringMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
TestInterface::SetTestStringMessage::SetTestStringMessage(const SetTestStringMessage *m) : Message("SetTestStringMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetTestStringMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get test_string value.
 * A test sring
 * @return test_string value
 */
char *
TestInterface::SetTestStringMessage::test_string() const
{
  return data->test_string;
}

/** Get maximum length of test_string value.
 * @return length of test_string value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::SetTestStringMessage::maxlenof_test_string() const
{
  return 30;
}

/** Set test_string value.
 * A test sring
 * @param new_test_string new test_string value
 */
void
TestInterface::SetTestStringMessage::set_test_string(const char * new_test_string)
{
  strncpy(data->test_string, new_test_string, sizeof(data->test_string));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
TestInterface::SetTestStringMessage::clone() const
{
  return new TestInterface::SetTestStringMessage(this);
}
/** @class TestInterface::CalculateMessage <interfaces/TestInterface.h>
 * CalculateMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_summand initial value for summand
 * @param ini_addend initial value for addend
 */
TestInterface::CalculateMessage::CalculateMessage(const int32_t ini_summand, const int32_t ini_addend) : Message("CalculateMessage")
{
  data_size = sizeof(CalculateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CalculateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->summand = ini_summand;
  data->addend = ini_addend;
  add_fieldinfo(IFT_INT32, "summand", 1, &data->summand);
  add_fieldinfo(IFT_INT32, "addend", 1, &data->addend);
}
/** Constructor */
TestInterface::CalculateMessage::CalculateMessage() : Message("CalculateMessage")
{
  data_size = sizeof(CalculateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CalculateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_INT32, "summand", 1, &data->summand);
  add_fieldinfo(IFT_INT32, "addend", 1, &data->addend);
}

/** Destructor */
TestInterface::CalculateMessage::~CalculateMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
TestInterface::CalculateMessage::CalculateMessage(const CalculateMessage *m) : Message("CalculateMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CalculateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get summand value.
 * Summand
 * @return summand value
 */
int32_t
TestInterface::CalculateMessage::summand() const
{
  return data->summand;
}

/** Get maximum length of summand value.
 * @return length of summand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::CalculateMessage::maxlenof_summand() const
{
  return 1;
}

/** Set summand value.
 * Summand
 * @param new_summand new summand value
 */
void
TestInterface::CalculateMessage::set_summand(const int32_t new_summand)
{
  data->summand = new_summand;
}

/** Get addend value.
 * Addend
 * @return addend value
 */
int32_t
TestInterface::CalculateMessage::addend() const
{
  return data->addend;
}

/** Get maximum length of addend value.
 * @return length of addend value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TestInterface::CalculateMessage::maxlenof_addend() const
{
  return 1;
}

/** Set addend value.
 * Addend
 * @param new_addend new addend value
 */
void
TestInterface::CalculateMessage::set_addend(const int32_t new_addend)
{
  data->addend = new_addend;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
TestInterface::CalculateMessage::clone() const
{
  return new TestInterface::CalculateMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
TestInterface::message_valid(const Message *message) const
{
  const SetTestIntMessage *m0 = dynamic_cast<const SetTestIntMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetTestStringMessage *m1 = dynamic_cast<const SetTestStringMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const CalculateMessage *m2 = dynamic_cast<const CalculateMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(TestInterface)
/// @endcond


} // end namespace fawkes
