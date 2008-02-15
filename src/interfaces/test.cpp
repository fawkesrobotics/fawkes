
/***************************************************************************
 *  test.cpp - Fawkes BlackBoard Interface - TestInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2006-2007  Tim Niemueller
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

#include <interfaces/test.h>

#include <cstring>
#include <cstdlib>

/** @class TestInterface interfaces/test.h
 * TestInterface Fawkes BlackBoard Interface.
 * Test interface. Use this to play around. Do NOT remove any fields, as this
      interface is used by BlackBoard QA.
 */


/** TEST_CONSTANT constant */
const int TestInterface::TEST_CONSTANT = 5;
/** TEST_FLOAT_CONSTANT constant */
const float TestInterface::TEST_FLOAT_CONSTANT = 1.2;

/** Constructor */
TestInterface::TestInterface() : Interface()
{
  data_size = sizeof(TestInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (TestInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
}

/** Destructor */
TestInterface::~TestInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get test_int value.
 * Test integer
 * @return test_int value
 */
int
TestInterface::test_int()
{
  return data->test_int;
}

/** Set test_int value.
 * Test integer
 * @param new_test_int new test_int value
 */
void
TestInterface::set_test_int(const int new_test_int)
{
  data->test_int = new_test_int;
}

/** Get _flags value.
 * Flags spit down by the writer
 * @return _flags value
 */
unsigned int
TestInterface::_flags()
{
  return data->_flags;
}

/** Set _flags value.
 * Flags spit down by the writer
 * @param new__flags new _flags value
 */
void
TestInterface::set__flags(const unsigned int new__flags)
{
  data->_flags = new__flags;
}

/** Get test_string value.
 * A test sring
 * @return test_string value
 */
char *
TestInterface::test_string()
{
  return data->test_string;
}

/** Set test_string value.
 * A test sring
 * @param new_test_string new test_string value
 */
void
TestInterface::set_test_string(const char * new_test_string)
{
  strncpy(data->test_string, new_test_string, sizeof(data->test_string));
}

/** Get result value.
 * Result of operation add operation from Calculate message.
 * @return result value
 */
int
TestInterface::result()
{
  return data->result;
}

/** Set result value.
 * Result of operation add operation from Calculate message.
 * @param new_result new result value
 */
void
TestInterface::set_result(const int new_result)
{
  data->result = new_result;
}

/** Get test_uint value.
 * Test unsigned int
 * @return test_uint value
 */
unsigned int
TestInterface::test_uint()
{
  return data->test_uint;
}

/** Set test_uint value.
 * Test unsigned int
 * @param new_test_uint new test_uint value
 */
void
TestInterface::set_test_uint(const unsigned int new_test_uint)
{
  data->test_uint = new_test_uint;
}

/** Get test_ulint value.
 * Test unsigned long int
 * @return test_ulint value
 */
unsigned long int
TestInterface::test_ulint()
{
  return data->test_ulint;
}

/** Set test_ulint value.
 * Test unsigned long int
 * @param new_test_ulint new test_ulint value
 */
void
TestInterface::set_test_ulint(const unsigned long int new_test_ulint)
{
  data->test_ulint = new_test_ulint;
}

/** Get test_lint value.
 * Test long int
 * @return test_lint value
 */
long int
TestInterface::test_lint()
{
  return data->test_lint;
}

/** Set test_lint value.
 * Test long int
 * @param new_test_lint new test_lint value
 */
void
TestInterface::set_test_lint(const long int new_test_lint)
{
  data->test_lint = new_test_lint;
}

/* =========== messages =========== */
/** @class TestInterface::SetTestIntMessage interfaces/test.h
 * SetTestIntMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_test_int initial value for test_int
 */
TestInterface::SetTestIntMessage::SetTestIntMessage(int ini_test_int) : Message("SetTestIntMessage")
{
  data_size = sizeof(SetTestIntMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTestIntMessage_data_t *)data_ptr;
  data->test_int = ini_test_int;
}
/** Constructor */
TestInterface::SetTestIntMessage::SetTestIntMessage() : Message("SetTestIntMessage")
{
  data_size = sizeof(SetTestIntMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTestIntMessage_data_t *)data_ptr;
}

/** Destructor */
TestInterface::SetTestIntMessage::~SetTestIntMessage()
{
  free(data_ptr);
}

/* Methods */
/** Get test_int value.
 * Test integer
 * @return test_int value
 */
int
TestInterface::SetTestIntMessage::test_int()
{
  return data->test_int;
}

/** Set test_int value.
 * Test integer
 * @param new_test_int new test_int value
 */
void
TestInterface::SetTestIntMessage::set_test_int(const int new_test_int)
{
  data->test_int = new_test_int;
}

/** @class TestInterface::SetTestStringMessage interfaces/test.h
 * SetTestStringMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_test_string initial value for test_string
 */
TestInterface::SetTestStringMessage::SetTestStringMessage(char * ini_test_string) : Message("SetTestStringMessage")
{
  data_size = sizeof(SetTestStringMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTestStringMessage_data_t *)data_ptr;
  strncpy(data->test_string, ini_test_string, 30);
}
/** Constructor */
TestInterface::SetTestStringMessage::SetTestStringMessage() : Message("SetTestStringMessage")
{
  data_size = sizeof(SetTestStringMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTestStringMessage_data_t *)data_ptr;
}

/** Destructor */
TestInterface::SetTestStringMessage::~SetTestStringMessage()
{
  free(data_ptr);
}

/* Methods */
/** Get test_string value.
 * A test sring
 * @return test_string value
 */
char *
TestInterface::SetTestStringMessage::test_string()
{
  return data->test_string;
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

/** @class TestInterface::CalculateMessage interfaces/test.h
 * CalculateMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_summand initial value for summand
 * @param ini_addend initial value for addend
 */
TestInterface::CalculateMessage::CalculateMessage(int ini_summand, int ini_addend) : Message("CalculateMessage")
{
  data_size = sizeof(CalculateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CalculateMessage_data_t *)data_ptr;
  data->summand = ini_summand;
  data->addend = ini_addend;
}
/** Constructor */
TestInterface::CalculateMessage::CalculateMessage() : Message("CalculateMessage")
{
  data_size = sizeof(CalculateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CalculateMessage_data_t *)data_ptr;
}

/** Destructor */
TestInterface::CalculateMessage::~CalculateMessage()
{
  free(data_ptr);
}

/* Methods */
/** Get summand value.
 * Summand
 * @return summand value
 */
int
TestInterface::CalculateMessage::summand()
{
  return data->summand;
}

/** Set summand value.
 * Summand
 * @param new_summand new summand value
 */
void
TestInterface::CalculateMessage::set_summand(const int new_summand)
{
  data->summand = new_summand;
}

/** Get addend value.
 * Addend
 * @return addend value
 */
int
TestInterface::CalculateMessage::addend()
{
  return data->addend;
}

/** Set addend value.
 * Addend
 * @param new_addend new addend value
 */
void
TestInterface::CalculateMessage::set_addend(const int new_addend)
{
  data->addend = new_addend;
}

/** Check if message is valid an can be queued.
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

