
/***************************************************************************
 *  test.cpp - Fawkes BlackBoard Interface - TestInterface
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

#include <interfaces/test.h>

#include <string.h>
#include <stdlib.h>

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
}
/** Destructor */
TestInterface::~TestInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get TestInt value.
 * Test integer
 * @return TestInt value
 */
int
TestInterface::getTestInt()
{
  return data->TestInt;
}

/** Set TestInt value.
 * Test integer
 * @param newTestInt new TestInt value
 */
void
TestInterface::setTestInt(int newTestInt)
{
  data->TestInt = newTestInt;
}

/** Get Flags value.
 * Flags spit down by the writer
 * @return Flags value
 */
unsigned int
TestInterface::getFlags()
{
  return data->Flags;
}

/** Set Flags value.
 * Flags spit down by the writer
 * @param newFlags new Flags value
 */
void
TestInterface::setFlags(unsigned int newFlags)
{
  data->Flags = newFlags;
}

/** Get TestString value.
 * A test sring
 * @return TestString value
 */
char *
TestInterface::getTestString()
{
  return data->TestString;
}

/** Set TestString value.
 * A test sring
 * @param newTestString new TestString value
 */
void
TestInterface::setTestString(char * newTestString)
{
  strncpy(data->TestString, newTestString, sizeof(data->TestString));
}

/** Get Result value.
 * Result of operation add operation from Calculate message.
 * @return Result value
 */
int
TestInterface::getResult()
{
  return data->Result;
}

/** Set Result value.
 * Result of operation add operation from Calculate message.
 * @param newResult new Result value
 */
void
TestInterface::setResult(int newResult)
{
  data->Result = newResult;
}

/** Get TestUInt value.
 * Test unsigned int
 * @return TestUInt value
 */
unsigned int
TestInterface::getTestUInt()
{
  return data->TestUInt;
}

/** Set TestUInt value.
 * Test unsigned int
 * @param newTestUInt new TestUInt value
 */
void
TestInterface::setTestUInt(unsigned int newTestUInt)
{
  data->TestUInt = newTestUInt;
}

/* =========== messages =========== */
/** @class TestInterface::SetTestIntMessage interfaces/test.h
 * SetTestIntMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniTestInt initial value for TestInt
 */
TestInterface::SetTestIntMessage::SetTestIntMessage(int iniTestInt) : Message()
{
  data_size = sizeof(SetTestIntMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (SetTestIntMessage_data_t *)data_ptr;
  data->TestInt = iniTestInt;
}
/** Constructor */
TestInterface::SetTestIntMessage::SetTestIntMessage() : Message()
{
  data_size = sizeof(SetTestIntMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (SetTestIntMessage_data_t *)data_ptr;
}
/** Destructor */
TestInterface::SetTestIntMessage::~SetTestIntMessage()
{
}
/* Methods */
/** Get TestInt value.
 * Test integer
 * @return TestInt value
 */
int
TestInterface::SetTestIntMessage::getTestInt()
{
  return data->TestInt;
}

/** Set TestInt value.
 * Test integer
 * @param newTestInt new TestInt value
 */
void
TestInterface::SetTestIntMessage::setTestInt(int newTestInt)
{
  data->TestInt = newTestInt;
}

/** @class TestInterface::SetTestStringMessage interfaces/test.h
 * SetTestStringMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniTestString initial value for TestString
 */
TestInterface::SetTestStringMessage::SetTestStringMessage(char * iniTestString) : Message()
{
  data_size = sizeof(SetTestStringMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (SetTestStringMessage_data_t *)data_ptr;
  strncpy(data->TestString, iniTestString, 30);
}
/** Constructor */
TestInterface::SetTestStringMessage::SetTestStringMessage() : Message()
{
  data_size = sizeof(SetTestStringMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (SetTestStringMessage_data_t *)data_ptr;
}
/** Destructor */
TestInterface::SetTestStringMessage::~SetTestStringMessage()
{
}
/* Methods */
/** Get TestString value.
 * A test sring
 * @return TestString value
 */
char *
TestInterface::SetTestStringMessage::getTestString()
{
  return data->TestString;
}

/** Set TestString value.
 * A test sring
 * @param newTestString new TestString value
 */
void
TestInterface::SetTestStringMessage::setTestString(char * newTestString)
{
  strncpy(data->TestString, newTestString, sizeof(data->TestString));
}

/** @class TestInterface::CalculateMessage interfaces/test.h
 * CalculateMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniSummand initial value for Summand
 * @param iniAddend initial value for Addend
 */
TestInterface::CalculateMessage::CalculateMessage(int iniSummand, int iniAddend) : Message()
{
  data_size = sizeof(CalculateMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (CalculateMessage_data_t *)data_ptr;
  data->Summand = iniSummand;
  data->Addend = iniAddend;
}
/** Constructor */
TestInterface::CalculateMessage::CalculateMessage() : Message()
{
  data_size = sizeof(CalculateMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (CalculateMessage_data_t *)data_ptr;
}
/** Destructor */
TestInterface::CalculateMessage::~CalculateMessage()
{
}
/* Methods */
/** Get Summand value.
 * Summand
 * @return Summand value
 */
int
TestInterface::CalculateMessage::getSummand()
{
  return data->Summand;
}

/** Set Summand value.
 * Summand
 * @param newSummand new Summand value
 */
void
TestInterface::CalculateMessage::setSummand(int newSummand)
{
  data->Summand = newSummand;
}

/** Get Addend value.
 * Addend
 * @return Addend value
 */
int
TestInterface::CalculateMessage::getAddend()
{
  return data->Addend;
}

/** Set Addend value.
 * Addend
 * @param newAddend new Addend value
 */
void
TestInterface::CalculateMessage::setAddend(int newAddend)
{
  data->Addend = newAddend;
}

/** Check if message is valid an can be queued.
 * @param message Message to check
 */
bool
TestInterface::messageValid(const Message *message) const
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
Interface *
private_newTestInterface()
{
  return new TestInterface();
}
/// @endcond
/** Create instance of TestInterface.
 * @return instance of TestInterface
 */
extern "C"
Interface *
newTestInterface()
{
  return private_newTestInterface();
}

/** Destroy TestInterface instance.
 * @param interface TestInterface instance to destroy.
 */
extern "C"
void
deleteTestInterface(Interface *interface)
{
  delete interface;
}

