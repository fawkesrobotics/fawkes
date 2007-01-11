
/***************************************************************************
 *  test.h - Fawkes BlackBoard Interface - TestInterface
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

#ifndef __INTERFACES_TEST_H_
#define __INTERFACES_TEST_H_

#include <interfaces/interface.h>
#include <interfaces/message.h>

class TestInterface : public Interface
{
/// @cond INTERNALS
 friend Interface *  private_newTestInterface();
/// @endcond
 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int TestUInt; /**< Test unsigned int */
    int TestInt; /**< Test integer */
    int Result; /**< Result of operation add operation from Calculate message. */
    unsigned int Flags : 8; /**< Flags spit down by the writer */
    char TestString[30]; /**< A test sring */
  } TestInterface_data_t;

  TestInterface_data_t *data;

 public:
  /* constants */
  static const int TEST_CONSTANT;
  static const float TEST_FLOAT_CONSTANT;

  /** Demonstrating enums */
  typedef enum {
    TEST_ENUM_1 /**< Item 1 */,
    TEST_ENUM_2 /**< Item 2 */
  } TestEnum;

  /* messages */
  class SetTestIntMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int TestInt; /**< Test integer */
    } SetTestIntMessage_data_t;

    SetTestIntMessage_data_t *data;

   public:
    SetTestIntMessage(int iniTestInt);
    SetTestIntMessage();
    ~SetTestIntMessage();

    /* Methods */
    int getTestInt();
    void setTestInt(int newTestInt);
  };

  class SetTestStringMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      char TestString[30]; /**< A test sring */
    } SetTestStringMessage_data_t;

    SetTestStringMessage_data_t *data;

   public:
    SetTestStringMessage(char * iniTestString);
    SetTestStringMessage();
    ~SetTestStringMessage();

    /* Methods */
    char * getTestString();
    void setTestString(char * newTestString);
  };

  class CalculateMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int Summand; /**< Summand */
      int Addend; /**< Addend */
    } CalculateMessage_data_t;

    CalculateMessage_data_t *data;

   public:
    CalculateMessage(int iniSummand, int iniAddend);
    CalculateMessage();
    ~CalculateMessage();

    /* Methods */
    int getSummand();
    void setSummand(int newSummand);
    int getAddend();
    void setAddend(int newAddend);
  };

  virtual bool messageValid(const Message *message) const;
 private:
  TestInterface();
  ~TestInterface();

 public:
  /* Methods */
  int getTestInt();
  void setTestInt(int newTestInt);
  unsigned int getFlags();
  void setFlags(unsigned int newFlags);
  char * getTestString();
  void setTestString(char * newTestString);
  int getResult();
  void setResult(int newResult);
  unsigned int getTestUInt();
  void setTestUInt(unsigned int newTestUInt);

};

#endif
