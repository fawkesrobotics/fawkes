
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

#ifndef __INTERFACES_TEST_H_
#define __INTERFACES_TEST_H_

#include <interface/interface.h>
#include <interface/message.h>

class TestInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(TestInterface)
 /// @endcond
 public:
  /* constants */
  static const int TEST_CONSTANT;
  static const float TEST_FLOAT_CONSTANT;

  /** Demonstrating enums */
  typedef enum {
    TEST_ENUM_1 /**< Item 1 */,
    TEST_ENUM_2 /**< Item 2 */
  } TestEnum;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int test_uint; /**< Test unsigned int */
    int test_int; /**< Test integer */
    int result; /**< Result of operation add operation from Calculate message. */
    unsigned long int test_ulint; /**< Test unsigned long int */
    long int test_lint; /**< Test long int */
    bool test_bool; /**< Test Bool */
    char _flags; /**< Flags spit down by the writer */
    char test_string[30]; /**< A test sring */
  } TestInterface_data_t;

  TestInterface_data_t *data;

 public:
  /* messages */
  class SetTestIntMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int test_int; /**< Test integer */
    } SetTestIntMessage_data_t;

    SetTestIntMessage_data_t *data;

   public:
    SetTestIntMessage(int ini_test_int);
    SetTestIntMessage();
    ~SetTestIntMessage();

    /* Methods */
    int test_int();
    void set_test_int(const int new_test_int);
  };

  class SetTestStringMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      char test_string[30]; /**< A test sring */
    } SetTestStringMessage_data_t;

    SetTestStringMessage_data_t *data;

   public:
    SetTestStringMessage(char * ini_test_string);
    SetTestStringMessage();
    ~SetTestStringMessage();

    /* Methods */
    char * test_string();
    void set_test_string(const char * new_test_string);
  };

  class CalculateMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int summand; /**< Summand */
      int addend; /**< Addend */
    } CalculateMessage_data_t;

    CalculateMessage_data_t *data;

   public:
    CalculateMessage(int ini_summand, int ini_addend);
    CalculateMessage();
    ~CalculateMessage();

    /* Methods */
    int summand();
    void set_summand(const int new_summand);
    int addend();
    void set_addend(const int new_addend);
  };

  virtual bool message_valid(const Message *message) const;
 private:
  TestInterface();
  ~TestInterface();

 public:
  /* Methods */
  bool is_test_bool();
  void set_test_bool(const bool new_test_bool);
  int test_int();
  void set_test_int(const int new_test_int);
  char _flags();
  void set__flags(const char new__flags);
  char * test_string();
  void set_test_string(const char * new_test_string);
  int result();
  void set_result(const int new_result);
  unsigned int test_uint();
  void set_test_uint(const unsigned int new_test_uint);
  unsigned long int test_ulint();
  void set_test_ulint(const unsigned long int new_test_ulint);
  long int test_lint();
  void set_test_lint(const long int new_test_lint);

};

#endif
