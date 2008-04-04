
/***************************************************************************
 *  test.h - Fawkes BlackBoard Interface - TestInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2006-2007  Tim Niemueller
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory.
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
    SetTestIntMessage(const int ini_test_int);
    SetTestIntMessage();
    ~SetTestIntMessage();

    SetTestIntMessage(const SetTestIntMessage *m);
    /* Methods */
    int test_int();
    void set_test_int(const int new_test_int);
    size_t maxlenof_test_int() const;
    virtual Message * clone() const;
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
    SetTestStringMessage(const char * ini_test_string);
    SetTestStringMessage();
    ~SetTestStringMessage();

    SetTestStringMessage(const SetTestStringMessage *m);
    /* Methods */
    char * test_string();
    void set_test_string(const char * new_test_string);
    size_t maxlenof_test_string() const;
    virtual Message * clone() const;
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
    CalculateMessage(const int ini_summand, const int ini_addend);
    CalculateMessage();
    ~CalculateMessage();

    CalculateMessage(const CalculateMessage *m);
    /* Methods */
    int summand();
    void set_summand(const int new_summand);
    size_t maxlenof_summand() const;
    int addend();
    void set_addend(const int new_addend);
    size_t maxlenof_addend() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  TestInterface();
  ~TestInterface();

 public:
  virtual Message * create_message(const char *type) const;

  /* Methods */
  bool is_test_bool();
  void set_test_bool(const bool new_test_bool);
  size_t maxlenof_test_bool() const;
  int test_int();
  void set_test_int(const int new_test_int);
  size_t maxlenof_test_int() const;
  char _flags();
  void set__flags(const char new__flags);
  size_t maxlenof__flags() const;
  char * test_string();
  void set_test_string(const char * new_test_string);
  size_t maxlenof_test_string() const;
  int result();
  void set_result(const int new_result);
  size_t maxlenof_result() const;
  unsigned int test_uint();
  void set_test_uint(const unsigned int new_test_uint);
  size_t maxlenof_test_uint() const;
  unsigned long int test_ulint();
  void set_test_ulint(const unsigned long int new_test_ulint);
  size_t maxlenof_test_ulint() const;
  long int test_lint();
  void set_test_lint(const long int new_test_lint);
  size_t maxlenof_test_lint() const;

};

#endif
