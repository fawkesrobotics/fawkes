
/***************************************************************************
 *  TestInterface.h - Fawkes BlackBoard Interface - TestInterface
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

#ifndef __INTERFACES_TESTINTERFACE_H_
#define __INTERFACES_TESTINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class TestInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(TestInterface)
 /// @endcond
 public:
  /* constants */
  static const int32_t TEST_CONSTANT;
  static const float TEST_FLOAT_CONSTANT;

  /** Demonstrating enums */
  typedef enum {
    TEST_ENUM_1 /**< Item 1 */,
    TEST_ENUM_2 /**< Item 2 */
  } TestEnum;
  const char * tostring_TestEnum(TestEnum value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    bool test_bool; /**< Test Bool */
    int32_t test_int; /**< Test integer */
    uint8_t flags; /**< Flags spit down by the writer */
    char test_string[30]; /**< A test sring */
    int32_t result; /**< Result of operation add operation from Calculate message. */
    uint32_t test_uint; /**< Test uint32 */
    uint64_t test_ulint; /**< Test unsigned long int */
    int64_t test_lint; /**< Test long int */
  } TestInterface_data_t;
#pragma pack(pop)

  TestInterface_data_t *data;

 public:
  /* messages */
  class SetTestIntMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      int32_t test_int; /**< Test integer */
    } SetTestIntMessage_data_t;
#pragma pack(pop)

    SetTestIntMessage_data_t *data;

   public:
    SetTestIntMessage(const int32_t ini_test_int);
    SetTestIntMessage();
    ~SetTestIntMessage();

    SetTestIntMessage(const SetTestIntMessage *m);
    /* Methods */
    int32_t test_int() const;
    void set_test_int(const int32_t new_test_int);
    size_t maxlenof_test_int() const;
    virtual Message * clone() const;
  };

  class SetTestStringMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      char test_string[30]; /**< A test sring */
    } SetTestStringMessage_data_t;
#pragma pack(pop)

    SetTestStringMessage_data_t *data;

   public:
    SetTestStringMessage(const char * ini_test_string);
    SetTestStringMessage();
    ~SetTestStringMessage();

    SetTestStringMessage(const SetTestStringMessage *m);
    /* Methods */
    char * test_string() const;
    void set_test_string(const char * new_test_string);
    size_t maxlenof_test_string() const;
    virtual Message * clone() const;
  };

  class CalculateMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      int32_t summand; /**< Summand */
      int32_t addend; /**< Addend */
    } CalculateMessage_data_t;
#pragma pack(pop)

    CalculateMessage_data_t *data;

   public:
    CalculateMessage(const int32_t ini_summand, const int32_t ini_addend);
    CalculateMessage();
    ~CalculateMessage();

    CalculateMessage(const CalculateMessage *m);
    /* Methods */
    int32_t summand() const;
    void set_summand(const int32_t new_summand);
    size_t maxlenof_summand() const;
    int32_t addend() const;
    void set_addend(const int32_t new_addend);
    size_t maxlenof_addend() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  TestInterface();
  ~TestInterface();

 public:
  /* Methods */
  bool is_test_bool() const;
  void set_test_bool(const bool new_test_bool);
  size_t maxlenof_test_bool() const;
  int32_t test_int() const;
  void set_test_int(const int32_t new_test_int);
  size_t maxlenof_test_int() const;
  uint8_t flags() const;
  void set_flags(const uint8_t new_flags);
  size_t maxlenof_flags() const;
  char * test_string() const;
  void set_test_string(const char * new_test_string);
  size_t maxlenof_test_string() const;
  int32_t result() const;
  void set_result(const int32_t new_result);
  size_t maxlenof_result() const;
  uint32_t test_uint() const;
  void set_test_uint(const uint32_t new_test_uint);
  size_t maxlenof_test_uint() const;
  uint64_t test_ulint() const;
  void set_test_ulint(const uint64_t new_test_ulint);
  size_t maxlenof_test_ulint() const;
  int64_t test_lint() const;
  void set_test_lint(const int64_t new_test_lint);
  size_t maxlenof_test_lint() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
