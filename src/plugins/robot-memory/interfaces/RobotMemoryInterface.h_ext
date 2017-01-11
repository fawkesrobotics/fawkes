
/***************************************************************************
 *  RobotMemoryInterface.h - Fawkes BlackBoard Interface - RobotMemoryInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  Frederik Zwilling
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

#ifndef __INTERFACES_ROBOTMEMORYINTERFACE_H_
#define __INTERFACES_ROBOTMEMORYINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class RobotMemoryInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(RobotMemoryInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    char error[1024]; /**< Error of last query */
    char result[1024]; /**< Result of last query */
  } RobotMemoryInterface_data_t;

  RobotMemoryInterface_data_t *data;

 public:
  /* messages */
  class QueryMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      char query[1024]; /**< Query as JSON string */
      char collection[1024]; /**< The collection to query */
    } QueryMessage_data_t;

    QueryMessage_data_t *data;

   public:
    QueryMessage(const char * ini_query, const char * ini_collection);
    QueryMessage();
    ~QueryMessage();

    QueryMessage(const QueryMessage *m);
    /* Methods */
    char * query() const;
    void set_query(const char * new_query);
    size_t maxlenof_query() const;
    char * collection() const;
    void set_collection(const char * new_collection);
    size_t maxlenof_collection() const;
    virtual Message * clone() const;
  };

  class InsertMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      char insert[1024]; /**< Document to insert as JSON string */
      char collection[1024]; /**< The collection to query */
    } InsertMessage_data_t;

    InsertMessage_data_t *data;

   public:
    InsertMessage(const char * ini_insert, const char * ini_collection);
    InsertMessage();
    ~InsertMessage();

    InsertMessage(const InsertMessage *m);
    /* Methods */
    char * insert() const;
    void set_insert(const char * new_insert);
    size_t maxlenof_insert() const;
    char * collection() const;
    void set_collection(const char * new_collection);
    size_t maxlenof_collection() const;
    virtual Message * clone() const;
  };

  class UpdateMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      char query[1024]; /**< Query as JSON string */
      char update[1024]; /**< Update as JSON string */
      char collection[1024]; /**< The collection to query */
    } UpdateMessage_data_t;

    UpdateMessage_data_t *data;

   public:
    UpdateMessage(const char * ini_query, const char * ini_update, const char * ini_collection);
    UpdateMessage();
    ~UpdateMessage();

    UpdateMessage(const UpdateMessage *m);
    /* Methods */
    char * query() const;
    void set_query(const char * new_query);
    size_t maxlenof_query() const;
    char * update() const;
    void set_update(const char * new_update);
    size_t maxlenof_update() const;
    char * collection() const;
    void set_collection(const char * new_collection);
    size_t maxlenof_collection() const;
    virtual Message * clone() const;
  };

  class RemoveMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      char query[1024]; /**< Query as JSON string */
      char collection[1024]; /**< The collection to query */
    } RemoveMessage_data_t;

    RemoveMessage_data_t *data;

   public:
    RemoveMessage(const char * ini_query, const char * ini_collection);
    RemoveMessage();
    ~RemoveMessage();

    RemoveMessage(const RemoveMessage *m);
    /* Methods */
    char * query() const;
    void set_query(const char * new_query);
    size_t maxlenof_query() const;
    char * collection() const;
    void set_collection(const char * new_collection);
    size_t maxlenof_collection() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  RobotMemoryInterface();
  ~RobotMemoryInterface();

 public:
  /* Methods */
  char * error() const;
  void set_error(const char * new_error);
  size_t maxlenof_error() const;
  char * result() const;
  void set_result(const char * new_result);
  size_t maxlenof_result() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
