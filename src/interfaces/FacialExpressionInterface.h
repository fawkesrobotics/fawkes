
/***************************************************************************
 *  FacialExpressionInterface.h - Fawkes BlackBoard Interface - FacialExpressionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2009  Bahram Maleki-Fard
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

#ifndef __INTERFACES_FACIALEXPRESSIONINTERFACE_H_
#define __INTERFACES_FACIALEXPRESSIONINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class FacialExpressionInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(FacialExpressionInterface)
 /// @endcond
 public:
  /* constants */

  /** Action types for moving brows */
  typedef enum {
    BROWS_DEFAULT /**< Reset */,
    BROWS_FROWN /**< Frown */,
    BROWS_LIFT /**< Lift */
  } brows_t;
  const char * tostring_brows_t(brows_t value) const;

  /** Action types for moving eyes */
  typedef enum {
    EYES_DEFAULT /**< Reset */,
    EYES_UP /**< Up */,
    EYES_DOWN /**< Down */,
    EYES_LEFT /**< Left */,
    EYES_RIGHT /**< Right */,
    EYES_COOL /**< Cool */,
    EYES_CROSS /**< Cross */,
    EYES_HEART /**< Heart */,
    EYES_DOLLAR /**< Dollar */
  } eyes_t;
  const char * tostring_eyes_t(eyes_t value) const;

  /** Action types for moving jowl */
  typedef enum {
    JOWL_DEFAULT /**< Reset */,
    JOWL_BLUSH /**< Blush */,
    JOWL_TEARS /**< Tears */
  } jowl_t;
  const char * tostring_jowl_t(jowl_t value) const;

  /** Action types for moving mouth */
  typedef enum {
    MOUTH_DEFAULT /**< Reset */,
    MOUTH_OPEN /**< Open */,
    MOUTH_CLOSE /**< Close */,
    MOUTH_SMILE /**< Smile */,
    MOUTH_SCOWL /**< Scowl */
  } mouth_t;
  const char * tostring_mouth_t(mouth_t value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    brows_t brows_action; /**< Type of action of brows */
    eyes_t eyes_action; /**< Type of action of eyes */
    jowl_t jowl_action; /**< Type of action of jown */
    mouth_t mouth_action; /**< Type of action of mouth */
  } FacialExpressionInterface_data_t;
#pragma pack(pop)

  FacialExpressionInterface_data_t *data;

 public:
  /* messages */
  class MoveBrowsMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      brows_t brows_action; /**< Type of action of brows */
    } MoveBrowsMessage_data_t;
#pragma pack(pop)

    MoveBrowsMessage_data_t *data;

   public:
    MoveBrowsMessage(const brows_t ini_brows_action);
    MoveBrowsMessage();
    ~MoveBrowsMessage();

    MoveBrowsMessage(const MoveBrowsMessage *m);
    /* Methods */
    brows_t brows_action() const;
    void set_brows_action(const brows_t new_brows_action);
    size_t maxlenof_brows_action() const;
    virtual Message * clone() const;
  };

  class MoveEyesMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      eyes_t eyes_action; /**< Type of action of eyes */
    } MoveEyesMessage_data_t;
#pragma pack(pop)

    MoveEyesMessage_data_t *data;

   public:
    MoveEyesMessage(const eyes_t ini_eyes_action);
    MoveEyesMessage();
    ~MoveEyesMessage();

    MoveEyesMessage(const MoveEyesMessage *m);
    /* Methods */
    eyes_t eyes_action() const;
    void set_eyes_action(const eyes_t new_eyes_action);
    size_t maxlenof_eyes_action() const;
    virtual Message * clone() const;
  };

  class MoveJowlMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      jowl_t jowl_action; /**< Type of action of jown */
    } MoveJowlMessage_data_t;
#pragma pack(pop)

    MoveJowlMessage_data_t *data;

   public:
    MoveJowlMessage(const jowl_t ini_jowl_action);
    MoveJowlMessage();
    ~MoveJowlMessage();

    MoveJowlMessage(const MoveJowlMessage *m);
    /* Methods */
    jowl_t jowl_action() const;
    void set_jowl_action(const jowl_t new_jowl_action);
    size_t maxlenof_jowl_action() const;
    virtual Message * clone() const;
  };

  class MoveMouthMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      mouth_t mouth_action; /**< Type of action of mouth */
    } MoveMouthMessage_data_t;
#pragma pack(pop)

    MoveMouthMessage_data_t *data;

   public:
    MoveMouthMessage(const mouth_t ini_mouth_action);
    MoveMouthMessage();
    ~MoveMouthMessage();

    MoveMouthMessage(const MoveMouthMessage *m);
    /* Methods */
    mouth_t mouth_action() const;
    void set_mouth_action(const mouth_t new_mouth_action);
    size_t maxlenof_mouth_action() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  FacialExpressionInterface();
  ~FacialExpressionInterface();

 public:
  /* Methods */
  brows_t brows_action() const;
  void set_brows_action(const brows_t new_brows_action);
  size_t maxlenof_brows_action() const;
  eyes_t eyes_action() const;
  void set_eyes_action(const eyes_t new_eyes_action);
  size_t maxlenof_eyes_action() const;
  jowl_t jowl_action() const;
  void set_jowl_action(const jowl_t new_jowl_action);
  size_t maxlenof_jowl_action() const;
  mouth_t mouth_action() const;
  void set_mouth_action(const mouth_t new_mouth_action);
  size_t maxlenof_mouth_action() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
