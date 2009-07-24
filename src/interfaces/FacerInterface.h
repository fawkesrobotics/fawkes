
/***************************************************************************
 *  FacerInterface.h - Fawkes BlackBoard Interface - FacerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
 *
 *  $Id: cpp_generator.cpp 2510 2009-06-09 09:32:58Z tim $
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

#ifndef __INTERFACES_FACERINTERFACE_H_
#define __INTERFACES_FACERINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class FacerInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(FacerInterface)
 /// @endcond
 public:
  /* constants */

  /** 
	This determines the current status of skill execution.
       */
  typedef enum {
    OPMODE_DISABLED /**< Facer will not process any images */,
    OPMODE_DETECTION /**< Facer will detect faces, but not try to recognize them. */,
    OPMODE_RECOGNITION /**< Facer will detect faces, and then try to recognize the most dominant face. */,
    OPMODE_LEARNING /**< Facer will gather images and learn an identity. */
  } if_facer_opmode_t;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int num_detections; /**< Number of currently detected faces */
    float sec_since_detection; /**< Time in seconds since the last successful detection. */
    bool learning_done; /**< True if opmode is learning and learning has been completed, false otherwise */
    if_facer_opmode_t opmode; /**< Current opmode. */
    char face_label[64]; /**< Label of the recognized face */
  } FacerInterface_data_t;

  FacerInterface_data_t *data;

 public:
  /* messages */
  class LearnFaceMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      char face_label[64]; /**< Label of the recognized face */
    } LearnFaceMessage_data_t;

    LearnFaceMessage_data_t *data;

   public:
    LearnFaceMessage(const char * ini_face_label);
    LearnFaceMessage();
    ~LearnFaceMessage();

    LearnFaceMessage(const LearnFaceMessage *m);
    /* Methods */
    char * face_label() const;
    void set_face_label(const char * new_face_label);
    size_t maxlenof_face_label() const;
    virtual Message * clone() const;
  };

  class SetOpmodeMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      if_facer_opmode_t opmode; /**< Current opmode. */
    } SetOpmodeMessage_data_t;

    SetOpmodeMessage_data_t *data;

   public:
    SetOpmodeMessage(const if_facer_opmode_t ini_opmode);
    SetOpmodeMessage();
    ~SetOpmodeMessage();

    SetOpmodeMessage(const SetOpmodeMessage *m);
    /* Methods */
    if_facer_opmode_t opmode() const;
    void set_opmode(const if_facer_opmode_t new_opmode);
    size_t maxlenof_opmode() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  FacerInterface();
  ~FacerInterface();

 public:
  /* Methods */
  if_facer_opmode_t opmode() const;
  void set_opmode(const if_facer_opmode_t new_opmode);
  size_t maxlenof_opmode() const;
  char * face_label() const;
  void set_face_label(const char * new_face_label);
  size_t maxlenof_face_label() const;
  bool is_learning_done() const;
  void set_learning_done(const bool new_learning_done);
  size_t maxlenof_learning_done() const;
  unsigned int num_detections() const;
  void set_num_detections(const unsigned int new_num_detections);
  size_t maxlenof_num_detections() const;
  float sec_since_detection() const;
  void set_sec_since_detection(const float new_sec_since_detection);
  size_t maxlenof_sec_since_detection() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);

};

} // end namespace fawkes

#endif
