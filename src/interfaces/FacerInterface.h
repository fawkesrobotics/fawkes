
/***************************************************************************
 *  FacerInterface.h - Fawkes BlackBoard Interface - FacerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
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
  const char * tostring_if_facer_opmode_t(if_facer_opmode_t value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    if_facer_opmode_t opmode; /**< 
      Current opmode.
     */
    uint32_t num_identities; /**< 
      The number of identities in the database.
     */
    uint32_t recognized_identity; /**< 
      The index of the recognized identity.
     */
    char recognized_name[64]; /**< 
      The name of the recognized identity.
     */
    uint32_t num_detections; /**< 
      Number of currently detected faces.
     */
    uint32_t num_recognitions; /**< 
      Number of recognized faces.
     */
    uint32_t most_likely_identity; /**< 
      The identity that was recognized most prevalently.
     */
    float history_ratio; /**< 
      The ratio of the most likely identity showing up in the history
      and the length of the history.
     */
    float sec_since_detection; /**< 
      Time in seconds since the last successful detection.
     */
    int32_t visibility_history; /**< 
      The number of consecutive sighting ( <= 1 ) and non-sightings
      ( >= -1 ), respectively.
     */
    bool learning_in_progress; /**< 
      Indicates whether a new identity is currently learnt. If
      learning is in progress only "old" faces can be recognized.
     */
    float recording_progress; /**< 
      Indicates the progress of recording images of a new face.
     */
    float bearing; /**< 
      The relative bearing to the recognized face in radians.
     */
    float slope; /**< 
      The relative slope to the recognized face in radians.
     */
    uint32_t requested_index; /**< 
      Index of the identity for which the name was requested.
     */
    char requested_name[64]; /**< 
      Requested name.
     */
  } FacerInterface_data_t;
#pragma pack(pop)

  FacerInterface_data_t *data;

 public:
  /* messages */
  class LearnFaceMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      char name[64]; /**< The name assigned to the new identity. */
    } LearnFaceMessage_data_t;
#pragma pack(pop)

    LearnFaceMessage_data_t *data;

   public:
    LearnFaceMessage(const char * ini_name);
    LearnFaceMessage();
    ~LearnFaceMessage();

    LearnFaceMessage(const LearnFaceMessage *m);
    /* Methods */
    char * name() const;
    void set_name(const char * new_name);
    size_t maxlenof_name() const;
    virtual Message * clone() const;
  };

  class SetOpmodeMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      if_facer_opmode_t opmode; /**< 
      Current opmode.
     */
    } SetOpmodeMessage_data_t;
#pragma pack(pop)

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

  class EnableIdentityMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint32_t index; /**< Index of the identity. */
      bool enable; /**< En-/disable flag. */
    } EnableIdentityMessage_data_t;
#pragma pack(pop)

    EnableIdentityMessage_data_t *data;

   public:
    EnableIdentityMessage(const uint32_t ini_index, const bool ini_enable);
    EnableIdentityMessage();
    ~EnableIdentityMessage();

    EnableIdentityMessage(const EnableIdentityMessage *m);
    /* Methods */
    uint32_t index() const;
    void set_index(const uint32_t new_index);
    size_t maxlenof_index() const;
    bool is_enable() const;
    void set_enable(const bool new_enable);
    size_t maxlenof_enable() const;
    virtual Message * clone() const;
  };

  class SetNameMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint32_t index; /**< Index of the identity. */
      char name[64]; /**< Name of the identity. */
    } SetNameMessage_data_t;
#pragma pack(pop)

    SetNameMessage_data_t *data;

   public:
    SetNameMessage(const uint32_t ini_index, const char * ini_name);
    SetNameMessage();
    ~SetNameMessage();

    SetNameMessage(const SetNameMessage *m);
    /* Methods */
    uint32_t index() const;
    void set_index(const uint32_t new_index);
    size_t maxlenof_index() const;
    char * name() const;
    void set_name(const char * new_name);
    size_t maxlenof_name() const;
    virtual Message * clone() const;
  };

  class GetNameMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint32_t index; /**< Index of the identity. */
    } GetNameMessage_data_t;
#pragma pack(pop)

    GetNameMessage_data_t *data;

   public:
    GetNameMessage(const uint32_t ini_index);
    GetNameMessage();
    ~GetNameMessage();

    GetNameMessage(const GetNameMessage *m);
    /* Methods */
    uint32_t index() const;
    void set_index(const uint32_t new_index);
    size_t maxlenof_index() const;
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
  uint32_t num_identities() const;
  void set_num_identities(const uint32_t new_num_identities);
  size_t maxlenof_num_identities() const;
  uint32_t recognized_identity() const;
  void set_recognized_identity(const uint32_t new_recognized_identity);
  size_t maxlenof_recognized_identity() const;
  char * recognized_name() const;
  void set_recognized_name(const char * new_recognized_name);
  size_t maxlenof_recognized_name() const;
  uint32_t num_detections() const;
  void set_num_detections(const uint32_t new_num_detections);
  size_t maxlenof_num_detections() const;
  uint32_t num_recognitions() const;
  void set_num_recognitions(const uint32_t new_num_recognitions);
  size_t maxlenof_num_recognitions() const;
  uint32_t most_likely_identity() const;
  void set_most_likely_identity(const uint32_t new_most_likely_identity);
  size_t maxlenof_most_likely_identity() const;
  float history_ratio() const;
  void set_history_ratio(const float new_history_ratio);
  size_t maxlenof_history_ratio() const;
  float sec_since_detection() const;
  void set_sec_since_detection(const float new_sec_since_detection);
  size_t maxlenof_sec_since_detection() const;
  int32_t visibility_history() const;
  void set_visibility_history(const int32_t new_visibility_history);
  size_t maxlenof_visibility_history() const;
  bool is_learning_in_progress() const;
  void set_learning_in_progress(const bool new_learning_in_progress);
  size_t maxlenof_learning_in_progress() const;
  float recording_progress() const;
  void set_recording_progress(const float new_recording_progress);
  size_t maxlenof_recording_progress() const;
  float bearing() const;
  void set_bearing(const float new_bearing);
  size_t maxlenof_bearing() const;
  float slope() const;
  void set_slope(const float new_slope);
  size_t maxlenof_slope() const;
  uint32_t requested_index() const;
  void set_requested_index(const uint32_t new_requested_index);
  size_t maxlenof_requested_index() const;
  char * requested_name() const;
  void set_requested_name(const char * new_requested_name);
  size_t maxlenof_requested_name() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
