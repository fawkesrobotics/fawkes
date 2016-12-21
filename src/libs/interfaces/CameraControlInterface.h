
/***************************************************************************
 *  CameraControlInterface.h - Fawkes BlackBoard Interface - CameraControlInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2014  Tim Niemueller
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

#ifndef __INTERFACES_CAMERACONTROLINTERFACE_H_
#define __INTERFACES_CAMERACONTROLINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class CameraControlInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(CameraControlInterface)
 /// @endcond
 public:
  /* constants */

  /** 
        Enumeration defining the possible effects. A camera must not necessarily
        implement all.
       */
  typedef enum {
    EFF_NONE /**< No effect. */,
    EFF_PASTEL /**< Pastel colors. */,
    EFF_NEGATIVE /**< Negative/Positive Reversal. */,
    EFF_BW /**< Monochrome Image. */,
    EFF_SOLARIZE /**< Enhanced Contrast. */
  } Effect;
  const char * tostring_Effect(Effect value) const;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    int32_t effect; /**< Currently active effect. */
    bool effect_supported; /**< Are effects supported? */
    uint32_t zoom; /**< Current zoom setting. */
    bool zoom_supported; /**< Is zooming supported? */
    uint32_t zoom_max; /**< Maximum zoom value */
    uint32_t zoom_min; /**< Minimum zoom */
    bool mirror; /**< Is the image mirrored? */
    bool mirror_supported; /**< Is mirroring supported? */
  } CameraControlInterface_data_t;

  CameraControlInterface_data_t *data;

  interface_enum_map_t enum_map_Effect;
 public:
  /* messages */
  class SetEffectMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      int32_t effect; /**< Currently active effect. */
    } SetEffectMessage_data_t;

    SetEffectMessage_data_t *data;

  interface_enum_map_t enum_map_Effect;
   public:
    SetEffectMessage(const Effect ini_effect);
    SetEffectMessage();
    ~SetEffectMessage();

    SetEffectMessage(const SetEffectMessage *m);
    /* Methods */
    Effect effect() const;
    void set_effect(const Effect new_effect);
    size_t maxlenof_effect() const;
    virtual Message * clone() const;
  };

  class SetZoomMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint32_t zoom; /**< Current zoom setting. */
    } SetZoomMessage_data_t;

    SetZoomMessage_data_t *data;

  interface_enum_map_t enum_map_Effect;
   public:
    SetZoomMessage(const uint32_t ini_zoom);
    SetZoomMessage();
    ~SetZoomMessage();

    SetZoomMessage(const SetZoomMessage *m);
    /* Methods */
    uint32_t zoom() const;
    void set_zoom(const uint32_t new_zoom);
    size_t maxlenof_zoom() const;
    virtual Message * clone() const;
  };

  class SetMirrorMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      bool mirror; /**< Is the image mirrored? */
    } SetMirrorMessage_data_t;

    SetMirrorMessage_data_t *data;

  interface_enum_map_t enum_map_Effect;
   public:
    SetMirrorMessage(const bool ini_mirror);
    SetMirrorMessage();
    ~SetMirrorMessage();

    SetMirrorMessage(const SetMirrorMessage *m);
    /* Methods */
    bool is_mirror() const;
    void set_mirror(const bool new_mirror);
    size_t maxlenof_mirror() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  CameraControlInterface();
  ~CameraControlInterface();

 public:
  /* Methods */
  Effect effect() const;
  void set_effect(const Effect new_effect);
  size_t maxlenof_effect() const;
  bool is_effect_supported() const;
  void set_effect_supported(const bool new_effect_supported);
  size_t maxlenof_effect_supported() const;
  uint32_t zoom() const;
  void set_zoom(const uint32_t new_zoom);
  size_t maxlenof_zoom() const;
  bool is_zoom_supported() const;
  void set_zoom_supported(const bool new_zoom_supported);
  size_t maxlenof_zoom_supported() const;
  uint32_t zoom_max() const;
  void set_zoom_max(const uint32_t new_zoom_max);
  size_t maxlenof_zoom_max() const;
  uint32_t zoom_min() const;
  void set_zoom_min(const uint32_t new_zoom_min);
  size_t maxlenof_zoom_min() const;
  bool is_mirror() const;
  void set_mirror(const bool new_mirror);
  size_t maxlenof_mirror() const;
  bool is_mirror_supported() const;
  void set_mirror_supported(const bool new_mirror_supported);
  size_t maxlenof_mirror_supported() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
