
/***************************************************************************
 *  VisualDisplay2DInterface.h - Fawkes BlackBoard Interface - VisualDisplay2DInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2009  Tim Niemueller
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

#ifndef __INTERFACES_VISUALDISPLAY2DINTERFACE_H_
#define __INTERFACES_VISUALDISPLAY2DINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class VisualDisplay2DInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(VisualDisplay2DInterface)
 /// @endcond
 public:
  /* constants */

  /** 
        Enumeration defining the possible line styles.
       */
  typedef enum {
    LS_SOLID /**< Solid line. */,
    LS_DASHED /**< Dashed line. */,
    LS_DOTTED /**< Dotted line. */,
    LS_DASH_DOTTED /**< Dashed and dotted line */
  } LineStyle;
  const char * tostring_LineStyle(LineStyle value) const;

  /** 
        Enumeration defining the possible anchor points. They are used
        for determining text alignment towards the reference point. The
	point is at the appropriate position of the bounding box of
        the text.
       */
  typedef enum {
    CENTERED /**< Vertically and horitontally centered. */,
    NORTH /**< Top and horiz. centered. */,
    EAST /**< Right and vert. centered. */,
    SOUTH /**< Bottom and horiz. centered. */,
    WEST /**< Left Right . */,
    NORTH_EAST /**< Top right. */,
    SOUTH_EAST /**< Bottom right. */,
    SOUTH_WEST /**< Bottom left. */,
    NORTH_WEST /**< Top left. */
  } Anchor;
  const char * tostring_Anchor(Anchor value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint32_t counter; /**< Field */
  } VisualDisplay2DInterface_data_t;
#pragma pack(pop)

  VisualDisplay2DInterface_data_t *data;

 public:
  /* messages */
  class AddCartLineMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float x[2]; /**< X coordinates of two points */
      float y[2]; /**< Y coordinates of two
    points */
      LineStyle style; /**< Style of this object. */
      uint8_t color[4]; /**< Color in RGBA */
    } AddCartLineMessage_data_t;
#pragma pack(pop)

    AddCartLineMessage_data_t *data;

   public:
    AddCartLineMessage(const float * ini_x, const float * ini_y, const LineStyle ini_style, const uint8_t * ini_color);
    AddCartLineMessage();
    ~AddCartLineMessage();

    AddCartLineMessage(const AddCartLineMessage *m);
    /* Methods */
    float * x() const;
    float x(unsigned int index) const;
    void set_x(unsigned int index, const float new_x);
    void set_x(const float * new_x);
    size_t maxlenof_x() const;
    float * y() const;
    float y(unsigned int index) const;
    void set_y(unsigned int index, const float new_y);
    void set_y(const float * new_y);
    size_t maxlenof_y() const;
    LineStyle style() const;
    void set_style(const LineStyle new_style);
    size_t maxlenof_style() const;
    uint8_t * color() const;
    uint8_t color(unsigned int index) const;
    void set_color(unsigned int index, const uint8_t new_color);
    void set_color(const uint8_t * new_color);
    size_t maxlenof_color() const;
    virtual Message * clone() const;
  };

  class AddCartCircleMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float x; /**< X coordinate of center point */
      float y; /**< Y coordinate of center point */
      float radius; /**< Radius of the circle. */
      LineStyle style; /**< Style of this object. */
      uint8_t color[4]; /**< Color in RGBA */
    } AddCartCircleMessage_data_t;
#pragma pack(pop)

    AddCartCircleMessage_data_t *data;

   public:
    AddCartCircleMessage(const float ini_x, const float ini_y, const float ini_radius, const LineStyle ini_style, const uint8_t * ini_color);
    AddCartCircleMessage();
    ~AddCartCircleMessage();

    AddCartCircleMessage(const AddCartCircleMessage *m);
    /* Methods */
    float x() const;
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y() const;
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    float radius() const;
    void set_radius(const float new_radius);
    size_t maxlenof_radius() const;
    LineStyle style() const;
    void set_style(const LineStyle new_style);
    size_t maxlenof_style() const;
    uint8_t * color() const;
    uint8_t color(unsigned int index) const;
    void set_color(unsigned int index, const uint8_t new_color);
    void set_color(const uint8_t * new_color);
    size_t maxlenof_color() const;
    virtual Message * clone() const;
  };

  class AddCartRectMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float x; /**< X coordinate of lower right corner */
      float y; /**< Y coordinate of lower right corner */
      float width; /**< Width of rectangle */
      float height; /**< Height of rectangle */
      LineStyle style; /**< Style of this object. */
      uint8_t color[4]; /**< Color in RGBA */
    } AddCartRectMessage_data_t;
#pragma pack(pop)

    AddCartRectMessage_data_t *data;

   public:
    AddCartRectMessage(const float ini_x, const float ini_y, const float ini_width, const float ini_height, const LineStyle ini_style, const uint8_t * ini_color);
    AddCartRectMessage();
    ~AddCartRectMessage();

    AddCartRectMessage(const AddCartRectMessage *m);
    /* Methods */
    float x() const;
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y() const;
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    float width() const;
    void set_width(const float new_width);
    size_t maxlenof_width() const;
    float height() const;
    void set_height(const float new_height);
    size_t maxlenof_height() const;
    LineStyle style() const;
    void set_style(const LineStyle new_style);
    size_t maxlenof_style() const;
    uint8_t * color() const;
    uint8_t color(unsigned int index) const;
    void set_color(unsigned int index, const uint8_t new_color);
    void set_color(const uint8_t * new_color);
    size_t maxlenof_color() const;
    virtual Message * clone() const;
  };

  class AddCartTextMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float x; /**< X coordinate of upper left corner */
      float y; /**< Y coordinate of upper left corner */
      char text[128]; /**< Width of rectangle */
      Anchor anchor; /**< Anchor which marks the
      alignment to the given point. */
      float size; /**< Font size (max height in m). */
      uint8_t color[4]; /**< Color in RGBA */
    } AddCartTextMessage_data_t;
#pragma pack(pop)

    AddCartTextMessage_data_t *data;

   public:
    AddCartTextMessage(const float ini_x, const float ini_y, const char * ini_text, const Anchor ini_anchor, const float ini_size, const uint8_t * ini_color);
    AddCartTextMessage();
    ~AddCartTextMessage();

    AddCartTextMessage(const AddCartTextMessage *m);
    /* Methods */
    float x() const;
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y() const;
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    char * text() const;
    void set_text(const char * new_text);
    size_t maxlenof_text() const;
    Anchor anchor() const;
    void set_anchor(const Anchor new_anchor);
    size_t maxlenof_anchor() const;
    float size() const;
    void set_size(const float new_size);
    size_t maxlenof_size() const;
    uint8_t * color() const;
    uint8_t color(unsigned int index) const;
    void set_color(unsigned int index, const uint8_t new_color);
    void set_color(const uint8_t * new_color);
    size_t maxlenof_color() const;
    virtual Message * clone() const;
  };

  class DeleteObjectMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint32_t object_id; /**< Object ID, which is
    the message ID of the Add* message. */
    } DeleteObjectMessage_data_t;
#pragma pack(pop)

    DeleteObjectMessage_data_t *data;

   public:
    DeleteObjectMessage(const uint32_t ini_object_id);
    DeleteObjectMessage();
    ~DeleteObjectMessage();

    DeleteObjectMessage(const DeleteObjectMessage *m);
    /* Methods */
    uint32_t object_id() const;
    void set_object_id(const uint32_t new_object_id);
    size_t maxlenof_object_id() const;
    virtual Message * clone() const;
  };

  class DeleteAllMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } DeleteAllMessage_data_t;
#pragma pack(pop)

    DeleteAllMessage_data_t *data;

   public:
    DeleteAllMessage();
    ~DeleteAllMessage();

    DeleteAllMessage(const DeleteAllMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  VisualDisplay2DInterface();
  ~VisualDisplay2DInterface();

 public:
  /* Methods */
  uint32_t counter() const;
  void set_counter(const uint32_t new_counter);
  size_t maxlenof_counter() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
