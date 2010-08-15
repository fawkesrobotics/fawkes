
/***************************************************************************
 *  VisualDisplay2DInterface.cpp - Fawkes BlackBoard Interface - VisualDisplay2DInterface
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

#include <interfaces/VisualDisplay2DInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class VisualDisplay2DInterface <interfaces/VisualDisplay2DInterface.h>
 * VisualDisplay2DInterface Fawkes BlackBoard Interface.
 * 
      This interface provides can be used by graphing applications to
      provide a graphing service to other components. This is intended
      to be used for debugging purposes. Usage of the interface should
      be optional to turn it off during a competition. 

      Add* messages will add the given object permanently, so the
      graphical display can be considered as a scenegraph. The message
      ID is becomes the ID and can be used to delete the object using
      the DeleteObjectMessage. With the DeleteAll message all objects
      can be removed (shall only remove objects added by the same
      sender, thus data drawn by other senders is not touched).

      The units shall be in meters and radians. Color is given as four
      byte RGBA value, one byte for each R, G, B and Alpha.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
VisualDisplay2DInterface::VisualDisplay2DInterface() : Interface()
{
  data_size = sizeof(VisualDisplay2DInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (VisualDisplay2DInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "counter", 1, &data->counter);
  add_messageinfo("AddCartLineMessage");
  add_messageinfo("AddCartCircleMessage");
  add_messageinfo("AddCartRectMessage");
  add_messageinfo("AddCartTextMessage");
  add_messageinfo("DeleteObjectMessage");
  add_messageinfo("DeleteAllMessage");
  unsigned char tmp_hash[] = {0xd9, 0x2, 0xad, 0xbb, 0x7a, 0x47, 0x40, 0x6a, 0x4f, 0x6d, 0xfa, 0xa, 0x20, 0x35, 0xe6, 0x1};
  set_hash(tmp_hash);
}

/** Destructor */
VisualDisplay2DInterface::~VisualDisplay2DInterface()
{
  free(data_ptr);
}
/** Convert LineStyle constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
VisualDisplay2DInterface::tostring_LineStyle(LineStyle value) const
{
  switch (value) {
  case LS_SOLID: return "LS_SOLID";
  case LS_DASHED: return "LS_DASHED";
  case LS_DOTTED: return "LS_DOTTED";
  case LS_DASH_DOTTED: return "LS_DASH_DOTTED";
  default: return "UNKNOWN";
  }
}
/** Convert Anchor constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
VisualDisplay2DInterface::tostring_Anchor(Anchor value) const
{
  switch (value) {
  case CENTERED: return "CENTERED";
  case NORTH: return "NORTH";
  case EAST: return "EAST";
  case SOUTH: return "SOUTH";
  case WEST: return "WEST";
  case NORTH_EAST: return "NORTH_EAST";
  case SOUTH_EAST: return "SOUTH_EAST";
  case SOUTH_WEST: return "SOUTH_WEST";
  case NORTH_WEST: return "NORTH_WEST";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get counter value.
 * Field
 * @return counter value
 */
uint32_t
VisualDisplay2DInterface::counter() const
{
  return data->counter;
}

/** Get maximum length of counter value.
 * @return length of counter value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::maxlenof_counter() const
{
  return 1;
}

/** Set counter value.
 * Field
 * @param new_counter new counter value
 */
void
VisualDisplay2DInterface::set_counter(const uint32_t new_counter)
{
  data->counter = new_counter;
  data_changed = true;
}

/* =========== message create =========== */
Message *
VisualDisplay2DInterface::create_message(const char *type) const
{
  if ( strncmp("AddCartLineMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddCartLineMessage();
  } else if ( strncmp("AddCartCircleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddCartCircleMessage();
  } else if ( strncmp("AddCartRectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddCartRectMessage();
  } else if ( strncmp("AddCartTextMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AddCartTextMessage();
  } else if ( strncmp("DeleteObjectMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DeleteObjectMessage();
  } else if ( strncmp("DeleteAllMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DeleteAllMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
VisualDisplay2DInterface::copy_values(const Interface *other)
{
  const VisualDisplay2DInterface *oi = dynamic_cast<const VisualDisplay2DInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(VisualDisplay2DInterface_data_t));
}

const char *
VisualDisplay2DInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "LineStyle") == 0) {
    return tostring_LineStyle((LineStyle)val);
  }
  if (strcmp(enumtype, "Anchor") == 0) {
    return tostring_Anchor((Anchor)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class VisualDisplay2DInterface::AddCartLineMessage <interfaces/VisualDisplay2DInterface.h>
 * AddCartLineMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_style initial value for style
 * @param ini_color initial value for color
 */
VisualDisplay2DInterface::AddCartLineMessage::AddCartLineMessage(const float * ini_x, const float * ini_y, const LineStyle ini_style, const uint8_t * ini_color) : Message("AddCartLineMessage")
{
  data_size = sizeof(AddCartLineMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddCartLineMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  memcpy(data->x, ini_x, sizeof(float) * 2);
  memcpy(data->y, ini_y, sizeof(float) * 2);
  data->style = ini_style;
  memcpy(data->color, ini_color, sizeof(uint8_t) * 4);
  add_fieldinfo(IFT_FLOAT, "x", 2, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 2, &data->y);
  add_fieldinfo(IFT_ENUM, "style", 1, &data->style, "LineStyle");
  add_fieldinfo(IFT_BYTE, "color", 4, &data->color);
}
/** Constructor */
VisualDisplay2DInterface::AddCartLineMessage::AddCartLineMessage() : Message("AddCartLineMessage")
{
  data_size = sizeof(AddCartLineMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddCartLineMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "x", 2, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 2, &data->y);
  add_fieldinfo(IFT_ENUM, "style", 1, &data->style, "LineStyle");
  add_fieldinfo(IFT_BYTE, "color", 4, &data->color);
}

/** Destructor */
VisualDisplay2DInterface::AddCartLineMessage::~AddCartLineMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
VisualDisplay2DInterface::AddCartLineMessage::AddCartLineMessage(const AddCartLineMessage *m) : Message("AddCartLineMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AddCartLineMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X coordinates of two points
 * @return x value
 */
float *
VisualDisplay2DInterface::AddCartLineMessage::x() const
{
  return data->x;
}

/** Get x value at given index.
 * X coordinates of two points
 * @param index index of value
 * @return x value
 * @exception Exception thrown if index is out of bounds
 */
float
VisualDisplay2DInterface::AddCartLineMessage::x(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->x[index];
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartLineMessage::maxlenof_x() const
{
  return 2;
}

/** Set x value.
 * X coordinates of two points
 * @param new_x new x value
 */
void
VisualDisplay2DInterface::AddCartLineMessage::set_x(const float * new_x)
{
  memcpy(data->x, new_x, sizeof(float) * 2);
}

/** Set x value at given index.
 * X coordinates of two points
 * @param new_x new x value
 * @param index index for of the value
 */
void
VisualDisplay2DInterface::AddCartLineMessage::set_x(unsigned int index, const float new_x)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->x[index] = new_x;
}
/** Get y value.
 * Y coordinates of two
    points
 * @return y value
 */
float *
VisualDisplay2DInterface::AddCartLineMessage::y() const
{
  return data->y;
}

/** Get y value at given index.
 * Y coordinates of two
    points
 * @param index index of value
 * @return y value
 * @exception Exception thrown if index is out of bounds
 */
float
VisualDisplay2DInterface::AddCartLineMessage::y(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->y[index];
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartLineMessage::maxlenof_y() const
{
  return 2;
}

/** Set y value.
 * Y coordinates of two
    points
 * @param new_y new y value
 */
void
VisualDisplay2DInterface::AddCartLineMessage::set_y(const float * new_y)
{
  memcpy(data->y, new_y, sizeof(float) * 2);
}

/** Set y value at given index.
 * Y coordinates of two
    points
 * @param new_y new y value
 * @param index index for of the value
 */
void
VisualDisplay2DInterface::AddCartLineMessage::set_y(unsigned int index, const float new_y)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->y[index] = new_y;
}
/** Get style value.
 * Style of this object.
 * @return style value
 */
VisualDisplay2DInterface::LineStyle
VisualDisplay2DInterface::AddCartLineMessage::style() const
{
  return data->style;
}

/** Get maximum length of style value.
 * @return length of style value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartLineMessage::maxlenof_style() const
{
  return 1;
}

/** Set style value.
 * Style of this object.
 * @param new_style new style value
 */
void
VisualDisplay2DInterface::AddCartLineMessage::set_style(const LineStyle new_style)
{
  data->style = new_style;
}

/** Get color value.
 * Color in RGBA
 * @return color value
 */
uint8_t *
VisualDisplay2DInterface::AddCartLineMessage::color() const
{
  return data->color;
}

/** Get color value at given index.
 * Color in RGBA
 * @param index index of value
 * @return color value
 * @exception Exception thrown if index is out of bounds
 */
uint8_t
VisualDisplay2DInterface::AddCartLineMessage::color(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->color[index];
}

/** Get maximum length of color value.
 * @return length of color value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartLineMessage::maxlenof_color() const
{
  return 4;
}

/** Set color value.
 * Color in RGBA
 * @param new_color new color value
 */
void
VisualDisplay2DInterface::AddCartLineMessage::set_color(const uint8_t * new_color)
{
  memcpy(data->color, new_color, sizeof(uint8_t) * 4);
}

/** Set color value at given index.
 * Color in RGBA
 * @param new_color new color value
 * @param index index for of the value
 */
void
VisualDisplay2DInterface::AddCartLineMessage::set_color(unsigned int index, const uint8_t new_color)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->color[index] = new_color;
}
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
VisualDisplay2DInterface::AddCartLineMessage::clone() const
{
  return new VisualDisplay2DInterface::AddCartLineMessage(this);
}
/** @class VisualDisplay2DInterface::AddCartCircleMessage <interfaces/VisualDisplay2DInterface.h>
 * AddCartCircleMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_radius initial value for radius
 * @param ini_style initial value for style
 * @param ini_color initial value for color
 */
VisualDisplay2DInterface::AddCartCircleMessage::AddCartCircleMessage(const float ini_x, const float ini_y, const float ini_radius, const LineStyle ini_style, const uint8_t * ini_color) : Message("AddCartCircleMessage")
{
  data_size = sizeof(AddCartCircleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddCartCircleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->radius = ini_radius;
  data->style = ini_style;
  memcpy(data->color, ini_color, sizeof(uint8_t) * 4);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "radius", 1, &data->radius);
  add_fieldinfo(IFT_ENUM, "style", 1, &data->style, "LineStyle");
  add_fieldinfo(IFT_BYTE, "color", 4, &data->color);
}
/** Constructor */
VisualDisplay2DInterface::AddCartCircleMessage::AddCartCircleMessage() : Message("AddCartCircleMessage")
{
  data_size = sizeof(AddCartCircleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddCartCircleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "radius", 1, &data->radius);
  add_fieldinfo(IFT_ENUM, "style", 1, &data->style, "LineStyle");
  add_fieldinfo(IFT_BYTE, "color", 4, &data->color);
}

/** Destructor */
VisualDisplay2DInterface::AddCartCircleMessage::~AddCartCircleMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
VisualDisplay2DInterface::AddCartCircleMessage::AddCartCircleMessage(const AddCartCircleMessage *m) : Message("AddCartCircleMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AddCartCircleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X coordinate of center point
 * @return x value
 */
float
VisualDisplay2DInterface::AddCartCircleMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartCircleMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X coordinate of center point
 * @param new_x new x value
 */
void
VisualDisplay2DInterface::AddCartCircleMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y coordinate of center point
 * @return y value
 */
float
VisualDisplay2DInterface::AddCartCircleMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartCircleMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y coordinate of center point
 * @param new_y new y value
 */
void
VisualDisplay2DInterface::AddCartCircleMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get radius value.
 * Radius of the circle.
 * @return radius value
 */
float
VisualDisplay2DInterface::AddCartCircleMessage::radius() const
{
  return data->radius;
}

/** Get maximum length of radius value.
 * @return length of radius value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartCircleMessage::maxlenof_radius() const
{
  return 1;
}

/** Set radius value.
 * Radius of the circle.
 * @param new_radius new radius value
 */
void
VisualDisplay2DInterface::AddCartCircleMessage::set_radius(const float new_radius)
{
  data->radius = new_radius;
}

/** Get style value.
 * Style of this object.
 * @return style value
 */
VisualDisplay2DInterface::LineStyle
VisualDisplay2DInterface::AddCartCircleMessage::style() const
{
  return data->style;
}

/** Get maximum length of style value.
 * @return length of style value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartCircleMessage::maxlenof_style() const
{
  return 1;
}

/** Set style value.
 * Style of this object.
 * @param new_style new style value
 */
void
VisualDisplay2DInterface::AddCartCircleMessage::set_style(const LineStyle new_style)
{
  data->style = new_style;
}

/** Get color value.
 * Color in RGBA
 * @return color value
 */
uint8_t *
VisualDisplay2DInterface::AddCartCircleMessage::color() const
{
  return data->color;
}

/** Get color value at given index.
 * Color in RGBA
 * @param index index of value
 * @return color value
 * @exception Exception thrown if index is out of bounds
 */
uint8_t
VisualDisplay2DInterface::AddCartCircleMessage::color(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->color[index];
}

/** Get maximum length of color value.
 * @return length of color value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartCircleMessage::maxlenof_color() const
{
  return 4;
}

/** Set color value.
 * Color in RGBA
 * @param new_color new color value
 */
void
VisualDisplay2DInterface::AddCartCircleMessage::set_color(const uint8_t * new_color)
{
  memcpy(data->color, new_color, sizeof(uint8_t) * 4);
}

/** Set color value at given index.
 * Color in RGBA
 * @param new_color new color value
 * @param index index for of the value
 */
void
VisualDisplay2DInterface::AddCartCircleMessage::set_color(unsigned int index, const uint8_t new_color)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->color[index] = new_color;
}
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
VisualDisplay2DInterface::AddCartCircleMessage::clone() const
{
  return new VisualDisplay2DInterface::AddCartCircleMessage(this);
}
/** @class VisualDisplay2DInterface::AddCartRectMessage <interfaces/VisualDisplay2DInterface.h>
 * AddCartRectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_width initial value for width
 * @param ini_height initial value for height
 * @param ini_style initial value for style
 * @param ini_color initial value for color
 */
VisualDisplay2DInterface::AddCartRectMessage::AddCartRectMessage(const float ini_x, const float ini_y, const float ini_width, const float ini_height, const LineStyle ini_style, const uint8_t * ini_color) : Message("AddCartRectMessage")
{
  data_size = sizeof(AddCartRectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddCartRectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->width = ini_width;
  data->height = ini_height;
  data->style = ini_style;
  memcpy(data->color, ini_color, sizeof(uint8_t) * 4);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "width", 1, &data->width);
  add_fieldinfo(IFT_FLOAT, "height", 1, &data->height);
  add_fieldinfo(IFT_ENUM, "style", 1, &data->style, "LineStyle");
  add_fieldinfo(IFT_BYTE, "color", 4, &data->color);
}
/** Constructor */
VisualDisplay2DInterface::AddCartRectMessage::AddCartRectMessage() : Message("AddCartRectMessage")
{
  data_size = sizeof(AddCartRectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddCartRectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_FLOAT, "width", 1, &data->width);
  add_fieldinfo(IFT_FLOAT, "height", 1, &data->height);
  add_fieldinfo(IFT_ENUM, "style", 1, &data->style, "LineStyle");
  add_fieldinfo(IFT_BYTE, "color", 4, &data->color);
}

/** Destructor */
VisualDisplay2DInterface::AddCartRectMessage::~AddCartRectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
VisualDisplay2DInterface::AddCartRectMessage::AddCartRectMessage(const AddCartRectMessage *m) : Message("AddCartRectMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AddCartRectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X coordinate of lower right corner
 * @return x value
 */
float
VisualDisplay2DInterface::AddCartRectMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartRectMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X coordinate of lower right corner
 * @param new_x new x value
 */
void
VisualDisplay2DInterface::AddCartRectMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y coordinate of lower right corner
 * @return y value
 */
float
VisualDisplay2DInterface::AddCartRectMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartRectMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y coordinate of lower right corner
 * @param new_y new y value
 */
void
VisualDisplay2DInterface::AddCartRectMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get width value.
 * Width of rectangle
 * @return width value
 */
float
VisualDisplay2DInterface::AddCartRectMessage::width() const
{
  return data->width;
}

/** Get maximum length of width value.
 * @return length of width value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartRectMessage::maxlenof_width() const
{
  return 1;
}

/** Set width value.
 * Width of rectangle
 * @param new_width new width value
 */
void
VisualDisplay2DInterface::AddCartRectMessage::set_width(const float new_width)
{
  data->width = new_width;
}

/** Get height value.
 * Height of rectangle
 * @return height value
 */
float
VisualDisplay2DInterface::AddCartRectMessage::height() const
{
  return data->height;
}

/** Get maximum length of height value.
 * @return length of height value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartRectMessage::maxlenof_height() const
{
  return 1;
}

/** Set height value.
 * Height of rectangle
 * @param new_height new height value
 */
void
VisualDisplay2DInterface::AddCartRectMessage::set_height(const float new_height)
{
  data->height = new_height;
}

/** Get style value.
 * Style of this object.
 * @return style value
 */
VisualDisplay2DInterface::LineStyle
VisualDisplay2DInterface::AddCartRectMessage::style() const
{
  return data->style;
}

/** Get maximum length of style value.
 * @return length of style value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartRectMessage::maxlenof_style() const
{
  return 1;
}

/** Set style value.
 * Style of this object.
 * @param new_style new style value
 */
void
VisualDisplay2DInterface::AddCartRectMessage::set_style(const LineStyle new_style)
{
  data->style = new_style;
}

/** Get color value.
 * Color in RGBA
 * @return color value
 */
uint8_t *
VisualDisplay2DInterface::AddCartRectMessage::color() const
{
  return data->color;
}

/** Get color value at given index.
 * Color in RGBA
 * @param index index of value
 * @return color value
 * @exception Exception thrown if index is out of bounds
 */
uint8_t
VisualDisplay2DInterface::AddCartRectMessage::color(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->color[index];
}

/** Get maximum length of color value.
 * @return length of color value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartRectMessage::maxlenof_color() const
{
  return 4;
}

/** Set color value.
 * Color in RGBA
 * @param new_color new color value
 */
void
VisualDisplay2DInterface::AddCartRectMessage::set_color(const uint8_t * new_color)
{
  memcpy(data->color, new_color, sizeof(uint8_t) * 4);
}

/** Set color value at given index.
 * Color in RGBA
 * @param new_color new color value
 * @param index index for of the value
 */
void
VisualDisplay2DInterface::AddCartRectMessage::set_color(unsigned int index, const uint8_t new_color)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->color[index] = new_color;
}
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
VisualDisplay2DInterface::AddCartRectMessage::clone() const
{
  return new VisualDisplay2DInterface::AddCartRectMessage(this);
}
/** @class VisualDisplay2DInterface::AddCartTextMessage <interfaces/VisualDisplay2DInterface.h>
 * AddCartTextMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_text initial value for text
 * @param ini_anchor initial value for anchor
 * @param ini_size initial value for size
 * @param ini_color initial value for color
 */
VisualDisplay2DInterface::AddCartTextMessage::AddCartTextMessage(const float ini_x, const float ini_y, const char * ini_text, const Anchor ini_anchor, const float ini_size, const uint8_t * ini_color) : Message("AddCartTextMessage")
{
  data_size = sizeof(AddCartTextMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddCartTextMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  strncpy(data->text, ini_text, 128);
  data->anchor = ini_anchor;
  data->size = ini_size;
  memcpy(data->color, ini_color, sizeof(uint8_t) * 4);
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_STRING, "text", 128, data->text);
  add_fieldinfo(IFT_ENUM, "anchor", 1, &data->anchor, "Anchor");
  add_fieldinfo(IFT_FLOAT, "size", 1, &data->size);
  add_fieldinfo(IFT_BYTE, "color", 4, &data->color);
}
/** Constructor */
VisualDisplay2DInterface::AddCartTextMessage::AddCartTextMessage() : Message("AddCartTextMessage")
{
  data_size = sizeof(AddCartTextMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (AddCartTextMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "x", 1, &data->x);
  add_fieldinfo(IFT_FLOAT, "y", 1, &data->y);
  add_fieldinfo(IFT_STRING, "text", 128, data->text);
  add_fieldinfo(IFT_ENUM, "anchor", 1, &data->anchor, "Anchor");
  add_fieldinfo(IFT_FLOAT, "size", 1, &data->size);
  add_fieldinfo(IFT_BYTE, "color", 4, &data->color);
}

/** Destructor */
VisualDisplay2DInterface::AddCartTextMessage::~AddCartTextMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
VisualDisplay2DInterface::AddCartTextMessage::AddCartTextMessage(const AddCartTextMessage *m) : Message("AddCartTextMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (AddCartTextMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get x value.
 * X coordinate of upper left corner
 * @return x value
 */
float
VisualDisplay2DInterface::AddCartTextMessage::x() const
{
  return data->x;
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartTextMessage::maxlenof_x() const
{
  return 1;
}

/** Set x value.
 * X coordinate of upper left corner
 * @param new_x new x value
 */
void
VisualDisplay2DInterface::AddCartTextMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y coordinate of upper left corner
 * @return y value
 */
float
VisualDisplay2DInterface::AddCartTextMessage::y() const
{
  return data->y;
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartTextMessage::maxlenof_y() const
{
  return 1;
}

/** Set y value.
 * Y coordinate of upper left corner
 * @param new_y new y value
 */
void
VisualDisplay2DInterface::AddCartTextMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get text value.
 * Width of rectangle
 * @return text value
 */
char *
VisualDisplay2DInterface::AddCartTextMessage::text() const
{
  return data->text;
}

/** Get maximum length of text value.
 * @return length of text value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartTextMessage::maxlenof_text() const
{
  return 128;
}

/** Set text value.
 * Width of rectangle
 * @param new_text new text value
 */
void
VisualDisplay2DInterface::AddCartTextMessage::set_text(const char * new_text)
{
  strncpy(data->text, new_text, sizeof(data->text));
}

/** Get anchor value.
 * Anchor which marks the
      alignment to the given point.
 * @return anchor value
 */
VisualDisplay2DInterface::Anchor
VisualDisplay2DInterface::AddCartTextMessage::anchor() const
{
  return data->anchor;
}

/** Get maximum length of anchor value.
 * @return length of anchor value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartTextMessage::maxlenof_anchor() const
{
  return 1;
}

/** Set anchor value.
 * Anchor which marks the
      alignment to the given point.
 * @param new_anchor new anchor value
 */
void
VisualDisplay2DInterface::AddCartTextMessage::set_anchor(const Anchor new_anchor)
{
  data->anchor = new_anchor;
}

/** Get size value.
 * Font size (max height in m).
 * @return size value
 */
float
VisualDisplay2DInterface::AddCartTextMessage::size() const
{
  return data->size;
}

/** Get maximum length of size value.
 * @return length of size value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartTextMessage::maxlenof_size() const
{
  return 1;
}

/** Set size value.
 * Font size (max height in m).
 * @param new_size new size value
 */
void
VisualDisplay2DInterface::AddCartTextMessage::set_size(const float new_size)
{
  data->size = new_size;
}

/** Get color value.
 * Color in RGBA
 * @return color value
 */
uint8_t *
VisualDisplay2DInterface::AddCartTextMessage::color() const
{
  return data->color;
}

/** Get color value at given index.
 * Color in RGBA
 * @param index index of value
 * @return color value
 * @exception Exception thrown if index is out of bounds
 */
uint8_t
VisualDisplay2DInterface::AddCartTextMessage::color(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->color[index];
}

/** Get maximum length of color value.
 * @return length of color value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::AddCartTextMessage::maxlenof_color() const
{
  return 4;
}

/** Set color value.
 * Color in RGBA
 * @param new_color new color value
 */
void
VisualDisplay2DInterface::AddCartTextMessage::set_color(const uint8_t * new_color)
{
  memcpy(data->color, new_color, sizeof(uint8_t) * 4);
}

/** Set color value at given index.
 * Color in RGBA
 * @param new_color new color value
 * @param index index for of the value
 */
void
VisualDisplay2DInterface::AddCartTextMessage::set_color(unsigned int index, const uint8_t new_color)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->color[index] = new_color;
}
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
VisualDisplay2DInterface::AddCartTextMessage::clone() const
{
  return new VisualDisplay2DInterface::AddCartTextMessage(this);
}
/** @class VisualDisplay2DInterface::DeleteObjectMessage <interfaces/VisualDisplay2DInterface.h>
 * DeleteObjectMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_object_id initial value for object_id
 */
VisualDisplay2DInterface::DeleteObjectMessage::DeleteObjectMessage(const uint32_t ini_object_id) : Message("DeleteObjectMessage")
{
  data_size = sizeof(DeleteObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DeleteObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->object_id = ini_object_id;
  add_fieldinfo(IFT_UINT32, "object_id", 1, &data->object_id);
}
/** Constructor */
VisualDisplay2DInterface::DeleteObjectMessage::DeleteObjectMessage() : Message("DeleteObjectMessage")
{
  data_size = sizeof(DeleteObjectMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DeleteObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "object_id", 1, &data->object_id);
}

/** Destructor */
VisualDisplay2DInterface::DeleteObjectMessage::~DeleteObjectMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
VisualDisplay2DInterface::DeleteObjectMessage::DeleteObjectMessage(const DeleteObjectMessage *m) : Message("DeleteObjectMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (DeleteObjectMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get object_id value.
 * Object ID, which is
    the message ID of the Add* message.
 * @return object_id value
 */
uint32_t
VisualDisplay2DInterface::DeleteObjectMessage::object_id() const
{
  return data->object_id;
}

/** Get maximum length of object_id value.
 * @return length of object_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
VisualDisplay2DInterface::DeleteObjectMessage::maxlenof_object_id() const
{
  return 1;
}

/** Set object_id value.
 * Object ID, which is
    the message ID of the Add* message.
 * @param new_object_id new object_id value
 */
void
VisualDisplay2DInterface::DeleteObjectMessage::set_object_id(const uint32_t new_object_id)
{
  data->object_id = new_object_id;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
VisualDisplay2DInterface::DeleteObjectMessage::clone() const
{
  return new VisualDisplay2DInterface::DeleteObjectMessage(this);
}
/** @class VisualDisplay2DInterface::DeleteAllMessage <interfaces/VisualDisplay2DInterface.h>
 * DeleteAllMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
VisualDisplay2DInterface::DeleteAllMessage::DeleteAllMessage() : Message("DeleteAllMessage")
{
  data_size = sizeof(DeleteAllMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DeleteAllMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
VisualDisplay2DInterface::DeleteAllMessage::~DeleteAllMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
VisualDisplay2DInterface::DeleteAllMessage::DeleteAllMessage(const DeleteAllMessage *m) : Message("DeleteAllMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (DeleteAllMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
VisualDisplay2DInterface::DeleteAllMessage::clone() const
{
  return new VisualDisplay2DInterface::DeleteAllMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
VisualDisplay2DInterface::message_valid(const Message *message) const
{
  const AddCartLineMessage *m0 = dynamic_cast<const AddCartLineMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const AddCartCircleMessage *m1 = dynamic_cast<const AddCartCircleMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const AddCartRectMessage *m2 = dynamic_cast<const AddCartRectMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const AddCartTextMessage *m3 = dynamic_cast<const AddCartTextMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const DeleteObjectMessage *m4 = dynamic_cast<const DeleteObjectMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const DeleteAllMessage *m5 = dynamic_cast<const DeleteAllMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(VisualDisplay2DInterface)
/// @endcond


} // end namespace fawkes
