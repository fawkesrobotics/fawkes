
/***************************************************************************
 *  visdisplay.cpp - Visual Display to show VisualDisplay2DInterface objects
 *
 *  Created: Thu Jan 07 23:48:49 2010
 *  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "visdisplay.h"

#include <interfaces/VisualDisplay2DInterface.h>

using namespace fawkes;


/** @class VisualDisplay2D "visdisplay.h"
 * 2D visualization processor for VisualDisplay2DInterface.
 * This class processes messages from the VisualDisplay2DInterface and
 * issues appropriate drawing commands to a Cairo drawing context.
 * @author Tim Niemueller
 */

/** Constructor. */
VisualDisplay2D::VisualDisplay2D()
{
  __interface = NULL;
}

/** Destructor. */
VisualDisplay2D::~VisualDisplay2D()
{
  for (__sit = __shapes.begin(); __sit != __shapes.end(); ++__sit) {
    delete __sit->second;
  }
  __shapes.clear();
}


/** Set interface.
 * @param interface interface to query for messages
 */
void
VisualDisplay2D::set_interface(fawkes::VisualDisplay2DInterface *interface)
{
  __interface = interface;
}


/** Process messages.
 * This processes the messages and builds up the internal object
 * representations.
 */
void
VisualDisplay2D::process_messages()
{
  while (! __interface->msgq_empty()) {
    if ( __interface->msgq_first_is<VisualDisplay2DInterface::AddCartLineMessage>() ) {
      VisualDisplay2DInterface::AddCartLineMessage *m = __interface->msgq_first<VisualDisplay2DInterface::AddCartLineMessage>();
      __shapes[m->id()] = new Line(m->x(0), m->y(0), m->x(1), m->y(1),
				   m->id(), m->sender_id(),
				   m->style(), m->color(0),
				   m->color(1), m->color(2), m->color(3));

    } else if ( __interface->msgq_first_is<VisualDisplay2DInterface::AddCartRectMessage>() ) {
      VisualDisplay2DInterface::AddCartRectMessage *m = __interface->msgq_first<VisualDisplay2DInterface::AddCartRectMessage>();
      __shapes[m->id()] = new Rectangle(m->x(), m->y(), m->width(), m->height(),
					m->id(), m->sender_id(),
					m->style(), m->color(0),
					m->color(1), m->color(2), m->color(3));

    } else if ( __interface->msgq_first_is<VisualDisplay2DInterface::AddCartCircleMessage>() ) {
      VisualDisplay2DInterface::AddCartCircleMessage *m = __interface->msgq_first<VisualDisplay2DInterface::AddCartCircleMessage>();
      __shapes[m->id()] = new Circle(m->x(), m->y(), m->radius(),
				     m->id(), m->sender_id(),
				     m->style(), m->color(0),
				     m->color(1), m->color(2), m->color(3));

    } else if ( __interface->msgq_first_is<VisualDisplay2DInterface::AddCartTextMessage>() ) {
      VisualDisplay2DInterface::AddCartTextMessage *m = __interface->msgq_first<VisualDisplay2DInterface::AddCartTextMessage>();
      __shapes[m->id()] = new Text(m->x(), m->y(), m->text(),
				   m->anchor(), m->size(),
				   m->id(), m->sender_id(),
				   m->color(0),
				   m->color(1), m->color(2), m->color(3));

    } else if (__interface->msgq_first_is<VisualDisplay2DInterface::DeleteAllMessage>() ) {
      for (__sit = __shapes.begin(); __sit != __shapes.end(); ++__sit) {
	delete __sit->second;
      }
      __shapes.clear();
    }

    __interface->msgq_pop();
  }
}


/** Draw objects.
 * This draws all objects currently enqueued by process_messages().
 * @param cr Cairo context to draw to
 */
void
VisualDisplay2D::draw(Cairo::RefPtr<Cairo::Context> cr)
{
  cr->save();
  for (__sit = __shapes.begin(); __sit != __shapes.end(); ++__sit) {
    float r, g, b, a;
    __sit->second->color(r, g, b, a);
    __sit->second->apply_style(cr);
    __sit->second->draw(cr);
  }
  cr->stroke();
  cr->restore();
}


/** @class VisualDisplay2D::Shape "visdisplay.h"
 * Class representing a shape.
 * All shapes inherit from the class and provide drawing primitives. The
 * internal object representations are instances of shapes.
 * @author Tim Niemueller
 *
 * @fn VisualDisplay2D::Shape::draw(Cairo::RefPtr<Cairo::Context> &cr)
 * Draw shape to Cairo context.
 * This method shall be implemented by a shape to draw itself using the
 * provided Cairo context.
 * @param cr reference to Cairo context. Note that this is a reference
 * bypassing the reference pointer. This is done for efficiency and with
 * the assumption that this method is only called by VisualDisplay2D::draw()
 * which itself has proper refptr handling.
 *
 * @fn inline void VisualDisplay2D::Shape::apply_style(Cairo::RefPtr<Cairo::Context> &cr)
 * Set style on context.
 * This method sets the style determined by the shape to the Cairo context.
 * @param cr reference to Cairo context. Note that this is a reference
 * bypassing the reference pointer. This is done for efficiency and with
 * the assumption that this method is only called by VisualDisplay2D::draw()
 * which itself has proper refptr handling.
 *
 * @fn inline unsigned int VisualDisplay2D::Shape::id()
 * Get shape ID.
 * @return shape ID
 *
 * @fn inline unsigned int VisualDisplay2D::Shape::owner()
 * Get owner ID.
 * @return owner ID
 *
 * @fn inline void VisualDisplay2D::Shape::color(float &r, float &g, float &b, float &a)
 * Get shape color.
 * @param r upon return contains red part of RGBA color
 * @param g upon return contains green part of RGBA color
 * @param b upon return contains blue part of RGBA color
 * @param a upon return contains alpha part of RGBA color
 */

/** Constructor.
 * @param id object ID
 * @param owner ID of the owner of the object
 * @param line_style drawing style of lines of shapes
 * @param r red part of RGBA color
 * @param g green part of RGBA color
 * @param b blue part of RGBA color
 * @param a alpha part of RGBA color
 */
VisualDisplay2D::Shape::Shape(unsigned int id, unsigned int owner,
			      VisualDisplay2DInterface::LineStyle line_style,
			      unsigned char r, unsigned char g,
			      unsigned char b, unsigned char a)
{
  _id         = id;
  _owner      = owner;
  _line_style = line_style;
  _color_r    = r / 255.f;
  _color_g    = g / 255.f;
  _color_b    = b / 255.f;
  _color_a    = a / 255.f;
}


/** Virtual empty destructor. */
VisualDisplay2D::Shape::~Shape()
{
}


/** @class VisualDisplay2D::Line "visdisplay.h"
 * Class representing a line.
 * Line represented by two end points in cartesian coordinates.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param x1 X coordinate of first point
 * @param y1 Y coordinate of first point
 * @param x2 X coordinate of second point
 * @param y2 Y coordinate of second point
 * @param id object ID
 * @param owner ID of the owner of the object
 * @param line_style drawing style of lines of shapes
 * @param r red part of RGBA color
 * @param g green part of RGBA color
 * @param b blue part of RGBA color
 * @param a alpha part of RGBA color
 */
VisualDisplay2D::Line::Line(float x1, float y1, float x2, float y2,
			    unsigned int id, unsigned int owner,
			    VisualDisplay2DInterface::LineStyle line_style,
			    unsigned char r, unsigned char g,
			    unsigned char b, unsigned char a)
  : Shape(id, owner, line_style, r, g, b, a)
{
  __x1 = x1;
  __y1 = y1;
  __x2 = x2;
  __y2 = y2;
}


void
VisualDisplay2D::Line::draw(Cairo::RefPtr<Cairo::Context> &cr)
{
  cr->move_to(__x1, __y1);
  cr->line_to(__x2, __y2);
  cr->stroke();
}



/** @class VisualDisplay2D::Rectangle "visdisplay.h"
 * Class representing a rectangle.
 * Rectangle represented the cartesian coordinates of the lower right corner
 * and its width and height.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param x X coordinate of lower right point
 * @param y Y coordinate of lower right  point
 * @param width width of rectangle
 * @param height height of rectangle
 * @param id object ID
 * @param owner ID of the owner of the object
 * @param line_style drawing style of lines of shapes
 * @param r red part of RGBA color
 * @param g green part of RGBA color
 * @param b blue part of RGBA color
 * @param a alpha part of RGBA color
 */
VisualDisplay2D::Rectangle::Rectangle(float x, float y, float width, float height,
				      unsigned int id, unsigned int owner,
				      VisualDisplay2DInterface::LineStyle line_style,
				      unsigned char r, unsigned char g,
				      unsigned char b, unsigned char a)
  : Shape(id, owner, line_style, r, g, b, a)
{
  __x      = x;
  __y      = y;
  __width  = width;
  __height = height;
}


void
VisualDisplay2D::Rectangle::draw(Cairo::RefPtr<Cairo::Context> &cr)
{
  cr->rectangle(__x, __y, __width, __height);
}



/** @class VisualDisplay2D::Circle "visdisplay.h"
 * Class representing a circle
 * Line represented by its center point and radius.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param x X coordinate of center point
 * @param y Y coordinate of center point
 * @param radius radius of the circle
 * @param id object ID
 * @param owner ID of the owner of the object
 * @param line_style drawing style of lines of shapes
 * @param r red part of RGBA color
 * @param g green part of RGBA color
 * @param b blue part of RGBA color
 * @param a alpha part of RGBA color
 */
VisualDisplay2D::Circle::Circle(float x, float y, float radius,
				unsigned int id, unsigned int owner,
				VisualDisplay2DInterface::LineStyle line_style,
				unsigned char r, unsigned char g,
				unsigned char b, unsigned char a)
  : Shape(id, owner, line_style, r, g, b, a)
{
  __x      = x;
  __y      = y;
  __radius = radius;
}


void
VisualDisplay2D::Circle::draw(Cairo::RefPtr<Cairo::Context> &cr)
{
  cr->arc(__x, __y, __radius, 0, 2*M_PI);
}


/** @class VisualDisplay2D::Text "visdisplay.h"
 * Class representing a text object.
 * Text is represented by a cartesian coordinate, which denotes a specific
 * point defined by the anchor, the text itself, and a text size.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param x X coordinate of anchor point
 * @param y Y coordinate of anchor point
 * @param text text to display
 * @param anchor anchor point relative to the text's bounding box
 * @param size height of font in meters
 * @param id object ID
 * @param owner ID of the owner of the object
 * @param r red part of RGBA color
 * @param g green part of RGBA color
 * @param b blue part of RGBA color
 * @param a alpha part of RGBA color
 */
VisualDisplay2D::Text::Text(float x, float y, std::string text,
			    fawkes::VisualDisplay2DInterface::Anchor anchor,
			    float size,
			    unsigned int id, unsigned int owner,
			    unsigned char r, unsigned char g,
			    unsigned char b, unsigned char a)
  : Shape(id, owner, fawkes::VisualDisplay2DInterface::LS_SOLID, r, g, b, a)
{
  __x      = x;
  __y      = y;
  __text   = text;
  __size   = size;
  __anchor = anchor;
}


void
VisualDisplay2D::Text::draw(Cairo::RefPtr<Cairo::Context> &cr)
{
  cr->save();
  cr->scale(-1, 1);
  cr->rotate(-0.5 * M_PI);
  cr->set_font_size(1.36 * __size);

  Cairo::TextExtents te;
  cr->get_text_extents(__text, te);

  float x = __x, y = __y;
  switch (__anchor) {
  case VisualDisplay2DInterface::CENTERED:
    x = __x - te.width / 2.;  y = __y + te.height / 2.; break;
  case VisualDisplay2DInterface::NORTH:
    x = __x - te.width / 2.;  y = __y + te.height; break;
  case VisualDisplay2DInterface::EAST:
    x = __x - te.width; y = __y + te.height / 2.; break;
  case VisualDisplay2DInterface::SOUTH:
    x = __x - te.width / 2.; break;
  case VisualDisplay2DInterface::WEST:
    y = __y + te.height / 2.; break;
  case VisualDisplay2DInterface::NORTH_EAST:
    x = __x - te.width;  y = __y + te.height; break;
  case VisualDisplay2DInterface::SOUTH_EAST:
    x = __x - te.width; break;
  case VisualDisplay2DInterface::SOUTH_WEST:
    break;
  case VisualDisplay2DInterface::NORTH_WEST:
    y = __y + te.height; break;
  }

  cr->move_to(x, y);
  cr->show_text(__text);
  cr->restore();
}
