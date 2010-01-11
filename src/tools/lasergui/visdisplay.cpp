
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

VisualDisplay2D::VisualDisplay2D()
{
  __interface = NULL;
}

VisualDisplay2D::~VisualDisplay2D()
{
  for (__sit = __shapes.begin(); __sit != __shapes.end(); ++__sit) {
    delete __sit->second;
  }
  __shapes.clear();
}

void
VisualDisplay2D::set_interface(fawkes::VisualDisplay2DInterface *interface)
{
  __interface = interface;
}


void
VisualDisplay2D::process_messages()
{
  while (! __interface->msgq_empty()) {
    if ( __interface->msgq_first_is<VisualDisplay2DInterface::AddCartLineMessage>() ) {
      VisualDisplay2DInterface::AddCartLineMessage *m = __interface->msgq_first<VisualDisplay2DInterface::AddCartLineMessage>();
      __shapes[m->id()] = new Line(m->x(0), m->y(0), m->x(1), m->y(1),
				   m->id(), m->sender_id(),
				   (LineStyle)m->style(), m->color(0),
				   m->color(1), m->color(2), m->color(3));
      printf("Enqueuing line %u (%f, %f) -> (%f, %f)\n", m->id(),
	     m->x(0), m->y(0), m->x(1), m->y(1));
    } else if (__interface->msgq_first_is<VisualDisplay2DInterface::DeleteAllMessage>() ) {
      for (__sit = __shapes.begin(); __sit != __shapes.end(); ++__sit) {
	delete __sit->second;
      }
      __shapes.clear();
    }

    __interface->msgq_pop();
  }
}


void
VisualDisplay2D::draw(Cairo::RefPtr<Cairo::Context> cr)
{
  cr->save();
  for (__sit = __shapes.begin(); __sit != __shapes.end(); ++__sit) {
    float r, g, b, a;
    __sit->second->color(r, g, b, a);
    printf("Drawing shape %u, color (%f, %f, %f, %f)\n", __sit->second->id(),
	   r, g, b, a);
    __sit->second->apply_style(cr);
    __sit->second->draw(cr);
  }
  cr->stroke();
  cr->restore();
}


VisualDisplay2D::Shape::Shape(unsigned int id, unsigned int owner,
			      LineStyle line_style,
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

VisualDisplay2D::Shape::~Shape()
{
}


VisualDisplay2D::Line::Line(float x1, float y1, float x2, float y2,
			    unsigned int id, unsigned int owner,
			    LineStyle line_style,
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
  printf("Moving to (%f, %f)\n", __x1, __y1);
  cr->move_to(__x1, __y1);
  printf("Line to (%f, %f)\n", __x2, __y2);
  cr->line_to(__x2, __y2);
  cr->stroke();
}



VisualDisplay2D::Rectangle::Rectangle(float x, float y, float width, float height,
				      unsigned int id, unsigned int owner,
				      LineStyle line_style,
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



VisualDisplay2D::Circle::Circle(float x, float y, float radius,
				unsigned int id, unsigned int owner,
				LineStyle line_style,
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


VisualDisplay2D::Text::Text(float x, float y, std::string text,
			    unsigned int id, unsigned int owner,
			    LineStyle line_style,
			    unsigned char r, unsigned char g,
			    unsigned char b, unsigned char a)
  : Shape(id, owner, line_style, r, g, b, a)
{
  __x      = x;
  __y      = y;
  __text   = text;
}


void
VisualDisplay2D::Text::draw(Cairo::RefPtr<Cairo::Context> &cr)
{
  cr->move_to(__x, __y);
  cr->text_path(__text);
  cr->fill();
}
