
/***************************************************************************
 *  visdisplay.h - Visual Display to show VisualDisplay2DInterface objects
 *
 *  Created: Thu Jan 07 23:36:15 2010
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

#ifndef __TOOLS_LASERGUI_VISDISPLAY_H_
#define __TOOLS_LASERGUI_VISDISPLAY_H_

#include <cairomm/context.h>
#include <string>
#include <map>

#include <interfaces/VisualDisplay2DInterface.h>

class VisualDisplay2D
{
 public:
  VisualDisplay2D();
  ~VisualDisplay2D();

  void set_interface(fawkes::VisualDisplay2DInterface *interface);

  void process_messages();
  void draw(Cairo::RefPtr<Cairo::Context> cr);

  class Shape {
   public:
    Shape(unsigned int id, unsigned int owner,
	  fawkes::VisualDisplay2DInterface::LineStyle line_style = fawkes::VisualDisplay2DInterface::LS_SOLID,
	  unsigned char r = 0, unsigned char g = 0,
	  unsigned char b = 0, unsigned char a = 0);
    virtual ~Shape();
    virtual void draw(Cairo::RefPtr<Cairo::Context> &cr) = 0;
    inline void apply_style(Cairo::RefPtr<Cairo::Context> &cr)
    { cr->set_source_rgba(_color_r, _color_g, _color_b, _color_a); }

    inline unsigned int id() { return _id; }
    inline unsigned int owner() { return _owner; }
    inline void color(float &r, float &g, float &b, float &a)
    { r = _color_r; g = _color_g; b = _color_b; a = _color_a; }
   protected:

    fawkes::VisualDisplay2DInterface::LineStyle  _line_style;	/**< Line style */
    float         _color_r;	/**< red part of RGBA object color */
    float         _color_g;	/**< green part of RGBA object color */
    float         _color_b;	/**< blue part of RGBA object color */
    float         _color_a;	/**< alpha part of RGBA object color */

    unsigned int  _id;		/**< Object ID */
    unsigned int  _owner;	/**< Owner ID */
  };

  class Line : public Shape {
   public:
    Line(float x1, float y1, float x2, float y2,
	 unsigned int id, unsigned int owner,
	 fawkes::VisualDisplay2DInterface::LineStyle line_style = fawkes::VisualDisplay2DInterface::LS_SOLID,
	 unsigned char r = 0, unsigned char g = 0,
	 unsigned char b = 0, unsigned char a = 0);
    void draw(Cairo::RefPtr<Cairo::Context> &cr);
  private:
    float __x1;
    float __y1;
    float __x2;
    float __y2;
  };

  class Rectangle : public Shape {
   public:
    Rectangle(float x, float y, float width, float height,
	      unsigned int id, unsigned int owner,
	      fawkes::VisualDisplay2DInterface::LineStyle line_style = fawkes::VisualDisplay2DInterface::LS_SOLID,
	      unsigned char r = 0, unsigned char g = 0,
	      unsigned char b = 0, unsigned char a = 0);
    void draw(Cairo::RefPtr<Cairo::Context> &cr);
   private:
    float __x;
    float __y;
    float __width;
    float __height;
  };

  class Circle : public Shape {
   public:
    Circle(float x, float y, float radius,
	   unsigned int id, unsigned int owner,
	   fawkes::VisualDisplay2DInterface::LineStyle line_style = fawkes::VisualDisplay2DInterface::LS_SOLID,
	   unsigned char r = 0, unsigned char g = 0,
	   unsigned char b = 0, unsigned char a = 0);
    void draw(Cairo::RefPtr<Cairo::Context> &cr);
   private:
    float __x;
    float __y;
    float __radius;
  };

  class Text : public Shape {
   public:
    Text(float x, float y, std::string text,
	 fawkes::VisualDisplay2DInterface::Anchor anchor,
	 float size,
	 unsigned int id, unsigned int owner,
	 unsigned char r = 0, unsigned char g = 0,
	 unsigned char b = 0, unsigned char a = 0);
    void draw(Cairo::RefPtr<Cairo::Context> &cr);
   private:
    float __x;
    float __y;
    std::string __text;
    float __size;
    fawkes::VisualDisplay2DInterface::Anchor __anchor;
  };

 private:
  std::map<unsigned int, Shape *>  __shapes;
  std::map<unsigned int, Shape *>::iterator  __sit;
  fawkes::VisualDisplay2DInterface  *__interface;
};


#endif
