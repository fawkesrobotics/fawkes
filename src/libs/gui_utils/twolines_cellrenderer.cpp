
/***************************************************************************
 *  twolines_cellrenderer.cpp - Gtk rell renderer for two lines of text
 *
 *  Created: Sat Nov 29 16:36:41 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <gui_utils/twolines_cellrenderer.h>

#include <gtkmm.h>
#include <gtk/gtkcellrenderer.h>
#include <glib-object.h>

#include <algorithm>
#include <cstring>
#include <cstdio>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class TwoLinesCellRenderer <gui_utils/twolines_cellrenderer.h>
 * Gtk cell renderer for two lines of text in a cell.
 * This cell renderer allows you to have two lines of text in a single
 * cell. It works by getting the text via two properties. The first line is
 * the primary line and printed "normally". The second line is the secondary
 * line and printed with a slightly smaller font.
 * @author Tim Niemueller
 */

/** Constructor. */
TwoLinesCellRenderer::TwoLinesCellRenderer()
  : Glib::ObjectBase(typeid(TwoLinesCellRenderer)),
    Gtk::CellRenderer(),
    __property_line1(*this, "line1", ""),
    __property_line2(*this, "line2", "")
{
}

/** Destructor. */
TwoLinesCellRenderer::~TwoLinesCellRenderer()
{
}

/** Get property proxy for first line.
 * @return property proxy for first line
 */
Glib::PropertyProxy<Glib::ustring>
TwoLinesCellRenderer::property_line1()
{
  return __property_line1.get_proxy();
}


/** Get property proxy for second line.
 * @return property proxy for second line
 */
Glib::PropertyProxy<Glib::ustring>
TwoLinesCellRenderer::property_line2()
{
  return __property_line2.get_proxy();
}


/** Get required size for cell.
 * @param widget widget
 * @param cell_area area of the cell
 * @param x_offset ignored
 * @param y_offset ignored
 * @param width upon return contains the required width of the cell
 * @param height upon return contains the required height of the cell
 */
void
TwoLinesCellRenderer::get_size_vfunc(Gtk::Widget &widget,
				     const Gdk::Rectangle *cell_area,
				     int *x_offset, int *y_offset,
				     int *width, int *height) const
{
  // Compute text width
  Glib::RefPtr<Pango::Layout> layout_ptr = widget.create_pango_layout(__property_line1);
  Pango::Rectangle rect = layout_ptr->get_pixel_logical_extents();
	
  int line1_width  = property_xpad() * 2 + rect.get_width();
  int line1_height = property_ypad() * 2 + rect.get_height();

  Glib::RefPtr<Pango::Layout> layout2 = widget.create_pango_layout(__property_line2);
  Glib::RefPtr<Gtk::Style> style = widget.get_style();
  Pango::FontDescription font2 = style->get_font();
  font2.set_size(Pango::SCALE_SMALL * font2.get_size());
  layout2->set_font_description(font2);
  Pango::Rectangle rect2 = layout2->get_pixel_logical_extents();
  layout2->set_ellipsize(Pango::ELLIPSIZE_END);

  int line2_height = property_ypad() * 2 + rect2.get_height();

  if ( width )  *width  = line1_width;
  if ( height ) *height = line1_height + 4 + line2_height;    
}


/** Render the cell.
 * This is called to render the cell.
 * @param window window
 * @param widget widget
 * @param background_area dimensions of the background area
 * @param cell_area dimensions of the cell area
 * @param expose_area dimensions of the exposed area
 * @param flags render flags
 */
void
TwoLinesCellRenderer::render_vfunc(const Glib::RefPtr<Gdk::Drawable> &window,
				   Gtk::Widget &widget,
				   const Gdk::Rectangle &background_area,
				   const Gdk::Rectangle &cell_area,
				   const Gdk::Rectangle &expose_area,
				   Gtk::CellRendererState flags)
{
  // Get cell size
  int x_offset = 0, y_offset = 0, width = 0, height = 0;
  get_size(widget, cell_area, x_offset, y_offset, width, height);
    
  // Create the graphic context
  Glib::RefPtr<Gdk::GC> gc = Gdk::GC::create(window);
    
  // Get cell state
  Gtk::StateType state;
  Gtk::StateType text_state;
  if ((flags & Gtk::CELL_RENDERER_SELECTED) != 0) {
    state = Gtk::STATE_SELECTED;
    text_state = (widget.has_focus()) ? Gtk::STATE_SELECTED : Gtk::STATE_ACTIVE;
  } else {
    state = Gtk::STATE_NORMAL;
    text_state = (widget.is_sensitive()) ? Gtk::STATE_NORMAL : Gtk::STATE_INSENSITIVE;
  }
	
  // Draw color text
  Glib::RefPtr<Gdk::Window> win = Glib::RefPtr<Gdk::Window>::cast_dynamic(window);    
  Glib::RefPtr<Pango::Layout> layout_ptr = widget.create_pango_layout(__property_line1);
  Pango::Rectangle rect1 = layout_ptr->get_pixel_logical_extents();
  widget.get_style()->paint_layout (win, text_state, true, cell_area,
				    widget, "cellrenderertext",
				    cell_area.get_x() + x_offset + 2 * property_xpad(),
				    cell_area.get_y() + y_offset + 2 * property_ypad(),
				    layout_ptr);

  Glib::RefPtr<Pango::Layout> layout2 = widget.create_pango_layout(__property_line2);
  Glib::RefPtr<Gtk::Style> style = widget.get_style();
  Pango::FontDescription font2 = style->get_font();
  font2.set_size(Pango::SCALE_SMALL * std::max(font2.get_size(), 8));
  layout2->set_font_description(font2);
  Pango::Rectangle rect2 = layout2->get_pixel_logical_extents();
  layout2->set_ellipsize(Pango::ELLIPSIZE_END);
  layout2->set_width((cell_area.get_width() - property_xpad()) * Pango::SCALE);
  widget.get_style()->paint_layout (win, text_state, true, cell_area,
				    widget, "cellrenderertext",
				    cell_area.get_x() + x_offset + property_xpad(),
				    cell_area.get_y() + y_offset + property_ypad() + rect1.get_height() + 4,
				    layout2);
}


} // end namespace fawkes
