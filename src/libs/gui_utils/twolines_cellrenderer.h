
/***************************************************************************
 *  twolines_cellrenderer.h - Gtk rell renderer for two lines of text
 *
 *  Created: Sat Nov 29 16:34:03 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_GUI_UTILS_TWOLINES_CELLRENDERER_H_
#define __LIBS_GUI_UTILS_TWOLINES_CELLRENDERER_H_

#include <gtkmm/cellrenderer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class TwoLinesCellRenderer : public Gtk::CellRenderer
{
 public:
  TwoLinesCellRenderer();
  ~TwoLinesCellRenderer();

  // Properties
  Glib::PropertyProxy<Glib::ustring> property_line1();
  Glib::PropertyProxy<Glib::ustring> property_line2();

 protected:
  virtual void get_size_vfunc (Gtk::Widget& widget, const Gdk::Rectangle* cell_area, int* x_offset, int* y_offset, int* width, int* height) const;
  virtual void render_vfunc (const Glib::RefPtr<Gdk::Drawable>& window, Gtk::Widget& widget, const Gdk::Rectangle& background_area, const Gdk::Rectangle& cell_area, const Gdk::Rectangle& expose_area, Gtk::CellRendererState flags);

 private:
  Glib::Property<Glib::ustring> __property_line1;
  Glib::Property<Glib::ustring> __property_line2;

};

} // end namespace fawkes

#endif
