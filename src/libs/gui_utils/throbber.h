
/***************************************************************************
 *  throbber.h - Fawkes throbber
 *
 *  Created: Tue Nov 04 16:36:38 2008
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

#ifndef __LIBS_GUI_UTILS_THROBBER_H_
#define __LIBS_GUI_UTILS_THROBBER_H_

#include <gtkmm.h>
#ifdef HAVE_GLADEMM
#  include <libglademm/xml.h>
#endif

namespace fawkes {

class Throbber : public Gtk::Image
{
 public:
  Throbber(Gtk::IconSize &icon_size);
#ifdef HAVE_GLADEMM
  Throbber(BaseObjectType* cobject,
	   const Glib::RefPtr<Gnome::Glade::Xml>& ref_glade);
#endif

  void set_timeout(unsigned int timeout);

  bool anim_running();

  void start_anim();
  void stop_anim();

  void set_stock(const Gtk::StockID& stock_id);

 private:
  void ctor(Gtk::IconSize icon_size);
  bool draw_next();

 private:
  unsigned int __current;
  std::vector<Glib::RefPtr<Gdk::Pixbuf> > __pixbufs;
  Gtk::IconSize __icon_size;

  sigc::connection __timeout_connection;
  unsigned int __timeout;
};

} // end namespace fawkes

#endif
