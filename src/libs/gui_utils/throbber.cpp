
/***************************************************************************
 *  throbber.cpp - Fawkes throbber
 *
 *  Created: Tue Nov 04 16:38:03 2008
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

#include <gui_utils/throbber.h>

#include <core/exception.h>
#include <algorithm>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

#define SPINNER_ICON_NAME		"process-working"
#define SPINNER_FALLBACK_ICON_NAME	"gnome-spinner"
#define SPINNER_DEFAULT_TIMEOUT         100


/** @class Throbber <gui_utils/throbber.h>
 * Simple Gtk Throbber/Spinner.
 * The throbber shows a spinning icon as a small image. It has been specifically
 * prepared to be used as a custom image Gtk::ToolItem in a Gtk::Toolbar.
 * The icon is defined by the currently active Gtk theme.
 * @author Tim Niemueller
 */

/** Constructor.
 * Special ctor to be used with Gtk::Builder's get_widget_derived().
 * @param cobject Gtk C object
 * @param builder Gtk builder
 */
Throbber::Throbber(BaseObjectType* cobject,
		   const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::Image(cobject)
{
  Gtk::Container *parent = get_parent();
  Gtk::ToolItem  *toolitem = dynamic_cast<Gtk::ToolItem *>(parent);
  if ( toolitem ) {
    ctor(toolitem->get_icon_size());
  } else {
    // We have no clue, just try button
    ctor(Gtk::IconSize(Gtk::ICON_SIZE_BUTTON));
  }
}


/** Constructor.
 * @param icon_size desired icon size. Be aware that the icon may not be available
 * in all sizes in the current theme.
 */
Throbber::Throbber(Gtk::IconSize &icon_size)
{
  ctor(icon_size);
}


void
Throbber::ctor(Gtk::IconSize icon_size)
{
  __timeout   = SPINNER_DEFAULT_TIMEOUT;
  __icon_size = icon_size;

  int isw = 0, ish = 0;
#if GTKMM_MAJOR_VERSION > 2 || ( GTKMM_MAJOR_VERSION == 2 && GTKMM_MINOR_VERSION >= 14 )
  Glib::RefPtr<Gtk::Settings> settings = Gtk::Settings::get_for_screen(get_screen());
  if ( ! Gtk::IconSize::lookup(icon_size, isw, ish, settings) ) {
    throw Exception("Could not get icon sizes");
  }
#else
  if ( ! Gtk::IconSize::lookup(icon_size, isw, ish) ) {
    throw Exception("Could not get icon sizes");
  }
#endif
  int requested_size = std::max(isw, ish);

  Glib::RefPtr<Gtk::IconTheme> icon_theme = Gtk::IconTheme::get_for_screen(get_screen());
  Gtk::IconInfo icon_info = icon_theme->lookup_icon(SPINNER_ICON_NAME,
						    requested_size,
						    Gtk::IconLookupFlags());
  if ( ! icon_info ) {
    icon_info = icon_theme->lookup_icon(SPINNER_FALLBACK_ICON_NAME,
					requested_size, Gtk::IconLookupFlags());
    if ( ! icon_info ) {
      throw Exception("Could not find neither default nor fallback throbber icon");
    }
  }

  int size = icon_info.get_base_size();

#ifdef GLIBMM_EXCEPTIONS_ENABLED
  Glib::RefPtr<Gdk::Pixbuf> icon = icon_info.load_icon();
#else
  std::auto_ptr<Glib::Error> error;
  Glib::RefPtr<Gdk::Pixbuf> icon = icon_info.load_icon(error);
#endif

  int pixwidth  = icon->get_width();
  int pixheight = icon->get_height();

  for (int y = 0; y < pixheight; y += size) {
    for (int x = 0; x < pixwidth ; x += size) {
      if ( (x + size <= icon->get_width()) &&
	   (y + size <= icon->get_height()) ) {
	Glib::RefPtr<Gdk::Pixbuf> p = Gdk::Pixbuf::create_subpixbuf(icon, x, y, size, size);
	__pixbufs.push_back(p);
      }
    }
  }

  if ( __pixbufs.empty() ) {
    throw Exception("Could not extract any throbber images from pixbuf");
  }

  __current = 0;
  set(__pixbufs.front());
}


/** Draw next image.
 * @return always true
 */
bool
Throbber::draw_next()
{
  __current = (__current + 1) % __pixbufs.size();
  if ( (__current == 0) && (__pixbufs.size() > 1) ) {
    __current = 1;
  }
  set(__pixbufs[__current]);  

  return true;
}


/** Set the animation timeout.
 * The animation timeout is the time between two frames. It defaults to 100ms.
 * @param timeout new timeout for animation in ms
 */
void
Throbber::set_timeout(unsigned int timeout)
{
  __timeout = timeout;
}


/** Check if animation is running.
 * @return true if animation is currently running, false otherwise.
 */
bool
Throbber::anim_running()
{
  return (__timeout_connection && __timeout_connection.connected());
}

/** Start animation. */
void
Throbber::start_anim()
{
  if ( ! __timeout_connection || ! __timeout_connection.connected()) {
    __timeout_connection = Glib::signal_timeout().connect(
	                     sigc::mem_fun(*this, &Throbber::draw_next), __timeout);
  }
}

/** Stop animation. */
void
Throbber::stop_anim()
{
  if (__timeout_connection && __timeout_connection.connected()) {
    __timeout_connection.disconnect();
  }

  __current = 0;
  set(__pixbufs.front());
}


/** Set image from stock ID.
 * The image will be overwritten by a running animation or when the
 * animation is started again. It will not be automatically reset to this
 * stock ID if the animation stops, rather you have to do this by yourself.
 * @param stock_id stock ID of image to set
 */
void
Throbber::set_stock(const Gtk::StockID& stock_id)
{
  set(stock_id, __icon_size);
}


} // end namespace fawkes
