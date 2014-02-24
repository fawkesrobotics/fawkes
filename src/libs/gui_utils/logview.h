
/***************************************************************************
 *  logview.h - Fawkes log view widget
 *
 *  Created: Mon Nov 02 13:08:29 2008
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

#ifndef __LIBS_GUI_UTILS_LOGVIEW_H_
#define __LIBS_GUI_UTILS_LOGVIEW_H_

#include <gtkmm.h>

#include <logging/logger.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FawkesNetworkClient;
class FawkesNetworkMessage;
class ConnectionDispatcher;

class LogView
  : public Gtk::TreeView
{
 public:
  LogView();
  LogView(const char *hostname, unsigned short int port);
  LogView(BaseObjectType* cobject,
	  const Glib::RefPtr<Gtk::Builder> &builder);
  ~LogView();

  void set_client(FawkesNetworkClient *client);
  FawkesNetworkClient *  get_client();

  void append_message(Logger::LogLevel log_level, struct timeval t,
		      const char *component, bool is_exception,
		      const char *message);

  void clear();

  ConnectionDispatcher *  get_connection_dispatcher() const;

 private:
  virtual void on_row_inserted(const Gtk::TreeModel::Path& path,
  			       const Gtk::TreeModel::iterator& iter);
  virtual void on_message_received(FawkesNetworkMessage *msg);
  virtual void on_client_connected();
  virtual void on_client_disconnected();
#if GTK_VERSION_GE(3,0)
  virtual bool on_draw(const Cairo::RefPtr<Cairo::Context> &cr);
#else
  virtual void on_expose_notify(GdkEventExpose *event);
#endif

  void ctor(const char *hostname = NULL, unsigned short int port = 0);

 private:
  class LogRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    LogRecord();

    /// @cond INTERNALS
    Gtk::TreeModelColumn<Glib::ustring> loglevel;
    Gtk::TreeModelColumn<Glib::ustring> time;
    Gtk::TreeModelColumn<Glib::ustring> component;
    Gtk::TreeModelColumn<Glib::ustring> message;
    Gtk::TreeModelColumn<Gdk::Color>    foreground;
    Gtk::TreeModelColumn<Gdk::Color>    background;
    /// @endcond
  };

  LogRecord __record;

  Glib::RefPtr<Gtk::ListStore> __list;

  ConnectionDispatcher *__connection_dispatcher;

  bool                  __have_recently_added_path;
  Gtk::TreeModel::Path  __recently_added_path;
};

} // end namespace fawkes


#endif
