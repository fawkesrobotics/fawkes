
/***************************************************************************
 *  logview.cpp - Fawkes log view widget
 *
 *  Created: Mon Nov 02 13:19:03 2008
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

#include <gui_utils/logview.h>
#include <gui_utils/connection_dispatcher.h>
#include <netcomm/fawkes/client.h>
#include <network_logger/network_logger.h>

#include <gtkmm.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class LogView <gui_utils/logview.h>
 * Log View widget.
 * This widget derives a Gtk::TreeView and provides an easy way to show
 * log messages in a GUI application.
 * @author Tim Niemueller
 */


/** Constructor. */
LogView::LogView()
{
  ctor();
}


/** Constructor.
 * @param hostname hostname to set for the FawkesNetworkClient.
 * @param port port to set for the FawkesNetworkClient.
 */
LogView::LogView(const char *hostname, unsigned short int port)
{
  ctor(hostname, port);
}


/** Constructor.
 * Special ctor to be used with Gtk::Builder's get_widget_derived().
 * @param cobject Gtk C object
 * @param builder Gtk builder
 */
LogView::LogView(BaseObjectType* cobject,
		 const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::TreeView(cobject)
{
  ctor();
}


/** Destructor. */
LogView::~LogView()
{
  FawkesNetworkClient *c = __connection_dispatcher->get_client();
  if (c && c->connected()) {
    FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NETWORKLOGGER,
							 NetworkLogger::MSGTYPE_UNSUBSCRIBE);
    c->enqueue(msg);
  }
  delete __connection_dispatcher;
}


/** Internal constructor method. */
void
LogView::ctor(const char *hostname, unsigned short int port)
{
  __list = Gtk::ListStore::create(__record);
  set_model(__list);
  __have_recently_added_path = false;

  __list->signal_row_inserted().connect(sigc::mem_fun(*this, &LogView::on_row_inserted));
  get_selection()->set_mode(Gtk::SELECTION_NONE);

  if ( (hostname != NULL) && (port != 0) ) {
    __connection_dispatcher =
      new ConnectionDispatcher(hostname, port, FAWKES_CID_NETWORKLOGGER);
  } else {
    __connection_dispatcher = new ConnectionDispatcher(FAWKES_CID_NETWORKLOGGER);
  }

  append_column("Level",     __record.loglevel);
  append_column("Time",      __record.time);
  int compcol = append_column("Component", __record.component);
  int msgcol  = append_column("Message",   __record.message);

  // We stored the number of columns, for an index (which starts at 0) we need
  // to subtract 1
  compcol -= 1;
  msgcol  -= 1;

  Glib::ListHandle<Gtk::TreeViewColumn *> columns = get_columns();
  int colnum = -1;
  for (Glib::ListHandle<Gtk::TreeViewColumn *>::iterator c = columns.begin();
       c != columns.end();
       ++c)
  {
    ++colnum;
#if GTK_VERSION_GE(3,0)
    Gtk::CellRenderer *cell_renderer = (*c)->get_first_cell();
#else
    Gtk::CellRenderer *cell_renderer = (*c)->get_first_cell_renderer();
#endif
    Gtk::CellRendererText *text_renderer =
      dynamic_cast<Gtk::CellRendererText *>(cell_renderer);
    if ( text_renderer ) {
#ifdef GLIBMM_PROPERTIES_ENABLED
      if ( (colnum == compcol) || (colnum == msgcol) ) {
	(*c)->set_resizable();
      }
      if ( colnum == compcol ) {
	text_renderer->property_ellipsize().set_value(Pango::ELLIPSIZE_END);
      }

      (*c)->add_attribute(text_renderer->property_background_gdk(), __record.background);
      (*c)->add_attribute(text_renderer->property_foreground_gdk(), __record.foreground);
#else
      (*c)->add_attribute(*text_renderer, "background-gdk", __record.background);
      (*c)->add_attribute(*text_renderer, "foreground-gdk", __record.background);
#endif
    }
  }

  __connection_dispatcher->signal_message_received().connect(sigc::mem_fun(*this, &LogView::on_message_received));
  __connection_dispatcher->signal_connected().connect(sigc::mem_fun(*this, &LogView::on_client_connected));
  __connection_dispatcher->signal_disconnected().connect(sigc::mem_fun(*this, &LogView::on_client_disconnected));
#if GTK_VERSION_LT(3,0)
  signal_expose_event().connect_notify(sigc::mem_fun(*this, &LogView::on_expose_notify));
#endif
}


/** Set FawkesNetworkClient instance.
 * @param client Fawkes network client to use
 */
void
LogView::set_client(FawkesNetworkClient *client)
{
  FawkesNetworkClient *c = __connection_dispatcher->get_client();
  if (c && c->connected()) {
    FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NETWORKLOGGER,
							 NetworkLogger::MSGTYPE_UNSUBSCRIBE);
    c->enqueue(msg);
  }
  __connection_dispatcher->set_client(client);
  if (client && client->connected()) {
    FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NETWORKLOGGER,
							 NetworkLogger::MSGTYPE_SUBSCRIBE);
    client->enqueue(msg);
  }
}


/** Get the used FawkesNetworkClient.
 * @return Fawkes network client instance
 */
FawkesNetworkClient *
LogView::get_client()
{
  return __connection_dispatcher->get_client();
}


/** Get ConnectionDispatcher instance that is used internally.
 * @return connection dispatcher
 */
ConnectionDispatcher *
LogView::get_connection_dispatcher() const
{
  return __connection_dispatcher;
}


/** Clear all records. */
void
LogView::clear()
{
  __list->clear();
}


/** Event handler when row inserted.
 * @param path path to element
 * @param iter iterator to inserted element
 */
void
LogView::on_row_inserted(const Gtk::TreeModel::Path& path,
			 const Gtk::TreeModel::iterator& iter)
{
  Gtk::TreeModel::Path vstart, vend;
  Gtk::TreeModel::Path prev = path;
  get_visible_range(vstart, vend);
  prev = path;
  if (! get_visible_range(vstart, vend) ||
      ( prev.prev() &&
	((vend == prev) ||
	 (__have_recently_added_path && (__recently_added_path == prev)))) ) {
    scroll_to_row(path);

    // the recently added stuff is required if multiple rows are inserted at
    // a time. In this case the widget wasn't redrawn and get_visible_range() does
    // not give the desired result and we have to "advance" it manually
    __have_recently_added_path = true;
    __recently_added_path = path;
  }
}

#if GTK_VERSION_GE(3,0)
bool
LogView::on_draw(const Cairo::RefPtr<Cairo::Context> &cr)
{
  __have_recently_added_path = false;
  return Gtk::TreeView::on_draw(cr);
}
#else
void
LogView::on_expose_notify(GdkEventExpose *event)
{
  __have_recently_added_path = false;
} 
#endif


void
LogView::on_client_connected()
{
  FawkesNetworkClient *c = __connection_dispatcher->get_client();
  if (c && c->connected()) {
    FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NETWORKLOGGER,
							 NetworkLogger::MSGTYPE_SUBSCRIBE);
    c->enqueue(msg);
    struct timeval t;
    gettimeofday(&t, NULL);
    append_message(Logger::LL_DEBUG, t, "LogView", false, "Connected");
  }
}

void
LogView::on_client_disconnected()
{
  struct timeval t;
  gettimeofday(&t, NULL);
  append_message(Logger::LL_ERROR, t, "LogView", false, "*** Connection died. ***");
}


/** Append a single message.
 * @param log_level log level
 * @param t time of the message
 * @param component component string for the message
 * @param is_exception true if essage was produced via an exception
 * @param message log message
 */
void
LogView::append_message(Logger::LogLevel log_level, struct timeval t,
			const char *component, bool is_exception,
			const char *message)
{
  const char *loglevel;
  const char *timestr;
  char* time = NULL;
  Gdk::Color color;
  bool set_foreground = false;
  bool set_background = false;

  switch ( log_level ) {
  case Logger::LL_DEBUG:
    loglevel = "DEBUG";
    color.set_rgb_p(0.4, 0.4, 0.4);
    set_foreground = true;
    break;
  case Logger::LL_INFO:
    loglevel = "INFO";
    break;
  case Logger::LL_WARN:
    loglevel = "WARN";
    color.set_rgb_p(1.0, 1.0, 0.7);
    set_background = true;
    break;
  case Logger::LL_ERROR:
    loglevel = "ERROR";
    color.set_rgb_p(1.0, 0.8, 0.8);
    set_background = true;
    break;
  default:
    loglevel = "NONE?";
    color.set_rgb_p(1.0, 0.0, 0.0);
    set_background = true;
    break;
  }

  struct tm time_tm;
  localtime_r(&(t.tv_sec), &time_tm);
  if (asprintf(&time, "%02d:%02d:%02d.%06ld", time_tm.tm_hour,
	       time_tm.tm_min, time_tm.tm_sec, t.tv_usec) == -1) {
    timestr = "OutOfMemory";
  } else {
    timestr = time;
  }

  Gtk::TreeModel::Row row  = *__list->append();
  row[__record.loglevel]   = loglevel;
  row[__record.time]       = timestr;
  row[__record.component]  = component;
  if ( is_exception ) {
    row[__record.message]    = std::string("[EXCEPTION] ") + message;
  } else {
    row[__record.message]    = message;
  }
  if ( set_foreground )  row[__record.foreground] = color;
  if ( set_background )  row[__record.background] = color;

  if (time) free(time);
}

/** Message received event handler.
 * @param msg Fawkes network message just recveived.
 */
void
LogView::on_message_received(FawkesNetworkMessage *msg)
{
  if ( (msg->cid() == FAWKES_CID_NETWORKLOGGER) &&
       (msg->msgid() == NetworkLogger::MSGTYPE_LOGMESSAGE) ) {

    NetworkLoggerMessageContent* content = msg->msgc<NetworkLoggerMessageContent>();

    append_message(content->get_loglevel(), content->get_time(),
		   content->get_component(),
		   content->is_exception(), content->get_message());

    delete content;
  }
}

/** @class LogView::LogRecord <gui_utils/logview.h>
 * TreeView record for LogView.
 */

/** Constructor. */
LogView::LogRecord::LogRecord()
{
  add(loglevel);
  add(time);
  add(component);
  add(message);
  add(foreground);
  add(background);
}



} // end namespace fawkes
