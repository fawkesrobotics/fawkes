
/***************************************************************************
 *  skillgui.h - Skill GUI
 *
 *  Created: Mon Nov 03 13:35:34 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __TOOLS_SKILLGUI_SKILLGUI_H_
#define __TOOLS_SKILLGUI_SKILLGUI_H_

#include <gui_utils/connection_dispatcher.h>

#include <gtkmm.h>
#include <gconfmm.h>
#include <libglademm/xml.h>

#define GCONF_PREFIX "/apps/fawkes/skillgui"

namespace fawkes {
  class BlackBoard;
  class SkillerInterface;
  class SkillerDebugInterface;
  class InterfaceDispatcher;
  class LogView;
  class Throbber;
}

#ifdef USE_PAPYRUS
class SkillGuiGraphViewport;
#else
class SkillGuiGraphDrawingArea;
#endif

class SkillGuiGtkWindow : public Gtk::Window
{
 public:  
  SkillGuiGtkWindow(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml> &refxml);
  ~SkillGuiGtkWindow();

 private:
  void close_bb();

  void on_connection_clicked();
  void on_connect();
  void on_disconnect();
  void on_exec_clicked();
  void on_skiller_data_changed();
  void on_skdbg_data_changed();
  void on_agdbg_data_changed();
  void on_exit_clicked();
  void on_controller_clicked();
  void on_stop_clicked();
  void on_config_changed();
  void on_contexec_toggled();
  void on_skill_changed();
  void on_graphupd_clicked();

 private:
  class SkillStringRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    SkillStringRecord();
    Gtk::TreeModelColumn<Glib::ustring> skillstring;
  };
  SkillStringRecord __sks_record;


  fawkes::BlackBoard *bb;

  fawkes::ConnectionDispatcher connection_dispatcher;
  fawkes::InterfaceDispatcher  *__skiller_ifd;
  fawkes::InterfaceDispatcher  *__skdbg_ifd;
  fawkes::InterfaceDispatcher  *__agdbg_ifd;

  Gtk::ToolButton        *tb_connection;
  Gtk::ToolButton        *tb_exit;
  Gtk::Button            *but_exec;
  Gtk::Button            *but_stop;
  Gtk::ComboBoxEntry     *cbe_skillstring;
  Gtk::ToggleToolButton  *tb_continuous;
  Gtk::Label             *lab_status;
  Gtk::Label             *lab_alive;
  Gtk::Label             *lab_continuous;
  Gtk::Label             *lab_skillstring;
  Gtk::Label             *lab_error;
  Gtk::ScrolledWindow    *scw_graph;
  Gtk::Notebook          *ntb_tabs;
  Gtk::DrawingArea       *drw_graph;
  Gtk::ToggleToolButton  *tb_skiller;
  Gtk::ToggleToolButton  *tb_agent;
  Gtk::ComboBoxText      *cb_graphlist;
  Gtk::ToolItem          *tb_graphlist;
  Gtk::ToolButton        *tb_graphsave;
  Gtk::ToolButton        *tb_graphupd;
  Gtk::ToolButton        *tb_controller;
  Gtk::ToolButton        *tb_zoomin;
  Gtk::ToolButton        *tb_zoomout;
  Gtk::ToolButton        *tb_zoomfit;
  Gtk::ToolButton        *tb_zoomreset;

  Glib::RefPtr<Gtk::ListStore> __sks_list;

  Glib::RefPtr<Gnome::Conf::Client> __gconf;

#ifdef USE_PAPYRUS
  SkillGuiGraphViewport  *pvp_graph;
#else
  SkillGuiGraphDrawingArea *gda;
#endif

  fawkes::SkillerInterface *__skiller_if;
  fawkes::SkillerDebugInterface *__skdbg_if;
  fawkes::SkillerDebugInterface *__agdbg_if;

  fawkes::LogView  *__logview;
  fawkes::Throbber *__throbber;

  bool __update_graph;
};

#endif
