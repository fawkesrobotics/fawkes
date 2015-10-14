
/***************************************************************************
 *  skillgui.h - Skill GUI
 *
 *  Created: Mon Nov 03 13:35:34 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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
#ifdef HAVE_GCONFMM
#  include <gconfmm.h>
#  define GCONF_PREFIX "/apps/fawkes/skillgui"
#endif

#include <interfaces/SkillerInterface.h>
#include <interfaces/SkillerDebugInterface.h>

#ifndef GTKMM_VERSION_GE
#  define GTKMM_VERSION_GE(major,minor) ((GTKMM_MAJOR_VERSION>major)||(GTKMM_MAJOR_VERSION==major)&&(GTKMM_MINOR_VERSION>=minor))
#endif

namespace fawkes {
  class BlackBoard;
  class InterfaceDispatcher;
  class LogView;
  class PluginTreeView;
}

#ifdef USE_PAPYRUS
class SkillGuiGraphViewport;
#else
class SkillGuiGraphDrawingArea;
#endif

class SkillGuiGtkWindow : public Gtk::Window
{
 public:  
  SkillGuiGtkWindow(BaseObjectType* cobject,
                    const Glib::RefPtr<Gtk::Builder> &builder);
  ~SkillGuiGtkWindow();

 private:
  void close_bb();
  void send_graphdir_message(fawkes::SkillerDebugInterface *iface,
			     fawkes::SkillerDebugInterface::GraphDirectionEnum gd);

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
  void on_skill_changed();
  void on_graphupd_clicked();
  void on_update_disabled();
  void on_recording_toggled();
  void on_graphdir_clicked();
  void on_graphdir_changed(fawkes::SkillerDebugInterface::GraphDirectionEnum gd);
  void on_graphcolor_toggled();

 private:
  class SkillStringRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    SkillStringRecord();
    /// @cond INTERNALS
    Gtk::TreeModelColumn<Glib::ustring> skillstring;
    /// @endcond
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
  Gtk::Button            *but_clearlog;
#if GTK_VERSION_GE(3,0)
  Gtk::ComboBox          *cbe_skillstring;
#else
  Gtk::ComboBoxEntry     *cbe_skillstring;
#endif
  Gtk::Label             *lab_status;
  Gtk::Label             *lab_alive;
  Gtk::Label             *lab_skillstring;
  Gtk::Label             *lab_error;
  Gtk::ScrolledWindow    *scw_graph;
  Gtk::Notebook          *ntb_tabs;
  Gtk::ToggleToolButton  *tb_skiller;
  Gtk::ToggleToolButton  *tb_agent;
  Gtk::ComboBoxText      *cb_graphlist;
  Gtk::ToolItem          *tb_graphlist;
  Gtk::ToolButton        *tb_graphsave;
  Gtk::ToolButton        *tb_graphopen;
  Gtk::ToolButton        *tb_graphupd;
  Gtk::ToggleToolButton  *tb_graphrecord;
  Gtk::ToolButton        *tb_controller;
  Gtk::ToolButton        *tb_zoomin;
  Gtk::ToolButton        *tb_zoomout;
  Gtk::ToolButton        *tb_zoomfit;
  Gtk::ToolButton        *tb_zoomreset;
#if GTKMM_VERSION_GE(2,20)
  Gtk::Spinner           *tb_spinner;
#endif

  Gtk::ToolButton        *tb_graphdir;
  Gtk::ToggleToolButton  *tb_graphcolored;

  Glib::RefPtr<Gtk::ListStore> __sks_list;

#ifdef HAVE_GCONFMM
  Glib::RefPtr<Gnome::Conf::Client> __gconf;
#endif

#ifdef USE_PAPYRUS
  SkillGuiGraphViewport  *pvp_graph;
#else
  SkillGuiGraphDrawingArea *gda;
#endif

  fawkes::SkillerInterface *__skiller_if;
  fawkes::SkillerDebugInterface *__skdbg_if;
  fawkes::SkillerDebugInterface *__agdbg_if;

  fawkes::LogView         *__logview;
  fawkes::PluginTreeView  *__trv_plugins;
};

#endif
