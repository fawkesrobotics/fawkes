
/***************************************************************************
 *  skillgui.cpp - Skill GUI
 *
 *  Created: Mon Nov 03 13:37:33 2008
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

#include "skillgui.h"

#include <utils/system/argparser.h>
#include <blackboard/remote.h>
#include <interfaces/SkillerInterface.h>
#include <netcomm/fawkes/client.h>

#include <gui_utils/logview.h>
#include <gui_utils/throbber.h>
#include <gui_utils/service_chooser_dialog.h>
#include <gui_utils/interface_dispatcher.h>

#include <cstring>

using namespace fawkes;


/** @class SkillGuiGtkWindow "skillgui.h"
 * Skill GUI main window.
 * The Skill GUI provides shows Skiller log messages and allows for
 * executing skills.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cobject C base object
 * @param refxml Glade XML
 */
SkillGuiGtkWindow::SkillGuiGtkWindow(BaseObjectType* cobject,
				 const Glib::RefPtr<Gnome::Glade::Xml> &refxml)
  : Gtk::Window(cobject)
{
  bb = NULL;
  __skiller_if = NULL;

  refxml->get_widget_derived("trv_log", __logview);
  refxml->get_widget("tb_connection", tb_connection);
  refxml->get_widget("tb_continuous", tb_continuous);
  refxml->get_widget("tb_stop", tb_stop);
  refxml->get_widget("tb_exit", tb_exit);
  refxml->get_widget("cbe_skillstring", cbe_skillstring);
  refxml->get_widget("but_exec", but_exec);
  refxml->get_widget("lab_status", lab_status);
  refxml->get_widget("lab_alive", lab_alive);
  refxml->get_widget("lab_continuous", lab_continuous);
  refxml->get_widget("lab_skillstring", lab_skillstring);

  refxml->get_widget_derived("img_throbber", __throbber);
  
  Gtk::SeparatorToolItem *spacesep;
  refxml->get_widget("tb_spacesep", spacesep);
  spacesep->set_expand();

  __sks_list = Gtk::ListStore::create(__sks_record);
  cbe_skillstring->set_model(__sks_list);
  cbe_skillstring->set_text_column(__sks_record.skillstring);

  connection_dispatcher.signal_connected().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_connect));
  connection_dispatcher.signal_disconnected().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_disconnect));

  tb_connection->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_connection_clicked));
  but_exec->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_exec_clicked));
  tb_exit->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_exit_clicked));
  tb_stop->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_stop_clicked));
}


/** Destructor. */
SkillGuiGtkWindow::~SkillGuiGtkWindow()
{
  __logview->set_client(NULL);
}


/** Event handler for connection button. */
void
SkillGuiGtkWindow::on_connection_clicked()
{
  if ( ! connection_dispatcher.get_client()->connected() ) {
    ServiceChooserDialog ssd(*this, connection_dispatcher.get_client());
    ssd.run_and_connect();
  } else {
    connection_dispatcher.get_client()->disconnect();
  } 
}


void
SkillGuiGtkWindow::on_exit_clicked()
{
  Gtk::Main::quit();
}


void
SkillGuiGtkWindow::on_stop_clicked()
{
  if ( bb && __skiller_if && __skiller_if->is_valid() && __skiller_if->has_writer() ) {
    SkillerInterface::StopExecMessage *sem = new SkillerInterface::StopExecMessage();
    __skiller_if->msgq_enqueue(sem);
  }
}

void
SkillGuiGtkWindow::close_bb()
{
  if ( bb ) {
    bb->unregister_listener(__ifd);
    delete __ifd;
    if ( __skiller_if->is_valid() && __skiller_if->has_writer() ) {
      SkillerInterface::ReleaseControlMessage *rcm = new SkillerInterface::ReleaseControlMessage();
      __skiller_if->msgq_enqueue(rcm);
    }
    bb->close(__skiller_if);
    delete bb;
    __skiller_if = NULL;
    bb = NULL;
  }
}

/** Event handler for connected event. */
void
SkillGuiGtkWindow::on_connect()
{
  try {
    if ( ! bb ) {
      bb           = new RemoteBlackBoard(connection_dispatcher.get_client());
      __skiller_if = bb->open_for_reading<SkillerInterface>("Skiller");
      on_skiller_data_changed();

      SkillerInterface::AcquireControlMessage *aqm = new SkillerInterface::AcquireControlMessage();
      __skiller_if->msgq_enqueue(aqm);
      __ifd = new InterfaceDispatcher("Skiller IFD", __skiller_if);
      bb->register_listener(__ifd, BlackBoard::BBIL_FLAG_DATA);
      __ifd->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_skiller_data_changed)));
    }
    tb_connection->set_stock_id(Gtk::Stock::DISCONNECT);
    __logview->set_client(connection_dispatcher.get_client());

  } catch (Exception &e) {
    Glib::ustring message = *(e.begin());
    Gtk::MessageDialog md(*this, message, /* markup */ false,
			  Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			  /* modal */ true);
    md.set_title("BlackBoard connection failed");
    md.run();

    close_bb();
    connection_dispatcher.get_client()->disconnect();
  }
}

/** Event handler for disconnected event. */
void
SkillGuiGtkWindow::on_disconnect()
{
  close_bb();

  tb_connection->set_stock_id(Gtk::Stock::CONNECT);
  __logview->set_client(NULL);
}


void
SkillGuiGtkWindow::on_exec_clicked()
{
  Glib::ustring sks = "";
  if ( cbe_skillstring->get_active_row_number() == -1 ) {
    Gtk::Entry *entry = cbe_skillstring->get_entry();
    sks = entry->get_text();
  } else {
    Gtk::TreeModel::Row row = *cbe_skillstring->get_active();
    row.get_value(cbe_skillstring->get_text_column(), sks);
  }

  __throbber->set_timeout(80);

  if ( sks != "" ) {

    if ( tb_continuous->get_active() ) {
      SkillerInterface::ExecSkillContinuousMessage *escm = new SkillerInterface::ExecSkillContinuousMessage(sks.c_str());
      __skiller_if->msgq_enqueue(escm);
    } else {
      SkillerInterface::ExecSkillMessage *esm = new SkillerInterface::ExecSkillMessage(sks.c_str());
      __skiller_if->msgq_enqueue(esm);
    }
    __throbber->start_anim();

    Gtk::TreeModel::Row row  = *__sks_list->prepend();
    row[__sks_record.skillstring] = sks;

    
    Gtk::TreeModel::iterator i = __sks_list->get_iter("0");
    unsigned int count = 0;
    while (bool(i)) {
      if ( ++count > 10 ) {
	i = __sks_list->erase(i);
	__throbber->stop_anim();
      } else {
	++i;
	__throbber->start_anim();
      }
    }
  }

}


void
SkillGuiGtkWindow::on_skiller_data_changed()
{
  __skiller_if->read();

  switch (__skiller_if->status()) {
  case SkillerInterface::S_INACTIVE:
    __throbber->stop_anim();
    lab_status->set_text("S_INACTIVE");
    break;
  case SkillerInterface::S_FINAL:
    __throbber->stop_anim();
    lab_status->set_text("S_FINAL");
    break;
  case SkillerInterface::S_RUNNING:
    lab_status->set_text("S_RUNNING");
    break;
  case SkillerInterface::S_FAILED:
    __throbber->stop_anim();
    lab_status->set_text("S_FAILED");
    break;
  }

  lab_skillstring->set_text(__skiller_if->skill_string());
  lab_continuous->set_text(__skiller_if->is_continuous() ? "Yes" : "No");
  lab_alive->set_text(__skiller_if->has_writer() ? "Yes" : "No");
}


SkillGuiGtkWindow::SkillStringRecord::SkillStringRecord()
{
  add(skillstring);
}
