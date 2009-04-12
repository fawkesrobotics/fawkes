
/***************************************************************************
 *  naostiffness_config_plugin.h - Config plugin for the nao joint stiffnesses
 *
 *  Created: Tue Apr  7 15:15:15 2009
 *  Copyright  2009  Tobias Kellner
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

#ifndef __TOOLS_CONFIG_EDITOR_NAOSTIFFNESS_CONFIG_PLUGIN_H_
#define __TOOLS_CONFIG_EDITOR_NAOSTIFFNESS_CONFIG_PLUGIN_H_

#include "config_editor_plugin.h"

#include <gtkmm.h>
#include <libglademm/xml.h>

#include <string>
#include <vector>

class NaoStiffnessConfigDialog : public Gtk::Dialog
{
public:
 struct nao_stiffnesses /**< joint stiffness values */
 {
   float hy; /**< head yaw */
   float hp; /**< head pitch */

   float lsp; /**< left  shoulder pitch */
   float rsp; /**< right shoulder pitch */
   float lsr; /**< left  shoulder roll */
   float rsr; /**< right shoulder roll */
   float ley; /**< left  elbow yaw */
   float rey; /**< right elbow yaw */
   float ler; /**< left  elbow roll */
   float rer; /**< right elbow roll */

   float lhyp; /**< left  hip yaw/pitch */
   float rhyp; /**< right hip yaw/pitch */
   float lhr;  /**< left  hip roll */
   float rhr;  /**< right hip roll */
   float lhp;  /**< left  hip pitch */
   float rhp;  /**< right hip pitch */
   float lkp;  /**< left  knee pitch */
   float rkp;  /**< right knee pitch */
   float lar;  /**< left  ankle roll */
   float rar;  /**< right ankle roll */
   float lap;  /**< left  ankle pitch */
   float rap;  /**< right ankle pitch */
 };

 public:
  NaoStiffnessConfigDialog(BaseObjectType *cobject,
                           const Glib::RefPtr<Gnome::Glade::Xml> &ref_xml);
  virtual ~NaoStiffnessConfigDialog();

  virtual void set_stiffnesses(const nao_stiffnesses &vals);
  virtual void get_stiffnesses(nao_stiffnesses &vals);

  virtual void on_checkbutton_lock_toggled();
  virtual void on_combobox_behaviour_changed();

  virtual std::string get_cur_behaviour();
  virtual void set_load_vals(sigc::slot<void> cb);

 private:
  Gtk::SpinButton *__hy;
  Gtk::SpinButton *__hp;

  Gtk::SpinButton *__lsp;
  Gtk::SpinButton *__rsp;
  Gtk::SpinButton *__lsr;
  Gtk::SpinButton *__rsr;
  Gtk::SpinButton *__ley;
  Gtk::SpinButton *__rey;
  Gtk::SpinButton *__ler;
  Gtk::SpinButton *__rer;

  Gtk::SpinButton *__lhyp;
  Gtk::SpinButton *__rhyp;
  Gtk::SpinButton *__lhr;
  Gtk::SpinButton *__rhr;
  Gtk::SpinButton *__lhp;
  Gtk::SpinButton *__rhp;
  Gtk::SpinButton *__lkp;
  Gtk::SpinButton *__rkp;
  Gtk::SpinButton *__lar;
  Gtk::SpinButton *__rar;
  Gtk::SpinButton *__lap;
  Gtk::SpinButton *__rap;

  Gtk::CheckButton *__lck;
  std::vector<sigc::connection> __connections;

  Gtk::ComboBox *__bhv;
  std::string __cur_bhv;

  sigc::slot<void> __load_vals;
};

class NaoStiffnessConfigPlugin : public ConfigEditorPlugin
{
 public:
   NaoStiffnessConfigPlugin(std::string glade_path);
  virtual ~NaoStiffnessConfigPlugin();

 protected:
  virtual void pre_run();
  virtual void post_run(int response);

  virtual Gtk::Dialog* load_dialog();

 private:
  virtual void load_vals();
  virtual void save_vals();

 private:
   NaoStiffnessConfigDialog::nao_stiffnesses __initial_vals;
};

#endif /* __TOOLS_CONFIG_EDITOR_NAOSTIFFNESS_CONFIG_PLUGIN_H_ */
