
/***************************************************************************
 *  naostiffness_config_plugin.cpp - Config plugin for the nao joint stiffnesses
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

#include "naostiffness_config_plugin.h"

#include <config/config.h>
#include <gui_utils/utils.h>

using namespace fawkes;
using std::string;
using sigc::mem_fun;
using sigc::compose;
using Gtk::SpinButton;
using Gtk::CheckButton;
using Gtk::ComboBox;


#define STIFFNESS_CFG_PATH "/naomotion/stiffness"

/** @class NaoStiffnessConfigDialog naostiffness_config_plugin.h
 * Config dialog of the config editor plugin for the nao joint stiffnesses.
 * @author Tobias Kellner
 */

/** Constructor.
 * Allows to construct a dialog by means of get_widget_derived(...).
 * @param cobject base object pointer
 * @param ref_xml Glade XML object representing the Glade input file
 */
NaoStiffnessConfigDialog::NaoStiffnessConfigDialog(BaseObjectType *cobject,
                                                   const Glib::RefPtr<Gnome::Glade::Xml> &ref_xml)
  : Gtk::Dialog(cobject)
{
  __hy = dynamic_cast<SpinButton *>(get_widget(ref_xml, "hy"));
  __hp = dynamic_cast<SpinButton *>(get_widget(ref_xml, "hp"));

  __lsp = dynamic_cast<SpinButton *>(get_widget(ref_xml, "lsp"));
  __rsp = dynamic_cast<SpinButton *>(get_widget(ref_xml, "rsp"));
  __lsr = dynamic_cast<SpinButton *>(get_widget(ref_xml, "lsr"));
  __rsr = dynamic_cast<SpinButton *>(get_widget(ref_xml, "rsr"));
  __ley = dynamic_cast<SpinButton *>(get_widget(ref_xml, "ley"));
  __rey = dynamic_cast<SpinButton *>(get_widget(ref_xml, "rey"));
  __ler = dynamic_cast<SpinButton *>(get_widget(ref_xml, "ler"));
  __rer = dynamic_cast<SpinButton *>(get_widget(ref_xml, "rer"));

  __lhyp = dynamic_cast<SpinButton *>(get_widget(ref_xml, "lhyp"));
  __rhyp = dynamic_cast<SpinButton *>(get_widget(ref_xml, "rhyp"));
  __lhr = dynamic_cast<SpinButton *>(get_widget(ref_xml, "lhr"));
  __rhr = dynamic_cast<SpinButton *>(get_widget(ref_xml, "rhr"));
  __lhp = dynamic_cast<SpinButton *>(get_widget(ref_xml, "lhp"));
  __rhp = dynamic_cast<SpinButton *>(get_widget(ref_xml, "rhp"));
  __lkp = dynamic_cast<SpinButton *>(get_widget(ref_xml, "lkp"));
  __rkp = dynamic_cast<SpinButton *>(get_widget(ref_xml, "rkp"));
  __lar = dynamic_cast<SpinButton *>(get_widget(ref_xml, "lar"));
  __rar = dynamic_cast<SpinButton *>(get_widget(ref_xml, "rar"));
  __lap = dynamic_cast<SpinButton *>(get_widget(ref_xml, "lap"));
  __rap = dynamic_cast<SpinButton *>(get_widget(ref_xml, "rap"));

  __def = dynamic_cast<CheckButton *>(get_widget(ref_xml, "checkbutton_default"));
  __lck = dynamic_cast<CheckButton *>(get_widget(ref_xml, "checkbutton_lock"));

  __lck->signal_toggled().connect(mem_fun(*this, &NaoStiffnessConfigDialog::on_checkbutton_lock_toggled));
  on_checkbutton_lock_toggled();

  __bhv = dynamic_cast<ComboBox *>(get_widget(ref_xml, "combobox_behaviour"));
  __bhv->set_active(0);
  __bhv->get_active()->get_value(0, __cur_bhv);
  __bhv->signal_changed().connect(mem_fun(*this, &NaoStiffnessConfigDialog::on_combobox_behaviour_changed));
}

/** Destructor. */
NaoStiffnessConfigDialog::~NaoStiffnessConfigDialog()
{
}

/** Set joint stiffness values in the dialog
 * @param vals structure containing the stiffness values
 */
void NaoStiffnessConfigDialog::set_stiffnesses(const nao_stiffnesses &vals)
{
  __hy->set_value(vals.hy);
  __hp->set_value(vals.hp);

  __lsp->set_value(vals.lsp);
  __rsp->set_value(vals.rsp);
  __lsr->set_value(vals.lsr);
  __rsr->set_value(vals.rsr);
  __ley->set_value(vals.ley);
  __rey->set_value(vals.rey);
  __ler->set_value(vals.ler);
  __rer->set_value(vals.rer);

  __lhyp->set_value(vals.lhyp);
  __rhyp->set_value(vals.rhyp);
  __lhr->set_value(vals.lhr);
  __rhr->set_value(vals.rhr);
  __lhp->set_value(vals.lhp);
  __rhp->set_value(vals.rhp);
  __lkp->set_value(vals.lkp);
  __rkp->set_value(vals.rkp);
  __lar->set_value(vals.lar);
  __rar->set_value(vals.rar);
  __lap->set_value(vals.lap);
  __rap->set_value(vals.rap);

  __lck->set_active(
                    __lsp->get_value() == __rsp->get_value() &&
                    __lsr->get_value() == __rsr->get_value() &&
                    __ley->get_value() == __rey->get_value() &&
                    __ler->get_value() == __rer->get_value() &&
                    __lhyp->get_value() == __rhyp->get_value() &&
                    __lhr->get_value() == __rhr->get_value() &&
                    __lhp->get_value() == __rhp->get_value() &&
                    __lkp->get_value() == __rkp->get_value() &&
                    __lar->get_value() == __rar->get_value() &&
                    __lap->get_value() == __rap->get_value()
                    );
}

/** Get joint stiffness values from the dialog
 * @param vals structure the stiffness values get written to
 */
void NaoStiffnessConfigDialog::get_stiffnesses(nao_stiffnesses &vals)
{
  vals.hy = __hy ->get_value();
  vals.hp = __hp->get_value();

  vals.lsp = __lsp->get_value();
  vals.rsp = __rsp->get_value();
  vals.lsr = __lsr->get_value();
  vals.rsr = __rsr->get_value();
  vals.ley = __ley->get_value();
  vals.rey = __rey->get_value();
  vals.ler = __ler->get_value();
  vals.rer = __rer->get_value();

  vals.lhyp = __lhyp->get_value();
  vals.rhyp = __rhyp->get_value();
  vals.lhr = __lhr->get_value();
  vals.rhr = __rhr->get_value();
  vals.lhp = __lhp->get_value();
  vals.rhp = __rhp->get_value();
  vals.lkp = __lkp->get_value();
  vals.rkp = __rkp->get_value();
  vals.lar = __lar->get_value();
  vals.rar = __rar->get_value();
  vals.lap = __lap->get_value();
  vals.rap = __rap->get_value();
}

/** Lock checkbox toggled handler. */
void NaoStiffnessConfigDialog::on_checkbutton_lock_toggled()
{
  bool locked = __lck->get_active();

  __rsp->set_sensitive(!locked);
  __rsr->set_sensitive(!locked);
  __rey->set_sensitive(!locked);
  __rer->set_sensitive(!locked);

  __rhyp->set_sensitive(!locked);
  __rhr->set_sensitive(!locked);
  __rhp->set_sensitive(!locked);
  __rkp->set_sensitive(!locked);
  __rar->set_sensitive(!locked);
  __rap->set_sensitive(!locked);

  if (locked)
  {
    __connections.push_back(__lsp->signal_value_changed().connect(compose(mem_fun(*__rsp, &SpinButton::set_value), mem_fun(*__lsp, &SpinButton::get_value))));
    __connections.push_back(__lsr->signal_value_changed().connect(compose(mem_fun(*__rsr, &SpinButton::set_value), mem_fun(*__lsr, &SpinButton::get_value))));
    __connections.push_back(__ley->signal_value_changed().connect(compose(mem_fun(*__rey, &SpinButton::set_value), mem_fun(*__ley, &SpinButton::get_value))));
    __connections.push_back(__ler->signal_value_changed().connect(compose(mem_fun(*__rer, &SpinButton::set_value), mem_fun(*__ler, &SpinButton::get_value))));

    __connections.push_back(__lhyp->signal_value_changed().connect(compose(mem_fun(*__rhyp, &SpinButton::set_value), mem_fun(*__lhyp, &SpinButton::get_value))));
    __connections.push_back(__lhr->signal_value_changed().connect(compose(mem_fun(*__rhr, &SpinButton::set_value), mem_fun(*__lhr, &SpinButton::get_value))));
    __connections.push_back(__lhp->signal_value_changed().connect(compose(mem_fun(*__rhp, &SpinButton::set_value), mem_fun(*__lhp, &SpinButton::get_value))));
    __connections.push_back(__lkp->signal_value_changed().connect(compose(mem_fun(*__rkp, &SpinButton::set_value), mem_fun(*__lkp, &SpinButton::get_value))));
    __connections.push_back(__lar->signal_value_changed().connect(compose(mem_fun(*__rar, &SpinButton::set_value), mem_fun(*__lar, &SpinButton::get_value))));
    __connections.push_back(__lap->signal_value_changed().connect(compose(mem_fun(*__rap, &SpinButton::set_value), mem_fun(*__lap, &SpinButton::get_value))));

    __rsp->set_value(__lsp->get_value());
    __rsr->set_value(__lsr->get_value());
    __rey->set_value(__ley->get_value());
    __rer->set_value(__ler->get_value());

    __rhyp->set_value(__lhyp->get_value());
    __rhr->set_value(__lhr->get_value());
    __rhp->set_value(__lhp->get_value());
    __rkp->set_value(__lkp->get_value());
    __rar->set_value(__lar->get_value());
    __rap->set_value(__lap->get_value());
  }
  else
  {
    while (!__connections.empty())
    {
      __connections.back().disconnect();
      __connections.pop_back();
    }
  }
}

/** Behaviour combobox changed handler. */
void NaoStiffnessConfigDialog::on_combobox_behaviour_changed()
{
  __bhv->get_active()->get_value(0, __cur_bhv);
  __load_vals();
}

/** Return currently selected behaviour.
 * @return a string representing the selected behaviour
 */
string NaoStiffnessConfigDialog::get_cur_behaviour()
{
  return __cur_bhv;
}

/** Return whether default checkbox is checked.
 * @return true if default is checked
 */
bool NaoStiffnessConfigDialog::get_save_default()
{
  return __def->get_active();
}

/** Set the callback function for loading values in the plugin.
 * Config is not accessible in the dialog, so it has to be done there.
 * @param cb the callback
 */
void NaoStiffnessConfigDialog::set_load_vals(sigc::slot<void> cb)
{
  __load_vals = cb;
}


/** @class NaoStiffnessConfigPlugin naostiffness_config_plugin.h
 * Config editor plugin for the Nao joint stiffness values.
 * @author Tobias Kellner
 */

/** Constructor.
 * @param glade_path path to the Glade file for the plugin's dialog
 */
NaoStiffnessConfigPlugin::NaoStiffnessConfigPlugin(string glade_path)
  : ConfigEditorPlugin(STIFFNESS_CFG_PATH, glade_path)
{
}

/** Destructor. */
NaoStiffnessConfigPlugin::~NaoStiffnessConfigPlugin()
{
}

/** Load joint stiffness values from config */
void
NaoStiffnessConfigPlugin::load_vals()
{
  try
  {
    NaoStiffnessConfigDialog* dlg = dynamic_cast<NaoStiffnessConfigDialog *>(m_dialog);

    string path = string(STIFFNESS_CFG_PATH"/").append(dlg->get_cur_behaviour());

    __initial_vals.hy = m_config->get_float(string(path).append("/head_yaw").c_str());
    __initial_vals.hp = m_config->get_float(string(path).append("/head_pitch").c_str());

    __initial_vals.lsp = m_config->get_float(string(path).append("/l_shoulder_pitch").c_str());
    __initial_vals.rsp = m_config->get_float(string(path).append("/r_shoulder_pitch").c_str());
    __initial_vals.lsr = m_config->get_float(string(path).append("/l_shoulder_roll").c_str());
    __initial_vals.rsr = m_config->get_float(string(path).append("/r_shoulder_roll").c_str());
    __initial_vals.ley = m_config->get_float(string(path).append("/l_elbow_yaw").c_str());
    __initial_vals.rey = m_config->get_float(string(path).append("/r_elbow_yaw").c_str());
    __initial_vals.ler = m_config->get_float(string(path).append("/l_elbow_roll").c_str());
    __initial_vals.rer = m_config->get_float(string(path).append("/r_elbow_roll").c_str());

    __initial_vals.lhyp = m_config->get_float(string(path).append("/l_hip_yaw_pitch").c_str());
    __initial_vals.rhyp = m_config->get_float(string(path).append("/r_hip_yaw_pitch").c_str());
    __initial_vals.lhr = m_config->get_float(string(path).append("/l_hip_roll").c_str());
    __initial_vals.rhr = m_config->get_float(string(path).append("/r_hip_roll").c_str());
    __initial_vals.lhp = m_config->get_float(string(path).append("/l_hip_roll").c_str());
    __initial_vals.rhp = m_config->get_float(string(path).append("/r_hip_roll").c_str());
    __initial_vals.lkp = m_config->get_float(string(path).append("/l_knee_pitch").c_str());
    __initial_vals.rkp = m_config->get_float(string(path).append("/r_knee_pitch").c_str());
    __initial_vals.lar = m_config->get_float(string(path).append("/l_ankle_roll").c_str());
    __initial_vals.rar = m_config->get_float(string(path).append("/r_ankle_roll").c_str());
    __initial_vals.lap = m_config->get_float(string(path).append("/l_ankle_pitch").c_str());
    __initial_vals.rap = m_config->get_float(string(path).append("/r_ankle_pitch").c_str());

    dlg->set_stiffnesses(__initial_vals);
  }
  catch (Exception &e)
  {
    printf("NaoStiffnessConfigPlugin: Could not read required config values.\n");

    __initial_vals.hy = -1.0;
    __initial_vals.hp = -1.0;

    __initial_vals.lsp = -1.0;
    __initial_vals.rsp = -1.0;
    __initial_vals.lsr = -1.0;
    __initial_vals.rsr = -1.0;
    __initial_vals.ley = -1.0;
    __initial_vals.rey = -1.0;
    __initial_vals.ler = -1.0;
    __initial_vals.rer = -1.0;

    __initial_vals.lhyp = -1.0;
    __initial_vals.rhyp = -1.0;
    __initial_vals.lhr = -1.0;
    __initial_vals.rhr = -1.0;
    __initial_vals.lhp = -1.0;
    __initial_vals.rhp = -1.0;
    __initial_vals.lkp = -1.0;
    __initial_vals.rkp = -1.0;
    __initial_vals.lar = -1.0;
    __initial_vals.rar = -1.0;
    __initial_vals.lap = -1.0;
    __initial_vals.rap = -1.0;
  }
}

/** Save joint stiffness values to config */
void
NaoStiffnessConfigPlugin::save_vals()
{
  try
  {
    NaoStiffnessConfigDialog::nao_stiffnesses vals;
    NaoStiffnessConfigDialog* dlg = dynamic_cast<NaoStiffnessConfigDialog *>(m_dialog);
    dlg->get_stiffnesses(vals);
    string path = string(STIFFNESS_CFG_PATH"/").append(dlg->get_cur_behaviour());

    void (Configuration::*my_set_float)(const char *, float) = NULL;

    if (dlg->get_save_default()) my_set_float = &Configuration::set_default_float;
    else my_set_float = &Configuration::set_float;

    if (vals.hy != __initial_vals.hy) (m_config->*my_set_float)(string(path).append("/head_yaw").c_str(), vals.hy);
    if (vals.hp != __initial_vals.hp) (m_config->*my_set_float)(string(path).append("/head_pitch").c_str(), vals.hp);

    if (vals.lsp != __initial_vals.lsp) (m_config->*my_set_float)(string(path).append("/l_shoulder_pitch").c_str(), vals.lsp);
    if (vals.rsp != __initial_vals.rsp) (m_config->*my_set_float)(string(path).append("/r_shoulder_pitch").c_str(), vals.rsp);
    if (vals.lsr != __initial_vals.lsr) (m_config->*my_set_float)(string(path).append("/l_shoulder_roll").c_str(), vals.lsr);
    if (vals.rsr != __initial_vals.rsr) (m_config->*my_set_float)(string(path).append("/r_shoulder_roll").c_str(), vals.rsr);
    if (vals.ley != __initial_vals.ley) (m_config->*my_set_float)(string(path).append("/l_elbow_yaw").c_str(), vals.ley);
    if (vals.rey != __initial_vals.rey) (m_config->*my_set_float)(string(path).append("/r_elbow_yaw").c_str(), vals.rey);
    if (vals.ler != __initial_vals.ler) (m_config->*my_set_float)(string(path).append("/l_elbow_roll").c_str(), vals.ler);
    if (vals.rer != __initial_vals.rer) (m_config->*my_set_float)(string(path).append("/r_elbow_roll").c_str(), vals.rer);

    if (vals.lhyp != __initial_vals.lhyp) (m_config->*my_set_float)(string(path).append("/l_hip_yaw_pitch").c_str(), vals.lhyp);
    if (vals.rhyp != __initial_vals.rhyp) (m_config->*my_set_float)(string(path).append("/r_hip_yaw_pitch").c_str(), vals.rhyp);
    if (vals.lhr != __initial_vals.lhr) (m_config->*my_set_float)(string(path).append("/l_hip_roll").c_str(), vals.lhr);
    if (vals.rhr != __initial_vals.rhr) (m_config->*my_set_float)(string(path).append("/r_hip_roll").c_str(), vals.rhr);
    if (vals.lhp != __initial_vals.lhp) (m_config->*my_set_float)(string(path).append("/l_hip_roll").c_str(), vals.lhp);
    if (vals.rhp != __initial_vals.rhp) (m_config->*my_set_float)(string(path).append("/r_hip_roll").c_str(), vals.rhp);
    if (vals.lkp != __initial_vals.lkp) (m_config->*my_set_float)(string(path).append("/l_knee_pitch").c_str(), vals.lkp);
    if (vals.rkp != __initial_vals.rkp) (m_config->*my_set_float)(string(path).append("/r_knee_pitch").c_str(), vals.rkp);
    if (vals.lar != __initial_vals.lar) (m_config->*my_set_float)(string(path).append("/l_ankle_roll").c_str(), vals.lar);
    if (vals.rar != __initial_vals.rar) (m_config->*my_set_float)(string(path).append("/r_ankle_roll").c_str(), vals.rar);
    if (vals.lap != __initial_vals.lap) (m_config->*my_set_float)(string(path).append("/l_ankle_pitch").c_str(), vals.lap);
    if (vals.rap != __initial_vals.rap) (m_config->*my_set_float)(string(path).append("/r_ankle_pitch").c_str(), vals.rap);
  }
  catch (Exception &e)
  {
    printf("NaoStiffnessConfigPlugin: Could not write required config values.\n");
    e.print_backtrace();
  }
}

void
NaoStiffnessConfigPlugin::pre_run()
{
  load_vals();
}

void
NaoStiffnessConfigPlugin::post_run(int response)
{
  switch (response)
  {
    case Gtk::RESPONSE_OK:
      save_vals();
      break;

    case Gtk::RESPONSE_CANCEL: //fall through
    case Gtk::RESPONSE_DELETE_EVENT:
      break;

    default:
      printf("Nao stiffness config plugin: unknown response\n");
      break;
  }
}

Gtk::Dialog*
NaoStiffnessConfigPlugin::load_dialog()
{
  NaoStiffnessConfigDialog *dlg = NULL;
  m_ref_xml->get_widget_derived("PluginDialog", dlg);
  dlg->set_load_vals(mem_fun(*this, &NaoStiffnessConfigPlugin::load_vals));

  return dlg;
}
