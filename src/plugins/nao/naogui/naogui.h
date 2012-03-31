
/***************************************************************************
 *  naogui.h - Nao GUI
 *
 *  Created: Mon Oct 27 17:10:58 2008
 *  Copyright  2008-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAO_TOOLS_NAOGUI_NAOGUI_H_
#define __PLUGINS_NAO_TOOLS_NAOGUI_NAOGUI_H_

#include <gui_utils/connection_dispatcher.h>

#include <gtkmm.h>

namespace fawkes {
  class BlackBoard;
  class Interface;
  class NaoJointPositionInterface;
  class NaoJointStiffnessInterface;
  class NaoSensorInterface;
  class NavigatorInterface;
  class HumanoidMotionInterface;
  class SpeechSynthInterface;
  class InterfaceDispatcher;
  class LedInterface;
  class SwitchInterface;
}

class NaoGuiGtkWindow : public Gtk::Window
{
 public:
  NaoGuiGtkWindow(BaseObjectType* cobject,
		  const Glib::RefPtr<Gtk::Builder> &builder);
  ~NaoGuiGtkWindow();

 private:
  void update_servo_value(Gtk::HScale *hsc, Gtk::Label *label, float value);
  void update_sensor_value(Gtk::Label *label, float value, bool show_decimal=true);
  void update_entry_value(Gtk::Entry *ent, float value, unsigned int width = 2);
  void update_ultrasonic_direction();
  void send_servo_msg(Gtk::HScale *hsc, unsigned int servo);
  void update_values(bool force = false);
  void update_jointpos_values(bool force = false);
  void update_sensor_values(bool force = false);
  bool servos_enabled() const;
  void update_tts();

  void on_led_data_changed(fawkes::Interface *iface);
  void on_button_data_changed(fawkes::Interface *iface);

  void on_stiffness_clicked();
  void on_control_toggled();
  void on_sv_copy_clicked();
  void on_us_emit_clicked();
  void on_us_auto_toggled();
  void on_goto_parkpos_clicked();
  void on_goto_zero_all_clicked();
  void on_get_up_clicked();
  void on_walkvel_exec_clicked();
  void on_ws_exec_clicked();
  void on_wsw_exec_clicked();
  void on_kick_exec_clicked();
  void on_standup_exec_clicked();
  void on_nav_exec_clicked();
  void on_wa_exec_clicked();
  void on_turn_exec_clicked();
  void on_motion_stop_clicked();
  void on_cf_read_clicked();
  void on_cf_write_clicked();
  void on_stiffness_write_clicked();
  void on_stiffness_read_clicked();
  void on_stiffness_global_toggled();
  void on_tts_exec_clicked();
  void on_slider_changed(Gtk::HScale *hsc, Gtk::Label *lab, unsigned int servo);
  void on_changed_speed();
  void on_connection_clicked();
  void on_connect();
  void on_disconnect();
  void on_exit_clicked();

  void on_control_leds_toggled();
  void on_led_tb_toggled(std::string iface_id, Gtk::ToggleButton *tb);
  void on_led_slider_changed(std::string iface_id, Gtk::Scale *scl);
  bool on_led_slider_button_release(GdkEventButton *event,
                                    std::string iface_id, Gtk::Scale *scl);

  void on_button_click_pressed(std::string iface_id);
  void on_button_click_released(std::string iface_id);

  bool convert_str2float(Glib::ustring sn, float *f);
  Glib::ustring convert_float2str(float f, unsigned int width = 2);

  void init();


 private:
  fawkes::BlackBoard *bb;
  fawkes::InterfaceDispatcher *ifd_jointpos;
  fawkes::InterfaceDispatcher *ifd_sensor;
  fawkes::InterfaceDispatcher *ifd_tts;
  fawkes::InterfaceDispatcher *ifd_leds;
  fawkes::InterfaceDispatcher *ifd_buttons;
  fawkes::NaoJointPositionInterface *jointpos_if;
  fawkes::NaoJointStiffnessInterface *jointstiff_if;
  fawkes::NaoSensorInterface *sensor_if;
  fawkes::NavigatorInterface *nao_navi_if;
  fawkes::SpeechSynthInterface *speechsynth_if;
  fawkes::HumanoidMotionInterface *hummot_fawkes_if;
  fawkes::HumanoidMotionInterface *hummot_naoqi_if;
  fawkes::ConnectionDispatcher connection_dispatcher;

  std::map<std::string, fawkes::LedInterface *> led_ifs;
  std::map<std::string, Gtk::Scale *> led_scales;
  std::map<std::string, Gtk::ToggleButton *> led_buttons;

  bool servo_enabled;
  bool global_stiffness_enabled;

  Gtk::Frame  *frm_servos;
  Gtk::Frame  *frm_sensors;
  Gtk::Frame  *frm_ultrasonic;

  Gtk::HScale *hsc_HeadYaw;
  Gtk::Label  *lab_HeadYaw;
  Gtk::HScale *hsc_HeadPitch;
  Gtk::Label  *lab_HeadPitch;
  Gtk::HScale *hsc_RShoulderPitch;
  Gtk::Label  *lab_RShoulderPitch;
  Gtk::HScale *hsc_RShoulderRoll;
  Gtk::Label  *lab_RShoulderRoll;
  Gtk::HScale *hsc_RElbowYaw;
  Gtk::Label  *lab_RElbowYaw;
  Gtk::HScale *hsc_RElbowRoll;
  Gtk::Label  *lab_RElbowRoll;
  Gtk::HScale *hsc_RWristYaw;
  Gtk::Label  *lab_RWristYaw;
  Gtk::HScale *hsc_RHand;
  Gtk::Label  *lab_RHand;
  Gtk::HScale *hsc_LShoulderPitch;
  Gtk::Label  *lab_LShoulderPitch;
  Gtk::HScale *hsc_LShoulderRoll;
  Gtk::Label  *lab_LShoulderRoll;
  Gtk::HScale *hsc_LElbowYaw;
  Gtk::Label  *lab_LElbowYaw;
  Gtk::HScale *hsc_LElbowRoll;
  Gtk::Label  *lab_LElbowRoll;
  Gtk::HScale *hsc_LWristYaw;
  Gtk::Label  *lab_LWristYaw;
  Gtk::HScale *hsc_LHand;
  Gtk::Label  *lab_LHand;
  Gtk::HScale *hsc_RHipYawPitch;
  Gtk::Label  *lab_RHipYawPitch;
  Gtk::HScale *hsc_RHipPitch;
  Gtk::Label  *lab_RHipPitch;
  Gtk::HScale *hsc_RHipRoll;
  Gtk::Label  *lab_RHipRoll;
  Gtk::HScale *hsc_RKneePitch;
  Gtk::Label  *lab_RKneePitch;
  Gtk::HScale *hsc_RAnklePitch;
  Gtk::Label  *lab_RAnklePitch;
  Gtk::HScale *hsc_RAnkleRoll;
  Gtk::Label  *lab_RAnkleRoll;
  Gtk::HScale *hsc_LHipYawPitch;
  Gtk::Label  *lab_LHipYawPitch;
  Gtk::HScale *hsc_LHipPitch;
  Gtk::Label  *lab_LHipPitch;
  Gtk::HScale *hsc_LHipRoll;
  Gtk::Label  *lab_LHipRoll;
  Gtk::HScale *hsc_LKneePitch;
  Gtk::Label  *lab_LKneePitch;
  Gtk::HScale *hsc_LAnklePitch;
  Gtk::Label  *lab_LAnklePitch;
  Gtk::HScale *hsc_LAnkleRoll;
  Gtk::Label  *lab_LAnkleRoll;
  Gtk::HScale *hsc_speed;
  Gtk::Label  *lab_speed;
  Gtk::ToolButton *tb_connection;
  Gtk::ToolButton *tb_stiffness;
  Gtk::ToggleToolButton *tb_control;
  Gtk::ToolButton *tb_getup;
  Gtk::ToolButton *tb_parkpos;
  Gtk::ToolButton *tb_zeroall;
  Gtk::ToolButton *tb_exit;
  Gtk::Label  *lab_l_fsr_fl;
  Gtk::Label  *lab_l_fsr_fr;
  Gtk::Label  *lab_l_fsr_rl;
  Gtk::Label  *lab_l_fsr_rr;
  Gtk::Label  *lab_r_fsr_fl;
  Gtk::Label  *lab_r_fsr_fr;
  Gtk::Label  *lab_r_fsr_rl;
  Gtk::Label  *lab_r_fsr_rr;
  Gtk::Label  *lab_r_cop;
  Gtk::Label  *lab_l_cop;
  Gtk::Label  *lab_r_total_weight;
  Gtk::Label  *lab_l_total_weight;
  Gtk::Label  *lab_chest_button;
  Gtk::Label  *lab_touch_front;
  Gtk::Label  *lab_touch_middle;
  Gtk::Label  *lab_touch_rear;
  Gtk::Label  *lab_l_bumper_l;
  Gtk::Label  *lab_l_bumper_r;
  Gtk::Label  *lab_r_bumper_l;
  Gtk::Label  *lab_r_bumper_r;
  Gtk::Label  *lab_accel_x;
  Gtk::Label  *lab_accel_y;
  Gtk::Label  *lab_accel_z;
  Gtk::Label  *lab_gyro_x;
  Gtk::Label  *lab_gyro_y;
  Gtk::Label  *lab_gyro_ref;
  Gtk::Label  *lab_angles_xy;
  Gtk::Label  *lab_ultrasonic_direction;
  Gtk::Label  *lab_ultrasonic_left0;
  Gtk::Label  *lab_ultrasonic_left1;
  Gtk::Label  *lab_ultrasonic_left2;
  Gtk::Label  *lab_ultrasonic_left3;
  Gtk::Label  *lab_ultrasonic_right0;
  Gtk::Label  *lab_ultrasonic_right1;
  Gtk::Label  *lab_ultrasonic_right2;
  Gtk::Label  *lab_ultrasonic_right3;
  Gtk::Label  *lab_battery_charge;
  Gtk::ToggleButton *but_us_auto;
  Gtk::Button       *but_us_emit;
  Gtk::ComboBox     *cmb_us_direction;

  Gtk::Button *but_sv_copy;

  Gtk::Button *but_stiffness_read;
  Gtk::Button *but_stiffness_write;
  Gtk::SpinButton *spb_stiffness_global;
  Gtk::CheckButton *chb_stiffness_global;
  Gtk::SpinButton *spb_HeadYaw;
  Gtk::SpinButton *spb_HeadPitch;
  Gtk::SpinButton *spb_RShoulderPitch;
  Gtk::SpinButton *spb_RShoulderRoll;
  Gtk::SpinButton *spb_RElbowYaw;
  Gtk::SpinButton *spb_RElbowRoll;
  Gtk::SpinButton *spb_RWristYaw;
  Gtk::SpinButton *spb_RHand;
  Gtk::SpinButton *spb_LShoulderPitch;
  Gtk::SpinButton *spb_LShoulderRoll;
  Gtk::SpinButton *spb_LElbowYaw;
  Gtk::SpinButton *spb_LElbowRoll;
  Gtk::SpinButton *spb_LWristYaw;
  Gtk::SpinButton *spb_LHand;
  Gtk::SpinButton *spb_RHipYawPitch;
  Gtk::SpinButton *spb_RHipPitch;
  Gtk::SpinButton *spb_RHipRoll;
  Gtk::SpinButton *spb_RKneePitch;
  Gtk::SpinButton *spb_RAnklePitch;
  Gtk::SpinButton *spb_RAnkleRoll;
  Gtk::SpinButton *spb_LHipYawPitch;
  Gtk::SpinButton *spb_LHipPitch;
  Gtk::SpinButton *spb_LHipRoll;
  Gtk::SpinButton *spb_LKneePitch;
  Gtk::SpinButton *spb_LAnklePitch;
  Gtk::SpinButton *spb_LAnkleRoll;

  Gtk::Label *lab_stiff_HeadYaw;
  Gtk::Label *lab_stiff_HeadPitch;
  Gtk::Label *lab_stiff_RShoulderPitch;
  Gtk::Label *lab_stiff_RShoulderRoll;
  Gtk::Label *lab_stiff_RElbowYaw;
  Gtk::Label *lab_stiff_RElbowRoll;
  Gtk::Label *lab_stiff_RWristYaw;
  Gtk::Label *lab_stiff_RHand;
  Gtk::Label *lab_stiff_LShoulderPitch;
  Gtk::Label *lab_stiff_LShoulderRoll;
  Gtk::Label *lab_stiff_LElbowYaw;
  Gtk::Label *lab_stiff_LElbowRoll;
  Gtk::Label *lab_stiff_LWristYaw;
  Gtk::Label *lab_stiff_LHand;
  Gtk::Label *lab_stiff_RHipYawPitch;
  Gtk::Label *lab_stiff_RHipPitch;
  Gtk::Label *lab_stiff_RHipRoll;
  Gtk::Label *lab_stiff_RKneePitch;
  Gtk::Label *lab_stiff_RAnklePitch;
  Gtk::Label *lab_stiff_RAnkleRoll;
  Gtk::Label *lab_stiff_LHipYawPitch;
  Gtk::Label *lab_stiff_LHipPitch;
  Gtk::Label *lab_stiff_LHipRoll;
  Gtk::Label *lab_stiff_LKneePitch;
  Gtk::Label *lab_stiff_LAnklePitch;
  Gtk::Label *lab_stiff_LAnkleRoll;

  Gtk::RadioButton *rad_motion_fawkes;
  Gtk::RadioButton *rad_motion_naoqi;
  Gtk::Button *but_stop;
  Gtk::Entry  *ent_ws_distance;
  Gtk::Button *but_ws_exec;
  Gtk::Entry  *ent_wsw_distance;
  Gtk::Button *but_wsw_exec;
  Gtk::Entry  *ent_wa_angle;
  Gtk::Entry  *ent_wa_radius;
  Gtk::Button *but_wa_exec;
  Gtk::Entry  *ent_turn_angle;
  Gtk::Button *but_turn_exec;
  Gtk::ComboBox *cmb_kick_leg;
  Gtk::Entry  *ent_kick_strength;
  Gtk::Button *but_kick_exec;
  Gtk::ComboBox *cmb_standup_from;
  Gtk::Button *but_standup_exec;

  Gtk::Entry  *ent_walkvel_x;
  Gtk::Entry  *ent_walkvel_y;
  Gtk::Entry  *ent_walkvel_theta;
  Gtk::Entry  *ent_walkvel_speed;
  Gtk::Button *but_walkvel_exec;

  Gtk::Entry  *ent_nav_x;
  Gtk::Entry  *ent_nav_y;
  Gtk::Entry  *ent_nav_ori;
  Gtk::Button *but_nav_exec;

  Gtk::Entry *ent_tts;
  Gtk::Button *but_tts_exec;
  Gtk::Label *lab_tts_active;

  Gtk::Scale *scl_chest_r;
  Gtk::Scale *scl_chest_g;
  Gtk::Scale *scl_chest_b;
  Gtk::Scale *scl_left_eye_r;
  Gtk::Scale *scl_left_eye_g;
  Gtk::Scale *scl_left_eye_b;
  Gtk::Scale *scl_right_eye_r;
  Gtk::Scale *scl_right_eye_g;
  Gtk::Scale *scl_right_eye_b;
  Gtk::Scale *scl_left_foot_r;
  Gtk::Scale *scl_left_foot_g;
  Gtk::Scale *scl_left_foot_b;
  Gtk::Scale *scl_right_foot_r;
  Gtk::Scale *scl_right_foot_g;
  Gtk::Scale *scl_right_foot_b;

  Gtk::ToggleButton *tb_left_ear_0;
  Gtk::ToggleButton *tb_left_ear_36;
  Gtk::ToggleButton *tb_left_ear_72;
  Gtk::ToggleButton *tb_left_ear_108;
  Gtk::ToggleButton *tb_left_ear_144;
  Gtk::ToggleButton *tb_left_ear_180;
  Gtk::ToggleButton *tb_left_ear_216;
  Gtk::ToggleButton *tb_left_ear_252;
  Gtk::ToggleButton *tb_left_ear_288;
  Gtk::ToggleButton *tb_left_ear_324;

  Gtk::ToggleButton *tb_right_ear_0;
  Gtk::ToggleButton *tb_right_ear_36;
  Gtk::ToggleButton *tb_right_ear_72;
  Gtk::ToggleButton *tb_right_ear_108;
  Gtk::ToggleButton *tb_right_ear_144;
  Gtk::ToggleButton *tb_right_ear_180;
  Gtk::ToggleButton *tb_right_ear_216;
  Gtk::ToggleButton *tb_right_ear_252;
  Gtk::ToggleButton *tb_right_ear_288;
  Gtk::ToggleButton *tb_right_ear_324;

  Gtk::ToggleButton *tb_control_leds;

  Gtk::Button *but_chestbut;
  Gtk::Button *but_head_front;
  Gtk::Button *but_head_middle;
  Gtk::Button *but_head_rear;
  Gtk::Button *but_lfoot_bumper;
  Gtk::Button *but_rfoot_bumper;

  /// @cond INTERNALS
  typedef struct {
    Gtk::Label  *lab_enabled;
    Gtk::Label  *lab_history;
    Gtk::Label  *lab_value;
    Gtk::Label  *lab_short;
    Gtk::Label  *lab_long;
    Gtk::Label  *lab_total;
  } ButtonLabelSet;
  /// @endcond
  std::map<std::string, ButtonLabelSet> button_labels;
  std::map<std::string, fawkes::SwitchInterface *> button_ifs;

  unsigned int update_cycle;
};

#endif
