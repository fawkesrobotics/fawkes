
/***************************************************************************
 *  firestation.h - Firestation
 *
 *  Created: Wed Oct 10 14:15:56 2007
 *  Copyright  2007  Daniel Beck
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

#ifndef __FIREVISION_TOOLS_FIRESTATION_FIRESTATION_H_
#define __FIREVISION_TOOLS_FIRESTATION_FIRESTATION_H_

#include <gtkmm.h>
#include <libglademm/xml.h>
#include <fvutils/base/roi.h>
#include <fvutils/color/colorspaces.h>
#include <netcomm/dns-sd/avahi_thread.h>

namespace firevision {
  class Camera;
  class SharedMemoryImageBuffer;
  class ShmImageLister;
  class Writer;
  class FuseImageListWidget;
}
namespace fawkes {
  class AvahiDispatcher;
}
class MirrorCalibTool;
class ColorTrainTool;
class ColorTrainWidget;
class FuseTransferWidget;

class Firestation : public Gtk::Window
{
 public:
  Firestation(Glib::RefPtr<Gnome::Glade::Xml> ref_xml);
  virtual ~Firestation();

  Gtk::Window& get_window() const;

 private:
  class ShmColumnRecord : public Gtk::TreeModel::ColumnRecord
  {
  public:
    ShmColumnRecord()
      {
	add(m_id); add(m_name);
      };
    Gtk::TreeModelColumn<int> m_id;
    Gtk::TreeModelColumn<Glib::ustring> m_name;
  };

  class FuseColumnRecord : public Gtk::TreeModel::ColumnRecord
    {
    public:
      FuseColumnRecord()
	{
	  add(m_id); add(m_name);
	  add(m_service_name); add(m_service_domain);
	  add(m_service_hostname); add(m_service_port);
	  add(m_image_id); add(m_image_width), add(m_image_height);
	  add(m_image_colorspace);
	};
      Gtk::TreeModelColumn<int> m_id;
      Gtk::TreeModelColumn<Glib::ustring> m_name;
      Gtk::TreeModelColumn<Glib::ustring> m_service_name;
      Gtk::TreeModelColumn<Glib::ustring> m_service_type;
      Gtk::TreeModelColumn<Glib::ustring> m_service_domain;
      Gtk::TreeModelColumn<Glib::ustring> m_service_hostname;
      Gtk::TreeModelColumn<unsigned short int> m_service_port;
      Gtk::TreeModelColumn<Glib::ustring> m_image_id;
      Gtk::TreeModelColumn<unsigned int> m_image_width;
      Gtk::TreeModelColumn<unsigned int> m_image_height;
      Gtk::TreeModelColumn<Glib::ustring> m_image_colorspace;
   };

  typedef enum
  {
    SRC_NONE,
    SRC_FILE,
    SRC_SHM,
    SRC_FUSE
  } ImageSource;

  typedef enum
  {
    MODE_VIEWER,
    MODE_COLOR_TRAIN,
    MODE_MIRROR_CALIB,
    MODE_MIRROR_CALIB_EVAL
  } OpMode;

  Gtk::Widget* get_widget(Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
			  const char* widget_name) const;

  void save_image();
  void exit();
  void close_camera();
  void update_image();
  bool call_update_image();
  void enable_cont_img_trans();
  void open_file();
  void open_folder();
  void open_shm();
  void open_fuse();
  void on_fuse_image_selected();
  void on_colormap_updated();
  bool image_click(GdkEventButton*);

  void on_service_added( fawkes::NetworkService* service );
  void on_service_removed( fawkes::NetworkService* service );

  void pre_open_img_src();
  void post_open_img_src();
  void resize_image(Gtk::Allocation& allocation);
  void draw_image();

  void ct_start();
  firevision::hint_t ct_get_fg_object();
  void ct_object_changed();

  double mc_line_angle_deg;
  void mc_load_mask();
  void mc_memorize();
  void mc_draw_line();
  bool mc_set_line_angle(Gtk::ScrollType scroll, double value);
  void mc_save();
  void mc_load();

  Glib::Dispatcher m_update_img;

  fawkes::AvahiDispatcher* m_avahi_dispatcher;

  // widgets
  Gtk::Window* m_wnd_main;
  Gtk::Dialog* m_dlg_open_shm;
  Gtk::Dialog* m_dlg_open_fuse;
  Gtk::CheckButton* m_ckb_fuse_jpeg;
  Gtk::CheckButton* m_ckb_cont_trans;
  Gtk::SpinButton* m_spb_update_time;
  Gtk::FileChooserDialog* m_fcd_open_image;
  Gtk::FileChooserDialog* m_fcd_save_image;
  Gtk::ToolButton* m_tbtn_open_file;
  Gtk::ToolButton* m_tbtn_open_folder;
  Gtk::ToolButton* m_tbtn_open_shm;
  Gtk::ToolButton* m_tbtn_open_fuse;
  Gtk::ToolButton* m_tbtn_update;
  Gtk::ToolButton* m_tbtn_save;
  Gtk::ToolButton* m_tbtn_close_camera;
  Gtk::ToolButton* m_tbtn_exit;
  Gtk::Image* m_img_image;
  Gtk::EventBox* m_evt_image;
  Gtk::TreeView* m_trv_shm_image_ids;
  Gtk::TreeView* m_trv_fuse_services;
  Gtk::Statusbar* m_stb_status;

  // color training widgets
  Gtk::ComboBox*    m_cmb_ct_type;
  Gtk::ToggleButton* m_btn_ct_start;
  Gtk::ToggleButton* m_btn_ct_seg;
  Gtk::SpinButton* m_spbtn_depth;
  Gtk::SpinButton* m_spbtn_width;
  Gtk::SpinButton* m_spbtn_height;

  // mirror calibration widgets
  Gtk::FileChooserDialog* m_fcd_mc_load_mask;
  Gtk::FileChooserDialog* m_fcd_mc_save;
  Gtk::FileChooserDialog* m_fcd_mc_load;
  Gtk::Button* m_btn_mc_load_mask;
  Gtk::Button* m_btn_mc_memorize;
  Gtk::Scale* m_scl_mc_line;
  Gtk::Button* m_btn_mc_load;
  Gtk::Button* m_btn_mc_save;
  Gtk::Entry* m_ent_mc_dist;
  Gtk::Entry* m_ent_mc_ori;

  ShmColumnRecord m_shm_columns;
  Glib::RefPtr<Gtk::ListStore> m_shm_list_store;

  FuseColumnRecord m_fuse_columns;
  Glib::RefPtr<Gtk::TreeStore> m_fuse_tree_store;

  firevision::SharedMemoryImageBuffer* m_shm_buffer;
  firevision::Camera* m_camera;
  firevision::Writer* m_img_writer;

  ImageSource m_img_src;
  OpMode m_op_mode;

  // image buffer
  unsigned char* m_yuv_orig_buffer;
  unsigned char* m_yuv_draw_buffer;
  unsigned char* m_yuv_scaled_buffer;
  unsigned char* m_rgb_scaled_buffer;

  unsigned int m_img_width;
  unsigned int m_img_height;
  unsigned int m_scaled_img_width;
  unsigned int m_scaled_img_height;
  unsigned int m_max_img_width;
  unsigned int m_max_img_height;

  firevision::colorspace_t m_img_cs;
  size_t m_img_size;

  bool m_cont_img_trans;

  bool m_enable_scaling;
  float m_scale_factor;

  MirrorCalibTool* m_calib_tool;
  ColorTrainWidget* m_ctw;
  FuseTransferWidget* m_ftw;
  firevision::FuseImageListWidget* m_filw;

  fawkes::AvahiThread* m_avahi_thread;
};

#endif /* __FIREVISION_TOOLS_FIRESTATION_FIRESTATION_H_ */
