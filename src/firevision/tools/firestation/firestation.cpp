
/***************************************************************************
 *  firestation.cpp - Firestation
 *
 *  Created: Wed Oct 10 14:19:30 2007
 *  Copyright  2007-2008  Daniel Beck
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

#include "firestation.h"
#include "mirror_calib.h"
#include "color_train_widget.h"
#include "fuse_transfer_widget.h"

#include <fvwidgets/fuse_image_list_widget.h>
#include <gui_utils/avahi_dispatcher.h>

#include <cams/fileloader.h>
#include <cams/shmem.h>
#include <cams/net.h>

#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>
#include <fvutils/color/yuv.h>
#include <fvutils/colormap/yuvcm.h>
#include <fvutils/scalers/lossy.h>
#include <fvutils/system/camargp.h>
#include <fvutils/writers/jpeg.h>
#include <fvutils/writers/fvraw.h>
#include <fvutils/draw/drawer.h>

#include <core/exception.h>

#include <gdkmm/pixbuf.h>

#include <arpa/inet.h>

#include <iostream>

using namespace std;
using namespace fawkes;
using namespace firevision;

/** @class Firestation "firestation.h"
 * Control GUI for vision related stuff.
 * @author Daniel Beck
 */

/** Constructor.
 * @param ref_xml reference pointer to the glade file
 */
Firestation::Firestation(Glib::RefPtr<Gnome::Glade::Xml> ref_xml)
{
  // --- main window ------------------------------------------------
  m_wnd_main = dynamic_cast<Gtk::Window*>( get_widget(ref_xml, "wndMain") );

  m_img_image = dynamic_cast<Gtk::Image*>( get_widget(ref_xml, "imgImage") );
  m_img_image->signal_size_allocate().connect( sigc::mem_fun(*this, &Firestation::resize_image) );

  m_evt_image = dynamic_cast<Gtk::EventBox*>( get_widget(ref_xml, "evtImageEventBox") );
  m_evt_image->signal_button_press_event().connect( sigc::mem_fun(*this, &Firestation::image_click) );

  m_trv_shm_image_ids = dynamic_cast<Gtk::TreeView*>( get_widget(ref_xml, "trvShmImageIds") );

  m_stb_status = dynamic_cast<Gtk::Statusbar*>( get_widget(ref_xml, "stbStatus") );

  m_ckb_cont_trans = dynamic_cast<Gtk::CheckButton*>( get_widget(ref_xml, "ckbContTrans") );
  m_ckb_cont_trans->signal_toggled().connect( sigc::mem_fun(*this, &Firestation::enable_cont_img_trans) );

  m_spb_update_time = dynamic_cast<Gtk::SpinButton*>( get_widget(ref_xml, "spbUpdateTime") );
  // ----------------------------------------------------------------


  // --- toolbar widgets --------------------------------------------
  m_tbtn_exit = dynamic_cast<Gtk::ToolButton*>( get_widget(ref_xml, "tbtnExit") );
  m_tbtn_exit->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::exit) );

  m_tbtn_close_camera = dynamic_cast<Gtk::ToolButton*>( get_widget(ref_xml, "tbtnCloseCamera") );
  m_tbtn_close_camera->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::close_camera) );

  m_tbtn_update = dynamic_cast<Gtk::ToolButton*>( get_widget(ref_xml, "tbtnUpdate") );
  m_tbtn_update->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::update_image) );

  m_tbtn_save = dynamic_cast<Gtk::ToolButton*>( get_widget(ref_xml, "tbtnSave") );
  m_tbtn_save->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::save_image) );

  m_tbtn_open_file = dynamic_cast<Gtk::ToolButton*>( get_widget(ref_xml, "tbtnOpenFile") );
  m_tbtn_open_file->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::open_file) );

  m_tbtn_open_folder = dynamic_cast<Gtk::ToolButton*>( get_widget(ref_xml, "tbtnOpenFolder") );
  m_tbtn_open_folder->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::open_folder) );

  m_tbtn_open_shm = dynamic_cast<Gtk::ToolButton*>( get_widget(ref_xml, "tbtnOpenShm") );
  m_tbtn_open_shm->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::open_shm) );

  m_tbtn_open_fuse = dynamic_cast<Gtk::ToolButton*>( get_widget(ref_xml, "tbtnOpenFuse") );
  m_tbtn_open_fuse->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::open_fuse) );
  // ----------------------------------------------------------------


  // --- dialogs ----------------------------------------------------
  ref_xml->get_widget("fcdOpenImage", m_fcd_open_image);
  if ( !m_fcd_open_image )
    {
      throw std::runtime_error("Couldn't find fcdOpenImage.");
    }

  Gtk::FileFilter* filter_jpg = new Gtk::FileFilter();
  filter_jpg->set_name("JPEG");
  filter_jpg->add_pattern("*.jpg");
  filter_jpg->add_pattern("*.jpeg");
  m_fcd_open_image->add_filter(*filter_jpg);

  Gtk::FileFilter* filter_fvraw = new Gtk::FileFilter();
  filter_fvraw->set_name("FVRaw");
  filter_fvraw->add_pattern("*.raw");
  filter_fvraw->add_pattern("*.fvraw");
  m_fcd_open_image->add_filter(*filter_fvraw);

  ref_xml->get_widget("fcdSaveImage", m_fcd_save_image);
  if ( !m_fcd_save_image )
    {
      throw std::runtime_error("Couldn't find fcdSaveImage.");
    }
  m_fcd_save_image->add_filter(*filter_jpg);
  m_fcd_save_image->add_filter(*filter_fvraw);

  ref_xml->get_widget("dlgOpenShm", m_dlg_open_shm);
  if (!m_dlg_open_shm)
    {
      throw std::runtime_error("Couldn't find dlgOpenShm.");
    }

  ref_xml->get_widget("trvShmImageIds", m_trv_shm_image_ids);
  if ( !m_trv_shm_image_ids )
    {
      throw std::runtime_error("Couldn't find trvShmImageIds.");
    }
  m_shm_list_store = Gtk::ListStore::create(m_shm_columns);
  m_trv_shm_image_ids->set_model(m_shm_list_store);
  m_trv_shm_image_ids->append_column("#", m_shm_columns.m_id);
  m_trv_shm_image_ids->append_column("Name", m_shm_columns.m_name);


  ref_xml->get_widget("dlgOpenFuse", m_dlg_open_fuse);
  if (!m_dlg_open_fuse)
    {
      throw std::runtime_error("Couldn't find dlgOpenFuse.");
    }

  ref_xml->get_widget("ckbFuseJpeg", m_ckb_fuse_jpeg);
  if (! m_ckb_fuse_jpeg )
    {
      throw std::runtime_error("Couldn't find ckbFuseJpeg.");
    }

  ref_xml->get_widget("trvFuseServices", m_trv_fuse_services);
  if ( !m_trv_fuse_services )
    {
      throw std::runtime_error("Couldn't find trvFuseServices.");
    }
  m_fuse_tree_store = Gtk::TreeStore::create(m_fuse_columns);
  m_trv_fuse_services->set_model(m_fuse_tree_store);
  //  m_trv_fuse_services->append_column("#", m_fuse_columns.m_id);
  m_trv_fuse_services->append_column("Name", m_fuse_columns.m_name);
  // ----------------------------------------------------------------


  // --- color train widget -----------------------------------------
  m_ctw = new ColorTrainWidget(this);
  m_cmb_ct_type  = dynamic_cast<Gtk::ComboBox*>( get_widget(ref_xml, "cmbCtObjectType") );
  m_cmb_ct_type->signal_changed().connect(sigc::mem_fun(*this, &Firestation::ct_object_changed));
  m_cmb_ct_type->set_active(0);

  m_btn_ct_start = dynamic_cast<Gtk::ToggleButton*>( get_widget(ref_xml, "btnCtStart") );
  m_btn_ct_start->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::ct_start) );

  m_ctw->update_image().connect( sigc::mem_fun(*this, &Firestation::draw_image) );
  m_ctw->colormap_updated().connect( sigc::mem_fun(*this, &Firestation::on_colormap_updated) );

  Gtk::Button* btn;
  btn = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, "btnCtUnselect") );
  m_ctw->set_reset_selection_btn(btn);

  btn = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, "btnCtAdd") );
  m_ctw->set_add_to_colormap_btn(btn);

  btn = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, "btnCtReset") );
  m_ctw->set_reset_colormap_btn(btn);

  btn = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, "btnCtSaveHistos") );
  m_ctw->set_save_histos_btn(btn);

  btn = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, "btnCtLoadHistos") );
  m_ctw->set_load_histos_btn(btn);

  btn = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, "btnCtSaveColormap") );
  m_ctw->set_save_colormap_btn(btn);

  btn = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, "btnCtLoadColormap") );
  m_ctw->set_load_colormap_btn(btn);

  Gtk::Scale* scl;
  scl = dynamic_cast<Gtk::Scale*>( get_widget(ref_xml, "sclCtThreshold") );
  m_ctw->set_threshold_scl(scl);

  scl = dynamic_cast<Gtk::Scale*>( get_widget(ref_xml, "sclCtMinProb") );
  m_ctw->set_min_prob_scl(scl);

  scl = dynamic_cast<Gtk::Scale*>( get_widget(ref_xml, "sclCtLayerSelector") );
  m_ctw->set_cm_layer_selector(scl);

  Gtk::Image* img;
  img = dynamic_cast<Gtk::Image*>( get_widget(ref_xml, "imgCtSegmentation") );
  m_ctw->set_segmentation_img(img);

  img = dynamic_cast<Gtk::Image*>( get_widget(ref_xml, "imgCtColormap") );
  m_ctw->set_colormap_img(img);

  Gtk::FileChooserDialog* fcd;
  fcd = dynamic_cast<Gtk::FileChooserDialog*>( get_widget(ref_xml, "fcdFilechooser") );
  m_ctw->set_filechooser_dlg(fcd);


  m_btn_ct_seg = dynamic_cast<Gtk::ToggleButton*>( get_widget(ref_xml, "btnCtSeg") );
  m_btn_ct_seg->signal_toggled().connect( sigc::mem_fun(*this, &Firestation::draw_image) );
  m_spbtn_depth = dynamic_cast<Gtk::SpinButton*>( get_widget(ref_xml, "spbtnCtCmDepth") );
  m_spbtn_width = dynamic_cast<Gtk::SpinButton*>( get_widget(ref_xml, "spbtnCtCmWidth") );
  m_spbtn_height = dynamic_cast<Gtk::SpinButton*>( get_widget(ref_xml, "spbtnCtCmHeight") );
  m_ctw->set_cm_selector(m_spbtn_depth, m_spbtn_width, m_spbtn_height);
  // ----------------------------------------------------------------


  // --- mirror calibration -----------------------------------------
  m_calib_tool = new MirrorCalibTool();

#ifndef HAVE_BULB_CREATOR
  Gtk::Notebook *nb = dynamic_cast<Gtk::Notebook*>( get_widget(ref_xml, "ntbOptions") );
  Gtk::HBox *box = dynamic_cast<Gtk::HBox*>( get_widget(ref_xml, "boxMirrorCalib") );
  nb->get_tab_label(*box)->set_sensitive(false);
  box->set_sensitive(false);
#endif /* HAVE_BULB_CREATOR */

  m_btn_mc_start = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, "btnMcStart") );
  m_btn_mc_start->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_start) );

  m_btn_mc_load = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, "btnCalibLoad") );
  m_btn_mc_load->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_load) );

  m_btn_mc_save = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, "btnCalibSave") );
  m_btn_mc_save->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_save) );

  m_ent_mc_dist = dynamic_cast<Gtk::Entry*>( get_widget(ref_xml, "entCalibDist") );
  m_ent_mc_ori = dynamic_cast<Gtk::Entry*>( get_widget(ref_xml, "entCalibOri") );

  m_fcd_mc_save = dynamic_cast<Gtk::FileChooserDialog*>( get_widget(ref_xml, "fcdCalibSave") );
  m_fcd_mc_load = dynamic_cast<Gtk::FileChooserDialog*>( get_widget(ref_xml, "fcdCalibLoad") );
  // ----------------------------------------------------------------


  // --- fuse transfer widget ---------------------------------------
  m_ftw = new FuseTransferWidget();

  Gtk::TreeView* trv = dynamic_cast<Gtk::TreeView*>( get_widget(ref_xml, "trvFuseRemoteLuts") );
  m_ftw->set_remote_lut_list_trv(trv);
  trv = dynamic_cast<Gtk::TreeView*>( get_widget(ref_xml, "trvFuseLocalLuts") );
  m_ftw->set_local_lut_list_trv(trv);
  img = dynamic_cast<Gtk::Image*>( get_widget(ref_xml, "imgFuseLocal") );
  m_ftw->set_local_img(img);
  img = dynamic_cast<Gtk::Image*>( get_widget(ref_xml, "imgFuseRemote") );
  m_ftw->set_remote_img(img);
  btn = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, "btnFuseUpload") );
  m_ftw->set_upload_btn(btn);
  scl = dynamic_cast<Gtk::Scale*>( get_widget(ref_xml, "sclLocalLayerSelector") );
  m_ftw->set_local_layer_selector(scl);
  scl = dynamic_cast<Gtk::Scale*>( get_widget(ref_xml, "sclRemoteLayerSelector") );
  m_ftw->set_remote_layer_selector(scl);
  // ----------------------------------------------------------------


  // --- fuse image list widget -------------------------------------
  m_filw = new FuseImageListWidget();
  trv = dynamic_cast<Gtk::TreeView*>( get_widget(ref_xml, "trvFuseImageList") );
  m_filw->set_image_list_trv(trv);
  Gtk::CheckButton* chk = dynamic_cast<Gtk::CheckButton*>( get_widget(ref_xml, "chkFuseImageListUpdate") );
  m_filw->set_auto_update_chk(chk);
  chk = dynamic_cast<Gtk::CheckButton*>( get_widget(ref_xml, "chkFuseCompression") );
  m_filw->set_toggle_compression_chk(chk);
  m_filw->image_selected().connect( sigc::mem_fun(*this, &Firestation::on_fuse_image_selected) );
  // ----------------------------------------------------------------

  m_yuv_orig_buffer   = 0;
  m_yuv_draw_buffer   = 0;
  m_yuv_scaled_buffer = 0;
  m_rgb_scaled_buffer = 0;

  m_img_width  = 0;
  m_img_height = 0;
  m_img_size   = 0;
  m_img_cs     = CS_UNKNOWN;

  m_img_writer = 0;
  m_camera     = 0;
  m_shm_buffer = 0;

  m_img_src = SRC_NONE;
  m_op_mode = MODE_VIEWER;

  m_cont_img_trans = false;

  m_max_img_width  = m_evt_image->get_width();
  m_max_img_height = m_evt_image->get_height();
  m_scaled_img_width  = m_evt_image->get_width();
  m_scaled_img_height = m_evt_image->get_height();
  m_scale_factor = 1.0;

  m_avahi_thread = new AvahiThread();
  m_avahi_dispatcher = new AvahiDispatcher;
  
  m_avahi_dispatcher->signal_service_added().connect( sigc::mem_fun( *this, &Firestation::on_service_added ) );
  m_avahi_dispatcher->signal_service_removed().connect( sigc::mem_fun( *this, &Firestation::on_service_removed ) );

  m_avahi_thread->watch_service("_fountain._tcp", m_avahi_dispatcher);
  m_avahi_thread->start();
}

/** Destructor. */
Firestation::~Firestation()
{
  if (m_yuv_orig_buffer)    free(m_yuv_orig_buffer);
  if (m_yuv_draw_buffer)    free(m_yuv_draw_buffer);
  if (m_yuv_scaled_buffer)  free(m_yuv_scaled_buffer);
  if (m_rgb_scaled_buffer)  free(m_rgb_scaled_buffer);

  delete m_camera;
  delete m_img_writer;

  delete m_calib_tool;
  delete m_ctw;
  delete m_ftw;
  delete m_filw;

  m_avahi_thread->cancel();
  m_avahi_thread->join();
  delete m_avahi_thread;
  delete m_avahi_dispatcher;

  delete m_wnd_main;
  delete m_fcd_open_image;
  delete m_fcd_save_image;
  delete m_dlg_open_shm;
  delete m_dlg_open_fuse;
}

Gtk::Widget*
Firestation::get_widget(Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
			const char* widget_name) const
{
  Gtk::Widget* widget;
  ref_xml->get_widget(widget_name, widget);
  if ( !widget )
    {
      std::string err_str = "Couldn't find widget ";
      err_str += std::string(widget_name);
      err_str += ".";
      throw runtime_error(err_str);
    }

  return widget;
}

/** Returns reference to main window.
 * @return reference to main window
 */
Gtk::Window&
Firestation::get_window() const
{
  return *m_wnd_main;
}

/** Exit routine. */
void
Firestation::exit()
{
  if (SRC_NONE != m_img_src)
    { m_camera->close(); }

  m_wnd_main->hide();
}

void
Firestation::close_camera()
{
  if (SRC_NONE == m_img_src)
    { return; }

  m_img_src = SRC_NONE;

  m_camera->close();
  delete m_camera;
  m_camera = 0;

  m_img_width = 0;
  m_img_height = 0;
  m_img_cs = CS_UNKNOWN;

  m_img_size = 0;

  m_img_image->clear();
  m_img_image->set("gtk-missing-image");

  m_ctw->set_src_buffer(NULL, 0, 0);
  m_ctw->set_draw_buffer(NULL);
}

/** Saves the current image. */
void
Firestation::save_image()
{
  if (m_img_src == SRC_NONE)
    { return; }

  m_fcd_save_image->set_transient_for(*this);

  int result = m_fcd_save_image->run();

  switch(result)
    {
    case(Gtk::RESPONSE_OK):
      {
	delete m_img_writer;

	Glib::ustring filter_name = m_fcd_save_image->get_filter()->get_name();
	if ( Glib::ustring("JPEG") == filter_name )
	  {
	    m_img_writer = new JpegWriter();
	  }
	else if( Glib::ustring("FVRaw") == filter_name )
	  {
	    m_img_writer = new FvRawWriter();
	  }
	else
	  {
	    cout << "save_file(): unknown file format" << endl;
	    break;
	  }

	std::string filename = m_fcd_save_image->get_filename();
	m_img_writer->set_filename( filename.c_str() );
	m_img_writer->set_dimensions(m_img_width, m_img_height);
	m_img_writer->set_buffer(m_img_cs, m_yuv_orig_buffer);
	m_img_writer->write();

	std::cout << "Save file: " <<  filename << std::endl;
	break;
      }

    case(Gtk::RESPONSE_CANCEL):
      break;

    default:
      break;
    }

  m_fcd_save_image->hide();
}

/** Reads in a new image for the current image source. */
void
Firestation::update_image()
{
  if (m_img_src == SRC_NONE)
    { return; }

  try
    {
      m_camera->capture();
      convert(m_img_cs, YUV422_PLANAR,
	      m_camera->buffer(), m_yuv_orig_buffer,
	      m_img_width, m_img_height);
      memcpy(m_yuv_draw_buffer, m_yuv_orig_buffer,
	     colorspace_buffer_size(YUV422_PLANAR, m_img_width, m_img_height));
      m_camera->dispose_buffer();

      draw_image();

      m_ctw->draw_segmentation_result();
    }
  catch (Exception& e)
    {
      e.print_trace();
    }
}

bool
Firestation::call_update_image()
{
  if ( !m_cont_img_trans )
    { return false; }

  update_image();

  return true;
}

void
Firestation::enable_cont_img_trans()
{
  if (m_cont_img_trans)
    {
      m_cont_img_trans = false;
      return;
    }

  int timeout = (int) rint( m_spb_update_time->get_value() );
  sigc::connection conn = Glib::signal_timeout().connect( sigc::mem_fun(*this, &Firestation::call_update_image), timeout);
  m_cont_img_trans = true;
}

/** Reads in an image from a file. */
void
Firestation::open_file()
{
  m_fcd_open_image->set_action(Gtk::FILE_CHOOSER_ACTION_OPEN);
  m_fcd_open_image->set_transient_for(*this);

  int result = m_fcd_open_image->run();

  switch(result)
    {
    case Gtk::RESPONSE_OK:
      {
	pre_open_img_src();

	std::string filename = m_fcd_open_image->get_filename();

	m_camera = new FileLoader( filename.c_str() );
	m_img_src = SRC_FILE;
	post_open_img_src();

	break;
      }

    case Gtk::RESPONSE_CANCEL:
      {
	break;
      }

    default:
      {
	break;
      }
    }

  m_fcd_open_image->hide();
}

/** Reads in images from a directory. */
void
Firestation::open_folder()
{
  m_fcd_open_image->set_action(Gtk::FILE_CHOOSER_ACTION_SELECT_FOLDER);
  m_fcd_open_image->set_transient_for(*this);

  int result = m_fcd_open_image->run();

  switch(result)
    {
    case Gtk::RESPONSE_OK:
      {
	pre_open_img_src();

	std::string extension;
	Glib::ustring filter_name = m_fcd_save_image->get_filter()->get_name();
	if ( Glib::ustring("JPEG") == filter_name )
	  { extension = "jpg"; }
	else if ( Glib::ustring("FVRaw") == filter_name )
	  { extension = "raw"; }

	std::string folder = m_fcd_open_image->get_current_folder();
	char* as;
	if (asprintf(&as, "file:file:dir=%s:ext=%s", folder.c_str(), extension.c_str()) != -1) {
	  CameraArgumentParser cap(as);
	  m_camera = new FileLoader( &cap );
	  m_img_src = SRC_FILE;
	  post_open_img_src();
	  free(as);
	} else {
	  printf("Cannot open folder, asprintf() ran out of memory");
	}

	break;
      }

    case Gtk::RESPONSE_CANCEL:
      {
	break;
      }

    default:
      {
	break;
      }
    }

  m_fcd_open_image->hide();
}

/** Opens a SHM image. */
void
Firestation::open_shm()
{
  unsigned int num_buffers = 0;
  SharedMemory::SharedMemoryIterator shmit;
  SharedMemoryImageBufferHeader* h = new SharedMemoryImageBufferHeader;
  shmit = SharedMemory::find(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h);

  if (shmit == SharedMemory::end())
    {
      m_stb_status->push("No SHM images found");
      return;
    }
  else
    {
      m_shm_list_store->clear();

      while ( shmit != SharedMemory::end() )
	{
	  ++num_buffers;
	  Gtk::TreeModel::Row row = *(m_shm_list_store->append());
	  row[m_shm_columns.m_id] = num_buffers;
	  const SharedMemoryImageBufferHeader* h = (SharedMemoryImageBufferHeader*)*shmit;
	  row[m_shm_columns.m_name] = h->image_id();
	  shmit++;
	}
    }

  m_dlg_open_shm->set_transient_for(*this);

  int result = m_dlg_open_shm->run();

  switch(result)
    {
    case Gtk::RESPONSE_OK:
      {
	delete m_shm_buffer;

	Gtk::TreeModel::Path path;
	Gtk::TreeViewColumn* column;
	m_trv_shm_image_ids->get_cursor(path, column);

	Gtk::TreeModel::iterator iter = m_shm_list_store->get_iter(path);

	if (iter)
	  {
	    Gtk::TreeModel::Row row = *iter;
	    if (row)
	      {
		Glib::ustring name = row[m_shm_columns.m_name];
		pre_open_img_src();

		try
		  {
		    m_camera = new SharedMemoryCamera( name.c_str() );
		  }
		catch (Exception& e)
		  {
		    e.print_trace();
		  }

		m_img_src = SRC_SHM;

		post_open_img_src();
	      }
	  }
	else
	  {
	    std::cout << "invalid iter" << std::endl;
	  }

	break;
      }

    case Gtk::RESPONSE_CANCEL:
      break;

    default:
      break;
    }

  m_dlg_open_shm->hide();
}

/** Connects to a FUSE server. */
void
Firestation::open_fuse()
{
  Gtk::TreeModel::Children children = m_fuse_tree_store->children();
  if ( 0 == children.size() )
    {
      m_stb_status->push("No FUSE services found");
      return;
    }

  m_trv_fuse_services->expand_all();
  m_dlg_open_fuse->set_transient_for(*this);

  int result = m_dlg_open_fuse->run();

  switch(result)
    {
    case Gtk::RESPONSE_OK:
      {
	Gtk::TreeModel::Path path;
	Gtk::TreeViewColumn* column;
	m_trv_fuse_services->get_cursor(path, column);

	Gtk::TreeModel::iterator iter = m_fuse_tree_store->get_iter(path);

	if (iter)
	  {
	    Gtk::TreeModel::Row row = *iter;
	    if (row)
	      {
 		Glib::ustring hostname = row[m_fuse_columns.m_service_hostname];
 		unsigned short int port = row[m_fuse_columns.m_service_port];
 		Glib::ustring image_id = row[m_fuse_columns.m_image_id];
 		bool jpeg = m_ckb_fuse_jpeg->get_active();

		pre_open_img_src();

		try
		  {
 		    m_camera = new NetworkCamera(hostname.c_str(), port, image_id.c_str(), jpeg);
 		    m_img_src = SRC_FUSE;
 		    post_open_img_src();
		  }
		catch (Exception& e)
		  {
		    m_img_src = SRC_NONE;
		    e.print_trace();
		  }
	      }
	  }
	else
	  {
	    std::cout << "invalid iter" << std::endl;
	  }

	break;
      }

    case Gtk::RESPONSE_CANCEL:
      break;

    default:
      break;
    }

  m_dlg_open_fuse->hide();
}

void
Firestation::pre_open_img_src()
{
  if (SRC_NONE != m_img_src)
    {
      m_camera->stop();
      m_camera->close();

      delete m_camera;
      m_camera = 0;

      m_img_src = SRC_NONE;
    }
}

/** Stuff that is executed after an image source has been selected. */
void
Firestation::post_open_img_src()
{
  if (m_img_src == SRC_NONE) { return; }

  try
    {
      m_camera->open();
      m_camera->start();
      m_camera->capture();
      m_img_width = m_camera->pixel_width();
      m_img_height = m_camera->pixel_height();
      m_img_cs = m_camera->colorspace();

      m_img_size = colorspace_buffer_size( m_img_cs,
					   m_img_width,
					   m_img_height );

      m_yuv_orig_buffer = malloc_buffer(YUV422_PLANAR, m_img_width, m_img_height);
      m_yuv_draw_buffer = malloc_buffer(YUV422_PLANAR, m_img_width, m_img_height);

      convert(m_img_cs, YUV422_PLANAR,
	      m_camera->buffer(), m_yuv_orig_buffer,
	      m_img_width, m_img_height);
      memcpy(m_yuv_draw_buffer, m_yuv_orig_buffer,
	     colorspace_buffer_size(YUV422_PLANAR, m_img_width, m_img_height));

      m_camera->dispose_buffer();

      m_tbtn_update->set_sensitive(true);
      m_tbtn_save->set_sensitive(true);

      draw_image();

      m_ctw->set_src_buffer(m_yuv_orig_buffer, m_img_width, m_img_height);
      m_ctw->set_draw_buffer(m_yuv_draw_buffer);
      m_ctw->draw_segmentation_result();
    }
  catch (Exception& e)
    {
      e.print_trace();
      printf("Opening camera failed.\n");
    }

}

void
Firestation::on_fuse_image_selected()
{
  string host_name;
  unsigned short port;
  string image_id;
  bool compression;

  m_filw->get_selected_image(host_name, port, image_id, compression);

  pre_open_img_src();

  try
    {
      m_camera = new NetworkCamera( host_name.c_str(), port, image_id.c_str(), compression );
      m_img_src = SRC_FUSE;
    }
  catch (Exception& e)
    {
      m_img_src = SRC_NONE;
      e.print_trace();
    }

  post_open_img_src();
}

void
Firestation::on_colormap_updated()
{
  m_ftw->set_current_colormap( m_ctw->get_colormap() );
}

/** Draws the image. */
void
Firestation::draw_image()
{
  if ( m_img_src == SRC_NONE ) { return; }

  LossyScaler scaler;
  scaler.set_original_buffer( m_yuv_draw_buffer );
  scaler.set_original_dimensions(m_img_width, m_img_height);
  scaler.set_scaled_dimensions(m_max_img_width, m_max_img_height);

  unsigned int scaled_width  = scaler.needed_scaled_width();
  unsigned int scaled_height = scaler.needed_scaled_height();

  if (scaled_width != m_scaled_img_width || scaled_height != m_scaled_img_height)
    {
      m_scaled_img_width  = scaled_width;
      m_scaled_img_height = scaled_height;
      m_scale_factor = scaler.get_scale_factor();
    }

  if (m_rgb_scaled_buffer)  free(m_rgb_scaled_buffer);
  if (m_yuv_scaled_buffer)  free(m_yuv_scaled_buffer);
  m_yuv_scaled_buffer = malloc_buffer(YUV422_PLANAR, m_scaled_img_width,
				      m_scaled_img_height);
  scaler.set_scaled_buffer(m_yuv_scaled_buffer);
  scaler.scale();

  if (m_btn_ct_seg->get_active()) {
    unsigned int sld_img_size = m_scaled_img_width * m_scaled_img_height;
    unsigned char u_seg = 255 / (unsigned int)pow(2, m_spbtn_width->get_value());
    unsigned char v_seg = 255 / (unsigned int)pow(2, m_spbtn_height->get_value());
    unsigned int u = 0;
    for (u = sld_img_size; u < sld_img_size + sld_img_size / 2; ++u) {
      m_yuv_scaled_buffer[u] = (m_yuv_scaled_buffer[u] / u_seg) * u_seg;
    }

    for (; u < 2 * sld_img_size; ++u) {
      m_yuv_scaled_buffer[u] = (m_yuv_scaled_buffer[u] / v_seg) * v_seg;
    }
  }

  if ( m_img_src == SRC_SHM )
    {
      SharedMemoryCamera* shm_camera = dynamic_cast<SharedMemoryCamera*>(m_camera);
      if ( shm_camera->shared_memory_image_buffer()->circle_found() )
	{
	  Drawer drawer;
	  drawer.set_buffer(m_yuv_scaled_buffer, m_scaled_img_width, m_scaled_img_height);
	  drawer.set_color(YUV_t::white());
	  unsigned int roi_x = (unsigned int) rint( shm_camera->shared_memory_image_buffer()->roi_x() * m_scale_factor );
	  unsigned int roi_y = (unsigned int) rint( shm_camera->shared_memory_image_buffer()->roi_y() * m_scale_factor );
	  unsigned int roi_width  = (unsigned int) rint( shm_camera->shared_memory_image_buffer()->roi_width() * m_scale_factor );
	  unsigned int roi_height = (unsigned int) rint( shm_camera->shared_memory_image_buffer()->roi_height() * m_scale_factor );
	  drawer.draw_rectangle( roi_x, roi_y, roi_width, roi_height );
	}
    }

  m_rgb_scaled_buffer = (unsigned char*) malloc( colorspace_buffer_size( RGB,
									 m_scaled_img_width,
									 m_scaled_img_height ) );

  convert( YUV422_PLANAR, RGB,
	   m_yuv_scaled_buffer, m_rgb_scaled_buffer,
	   m_scaled_img_width, m_scaled_img_height );

  Glib::RefPtr<Gdk::Pixbuf> image = Gdk::Pixbuf::create_from_data( m_rgb_scaled_buffer,
								   Gdk::COLORSPACE_RGB,
								   false,
								   8,
								   m_scaled_img_width,
								   m_scaled_img_height,
								   3 * m_scaled_img_width );

  m_img_image->set(image);
}

/** Signal handler that is called whenever the window size is changed.
 * @param allocation a Gtk allocation
 */
void
Firestation::resize_image(Gtk::Allocation& allocation)
{
  unsigned int new_width = (unsigned int) allocation.get_width();
  unsigned int new_height = (unsigned int) allocation.get_height();

  if (new_width != m_max_img_width ||  new_height != m_max_img_height)
    {
      m_max_img_width = new_width;
      m_max_img_height = new_height;
      draw_image();
    }
}

/** Handles mouse clicks in the image area.
 * @param event a Gtk event
 * @return true if signal was handled
 */
bool
Firestation::image_click(GdkEventButton* event)
{
  unsigned int offset_x;
  unsigned int offset_y;

  offset_x = (m_max_img_width - m_scaled_img_width) / 2;
  offset_y = (m_max_img_height - m_scaled_img_height) / 2;

  offset_x = offset_x > m_max_img_width ? 0 : offset_x;
  offset_y = offset_y > m_max_img_height ? 0 : offset_y;

  unsigned int image_x;
  unsigned int image_y;

  image_x = (unsigned int)rint( (event->x - offset_x) / m_scale_factor);
  image_y = (unsigned int)rint( (event->y - offset_y) / m_scale_factor);

  if ( image_x < 0 || image_x > m_img_width ||
       image_y < 0 || image_y > m_img_height )
    { return true; }

  switch (m_op_mode)
    {
    case MODE_VIEWER:
      if (m_img_src != SRC_NONE)
	{
	  register unsigned char y;
	  register unsigned char u;
	  register unsigned char v;
	  YUV422_PLANAR_YUV( m_yuv_orig_buffer,
			     m_img_width,
			     m_img_height,
			     image_x,
			     image_y,
			     y, u, v );
	  printf( "Y=%d  U=%d  Y=%d @ (%d, %d)\n",
		  (unsigned int) y, (unsigned int) u, (unsigned int) v,
		  image_x, image_y );
	}
      break;

    case MODE_COLOR_TRAIN:
      m_ctw->click(image_x, image_y, event->button);
      draw_image();
     break;

    case MODE_MIRROR_CALIB:
      {
	m_calib_tool->step(image_x, image_y);

	bool show;
	float next_dist;
	float next_ori;
	show = m_calib_tool->get_next(&next_dist, &next_ori);

	if (show)
	  {
	    char* next_dist_char = (char*) malloc(10);
	    char* next_ori_char = (char*) malloc(10);

	    sprintf(next_dist_char, "%2f", next_dist);
	    sprintf(next_ori_char, "%2f", next_ori);
	    m_ent_mc_dist->set_text(Glib::ustring(next_dist_char));
	    m_ent_mc_ori->set_text(Glib::ustring(next_ori_char));

	    free(next_dist_char);
	    free(next_ori_char);
	  }
	else
	  {
	    m_ent_mc_dist->set_text("");
	    m_ent_mc_ori->set_text("");
	  }

	break;
      }

    case MODE_MIRROR_CALIB_EVAL:
      {
	float dist;
	float phi;
	m_calib_tool->eval(image_x, image_y, &dist, &phi);
	printf("Distance: %2f\t Phi: %2f\n", dist, phi);
	break;
      }

    default:
      break;
    }

  return true;
}

/** Starts the color training. */
void
Firestation::ct_start()
{
  if (m_op_mode == MODE_COLOR_TRAIN)
    {
      m_op_mode = MODE_VIEWER;
      m_stb_status->push("Leaving color training mode");
    }
  else
    {
      if (m_img_src != SRC_NONE)
	{
	  m_ctw->set_fg_object( ct_get_fg_object() );

	  m_op_mode = MODE_COLOR_TRAIN;

	  m_stb_status->push("Entering color training mode");
	}
    }
}

hint_t
Firestation::ct_get_fg_object()
{
	int active = m_cmb_ct_type->get_active_row_number();
	switch(active)
	{
		case 0: //Ball
			return H_BALL;

		case 1: //Field
			return H_FIELD;

		case 2: //Lines
			return H_LINE;

		case 3: //Robot (Team A or all)
			return H_ROBOT;

		case 4: //Robot (Team B)
			return H_ROBOT_OPP;

		case 5: //Goal (yellow)
			return H_GOAL_YELLOW;

                case 6: //Goal (sky-blue)
                        return H_GOAL_BLUE;

                case 7: //Background
                        return H_UNKNOWN;

		default:
			printf("ct_get_fg_object(): UNKNOWN\n");
			return H_UNKNOWN;
	}
}

void
Firestation::ct_object_changed()
{
	hint_t object = ct_get_fg_object();
	m_ctw->set_fg_object(object);
}

/** Start the mirror calibration process. */
void
Firestation::mc_start()
{
  if (m_op_mode == MODE_MIRROR_CALIB)
    {
      m_op_mode = MODE_VIEWER;
      m_stb_status->push("Leaving mirror calibration mode");
    }
  else
    {
      if (m_img_src != SRC_NONE)
	{
	  m_calib_tool->set_img_dimensions(m_img_width, m_img_height);
	  m_calib_tool->start();

	  m_op_mode = MODE_MIRROR_CALIB;

	  bool show;
	  float next_dist;
	  float next_ori;
	  show = m_calib_tool->get_next(&next_dist, &next_ori);

	  if (show)
	    {
	      char* next_dist_char = (char*) malloc(10);
	      char* next_ori_char = (char*) malloc(10);

	      sprintf(next_dist_char, "%2f", next_dist);
	      sprintf(next_ori_char, "%2f", next_ori);
	      m_ent_mc_dist->set_text(Glib::ustring(next_dist_char));
	      m_ent_mc_ori->set_text(Glib::ustring(next_ori_char));

	      free(next_dist_char);
	      free(next_ori_char);
	    }
	  else
	    {
	      m_ent_mc_dist->set_text("");
	      m_ent_mc_ori->set_text("");
	    }

	  m_stb_status->push("Entering mirror calibration mode");
	}
    }
}

/** Load mirror calibration data from a file. */
void
Firestation::mc_load()
{
  m_fcd_mc_load->set_transient_for(*this);

  Gtk::FileFilter filter_mirror;
  filter_mirror.set_name("Mirror Calibration");
  filter_mirror.add_pattern("*.mirror");
  filter_mirror.add_pattern("*.bulb");
  m_fcd_mc_load->add_filter(filter_mirror);

  int result = m_fcd_mc_load->run();

  switch(result)
    {
    case Gtk::RESPONSE_OK:
      {
	std::string filename = m_fcd_mc_load->get_filename();
	m_calib_tool->load( filename.c_str() );
	m_op_mode = MODE_MIRROR_CALIB_EVAL;
	break;
      }
    case Gtk::RESPONSE_CANCEL:
      break;
    default:
      break;
    }

  m_fcd_mc_load->hide();
}

/** Save calibration data to a file. */
void
Firestation::mc_save()
{
  m_fcd_mc_save->set_transient_for(*this);

  int result = m_fcd_mc_save->run();

  switch(result)
    {
    case(Gtk::RESPONSE_OK):
      {
	std::string filename = m_fcd_mc_save->get_filename();

	m_calib_tool->save( filename.c_str() );
	break;
      }

    case(Gtk::RESPONSE_CANCEL):
      break;

    default:
      break;
    }

  m_fcd_mc_save->hide();

}

void
Firestation::on_service_added( NetworkService* service )
{
  const char* host        = service->host();
  const char* name        = service->name();
  const char* type        = service->type();
  const char* domain      = service->domain();
  unsigned short int port = service->port();
  
  std::vector<FUSE_imageinfo_t> image_list;
  NetworkCamera cam(host, port);
  try
    {
      cam.open();
      cam.start();
      image_list = cam.image_list();
    }
  catch (Exception& e)
    {
      e.append("Could not open camera on %s:%d", host, port);
      e.print_trace();
      return;
    }
  cam.close();

#ifdef DEBUG_PRINT
  printf("%zu images available on host %s.\n", image_list.size(), host);
#endif /* DEBUG_PRINT */

  std::vector<FUSE_imageinfo_t>::iterator fit;

  Gtk::TreeModel::Children children = m_fuse_tree_store->children();
  Gtk::TreeModel::Row row = *(m_fuse_tree_store->append());
  row[m_fuse_columns.m_id] = children.size();
  row[m_fuse_columns.m_name] = Glib::ustring(name);
  row[m_fuse_columns.m_service_name] = Glib::ustring(name);
  row[m_fuse_columns.m_service_type] = Glib::ustring(type);
  row[m_fuse_columns.m_service_domain] = Glib::ustring(domain);
  row[m_fuse_columns.m_service_hostname] = Glib::ustring(host);
  row[m_fuse_columns.m_service_port] = port;

  for (fit = image_list.begin(); fit != image_list.end(); ++fit)
    {
      Gtk::TreeModel::Row childrow = *(m_fuse_tree_store->append(row.children()));
      childrow[m_fuse_columns.m_name] = Glib::ustring(fit->image_id);
      childrow[m_fuse_columns.m_service_name] = Glib::ustring(name);
      childrow[m_fuse_columns.m_service_type] = Glib::ustring(type);
      childrow[m_fuse_columns.m_service_domain] = Glib::ustring(domain);
      childrow[m_fuse_columns.m_service_hostname] = Glib::ustring(host);
      childrow[m_fuse_columns.m_service_port] = port;
      childrow[m_fuse_columns.m_image_id] = Glib::ustring(fit->image_id);
      childrow[m_fuse_columns.m_image_width] = fit->width;
      childrow[m_fuse_columns.m_image_height] = fit->height;
      childrow[m_fuse_columns.m_image_colorspace] = Glib::ustring( colorspace_to_string((colorspace_t) fit->colorspace) );
    }

  m_ftw->add_fountain_service(name, host, port);
  m_filw->add_fountain_service(name, host, port);
}

void
Firestation::on_service_removed( NetworkService* service )
{
  const char* name   = service->name();
  const char* type   = service->type();
  const char* domain = service->domain();

  Gtk::TreeModel::Children children = m_fuse_tree_store->children();
  Gtk::TreeModel::iterator rit;
  for (rit = children.begin(); rit != children.end(); ++rit)
    {
      Glib::ustring n = (*rit)[m_fuse_columns.m_service_name];
      Glib::ustring t = (*rit)[m_fuse_columns.m_service_type];
      Glib::ustring d = (*rit)[m_fuse_columns.m_service_domain];

      if ( strcmp( n.c_str(), name) == 0 &&
	   strcmp( t.c_str(), type) == 0 &&
	   strcmp( d.c_str(), domain) == 0 )
	{
	  m_fuse_tree_store->erase(rit);
	}
    }

  m_ftw->remove_fountain_service(name);
  m_filw->remove_fountain_service(name);
}
