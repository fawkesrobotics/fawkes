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
#include "color_train_widget.h"
#include "fuse_transfer_widget.h"

#include <utils/math/angle.h>

#include <fvwidgets/fuse_image_list_widget.h>
#include <gui_utils/avahi_dispatcher.h>

#include <fvcams/fileloader.h>
#include <fvcams/shmem.h>
#include <fvcams/net.h>

#ifdef HAVE_MIRROR_CALIB
#include <fvmodels/mirror/mirror_calib.h>
#endif

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
 * @param builder Gtk Builder
 */
Firestation::Firestation(Glib::RefPtr<Gtk::Builder> builder)
{
  // --- main window ------------------------------------------------
  builder->get_widget("wndMain", m_wnd_main);
  builder->get_widget("imgImage", m_img_image);
  builder->get_widget("evtImageEventBox", m_evt_image);
  builder->get_widget("trvShmImageIds", m_trv_shm_image_ids);
  builder->get_widget("stbStatus", m_stb_status);
  builder->get_widget("ckbContTrans", m_ckb_cont_trans);
  builder->get_widget("spbUpdateTime", m_spb_update_time);

  m_img_image->signal_size_allocate().connect( sigc::mem_fun(*this, &Firestation::resize_image) );
  m_evt_image->signal_button_press_event().connect( sigc::mem_fun(*this, &Firestation::image_click) );
  m_ckb_cont_trans->signal_toggled().connect( sigc::mem_fun(*this, &Firestation::enable_cont_img_trans) );

  // ----------------------------------------------------------------


  // --- toolbar widgets --------------------------------------------
  builder->get_widget("tbtnExit", m_tbtn_exit);
  builder->get_widget("tbtnCloseCamera", m_tbtn_close_camera);
  builder->get_widget("tbtnUpdate", m_tbtn_update);
  builder->get_widget("tbtnSave", m_tbtn_save);
  builder->get_widget("tbtnOpenFile", m_tbtn_open_file);
  builder->get_widget("tbtnOpenFolder", m_tbtn_open_folder);
  builder->get_widget("tbtnOpenShm", m_tbtn_open_shm);
  builder->get_widget("tbtnOpenFuse", m_tbtn_open_fuse);

  m_tbtn_exit->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::exit) );
  m_tbtn_close_camera->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::close_camera) );
  m_tbtn_update->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::update_image) );
  m_tbtn_save->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::save_image) );
  m_tbtn_open_file->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::open_file) );
  m_tbtn_open_folder->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::open_folder) );
  m_tbtn_open_shm->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::open_shm) );
  m_tbtn_open_fuse->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::open_fuse) );
  // ----------------------------------------------------------------


  // --- dialogs ----------------------------------------------------
  builder->get_widget("fcdOpenImage", m_fcd_open_image);
  builder->get_widget("fcdSaveImage", m_fcd_save_image);
  builder->get_widget("dlgOpenShm", m_dlg_open_shm);
  builder->get_widget("trvShmImageIds", m_trv_shm_image_ids);
  builder->get_widget("dlgOpenFuse", m_dlg_open_fuse);
  builder->get_widget("ckbFuseJpeg", m_ckb_fuse_jpeg);
  builder->get_widget("trvFuseServices", m_trv_fuse_services);

#if GTK_VERSION_GE(3,0)
  Glib::RefPtr<Gtk::FileFilter> filter_jpg = Gtk::FileFilter::create();
#else
  Gtk::FileFilter *filter_jpg = Gtk::manage(new Gtk::FileFilter());
#endif
  filter_jpg->set_name("JPEG");
  filter_jpg->add_pattern("*.jpg");
  filter_jpg->add_pattern("*.jpeg");

#if GTK_VERSION_GE(3,0)
  Glib::RefPtr<Gtk::FileFilter> filter_fvraw = Gtk::FileFilter::create();
#else
  Gtk::FileFilter *filter_fvraw = Gtk::manage(new Gtk::FileFilter());
#endif
  filter_fvraw->set_name("FVRaw");
  filter_fvraw->add_pattern("*.raw");
  filter_fvraw->add_pattern("*.fvraw");

#if GTK_VERSION_GE(3,0)
  m_fcd_open_image->add_filter(filter_jpg);
  m_fcd_open_image->add_filter(filter_fvraw);

  m_fcd_save_image->add_filter(filter_jpg);
  m_fcd_save_image->add_filter(filter_fvraw);

#else
  m_fcd_open_image->add_filter(*filter_jpg);
  m_fcd_open_image->add_filter(*filter_fvraw);

  m_fcd_save_image->add_filter(*filter_jpg);
  m_fcd_save_image->add_filter(*filter_fvraw);
#endif

  m_shm_list_store = Gtk::ListStore::create(m_shm_columns);
  m_trv_shm_image_ids->set_model(m_shm_list_store);
  m_trv_shm_image_ids->append_column("#", m_shm_columns.m_id);
  m_trv_shm_image_ids->append_column("Name", m_shm_columns.m_name);

  m_fuse_tree_store = Gtk::TreeStore::create(m_fuse_columns);
  m_trv_fuse_services->set_model(m_fuse_tree_store);
  //  m_trv_fuse_services->append_column("#", m_fuse_columns.m_id);
  m_trv_fuse_services->append_column("Name", m_fuse_columns.m_name);
  // ----------------------------------------------------------------


  // --- color train widget -----------------------------------------
  m_ctw = new ColorTrainWidget(this);
  builder->get_widget("cmbCtObjectType", m_cmb_ct_type);
  builder->get_widget("btnCtStart", m_btn_ct_start);
  builder->get_widget("btnCtSeg", m_btn_ct_seg);
  builder->get_widget("spbtnCtCmDepth", m_spbtn_depth);
  builder->get_widget("spbtnCtCmWidth", m_spbtn_width);
  builder->get_widget("spbtnCtCmHeight", m_spbtn_height);

  m_cmb_ct_type->signal_changed().connect(sigc::mem_fun(*this, &Firestation::ct_object_changed));
  m_cmb_ct_type->set_active(0);

  m_btn_ct_start->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::ct_start) );

  m_ctw->update_image().connect( sigc::mem_fun(*this, &Firestation::draw_image) );
  m_ctw->colormap_updated().connect( sigc::mem_fun(*this, &Firestation::on_colormap_updated) );

  Gtk::Button* btn;
  builder->get_widget("btnCtUnselect", btn);
  m_ctw->set_reset_selection_btn(btn);

  builder->get_widget("btnCtAdd", btn);
  m_ctw->set_add_to_colormap_btn(btn);

  builder->get_widget("btnCtReset", btn);
  m_ctw->set_reset_colormap_btn(btn);

  builder->get_widget("btnCtSaveHistos", btn);
  m_ctw->set_save_histos_btn(btn);

  builder->get_widget("btnCtLoadHistos", btn);
  m_ctw->set_load_histos_btn(btn);

  builder->get_widget("btnCtSaveColormap", btn);
  m_ctw->set_save_colormap_btn(btn);

  builder->get_widget("btnCtLoadColormap", btn);
  m_ctw->set_load_colormap_btn(btn);

  Gtk::Scale* scl;
  builder->get_widget("sclCtThreshold", scl);
  m_ctw->set_threshold_scl(scl);

  builder->get_widget("sclCtMinProb", scl);
  m_ctw->set_min_prob_scl(scl);

  builder->get_widget("sclCtLayerSelector", scl);
  m_ctw->set_cm_layer_selector(scl);

  Gtk::Image* img;
  builder->get_widget("imgCtSegmentation", img);
  m_ctw->set_segmentation_img(img);

  builder->get_widget("imgCtColormap", img);
  m_ctw->set_colormap_img(img);

  Gtk::FileChooserDialog* fcd;
  builder->get_widget("fcdFilechooser", fcd);
  m_ctw->set_filechooser_dlg(fcd);


  m_btn_ct_seg->signal_toggled().connect( sigc::mem_fun(*this, &Firestation::draw_image) );
  m_ctw->set_cm_selector(m_spbtn_depth, m_spbtn_width, m_spbtn_height);
  // ----------------------------------------------------------------


  // --- mirror calibration -----------------------------------------
#ifdef HAVE_MIRROR_CALIB
  m_calib_tool = new firevision::MirrorCalibTool();
#endif

#ifndef HAVE_MIRROR_CALIB
  Gtk::Notebook *nb;
  Gtk::HBox *box;
  builder->get_widget("ntbOptions", nb);
  builder->get_widget("boxMirrorCalib", box);
  nb->get_tab_label(*box)->set_sensitive(false);
  box->set_sensitive(false);
#endif

  builder->get_widget("sclMcLine", m_scl_mc_line);
  m_scl_mc_line->signal_change_value().connect( sigc::mem_fun(*this, &Firestation::mc_on_line_angle_changed) );

  builder->get_widget("btnMcLoadMask", m_btn_mc_load_mask);
  m_btn_mc_load_mask->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_load_mask) );

  //builder->get_widget("btnMcStart", m_btn_mc_start);
  builder->get_widget("btnCalibLoad", m_btn_mc_load);
  builder->get_widget("btnCalibSave", m_btn_mc_save);
  builder->get_widget("entCalibDist", m_ent_mc_dist);
  builder->get_widget("entCalibOri", m_ent_mc_ori);
  builder->get_widget("fcdCalibSave", m_fcd_mc_save);
  builder->get_widget("fcdCalibLoad", m_fcd_mc_load);

  //m_btn_mc_start->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_start) );
  m_btn_mc_load->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_load) );
  m_btn_mc_save->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_save) );

  builder->get_widget("btnMcSetCenter", m_btn_mc_set_center);
  m_btn_mc_set_center->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_set_center) );

  builder->get_widget("btnMcMemorize", m_btn_mc_memorize);
  m_btn_mc_memorize->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_memorize) );

  builder->get_widget("btnMcSimulateClicks", m_btn_mc_simulate_clicks);
  m_btn_mc_simulate_clicks->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_simulate_clicks) );

  builder->get_widget("entCalibDist", m_ent_mc_dist);
  builder->get_widget("entCalibOri", m_ent_mc_ori);

  // ----------------------------------------------------------------
  builder->get_widget("fcdMcLoadMask", m_fcd_mc_load_mask);
  builder->get_widget("fcdCalibSave", m_fcd_mc_save);
  builder->get_widget("fcdCalibLoad", m_fcd_mc_load);
  // ----------------------------------------------------------------


  // --- fuse transfer widget ---------------------------------------
  m_ftw = new FuseTransferWidget();

  Gtk::TreeView* trv;
  builder->get_widget("trvFuseRemoteLuts", trv);
  m_ftw->set_remote_lut_list_trv(trv);
  builder->get_widget("trvFuseLocalLuts", trv);
  m_ftw->set_local_lut_list_trv(trv);
  builder->get_widget("imgFuseLocal", img);
  m_ftw->set_local_img(img);
  builder->get_widget("imgFuseRemote", img);
  m_ftw->set_remote_img(img);
  builder->get_widget("btnFuseUpload", btn);
  m_ftw->set_upload_btn(btn);
  builder->get_widget("sclLocalLayerSelector", scl);
  m_ftw->set_local_layer_selector(scl);
  builder->get_widget("sclRemoteLayerSelector", scl);
  m_ftw->set_remote_layer_selector(scl);
  // ----------------------------------------------------------------


  // --- fuse image list widget -------------------------------------
  m_filw = new FuseImageListWidget();
  builder->get_widget("trvFuseImageList", trv);
  m_filw->set_image_list_trv(trv);
  Gtk::CheckButton* chk;
  builder->get_widget("chkFuseImageListUpdate", chk);
  m_filw->set_auto_update_chk(chk);
  builder->get_widget("chkFuseCompression", chk);
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

  mc_line_angle_deg = 0.0;

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

#ifdef HAVE_MIRROR_CALIB
  delete m_calib_tool;
#endif
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

      mc_draw_line();
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

  if (event != NULL) {
    image_x = (unsigned int)rint( (event->x - offset_x) / m_scale_factor);
    image_y = (unsigned int)rint( (event->y - offset_y) / m_scale_factor);
  } else {
    image_x = 0;
    image_y = 0;
  }

  if ( image_x > m_img_width || image_y > m_img_height )
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
#ifdef HAVE_MIRROR_CALIB
        if (m_btn_mc_set_center->get_active()) {
          m_calib_tool->set_center(image_x, image_y);
          m_btn_mc_set_center->set_active(false);
          mc_draw_line();
          printf("Setting center to %d, %d\n", image_x, image_y);
        } else {
          printf("Using center to %d, %d\n", m_calib_tool->center_x(), m_calib_tool->center_y());
          m_calib_tool->next_step();
          const unsigned char* last_yuv = m_calib_tool->get_last_yuv_buffer();
          memcpy(m_yuv_draw_buffer, last_yuv, m_img_size);
          memcpy(m_yuv_orig_buffer, last_yuv, m_img_size);
          m_calib_tool->draw_mark_lines(m_yuv_draw_buffer);
          draw_image();
          m_stb_status->push(m_calib_tool->get_state_description());
        }
#else
        printf("IPP and OpenCV not installed; mirror calibration does not work.\n");
#endif
	break;
      }

    case MODE_MIRROR_CALIB_EVAL:
      {
#ifdef HAVE_MIRROR_CALIB
	float dist;
	float phi;
	m_calib_tool->eval(image_x, image_y, &dist, &phi);
        phi = normalize_mirror_rad(phi);
        printf("(%d, %d) = POLAR(%.2f deg, %.2f meters)\n",
            image_x, image_y,
            rad2deg(phi), dist);
        //printf("Distance: %2f\t Phi: %2f\n", dist, phi);
#else
        printf("IPP and OpenCV not installed; mirror calibration does not work.\n");
#endif
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

void
Firestation::mc_draw_line()
{
  if (m_img_src != SRC_NONE) {
#ifdef HAVE_MIRROR_CALIB
    memcpy(m_yuv_draw_buffer, m_yuv_orig_buffer, m_img_size);
    MirrorCalibTool::draw_line(m_yuv_draw_buffer,
                               mc_line_angle_deg,
                               m_calib_tool->center_x(),
                               m_calib_tool->center_y(),
                               m_img_width,
                               m_img_height);
    draw_image();
#else
        printf("IPP and OpenCV not installed; mirror calibration does not work.\n");
#endif
  }
}

bool
Firestation::mc_on_line_angle_changed(Gtk::ScrollType scroll, double value)
{
  mc_line_angle_deg = -1.0f * value;
  mc_line_angle_deg =
    rad2deg(normalize_mirror_rad(deg2rad(mc_line_angle_deg)));
  // Why -1.0f * value?
  // We want to display angles from the robot's real-world perspective.
  // We want to calculate with angles from the (mirrored!) image's perspective.
  // So when the user chooses 90 degrees, he wants to look to the left from the
  // robots perspective. But due to the mirroring, that's the right side in the
  // image, so we take -90 degrees.
  mc_draw_line();
  return true;
}

void
Firestation::mc_load_mask()
{
  m_fcd_mc_load_mask->set_transient_for(*this);

#if GTK_VERSION_GE(3,0)
  Glib::RefPtr<Gtk::FileFilter> filter_mirror = Gtk::FileFilter::create();
#else
  Gtk::FileFilter *filter_mirror = Gtk::manage(new Gtk::FileFilter());
#endif
  filter_mirror->set_name("Robot Mask");
  filter_mirror->add_pattern("*.pnm");
#if GTK_VERSION_GE(3,0)
  m_fcd_mc_load_mask->add_filter(filter_mirror);
#else
  m_fcd_mc_load_mask->add_filter(*filter_mirror);
#endif

  int result = m_fcd_mc_load_mask->run();

  switch(result)
    {
    case Gtk::RESPONSE_OK:
      {
#ifdef HAVE_MIRROR_CALIB
	std::string filename = m_fcd_mc_load_mask->get_filename();
	m_calib_tool->load_mask( filename.c_str() );
	//m_op_mode = MODE_MIRROR_CALIB_EVAL;
#else
        printf("IPP and OpenCV not installed; mirror calibration does not work.\n");
#endif
	break;
      }
    case Gtk::RESPONSE_CANCEL:
      break;
    default:
      break;
    }

  m_fcd_mc_load_mask->hide();
}

/** Enters MODE_MIRROR_CALIB mode which waits for a click to mark the
 * preliminary center of the image. */
void
Firestation::mc_set_center()
{
  m_op_mode = MODE_MIRROR_CALIB;
}

/** Start the mirror calibration process. */
void
Firestation::mc_memorize()
{
  /* if (m_op_mode == MODE_MIRROR_CALIB)
    {
      m_op_mode = MODE_VIEWER;
      m_stb_status->push("Leaving mirror calibration mode");
    }
  else */ if (m_img_src != SRC_NONE)
    {
#ifdef HAVE_MIRROR_CALIB
      double ori = mc_line_angle_deg;
      std::cout << "Starting calibration for ori = " << ori << std::endl;
      m_calib_tool->push_back(m_yuv_orig_buffer, m_img_size,
                              m_img_width, m_img_height, deg2rad(ori));
      m_op_mode = MODE_MIRROR_CALIB;
      std::cout << "Initialization for ori = " << ori << " completed" << std::endl;

      mc_line_angle_deg -= 120.0;
      mc_line_angle_deg =
        rad2deg(normalize_mirror_rad(deg2rad(mc_line_angle_deg)));
      m_scl_mc_line->set_value(-1.0f * mc_line_angle_deg);
      // Why -1.0f * mc_line_angle_deg?
      // We want to display angles from the robot's real-world perspective.
      // We want to calculate with angles from the (mirrored!) image's perspective.
      // So when the user chooses 90 degrees, he wants to look to the left from the
      // robots perspective. But due to the mirroring, that's the right side in the
      // image, so we take -90 degrees.
      mc_draw_line();
#else
        printf("IPP and OpenCV not installed; mirror calibration does not work.\n");
#endif
  }
}

/** Start the mirror calibration process. */
void
Firestation::mc_simulate_clicks()
{
  for (int i = 1; i <= 3; ++i) {
    image_click(NULL); // SHARPENING
    for (int j = 1; j <= 8; ++j) { image_click(NULL); } // EDGE_DETECTION
    for (int j = 1; j <= 2*8; ++j) { image_click(NULL); } // COMBINATION
    image_click(NULL); // PRE_MARKING
    image_click(NULL); // FINAL_MARKING
  }
}

/** Load mirror calibration data from a file. */
void
Firestation::mc_load()
{
  m_fcd_mc_load->set_transient_for(*this);

#if GTK_VERSION_GE(3,0)
  Glib::RefPtr<Gtk::FileFilter> filter_mirror = Gtk::FileFilter::create();
#else
  Gtk::FileFilter *filter_mirror = Gtk::manage(new Gtk::FileFilter());
#endif
  filter_mirror->set_name("Mirror Calibration");
  filter_mirror->add_pattern("*.mirror");
  filter_mirror->add_pattern("*.bulb");
#if GTK_VERSION_GE(3,0)
  m_fcd_mc_load->add_filter(filter_mirror);
#else
  m_fcd_mc_load->add_filter(*filter_mirror);
#endif

  int result = m_fcd_mc_load->run();

  switch(result)
    {
    case Gtk::RESPONSE_OK:
      {
#ifdef HAVE_MIRROR_CALIB
	std::string filename = m_fcd_mc_load->get_filename();
	m_calib_tool->load( filename.c_str() );
	m_op_mode = MODE_MIRROR_CALIB_EVAL;
#else
        printf("IPP and OpenCV not installed; mirror calibration does not work.\n");
#endif
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
#ifdef HAVE_MIRROR_CALIB
	std::string filename = m_fcd_mc_save->get_filename();

	m_calib_tool->save( filename.c_str() );
#else
        printf("IPP and OpenCV not installed; mirror calibration does not work.\n");
#endif
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
