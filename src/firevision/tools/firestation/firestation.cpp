
/***************************************************************************
 *  firestation.cpp - Firestation
 *
 *  Created: Wed Oct 10 14:19:30 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <tools/firestation/firestation.h>
#include <tools/firestation/mirror_calib.h>
#include <tools/firestation/color_train.h>

#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>
#include <fvutils/color/yuv.h>
#include <fvutils/scalers/lossy.h>
#include <fvutils/writers/jpeg.h>
#include <fvutils/readers/jpeg.h>

#include <cams/net.h>
#include <core/exception.h>

#include <gdkmm/pixbuf.h>
#include <arpa/inet.h>

#include <iostream>

/** @class Firestation tools/firestation/firestation.h
 * Control GUI for vision related stuff.
 */

using namespace std;

/** Constructor.
 * @param ref_xml reference pointer to the glade file
 */
Firestation::Firestation(Glib::RefPtr<Gnome::Glade::Xml> ref_xml)
{
  m_shm_buffer = 0;
  m_yuv_orig_buffer = 0;
  m_yuv_draw_buffer = 0;
  m_yuv_scaled_buffer = 0;
  m_rgb_scaled_buffer = 0;
  m_img_src_ready = false;
  m_img_writer = 0;
  m_img_reader = 0;

  m_calib_tool = new MirrorCalibTool();
  m_color_tool = new ColorTrainTool();

  m_net_cam = 0;

  m_img_src = SRC_NONE;
  m_op_mode = MODE_VIEWER;

  // --- main window ------------------------------------------------
  ref_xml->get_widget("wndMain", m_wnd_main);
  if ( !m_wnd_main )
    {
      throw std::runtime_error("Couldn't find wndMain.");
    }

  ref_xml->get_widget("imgImage", m_img_image);
  if ( !m_img_image )
    {
      throw std::runtime_error("Couldn't find imgImage.");
    }
  m_img_image->signal_size_allocate().connect( sigc::mem_fun(*this, &Firestation::resize_image) );
    
  ref_xml->get_widget("evtImageEventBox", m_evt_image);
  if ( !m_evt_image )
    {
      throw std::runtime_error("Couldn't find evtImageEventBox.");
    }
  m_evt_image->signal_button_press_event().connect( sigc::mem_fun(*this, &Firestation::image_click) );


  ref_xml->get_widget("trvShmImageIds", m_trv_shm_image_ids);
  if ( !m_trv_shm_image_ids )
    {
      throw std::runtime_error("Couldn't find trvShmImageIds.");
    }

  ref_xml->get_widget("stbStatus", m_stb_status);
  if ( !m_stb_status )
    {
      throw std::runtime_error("Couldn't find stbStatus.");
    }
  // ----------------------------------------------------------------


  // --- toolbar widgets --------------------------------------------
  ref_xml->get_widget("tbtnExit", m_tbtn_exit);
  if ( !m_tbtn_exit )
    {
      throw std::runtime_error("Couldn't find tbtmExit.");
    }
  m_tbtn_exit->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::exit) );
  
  ref_xml->get_widget("tbtnUpdate", m_tbtn_update);
  if ( !m_tbtn_update )
    {
      throw std::runtime_error("Couldn't find tbtnUpdate.");
    }
  m_tbtn_update->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::update_image) );

  ref_xml->get_widget("tbtnSave", m_tbtn_save);
  if ( !m_tbtn_save )
    {
      throw std::runtime_error("Couldn't find tbtnSave.");
    }
  m_tbtn_save->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::save_image) );
  
  ref_xml->get_widget("tbtnOpenFile", m_tbtn_open_file);
  if ( !m_tbtn_open_file )
    {
      throw std::runtime_error("Couldn't find tbtnOpenFile.");
    }
  m_tbtn_open_file->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::open_file) );
  
  ref_xml->get_widget("tbtnOpenShm", m_tbtn_open_shm);
  if ( !m_tbtn_open_shm )
    {
      throw std::runtime_error("Couldn't find tbtnOpenShm.");
    }
  m_tbtn_open_shm->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::open_shm) );

  ref_xml->get_widget("tbtnOpenFuse", m_tbtn_open_fuse);
  if ( !m_tbtn_open_fuse )
    {
      throw std::runtime_error("Couldn't find tbtnOpenFuse.");
    }
  m_tbtn_open_fuse->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::open_fuse) );
  // ----------------------------------------------------------------


  // --- dialogs ----------------------------------------------------
  ref_xml->get_widget("fcdOpenImage", m_fcd_open_image);
  if ( !m_fcd_open_image )
    {
      throw std::runtime_error("Couldn't find fcdOpenImage.");
    }

  ref_xml->get_widget("fcdSaveImage", m_fcd_save_image);
  if ( !m_fcd_save_image )
    {
      throw std::runtime_error("Couldn't find fcdSaveImage.");
    }

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


  // --- color training ---------------------------------------------
  ref_xml->get_widget("rbtCtBall", m_rbt_ct_ball);
  if ( !m_rbt_ct_ball)
    {
      throw std::runtime_error("Couldn't find rbtCtBall.");
    }

  ref_xml->get_widget("rbtCtField", m_rbt_ct_field);
  if ( !m_rbt_ct_field)
    {
      throw std::runtime_error("Couldn't find rbtCtField.");
    }

  ref_xml->get_widget("rbtCtLines", m_rbt_ct_lines);
  if ( !m_rbt_ct_lines)
    {
      throw std::runtime_error("Couldn't find rbtCtLines.");
    }

  ref_xml->get_widget("btnCtUnselect", m_btn_ct_unselect);
  if ( !m_btn_ct_unselect )
    {
      throw std::runtime_error("Couldn't find btnCtUnselect.");
    }
  m_btn_ct_unselect->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::ct_unselect) );

  ref_xml->get_widget("btnCtAdd", m_btn_ct_add);
  if ( !m_btn_ct_add )
    {
      throw std::runtime_error("Couldn't find btnCtAdd.");
    }
  m_btn_ct_add->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::ct_add) );
  
  ref_xml->get_widget("btnCtStart", m_btn_ct_start);
  if ( !m_btn_ct_start )
    {
      throw std::runtime_error("Couldn't find btnCtStart.");
    }
  m_btn_ct_start->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::ct_start) );
  
  ref_xml->get_widget("btnCtSaveHistos", m_btn_ct_save_histos);
  if ( !m_btn_ct_save_histos )
    {
      throw std::runtime_error("Couldn't find btnCtSaveHistos.");
    }
  m_btn_ct_save_histos->signal_clicked().connect( sigc::mem_fun(*m_color_tool, &ColorTrainTool::save_histos) );
  
  ref_xml->get_widget("sclCtThreshold", m_scl_ct_threshold);
  if ( !m_scl_ct_threshold )
    {
      throw std::runtime_error("Couldn't find sclCtThreshold.");
    }
  m_scl_ct_threshold->signal_change_value().connect( sigc::mem_fun(*m_color_tool, &ColorTrainTool::set_threshold_sh) );
  
  ref_xml->get_widget("sclCtMinProb", m_scl_ct_minprob);
  if ( !m_scl_ct_minprob )
    {
      throw std::runtime_error("Couldn't find sclCtMinProb.");
    }
  m_scl_ct_minprob->signal_change_value().connect( sigc::mem_fun(*m_color_tool, &ColorTrainTool::set_min_prob_sh) );
  
  ref_xml->get_widget("wndCtColorMap", m_wnd_ct_colormap);
  if ( !m_wnd_ct_colormap )
    {
      throw std::runtime_error("Couldn't find wndCtColorMap.");
    }
  
  ref_xml->get_widget("imgCtSegmentation", m_img_ct_segmentation);
  if ( !m_img_ct_segmentation )
    {
      throw std::runtime_error("Couldn't find imgCtSegmentation.");
    }

  ref_xml->get_widget("imgCtColormapLocal", m_img_ct_colormap_local);
  if ( !m_img_ct_colormap_local )
    {
      throw std::runtime_error("Couldn't find imgCtColormapLocal.");
    }
  
  ref_xml->get_widget("imgCtColormapRemote", m_img_ct_colormap_remote);
  if ( !m_img_ct_colormap_remote )
    {
      throw std::runtime_error("Couldn't find imgCtColormapRemote.");
    }
  
  ref_xml->get_widget("fcdCtSaveColormap", m_fcd_ct_save_colormap);
  if ( !m_fcd_ct_save_colormap)
    {
      throw std::runtime_error("Couldn't find fcdCtSaveColormap.");
    }
  
  ref_xml->get_widget("btnCtSaveColormap", m_btn_ct_save_colormap);
  if ( ! m_btn_ct_save_colormap )
    {
      throw std::runtime_error("Couldn't find btnCtSaveColormap.");
    }
  m_btn_ct_save_colormap->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::ct_save_colormap) );

  ref_xml->get_widget("fcdCtLoadColormap", m_fcd_ct_load_colormap);
  if ( !m_fcd_ct_load_colormap)
    {
      throw std::runtime_error("Couldn't find fcdCtLoadColormap.");
    }

  ref_xml->get_widget("btnCtLoadColormap", m_btn_ct_load_colormap);
  if ( ! m_btn_ct_load_colormap )
    {
      throw std::runtime_error("Couldn't find btnCtLoadColormap.");
    }
  m_btn_ct_load_colormap->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::ct_load_colormap) );
  // ----------------------------------------------------------------


  // --- mirror calibration -----------------------------------------
  ref_xml->get_widget("btnMcStart", m_btn_mc_start);
  if ( !m_btn_mc_start )
    {
      throw std::runtime_error("Couldn't find btnMcStart.");
    }
  m_btn_mc_start->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_start) );

  ref_xml->get_widget("entCalibDist", m_ent_mc_dist);
  if ( ! m_ent_mc_dist ) 
    {
      throw std::runtime_error("Couldn't find entCalibDist.");
    }

  ref_xml->get_widget("entCalibOri", m_ent_mc_ori);
  if ( ! m_ent_mc_ori ) 
    {
      throw std::runtime_error("Couldn't find entCalibOri.");
    }

  ref_xml->get_widget("btnCalibLoad", m_btn_mc_load);
  if ( ! m_btn_mc_load )
    {
      throw std::runtime_error("Couldn't find btnCalibLoad.");
    }
  m_btn_mc_load->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_load) );

  ref_xml->get_widget("btnCalibSave", m_btn_mc_save);
  if ( ! m_btn_mc_save )
    {
      throw std::runtime_error("Couldn't find btnCalibSave.");
    }
  m_btn_mc_save->signal_clicked().connect( sigc::mem_fun(*this, &Firestation::mc_save) );

  ref_xml->get_widget("fcdCalibSave", m_fcd_mc_save);
  if ( ! m_fcd_mc_save )
    {
      throw std::runtime_error("Couldn't find fcdCalibSave.");
    }
  
  ref_xml->get_widget("fcdCalibLoad", m_fcd_mc_load);
  if ( ! m_fcd_mc_load )
    {
      throw std::runtime_error("Couldn't find fcdCalibLoad.");
    }
  // ----------------------------------------------------------------


  m_scaled_img_width = m_evt_image->get_width();
  m_scaled_img_height = m_evt_image->get_height();

  m_avahi_thread = new AvahiThread();
  m_avahi_thread->watch("_fountain._tcp", this);
  m_avahi_thread->start();
}

/** Destructor. */
Firestation::~Firestation()
{
  m_avahi_thread->cancel();
  m_avahi_thread->join();

  free(m_yuv_orig_buffer);
  free(m_yuv_draw_buffer);
  free(m_yuv_scaled_buffer);
  free(m_rgb_scaled_buffer);
  
  delete m_net_cam;
  delete m_shm_buffer;
  delete m_img_writer;
  delete m_img_reader;
  delete m_calib_tool;
  delete m_color_tool;
  delete m_avahi_thread;
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
  m_wnd_main->hide();
}

/** Saves the current image. */
void
Firestation::save_image()
{
  if (!m_img_src_ready)
    {
      return;
    }

  m_fcd_save_image->set_transient_for(*this);

  Gtk::FileFilter filter_jpg;
  filter_jpg.set_name("Jpeg");
  filter_jpg.add_pattern("*.jpg");
  filter_jpg.add_pattern("*.jpeg");
  m_fcd_save_image->add_filter(filter_jpg);

  Gtk::FileFilter filter_any;
  filter_any.set_name("Any format");
  filter_any.add_pattern("*");
  m_fcd_save_image->add_filter(filter_any);

  int result = m_fcd_save_image->run();

  switch(result) 
    {
    case(Gtk::RESPONSE_OK):
      {
	std::string filename = m_fcd_save_image->get_filename();
	
	delete m_img_writer;
	m_img_writer = new JpegWriter();
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
  if (!m_img_src_ready)
    {
      return;
    }

  switch(m_img_src)
    {
    case SRC_NONE:
      break;

    case SRC_FILE:
      scale_image();
      draw_image();
      break;

    case SRC_SHM:
      /* TODO */
      scale_image();
      draw_image();
      break;

    case SRC_FUSE:
      m_net_cam->capture();
      memcpy(m_yuv_orig_buffer, m_net_cam->buffer(), m_img_size);
      memcpy(m_yuv_draw_buffer, m_net_cam->buffer(), m_img_size);
      m_net_cam->dispose_buffer();

      scale_image();
      draw_image();
      break;

    default:
      break;
    }

  draw_segmentation_result();
}

/** Reads in an image from a file. */
void
Firestation::open_file()
{
  m_img_src = SRC_FILE;

  m_fcd_open_image->set_transient_for(*this);
  
  Gtk::FileFilter filter_jpg;
  filter_jpg.set_name("Jpeg");
  filter_jpg.add_pattern("*.jpg");
  filter_jpg.add_pattern("*.jpeg");
  m_fcd_open_image->add_filter(filter_jpg);
  
  int result = m_fcd_open_image->run();
	
  switch(result)
    {
    case Gtk::RESPONSE_OK:
      {
	// allocate memory for the uncompressed image
	unsigned char* tmp_buffer = (unsigned char*) malloc(10000000);
	std::string filename = m_fcd_open_image->get_filename();
	
	// TODO: different file formats
	m_img_reader = new JpegReader(filename.c_str());
	m_img_reader->set_buffer(tmp_buffer);
	m_img_reader->read();
	
	m_img_width = m_img_reader->pixel_width();
	m_img_height = m_img_reader->pixel_height();
	m_img_cs = m_img_reader->colorspace();
	
	m_img_size = colorspace_buffer_size( m_img_cs, 
					     m_img_width, 
					     m_img_height );
	
	m_yuv_orig_buffer = (unsigned char*) malloc(m_img_size);
	m_yuv_draw_buffer = (unsigned char*) malloc(m_img_size);
	memcpy(m_yuv_orig_buffer, tmp_buffer, m_img_size);
	memcpy(m_yuv_draw_buffer, tmp_buffer, m_img_size);
	free(tmp_buffer);
	
	m_img_src_ready = true;
	
	scale_image();
	draw_image();
	
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

/** Opens a SHM image. */
void
Firestation::open_shm()
{
  m_img_src = SRC_SHM;

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
		
		try
		  {
		    m_shm_buffer = new SharedMemoryImageBuffer(name.c_str());
		  }
		catch (Exception& e)
		  {
		    e.print_trace();
		  }
		
		m_img_src_ready = true;

		scale_image();
		draw_image();
		
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
  m_img_src = SRC_FUSE;

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
		
		try
		  {
		    delete m_net_cam;
		    m_net_cam = new NetworkCamera(hostname.c_str(), port, image_id.c_str(), jpeg);
		    m_net_cam->open();
		    m_net_cam->start();
		    m_net_cam->capture();
		    
		    m_img_cs = m_net_cam->colorspace();
		    
		    if ( YUV422_PLANAR == m_img_cs )
		      {
			m_img_size = m_net_cam->buffer_size();
			m_img_width = m_net_cam->pixel_width();
			m_img_height = m_net_cam->pixel_height();
			m_yuv_orig_buffer = (unsigned char*) malloc(m_img_size);
			m_yuv_draw_buffer = (unsigned char*) malloc(m_img_size);
			memcpy(m_yuv_orig_buffer, m_net_cam->buffer(), m_img_size);
			memcpy(m_yuv_draw_buffer, m_net_cam->buffer(), m_img_size);
		      }
		    
		    m_net_cam->dispose_buffer();
		  }
		catch (Exception& e)
		  {
		    e.print_trace();
		  }
		
		m_img_src_ready = true;
		
		scale_image();
		draw_image();

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
  
  m_dlg_open_fuse->hide();
}

/** Stuff that is executed after an image source has been opened
 * successfully.
 */ 
void
Firestation::post_open_img_src()
{
  if (m_img_src_ready)
    {
      m_tbtn_update->set_sensitive(true);
      m_tbtn_save->set_sensitive(true);

      m_color_tool->initialize( m_img_width, m_img_height,
				m_yuv_orig_buffer, m_yuv_draw_buffer, ct_get_fg_object() ); 

      draw_segmentation_result();
    }
}

/** Handles the scaling of the displayed image.
 * @return true if redraw is necessary
 */
bool
Firestation::scale_image()
{
  if (m_img_src_ready)
    {
      float scale_factor_width = m_scaled_img_width / (float) m_img_width;
      float scale_factor_height = m_scaled_img_height / (float) m_img_height;
      
      float new_scale_factor;
      new_scale_factor = (scale_factor_width < scale_factor_height) ? scale_factor_width : scale_factor_height;
      if ( new_scale_factor != m_scale_factor )
	{
	  m_scale_factor = new_scale_factor;
	  m_scaled_img_width = (unsigned int) floor(m_img_width * m_scale_factor);
	  m_scaled_img_height = (unsigned int) floor(m_img_height * m_scale_factor);
	  m_img_image->set_size_request(m_scaled_img_width, m_scaled_img_height);
	  m_evt_image->set_size_request(m_scaled_img_width, m_scaled_img_height);
	  
	  return true;
	}
    }
  
  return false;
}

/** Draws the image. */
void
Firestation::draw_image()
{
  if ( ! m_img_src_ready )
    {
      return ;
    }

  LossyScaler scaler;
  scaler.set_original_buffer( m_yuv_draw_buffer );
  scaler.set_original_dimensions(m_img_width, m_img_height);
  scaler.set_scale_factor(m_scale_factor);
  scaler.set_scaled_dimensions(m_scaled_img_width, m_scaled_img_height);
  m_scaled_img_width = scaler.needed_scaled_width();
  m_scaled_img_height = scaler.needed_scaled_height();
  free(m_rgb_scaled_buffer);
  free(m_yuv_scaled_buffer);
  m_yuv_scaled_buffer = (unsigned char*) malloc( colorspace_buffer_size( m_img_cs,
									 m_scaled_img_width,
									 m_scaled_img_height ) );
  scaler.set_scaled_buffer(m_yuv_scaled_buffer);
  scaler.scale();
  
  m_rgb_scaled_buffer = (unsigned char*) malloc( colorspace_buffer_size( RGB,
									 m_scaled_img_width,
									 m_scaled_img_height ) );
  
  convert( m_img_cs, RGB,
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
  m_scaled_img_width = (unsigned int) allocation.get_width();
  m_scaled_img_height = (unsigned int) allocation.get_height();
  if ( scale_image() )
    {
      draw_image();
    } 
}

void
Firestation::draw_segmentation_result()
{
  if ( !m_color_tool->initialized() )
    {
      return;
    }

  float scale_w = 300.0 / m_img_width;  /* TODO */
  float scale_h = 200.0 / m_img_height;  /* TODO */
  float scale_factor = scale_w < scale_h ? scale_w : scale_h;

  m_color_tool->perform_segmentation();

  LossyScaler scaler;
  scaler.set_original_buffer( m_color_tool->seg_buffer() );
  scaler.set_original_dimensions(m_img_width, m_img_height);
  scaler.set_scale_factor(scale_factor);
  scaler.set_scaled_dimensions(200, 200);  /* TODO */
  unsigned int scaled_width = scaler.needed_scaled_width();
  unsigned int scaled_height = scaler.needed_scaled_height();

  unsigned char* scaled_buffer = (unsigned char*) malloc( colorspace_buffer_size( m_img_cs,
										  scaled_width,
										  scaled_height ) );
  scaler.set_scaled_buffer(scaled_buffer);
  scaler.scale();

  unsigned char* rgb_buffer = (unsigned char*) malloc( colorspace_buffer_size( RGB,
									       scaled_width,
									       scaled_height ) );

  convert( m_img_cs, RGB,
	   scaled_buffer, rgb_buffer,
	   scaled_width, scaled_height);

  free(scaled_buffer);

  Glib::RefPtr<Gdk::Pixbuf> image = Gdk::Pixbuf::create_from_data( rgb_buffer,
								   Gdk::COLORSPACE_RGB,
								   false,
								   8,
								   scaled_width,
								   scaled_height,
								   3 * scaled_width );

  m_img_ct_segmentation->set(image);
}

/** Handles mouse clicks in the image area.
 * @param event a Gtk event
 * @return true if signal was handled
 */
bool
Firestation::image_click(GdkEventButton* event)
{
  unsigned int image_x;
  unsigned int image_y;

  image_x = (unsigned int)rint(event->x / m_scale_factor);
  image_y = (unsigned int)rint(event->y / m_scale_factor);

  //  cout << "X: " << image_x << " Y: " << image_y << endl;

  switch (m_op_mode)
    {
    case MODE_VIEWER:
      if (m_img_src_ready)
	{
	  switch( m_img_cs )
	    {
	    case YUV422_PLANAR:
	      register unsigned char y;
	      register unsigned char u;
	      register unsigned char v;
	      YUV422_PLANAR_YUV( m_yuv_orig_buffer,
				 m_img_width,
				 m_img_height,
				 (unsigned int)rint(event->x / m_scale_factor),
				 (unsigned int)rint(event->y / m_scale_factor),
				 y, u, v );
	      cout << "Y: " << (unsigned int)y 
		   << " U: " << (unsigned int)u 
		   << " V: " << (unsigned int)v << endl;
	      break;
	    default:
	  cout << "Unhandled colorspace" << endl;
	    }
	}
      break;

    case MODE_COLOR_TRAIN:
      m_color_tool->step( (unsigned int) rint(event->x / m_scale_factor),
			  (unsigned int) rint(event->y / m_scale_factor) );
      draw_image();
     break;

    case MODE_MIRROR_CALIB:
      {
	m_calib_tool->step( (unsigned int) rint(event->x / m_scale_factor),
			    (unsigned int) rint(event->y / m_scale_factor) );

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
      if (m_img_src_ready)
	{
	  if ( !m_color_tool->initialized() )
	    {
	      m_color_tool->initialize( m_img_width, m_img_height,
					m_yuv_orig_buffer, m_yuv_draw_buffer,
					ct_get_fg_object() );
	    }
	  m_color_tool->set_fg_object( ct_get_fg_object() );

	  m_color_tool->set_threshold((unsigned int) rint(m_scl_ct_threshold->get_value()));
	  m_color_tool->set_min_prob((float) m_scl_ct_minprob->get_value());
	  
	  m_op_mode = MODE_COLOR_TRAIN;

	  m_stb_status->push("Entering color training mode");
	}
    }
}

/** Reset the selection of the magic wand. */
void
Firestation::ct_unselect()
{
  m_color_tool->unselect();
  draw_image();
}

/** Add magic wand selection to colormap. */
void
Firestation::ct_add()
{
  m_color_tool->add();
  ct_draw_colormaps();
  draw_segmentation_result();
}

/** Save the current colormap to disk. */
void
Firestation::ct_save_colormap()
{
  m_fcd_ct_save_colormap->set_transient_for(*this);

  int result = m_fcd_ct_save_colormap->run();

  switch(result) 
    {
    case(Gtk::RESPONSE_OK):
      {
	std::string filename = m_fcd_ct_save_colormap->get_filename();
	m_color_tool->save_colormap( filename.c_str() );
	break;
      }

    case(Gtk::RESPONSE_CANCEL):
      break;

    default:
      break;
    }
  
  m_fcd_ct_save_colormap->hide();
}

hint_t
Firestation::ct_get_fg_object()
{
  if ( m_rbt_ct_ball->get_active() )
    {
      return H_BALL;
    }
  else if ( m_rbt_ct_field->get_active() )
    {
      return H_FIELD;
    }
  else if ( m_rbt_ct_lines->get_active() )
    {
      return H_LINE;
    }
  else
    {
      return H_UNKNOWN;
    }
}

void
Firestation::ct_draw_colormaps()
{
  unsigned int width = 128;  /* TODO */
  unsigned int height = 128;  /* TODO */
  unsigned char* buffer = m_color_tool->colormap_rgb(&width, &height);
  printf("colormap dimensions %d x %d\n", width, height);

  Glib::RefPtr<Gdk::Pixbuf> image = Gdk::Pixbuf::create_from_data( buffer,
								   Gdk::COLORSPACE_RGB,
								   false,
								   8,
								   width, height,
								   3 * width );
  m_img_ct_colormap_local->set(image);
}

void
Firestation::ct_load_colormap()
{
  m_fcd_ct_load_colormap->set_transient_for(*this);

  int result = m_fcd_ct_load_colormap->run();

  switch(result)
    {
    case (Gtk::RESPONSE_OK):
      {
	std::string filename = m_fcd_ct_load_colormap->get_filename();
	m_color_tool->load_colormap( filename.c_str() );
	
	if (m_img_src_ready)
	  {
	    ct_draw_colormaps();
	    draw_segmentation_result();
	  }

	break;
      }

    case (Gtk::RESPONSE_CANCEL):
      break;

    default:
      break;
    }

  m_fcd_ct_load_colormap->hide();
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
      if (m_img_src_ready)
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
Firestation::all_for_now()
{
}

void
Firestation::cache_exhausted()
{
}

void
Firestation::browse_failed( const char* name,
			      const char* type,
			      const char* domain )
{
}

void
Firestation::service_added( const char* name,
			      const char* type,
			      const char* domain,
			      const char* host_name,
			      const struct sockaddr* addr,
			      const socklen_t addr_size,
			      uint16_t port,
			      std::list<std::string>& txt,
			      int flags )
{
  std::vector<FUSE_imageinfo_t> image_list;
  NetworkCamera cam(host_name, port);
  cam.open();
  image_list = cam.image_list();
 
  std::vector<FUSE_imageinfo_t>::iterator fit;

  Gtk::TreeModel::Children children = m_fuse_tree_store->children();
  Gtk::TreeModel::Row row = *(m_fuse_tree_store->append());
  row[m_fuse_columns.m_id] = children.size();
  row[m_fuse_columns.m_name] = Glib::ustring(name);
  row[m_fuse_columns.m_service_name] = Glib::ustring(name);
  row[m_fuse_columns.m_service_type] = Glib::ustring(type);
  row[m_fuse_columns.m_service_domain] = Glib::ustring(domain);
  row[m_fuse_columns.m_service_hostname] = Glib::ustring(host_name);
  row[m_fuse_columns.m_service_port] = port;

  for (fit = image_list.begin(); fit != image_list.end(); ++fit)
    {
      Gtk::TreeModel::Row childrow = *(m_fuse_tree_store->append(row.children()));
      childrow[m_fuse_columns.m_name] = Glib::ustring(fit->image_id);
      childrow[m_fuse_columns.m_service_name] = Glib::ustring(name);
      childrow[m_fuse_columns.m_service_type] = Glib::ustring(type);
      childrow[m_fuse_columns.m_service_domain] = Glib::ustring(domain);
      childrow[m_fuse_columns.m_service_hostname] = Glib::ustring(host_name);
      childrow[m_fuse_columns.m_service_port] = port;
      childrow[m_fuse_columns.m_image_id] = Glib::ustring(fit->image_id);
      childrow[m_fuse_columns.m_image_width] = fit->width;
      childrow[m_fuse_columns.m_image_height] = fit->height;
      childrow[m_fuse_columns.m_image_colorspace] = Glib::ustring( colorspace_to_string((colorspace_t) fit->colorspace) );
    }
}

void
Firestation::service_removed( const char* name,
				const char* type,
				const char* domain )
{
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
}
