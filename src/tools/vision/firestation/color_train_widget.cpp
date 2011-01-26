
/***************************************************************************
 *  color_train_widget.cpp - Color training widget
 *
 *  Created: Thu Mar 20 22:19:36 2008
 *  Copyright  2008  Daniel Beck
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

#include "color_train_widget.h"
#include "colormap_viewer_widget.h"
#include <fvutils/color/yuv.h>
#include <fvutils/color/zauberstab.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>
#include <fvutils/draw/drawer.h>
#include <fvutils/scalers/lossy.h>
#include <fvutils/colormap/bayes/bayes_generator.h>
#include <fvutils/colormap/yuvcm.h>
#include <fvutils/colormap/cmfile.h>

#include <fvutils/writers/jpeg.h>

#include <fvutils/color/color_object_map.h>

#include <core/exceptions/software.h>

using namespace firevision;

/** @class ColorTrainWidget "color_train_widget.h"
 * This widget implements the complete color training process.
 *
 * @author Daniel Beck
 */

/** Constructor.
 * @param parent the parent window
 */
ColorTrainWidget::ColorTrainWidget(Gtk::Window* parent)
{
  m_generator = 0;
  m_zauberstab = new Zauberstab();
  m_cvw = new ColormapViewerWidget();

  m_src_buffer = 0;
  m_draw_buffer = 0;

  m_wnd_parent = parent;
  m_btn_reset_selection = 0;
  m_btn_add_to_colormap = 0;
  m_btn_reset_colormap = 0;
  m_btn_load_histos = 0;
  m_btn_save_histos = 0;
  m_btn_load_colormap = 0;
  m_btn_save_colormap = 0;
  m_spbtn_cm_depth = 0;
  m_spbtn_cm_width = 0;
  m_spbtn_cm_height = 0;
  m_img_segmentation = 0;
  m_scl_threshold = 0;
  m_scl_min_prob = 0;
  m_fcd_filechooser = 0;
}

/** Destructor. */
ColorTrainWidget::~ColorTrainWidget()
{
  delete m_cvw;
  delete m_generator;
  delete m_zauberstab;
}

/** Set the current foreground object.
 * @param fg_object the foreground object
 */
void
ColorTrainWidget::set_fg_object(hint_t fg_object)
{
  m_fg_object = fg_object;
}

/** Set the buffer containing the image data.
 * @param yuv422_buffer the YUV422_PLANAR buffer holding the image data
 * @param img_width the width of the image
 * @param img_height the height of the image
 */
void
ColorTrainWidget::set_src_buffer(unsigned char* yuv422_buffer,
		    unsigned int img_width, unsigned int img_height)
{
  m_img_width = img_width;
  m_img_height = img_height;
  m_src_buffer = yuv422_buffer;
  m_img_cs = YUV422_PLANAR;
  m_img_size = colorspace_buffer_size( m_img_cs, m_img_width, m_img_height );

  if (yuv422_buffer)
    {
      m_zauberstab->deleteRegion();
      m_zauberstab->setBuffer(m_src_buffer, m_img_width, m_img_height);
      m_zauberstab->setThreshold(10);
    }
  else
    {
      m_img_segmentation->clear();
      m_img_segmentation->set("gtk-missing-image");
    }
}

/** Set the buffer to draw the selection into.
 * It is assumed that this buffer has the same dimensions as the buffer holding
 * the soruce image.
 * @param buffer the draw buffer
 */
void
ColorTrainWidget::set_draw_buffer(unsigned char* buffer)
{
  m_draw_buffer = buffer;
}

/** The user clicked into the image.
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param button 1 for left click, 3 for right click @see GdkEventButton
 */
void
ColorTrainWidget::click(unsigned int x, unsigned int y, unsigned int button)
{
  if (m_src_buffer == 0 || m_draw_buffer == 0)
    { return; }

  if ( m_zauberstab->isEmptyRegion() )
    {
      if (button == MOUSE_BUTTON_LEFT) //left click
			{
				m_zauberstab->findRegion(x, y);
			}
    }
  else
    {
      if (button == MOUSE_BUTTON_LEFT) //left click
			{
				m_zauberstab->addRegion(x, y);
			}

      if (button == MOUSE_BUTTON_RIGHT) //right click
			{
				m_zauberstab->deleteRegion(x, y);
			}
    }

  memcpy(m_draw_buffer, m_src_buffer, m_img_size);

  ZRegion *region = m_zauberstab->getRegion();
  Drawer *d = new Drawer();
  d->set_buffer( m_draw_buffer, m_img_width, m_img_height );

  for (unsigned int s = 0; s < region->slices->size(); s++)
    {
      d->draw_rectangle_inverted( region->slices->at(s)->leftX,
				region->slices->at(s)->y,
				region->slices->at(s)->rightX - region->slices->at(s)->leftX,
				1 );
    }

  delete d;

  m_signal_update_image();
}

/** Reset the selection. */
void
ColorTrainWidget::reset_selection()
{
  if (m_zauberstab)
    { m_zauberstab->deleteRegion(); }

  if( m_src_buffer && m_draw_buffer )
    { memcpy(m_draw_buffer, m_src_buffer, m_img_size); }

  m_signal_update_image();
}

/** Set the button to reset the selection.
 * @param btn the reset selection button
 */
void
ColorTrainWidget::set_reset_selection_btn(Gtk::Button* btn)
{
  m_btn_reset_selection = btn;
  m_btn_reset_selection->signal_clicked().connect( sigc::mem_fun(*this, &ColorTrainWidget::reset_selection) );
}

/** Set the button to trigger the generation of the colormap.
 * @param btn a Button
 */
void
ColorTrainWidget::set_add_to_colormap_btn(Gtk::Button* btn)
{
  m_btn_add_to_colormap = btn;
  m_btn_add_to_colormap->signal_clicked().connect( sigc::mem_fun(*this, &ColorTrainWidget::add_to_colormap) );
}

/** Set the button to reset the colormap.
 * @param btn a Button
 */
void
ColorTrainWidget::set_reset_colormap_btn(Gtk::Button* btn)
{
  m_btn_reset_colormap = btn;
  m_btn_reset_colormap->signal_clicked().connect( sigc::mem_fun(*this, &ColorTrainWidget::reset_colormap) );
}

/** Set the buffon to open a dialog to load histograms.
 * @param btn a Button
 */
void
ColorTrainWidget::set_load_histos_btn(Gtk::Button* btn)
{
  m_btn_load_histos = btn;
  m_btn_load_histos->signal_clicked().connect( sigc::mem_fun(*this, &ColorTrainWidget::load_histograms) );
}

/** Set the buffon to open a dialog to save histograms.
 * @param btn a Button
 */
void
ColorTrainWidget::set_save_histos_btn(Gtk::Button* btn)
{
  m_btn_save_histos = btn;
  m_btn_save_histos->signal_clicked().connect( sigc::mem_fun(*this, &ColorTrainWidget::save_histograms) );
}

/** Set the buffon to open a dialog to load a colormap.
 * @param btn a Button
 */
void
ColorTrainWidget::set_load_colormap_btn(Gtk::Button* btn)
{
  m_btn_load_colormap = btn;
  m_btn_load_colormap->signal_clicked().connect( sigc::mem_fun(*this, &ColorTrainWidget::load_colormap) );
}

/** Set the buffon to open a dialog to save a colormap.
 * @param btn a Button
 */
void
ColorTrainWidget::set_save_colormap_btn(Gtk::Button* btn)
{
  m_btn_save_colormap = btn;
  m_btn_save_colormap->signal_clicked().connect( sigc::mem_fun(*this, &ColorTrainWidget::save_colormap) );
}

/** Set the image to render the colormap into.
 * @param img an Image
 */
void
ColorTrainWidget::set_colormap_img(Gtk::Image* img)
{
  m_cvw->set_colormap_img(img);
}

/** Set the image to render the segmented image into.
 * @param img an Image
 */
void
ColorTrainWidget::set_segmentation_img(Gtk::Image* img)
{
  m_img_segmentation = img;
  m_seg_img_max_width  = m_img_segmentation->get_width();
  m_seg_img_max_height = m_img_segmentation->get_height();
  m_img_segmentation->signal_size_allocate().connect( sigc::mem_fun( *this, &ColorTrainWidget::resize_seg_image) );
}

void
ColorTrainWidget::resize_seg_image(Gtk::Allocation& allocation)
{
  unsigned int new_width = (unsigned int) allocation.get_width();
  unsigned int new_height = (unsigned int) allocation.get_height();

  if (new_width != m_seg_img_max_width ||  new_height != m_seg_img_max_height)
    {
      m_seg_img_max_width = new_width;
      m_seg_img_max_height = new_height;
      draw_segmentation_result();
    }
}

/** Set the scale to control the selection threshold.
 * @param scl a Scale
 */
void
ColorTrainWidget::set_threshold_scl(Gtk::Scale* scl)
{
  m_scl_threshold = scl;
  m_scl_threshold->signal_change_value().connect( sigc::mem_fun(*this, &ColorTrainWidget::set_threshold) );
}

/** Set the scale to control the minimum probability.
 * @param scl a Scale
 */
void
ColorTrainWidget::set_min_prob_scl(Gtk::Scale* scl)
{
  m_scl_min_prob = scl;
  m_scl_min_prob->signal_change_value().connect( sigc::mem_fun(*this, &ColorTrainWidget::set_min_prob) );
}

/** Set the filechooser dialog to be used by this widget.
 * @param dlg a FileChooserDialog
 */
void
ColorTrainWidget::set_filechooser_dlg(Gtk::FileChooserDialog* dlg)
{
  m_fcd_filechooser = dlg;
}

/** Set the widget to choose the layer of the colormap to display.
 * @param scl a Scale
 */
void
ColorTrainWidget::set_cm_layer_selector(Gtk::Scale* scl)
{
  m_cvw->set_layer_selector(scl);
}

/** Set the widget to adjust the depth of the colormap.
 * @param depth SpinButton to set the Y-resolution of the color map
 * @param width SpinButton to set the U-resolution of the color map
 * @param height SpinButton to set the V-resolution of the color map
 */
void
ColorTrainWidget::set_cm_selector(Gtk::SpinButton* depth, Gtk::SpinButton* width, Gtk::SpinButton* height)
{
  m_spbtn_cm_depth = depth;
  m_spbtn_cm_width = width;
  m_spbtn_cm_height = height;
}

/** Access the signal that is emitted whenever a redraw of the image is necessary.
 * @return reference to a Dispatcher.
 */
Glib::Dispatcher&
ColorTrainWidget::update_image()
{
  return m_signal_update_image;
}

/** Access the signal that is emitted whenever the colormap has changed.
 * @return reference to a Dispatcher.
 */
Glib::Dispatcher&
ColorTrainWidget::colormap_updated()
{
  return m_signal_colormap_updated;
}

/** Open a dialog to load a histogram. */
void
ColorTrainWidget::load_histograms()
{
  if ( !m_fcd_filechooser )
    { return; }

  m_fcd_filechooser->set_title("Load histograms");
  m_fcd_filechooser->set_action(Gtk::FILE_CHOOSER_ACTION_OPEN);

  m_fcd_filechooser->set_transient_for(*m_wnd_parent);

  int result = m_fcd_filechooser->run();

   switch(result)
     {
     case (Gtk::RESPONSE_OK):
       {
	 std::string filename = m_fcd_filechooser->get_filename();
	 if (!m_generator)
	   { m_generator = new BayesColormapGenerator(); }
	 m_generator->load_histograms( filename.c_str() );
	 m_generator->calc();
	 m_signal_colormap_updated();

	 YuvColormap *cur = m_generator->get_current();
         if (m_spbtn_cm_depth) m_spbtn_cm_depth->set_value(log(cur->depth()) / log(2));
         if (m_spbtn_cm_width) m_spbtn_cm_width->set_value(log(cur->width()) / log(2));
         if (m_spbtn_cm_height) m_spbtn_cm_height->set_value(log(cur->height()) / log(2));

	 m_cvw->set_colormap(cur);
	 m_cvw->draw();
	 draw_segmentation_result();
 	break;
       }

     case (Gtk::RESPONSE_CANCEL):
       break;

     default:
       break;
     }

   m_fcd_filechooser->hide();
}

/** Open a dialog to save a histogram. */
void
ColorTrainWidget::save_histograms()
{
  if ( !m_fcd_filechooser )
    { return; }

  m_fcd_filechooser->set_title("Save histograms");
  m_fcd_filechooser->set_action(Gtk::FILE_CHOOSER_ACTION_SAVE);

  m_fcd_filechooser->set_transient_for(*m_wnd_parent);

  int result = m_fcd_filechooser->run();

   switch(result)
     {
     case (Gtk::RESPONSE_OK):
       {
	 std::string filename = m_fcd_filechooser->get_filename();
	 m_generator->save_histograms( filename.c_str() );
	 break;
       }

     case (Gtk::RESPONSE_CANCEL):
       break;

     default:
       break;
     }

   m_fcd_filechooser->hide();
}

/** Generate a new colormap by adding the current histograms. */
void
ColorTrainWidget::add_to_colormap()
{
  if ( !m_src_buffer )
    { return; }

  unsigned int cm_depth;
  if (m_spbtn_cm_depth)
    { cm_depth = (unsigned int) rint( pow(2.0, m_spbtn_cm_depth->get_value()) ); }
  else
    { cm_depth = 1; }

  unsigned int cm_width;
  if (m_spbtn_cm_width)
    { cm_width = (unsigned int) rint( pow(2.0, m_spbtn_cm_width->get_value()) ); }
  else
    { cm_width = 256; }

  unsigned int cm_height;
  if (m_spbtn_cm_height)
    { cm_height = (unsigned int) rint( pow(2.0, m_spbtn_cm_height->get_value()) ); }
  else
    { cm_height = 256; }

  if ( !m_generator
      || cm_depth  != m_generator->get_current()->depth()
      || cm_width  != m_generator->get_current()->width()
      || cm_height != m_generator->get_current()->height())
    {
      delete m_generator;
      m_generator = new BayesColormapGenerator(cm_depth, H_UNKNOWN, cm_width, cm_height);
      m_cvw->set_colormap( m_generator->get_current() );
    }

  if (m_fg_object == H_UNKNOWN)
    {
      printf("CTW::add_to_colormap(): no fg object set\n");
      return;
    }

  m_generator->set_fg_object(m_fg_object);
  m_generator->reset_undo();
  m_generator->set_buffer(m_src_buffer, m_img_width, m_img_height);
  m_generator->set_selection( m_zauberstab->getSelection() );
  m_generator->consider();
  m_generator->calc();
  m_signal_colormap_updated();

  // update colormap image
  m_cvw->draw(-1);

  // update segmentation image
  draw_segmentation_result();
}

/** Reset the colormap. */
void
ColorTrainWidget::reset_colormap()
{
  Gtk::MessageDialog dialog(*m_wnd_parent, "Are you sure you want to reset the colormap?",
                            false, Gtk::MESSAGE_QUESTION, Gtk::BUTTONS_OK_CANCEL);

  int result = dialog.run();

  if (result != Gtk::RESPONSE_OK) return;

  if (m_generator)
    {
      m_generator->reset();
      m_signal_colormap_updated();

      if (m_cvw)
	{ m_cvw->draw(); }

      draw_segmentation_result();
    }
}

/** Open a dialog to load a colormap. */
void
ColorTrainWidget::load_colormap()
{
  if ( !m_fcd_filechooser )
    { return; }

  m_fcd_filechooser->set_title("Load colormap colormap");
  m_fcd_filechooser->set_action(Gtk::FILE_CHOOSER_ACTION_OPEN);

  m_fcd_filechooser->set_transient_for(*m_wnd_parent);

  int result = m_fcd_filechooser->run();

   switch(result)
     {
     case (Gtk::RESPONSE_OK):
       {
	 delete m_generator;

	 std::string filename = m_fcd_filechooser->get_filename();
	 ColormapFile cmf;
	 cmf.read(filename.c_str());
	 Colormap *tcm = cmf.get_colormap();
	 YuvColormap *tycm = dynamic_cast<YuvColormap *>(tcm);
	 if ( ! tycm ) {
	   delete tcm;
	   throw fawkes::TypeMismatchException("File does not contain a YUV colormap");
	 }
         unsigned int cm_depth = tcm->depth();
         unsigned int cm_width = tcm->width();
         unsigned int cm_height = tcm->height();
	 m_generator = new BayesColormapGenerator(cm_depth, H_UNKNOWN, cm_width, cm_height);
	 YuvColormap *current = m_generator->get_current();
	 *current = *tycm;
	 delete tcm;

         if (m_spbtn_cm_depth) m_spbtn_cm_depth->set_value(log(cm_depth) / log(2));
         if (m_spbtn_cm_width) m_spbtn_cm_width->set_value(log(cm_width) / log(2));
         if (m_spbtn_cm_height) m_spbtn_cm_height->set_value(log(cm_height) / log(2));

	 m_signal_colormap_updated();
	 m_cvw->set_colormap( m_generator->get_current() );
	 m_cvw->draw();
	 draw_segmentation_result();
 	break;
       }

     case (Gtk::RESPONSE_CANCEL):
       break;

     default:
       break;
     }

   m_fcd_filechooser->hide();
}

/** Open a dialog to save a colormap. */
void
ColorTrainWidget::save_colormap()
{
  if ( !m_fcd_filechooser )
    { return; }

  m_fcd_filechooser->set_title("Save colormap colormap");
  m_fcd_filechooser->set_action(Gtk::FILE_CHOOSER_ACTION_SAVE);

  m_fcd_filechooser->set_transient_for(*m_wnd_parent);

  int result = m_fcd_filechooser->run();

  switch(result)
    {
    case(Gtk::RESPONSE_OK):
      {
        std::string filename = m_fcd_filechooser->get_filename();
	YuvColormap *current = m_generator->get_current();
	ColormapFile cmf(current->depth(), current->width(), current->height());
	cmf.add_colormap(current);
	cmf.write( filename.c_str() );
        break;
      }

    case(Gtk::RESPONSE_CANCEL):
      break;

    default:
      break;
    }

   m_fcd_filechooser->hide();
}

/** Get the current colormap.
 * @return the current colormap
 */
YuvColormap *
ColorTrainWidget::get_colormap() const
{
  if ( !m_generator )
    { return 0; }

  return m_generator->get_current();
}

bool
ColorTrainWidget::set_threshold(Gtk::ScrollType scroll, double value)
{
  unsigned int threshold = (unsigned int) rint(value);
  m_zauberstab->setThreshold(threshold);

  return true;
}

bool
ColorTrainWidget::set_min_prob(Gtk::ScrollType scroll, double value)
{
  if ( !m_generator )
    { return true; }

  m_generator->set_min_probability(value);

  return true;
}

void
ColorTrainWidget::reset_gui()
{
  m_scl_min_prob->set_value(0.0);
}

/** Render the result of segmenting the image in the source buffer considering the current
 * colormap into the specified Image.
 */
void
ColorTrainWidget::draw_segmentation_result()
{
  if ( !m_src_buffer || !m_img_segmentation || !m_generator)
    { return; }

  unsigned char* seg_buffer = (unsigned char*) malloc(m_img_size);
  bzero(seg_buffer, m_img_size);

  Drawer d;
  d.set_buffer(seg_buffer, m_img_width, m_img_height);

  YuvColormap* cm = m_generator->get_current();

  for (unsigned int w = 0; w < m_img_width; ++w)
    {
      for (unsigned int h = 0; h < m_img_height; ++h)
	{
	  unsigned int y = YUV422_PLANAR_Y_AT(m_src_buffer, m_img_width, w, h);
	  unsigned int u = YUV422_PLANAR_U_AT(m_src_buffer, m_img_width, m_img_height, w, h);
	  unsigned int v = YUV422_PLANAR_V_AT(m_src_buffer, m_img_width, m_img_height, w, h);

		d.set_color(ColorObjectMap::get_color(cm->determine(y, u, v)));
		d.color_point(w, h);
	}
    }

  LossyScaler scaler;
  scaler.set_original_buffer(seg_buffer);
  scaler.set_original_dimensions(m_img_width, m_img_height);
  scaler.set_scaled_dimensions(m_seg_img_max_width, m_seg_img_max_height);
  unsigned int width  = scaler.needed_scaled_width();
  unsigned int height = scaler.needed_scaled_height();

  unsigned char* scaled_buffer = (unsigned char*) malloc( colorspace_buffer_size( m_img_cs,
										  width,
										  height ) );
  scaler.set_scaled_buffer(scaled_buffer);
  scaler.scale();

  unsigned char* rgb_buffer = (unsigned char*) malloc( colorspace_buffer_size( RGB,
									       width,
									       height ) );
  convert(m_img_cs, RGB, scaled_buffer, rgb_buffer, width, height);

  Glib::RefPtr<Gdk::Pixbuf> image = Gdk::Pixbuf::create_from_data( rgb_buffer,
								   Gdk::COLORSPACE_RGB,
								   false,
								   8,
								   width,
								   height,
								   3 * width,
								   Gdk::Pixbuf::SlotDestroyData(&free_rgb_buffer));

  m_img_segmentation->set(image);

  free(scaled_buffer);
  free(seg_buffer);
}

/** Callback to free the rgb buffer
 * @param rgb_buffer pointer to the buffer
 */
void ColorTrainWidget::free_rgb_buffer(const guint8* rgb_buffer)
{
	free(const_cast<guint8 *>(rgb_buffer));
}
