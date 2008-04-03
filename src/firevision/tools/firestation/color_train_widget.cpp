
/***************************************************************************
 *  color_train_widget.cpp - Color training widget
 *
 *  Created: Thu Mar 20 22:19:36 2008
 *  Copyright  2006  Daniel Beck
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

#include <tools/firestation/color_train_widget.h>
#include <tools/firestation/lut_viewer_widget.h>
#include <fvutils/color/yuv.h>
#include <fvutils/color/zauberstab.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>
#include <fvutils/draw/drawer.h>
#include <fvutils/scalers/lossy.h>
#include <fvutils/colormap/bayes/bayes_generator.h>
#include <fvutils/colormap/yuvcm.h>
#include <fvutils/colormap/cmfile.h>

#include <core/exceptions/software.h>

/** @class ColorTrainWidget tools/firestation/color_train_widget.h
 * This widget implements the complete color training process.
 *
 * @author Daniel Beck
 */

/** Constructor.
 * @param parent the parent window
 */
ColorTrainWidget::ColorTrainWidget(Gtk::Window* parent)
{
  m_lut_depth = 8;

  m_generator = new BayesColormapGenerator(m_lut_depth);
  m_zauberstab = new Zauberstab();
  m_lvw = new LutViewerWidget();
  m_lvw->set_colormap( m_generator->get_current() );

  m_src_buffer = 0;
  m_draw_buffer = 0;

  m_wnd_parent = parent;
  m_btn_reset_selection = 0;
  m_btn_add_to_lut = 0;
  m_btn_reset_lut = 0;
  m_btn_load_histos = 0;
  m_btn_save_histos = 0;
  m_btn_load_lut = 0;
  m_btn_save_lut = 0;
  m_img_segmentation = 0;
  m_scl_threshold = 0;
  m_scl_min_prob = 0;
  m_fcd_filechooser = 0;

  m_update_img = 0;
}

/** Destructor. */
ColorTrainWidget::~ColorTrainWidget()
{
  delete m_lvw;
  delete m_generator;
  delete m_zauberstab;
}

/** Set the current foreground object.
 * @param object the foreground object
 */
void
ColorTrainWidget::set_fg_object(hint_t object)
{
  m_fg_object = object;
  m_generator->set_fg_object(m_fg_object);
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
  m_img_ratio = m_img_width / float(m_img_width);
  m_src_buffer = yuv422_buffer;
  m_img_cs = YUV422_PLANAR;
  m_img_size = colorspace_buffer_size( m_img_cs, m_img_width, m_img_height );

  m_zauberstab->setBuffer(m_src_buffer, m_img_width, m_img_height);
  m_zauberstab->setThreshold(10);
}

/** Set the buffer to draw the selection into.
 * It is assumed that this buffer has the same dimensions as the buffer holding the soruce
 * image.
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
 */
void
ColorTrainWidget::click(unsigned int x, unsigned int y)
{
  if (m_src_buffer == 0 || m_draw_buffer == 0)
    { return; } 

  if ( m_zauberstab->isEmptyRegion() )
    {
      m_zauberstab->findRegion(x, y);
    }
  else
    {
      m_zauberstab->addRegion(x, y);
    }
  
  memcpy(m_draw_buffer, m_src_buffer, m_img_size);
  
  ZRegion *region = m_zauberstab->getRegion();
  Drawer *d = new Drawer();
  d->setBuffer( m_draw_buffer, m_img_width, m_img_height );	  
  
  for (unsigned int s = 0; s < region->slices->size(); s++) 
    {
      d->drawRectangleInverted( region->slices->at(s)->leftX,
				region->slices->at(s)->y,
				region->slices->at(s)->rightX - region->slices->at(s)->leftX,
				1 ); 
    }
  
  delete d;

  if ( m_update_img )
    { m_update_img->emit(); }
}

/** Reset the selection. */
void
ColorTrainWidget::reset_selection()
{
  if (m_zauberstab)
    { m_zauberstab->deleteRegion(); }

  if( m_src_buffer && m_draw_buffer )
    { memcpy(m_draw_buffer, m_src_buffer, m_img_size); }

  if ( m_update_img )
    { m_update_img->emit(); }
}

/** Set the depth of the LUT that is generated.
 * Note: this invalidates the pointer returned by get_lut().
 * @param lut_depth the depth of the LUT
 */
void
ColorTrainWidget::set_lut_depth(unsigned int lut_depth)
{
  m_lut_depth = lut_depth;
  
  delete m_generator;
  m_generator = new BayesColormapGenerator(m_lut_depth);
  m_lvw->set_colormap( m_generator->get_current() );
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

/** Set the button to trigger the generation of the LUT.
 * @param btn a Button
 */
void
ColorTrainWidget::set_add_to_lut_btn(Gtk::Button* btn)
{
  m_btn_add_to_lut = btn;
  m_btn_add_to_lut->signal_clicked().connect( sigc::mem_fun(*this, &ColorTrainWidget::add_to_lut) );
}

/** Set the button to reset the LUT.
 * @param btn a Button
 */
void
ColorTrainWidget::set_reset_lut_btn(Gtk::Button* btn)
{
  m_btn_reset_lut = btn;
  m_btn_reset_lut->signal_clicked().connect( sigc::mem_fun(*this, &ColorTrainWidget::reset_lut) );
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

/** Set the buffon to open a dialog to load a LUT.
 * @param btn a Button
 */
void
ColorTrainWidget::set_load_lut_btn(Gtk::Button* btn)
{
  m_btn_load_lut = btn;
  m_btn_load_lut->signal_clicked().connect( sigc::mem_fun(*this, &ColorTrainWidget::load_lut) );
}

/** Set the buffon to open a dialog to save a LUT.
 * @param btn a Button
 */
void
ColorTrainWidget::set_save_lut_btn(Gtk::Button* btn)
{
  m_btn_save_lut = btn;
  m_btn_save_lut->signal_clicked().connect( sigc::mem_fun(*this, &ColorTrainWidget::save_lut) );
}

/** Set the image to render the LUT into.
 * @param img an Image
 */
void
ColorTrainWidget::set_lut_img(Gtk::Image* img)
{
  m_lvw->set_lut_img(img);
}

/** Set the image to render the segmented image into.
 * @param img an Image
 */
void
ColorTrainWidget::set_segmentation_img(Gtk::Image* img)
{
  m_img_segmentation = img;
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

/** Set the signal that is emmitted whenever the draw buffer was changed and a redraw is
 * necessary.
 *@param update_img a Dispatcher
 */
void
ColorTrainWidget::set_update_img_signal(Glib::Dispatcher* update_img)
{
  m_update_img = update_img;
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
	 m_generator->load_histograms( filename.c_str() );

	 m_lvw->draw();
	 // draw_segmentation_result();
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

/** Generate a new LUT by adding the current histograms. */
void
ColorTrainWidget::add_to_lut()
{
  if ( !m_src_buffer )
    { return; }

  if (m_fg_object == H_UNKNOWN)
    {
      printf("CTW::add_to_lut(): no fg object set\n");
      return;
    }

  m_generator->reset_undo();
  m_generator->set_buffer(m_src_buffer, m_img_width, m_img_height);
  m_generator->set_selection( m_zauberstab->getSelection() );
  m_generator->consider();
  m_generator->calc();

  // update lut image
  m_lvw->draw();

  // update segmentation image
  draw_segmentation_result();
}

/** Reset the LUT. */
void
ColorTrainWidget::reset_lut()
{
  // TODO
}

/** Open a dialog to load a LUT. */
void
ColorTrainWidget::load_lut()
{
  if ( !m_fcd_filechooser )
    { return; }

  m_fcd_filechooser->set_title("Load colormap LUT");
  m_fcd_filechooser->set_action(Gtk::FILE_CHOOSER_ACTION_OPEN);
  
  m_fcd_filechooser->set_transient_for(*m_wnd_parent);

  int result = m_fcd_filechooser->run();

   switch(result)
     {
     case (Gtk::RESPONSE_OK):
       {
	 std::string filename = m_fcd_filechooser->get_filename();
	 ColormapFile cmf;
	 cmf.read(filename.c_str());
	 Colormap *tcm = cmf.get_colormap();
	 YuvColormap *tycm = dynamic_cast<YuvColormap *>(tcm);
	 if ( ! tycm ) {
	   delete tcm;
	   throw TypeMismatchException("File does not contain a YUV colormap");
	 }
	 YuvColormap *current = m_generator->get_current();
	 *current = *tycm;
	 delete tcm;

	 m_lvw->draw();
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

/** Open a dialog to save a LUT. */
void
ColorTrainWidget::save_lut()
{
  if ( !m_fcd_filechooser )
    { return; }

  m_fcd_filechooser->set_title("Save colormap LUT");
  m_fcd_filechooser->set_action(Gtk::FILE_CHOOSER_ACTION_SAVE);
  
  m_fcd_filechooser->set_transient_for(*m_wnd_parent);

  int result = m_fcd_filechooser->run();

  switch(result)
    {
    case(Gtk::RESPONSE_OK):
      {
        std::string filename = m_fcd_filechooser->get_filename();
	ColormapFile cmf;
	cmf.add_colormap(m_generator->get_current());
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

/** Get the current LUT.
 * @return the current LUT
 */
YuvColormap *
ColorTrainWidget::get_colormap() const
{
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
  m_generator->set_min_probability(value);

  return true;
}

/** Render the result of segmenting the image in the source buffer considering the current
 * LUT into the specified Image.
 */
void
ColorTrainWidget::draw_segmentation_result()
{
  if ( !m_src_buffer || !m_img_segmentation ) 
    { return; }

  unsigned char* seg_buffer = (unsigned char*) malloc(m_img_size);
  bzero(seg_buffer, m_img_size);

  Drawer* d = new Drawer();
  d->setBuffer(seg_buffer, m_img_width, m_img_height);
  
  YuvColormap* cm = m_generator->get_current();
  
  for (unsigned int w = 0; w < m_img_width; ++w)
    {
      for (unsigned int h = 0; h < m_img_height; ++h)
	{
	  unsigned int y = YUV422_PLANAR_Y_AT(m_src_buffer, m_img_width, w, h);
	  unsigned int u = YUV422_PLANAR_U_AT(m_src_buffer, m_img_width, m_img_height, w, h);
	  unsigned int v = YUV422_PLANAR_V_AT(m_src_buffer, m_img_width, m_img_height, w, h);

	  color_t result;
	  result = cm->determine(y, u, v);				  
	  
	  if (C_GREEN == result)
	    {
	      d->setColor(240, 0, 0);
	    }
	  else if (C_ORANGE == result)
	    {
	      d->setColor(127, 30, 230);
	    }
	  else if (C_BACKGROUND == result)
	    {
	      d->setColor(50, 127, 127);
	    }
	  else if (C_WHITE == result)
	    {
	      d->setColor(255, 128, 128);
	    }
	  else
	    {
	      d->setColor(0, 127, 127);
	    }

	  d->colorPoint(w, h);
	}
    }

  // scale, convert to RGB, draw image
  unsigned int width = (unsigned int) m_img_segmentation->get_width();
  unsigned int height = (unsigned int) m_img_segmentation->get_height();
  float ratio = width / float(height);
  
  if (ratio < m_img_ratio)
    { height = (unsigned int) rint(width * m_img_ratio); }
  else if (ratio > m_img_ratio)
    { width = (unsigned int) rint(height / m_img_ratio); }
  
  LossyScaler scaler;
  scaler.set_original_buffer(seg_buffer);
  scaler.set_original_dimensions(m_img_width, m_img_height);
  scaler.set_scaled_dimensions(width, height);

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
								   3 * height );
  free(seg_buffer);
  free(scaled_buffer);
  free(rgb_buffer);

  m_img_segmentation->set(image);
}
