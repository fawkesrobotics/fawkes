
/***************************************************************************
 *  colormap_viewer_widget.cpp - Viewer widget for colormaps
 *
 *  Created: Thu Mar 20 19:08:04 2008
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

#include <tools/firestation/colormap_viewer_widget.h>
#include <fvutils/colormap/colormap.h>
#include <fvutils/scalers/lossy.h>
#include <fvutils/color/conversions.h>

using namespace firevision;

/** @class ColormapViewerWidget "colormap_viewer_widget.h"
 * Select a layer from a colormap and render it to a Gtk::Image.
 * @author Daniel Beck
 */

/** Constructor. */
ColormapViewerWidget::ColormapViewerWidget()
{
  m_cm = 0;
  m_img_colormap = 0;
  m_scl_layer_selector = 0;
  m_colormap_img_buf = 0;
}

/** Destructor. */
ColormapViewerWidget::~ColormapViewerWidget()
{
  free(m_colormap_img_buf);
}

/** Set the colormap to display.
 * @param cm colormap
 */
void
ColormapViewerWidget::set_colormap(Colormap *cm)
{
  m_cm = cm;

  if (m_scl_layer_selector)
    {
      double max = m_cm->deepness();
      m_scl_layer_selector->set_range(0.0, max);
      m_scl_layer_selector->set_increments(1.0, 1.0);
      m_scl_layer_selector->set_value(0.0);
    }
}

/** Set the image to render into.
 * @param img the Image
 */
void
ColormapViewerWidget::set_colormap_img(Gtk::Image* img)
{
  m_img_colormap = img;
}

/** Set the selector widget to choose the layer of the colormap which gets rendered.
 * @param scl a Gtk::Scale
 */
void
ColormapViewerWidget::set_layer_selector(Gtk::Scale* scl)
{
  m_scl_layer_selector = scl;

  double max;
  if (m_cm)
    { max = m_cm->deepness(); }
  else
    { max = 256.0; }
  m_scl_layer_selector->set_range(0.0, max);
  m_scl_layer_selector->set_increments(1.0, 1.0);
  m_scl_layer_selector->set_value(0.0);

  m_scl_layer_selector->signal_change_value().connect( sigc::mem_fun(*this, &ColormapViewerWidget::on_layer_selected) );
}

bool
ColormapViewerWidget::on_layer_selected(Gtk::ScrollType scroll, double value)
{
  unsigned int layer = (unsigned int) rint(value);
  draw(layer);

  return true;
}

/** Draw the colormap.
 * @param layer the plane in the third dimension of the colormap to be drawn
 */
void
ColormapViewerWidget::draw(unsigned int layer)
{
  if (m_cm == 0 || m_img_colormap == 0)
    { return; }

  if (layer >= m_cm->deepness() )
    {
      if (!m_scl_layer_selector) return;
      else layer = (unsigned int) rint(m_scl_layer_selector->get_value());
    }

  unsigned int cm_layer = (layer * m_cm->depth()) / m_cm->deepness();

  unsigned char* colormap_buffer = (unsigned char*) malloc( colorspace_buffer_size(YUV422_PLANAR, m_cm->image_width(), m_cm->image_height()) );
  m_cm->to_image(colormap_buffer, cm_layer);

  unsigned int img_width  = (unsigned int) m_img_colormap->get_width();
  unsigned int img_height = (unsigned int) m_img_colormap->get_height();

  img_width  = (img_width < img_height) ? img_width : img_height;
  img_height = (img_width < img_height) ? img_width : img_height;

  // scale
  LossyScaler scaler;
  scaler.set_original_buffer(colormap_buffer);
  scaler.set_original_dimensions(m_cm->image_width(), m_cm->image_height());
  scaler.set_scaled_dimensions(img_width, img_height);
  //unsigned int scaled_width  = scaler.needed_scaled_width();
  //unsigned int scaled_height = scaler.needed_scaled_height();
  unsigned char* scaled_colormap_buffer = (unsigned char*) malloc( colorspace_buffer_size(YUV422_PLANAR, img_width, img_height) );
  scaler.set_scaled_buffer(scaled_colormap_buffer);
  scaler.scale();

  free(m_colormap_img_buf);
  m_colormap_img_buf = (unsigned char*) malloc( colorspace_buffer_size(RGB, img_width, img_height) );
  convert(YUV422_PLANAR, RGB, scaled_colormap_buffer, m_colormap_img_buf, img_width, img_height);

  Glib::RefPtr<Gdk::Pixbuf> colormap_image =
    Gdk::Pixbuf::create_from_data( m_colormap_img_buf,
				   Gdk::COLORSPACE_RGB,
				   false,
				   8,
				   img_width, img_height,
				   3 * img_width);
  m_img_colormap->set(colormap_image);

  free(colormap_buffer);
  free(scaled_colormap_buffer);
}
