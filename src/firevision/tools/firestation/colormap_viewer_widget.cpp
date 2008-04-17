
/***************************************************************************
 *  colormap_viewer_widget.cpp - Viewer widget for colormaps
 *
 *  Created: Thu Mar 20 19:08:04 2008
 *  Copyright  2008  Daniel Beck
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

#include <tools/firestation/colormap_viewer_widget.h>
#include <models/color/lookuptable.h>
#include <fvutils/scalers/lossy.h>
#include <fvutils/color/conversions.h>

/** @class ColormapViewerWidget <tools/firestation/colormap_viewer_widget.h>
 * Displays a colormap in an Gtk::Image.
 * @author Daniel Beck
 */

/** Constructor. */
ColormapViewerWidget::ColormapViewerWidget()
{
  m_cm = 0;
  m_img_colormap = 0;
  m_colormap_img_buf = 0;
}

/** Destructor. */
ColormapViewerWidget::~ColormapViewerWidget()
{
  free(m_colormap_img_buf);
  delete m_img_colormap;
}

/** Set the colormap to display.
 * @param cm colormap
 */
void
ColormapViewerWidget::set_colormap(YuvColormap *cm)
{
  m_cm = cm;
}

/** Set the image to render into.
 * @param img the Image
 */
void
ColormapViewerWidget::set_colormap_img(Gtk::Image* img)
{
  m_img_colormap = img;
}

/** Draw the COLORMAP. 
 * @param y_layer the layer on the Y axis to be drawn
 */
void
ColormapViewerWidget::draw(unsigned int y_layer)
{
  if (m_cm == 0 || m_img_colormap == 0)
    { return; }

  if ( y_layer < 0 || y_layer > 255 )
    { return; }

  unsigned int layer = (y_layer * m_cm->depth()) / 255;

  unsigned char* colormap_buffer = (unsigned char*) malloc( colorspace_buffer_size(YUV422_PLANAR, m_cm->width() * 2, m_cm->height() * 2) );
  m_cm->to_image(colormap_buffer, layer);

  unsigned int img_width = (unsigned int) m_img_colormap->get_width();
  unsigned int img_height = (unsigned int) m_img_colormap->get_height();

  img_width = (img_width < img_height) ? img_width : img_height;
  img_height = (img_width < img_height) ? img_width : img_height;
  
  // scale
  LossyScaler scaler;
  scaler.set_original_buffer(colormap_buffer);
  scaler.set_original_dimensions(m_cm->width() * 2, m_cm->height() * 2);
  scaler.set_scaled_dimensions(img_width, img_height);
  unsigned char* scaled_colormap_buffer = (unsigned char*) malloc( colorspace_buffer_size(YUV422_PLANAR, img_width, img_height) );
  scaler.set_scaled_buffer(scaled_colormap_buffer);
  scaler.scale();
  
  free(m_colormap_img_buf);
  m_colormap_img_buf = (unsigned char*) malloc( colorspace_buffer_size(RGB, img_width, img_height) );
  convert(YUV422_PLANAR, RGB, scaled_colormap_buffer, m_colormap_img_buf, img_width, img_height);
  
  Glib::RefPtr<Gdk::Pixbuf> colormap_image = Gdk::Pixbuf::create_from_data( m_colormap_img_buf,
									    Gdk::COLORSPACE_RGB,
									    false,
									    8,
									    img_width, img_height,
									    3 * img_width);
  m_img_colormap->set(colormap_image);

  free(colormap_buffer);
  free(scaled_colormap_buffer);
}
