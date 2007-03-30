
/***************************************************************************
 *  colormodel.cpp - Abstract class defining a color model
 *
 *  Generated: Wed Mar 21 18:38:17 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <models/color/colormodel.h>

/** @class ColorModel <models/color/colormodel.h>
 * Color model interface.
 * This interface defines the API for color models.
 *
 * @fn color_t ColorModel::determine(unsigned int y, unsigned int u, unsigned int v) const
 * Determine classification of YUV pixel.
 * Given a pixel in the YUV colorspace the colormodel determines the color
 * classification based on some a-priori knowledge.
 * @param y Y value
 * @param u U value
 * @param v V value
 *
 * @fn const char * ColorModel::getName()
 * Get name of color model.
 * @return name of color model.
 *
 * @author Tim Niemueller
 */


/** Virtual empty destructor. */
ColorModel::~ColorModel()
{
}
