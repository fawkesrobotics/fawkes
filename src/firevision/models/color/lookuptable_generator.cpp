
/**************************************************************************
 *  lookuptable_generator.cpp - interface for generating arbitrary color
 *                              lookup tables
 *
 *  Generated: Tue Mar 27 17:07:15 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ***************************************************************************/

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

#include <models/color/lookuptable_generator.h>
#include <models/color/lookuptable.h>
#include <fvutils/statistical/histogram.h>

/** @class ColorLutGenerator <models/color/lookuptable_generator.h>
 * Interface for color LUT generators.
 *
 * @fn void ColorLutGenerator::set_buffer(unsigned char *buffer, unsigned int width, unsigned int height)
 * Set image buffer.
 * Set the image buffer that is to be considered next.
 * @param buffer image buffer (YUV422 planar format assumed)
 * @param width width of image in pixels
 * @param height height of image in pixels
 *
 * @fn ColorModelLookupTable *  ColorLutGenerator::get_current()
 * Get the current LUT.
 * With this method you can access the current LUT. This is useful to display the
 * current results as "what would happen if we'd use this LUT?".
 * @return current colormap
 *
 * @fn void ColorLutGenerator::consider()
 * Considers the given buffer and extracts the needed information.
 * @see setBuffer()
 *
 * @fn void ColorLutGenerator::calc()
 * Calculate LUT.
 * Does the calculation of the lookup table without extracting any further information
 * from the given buffer.
 *
 * @fn void ColorLutGenerator::undo()
 * Undo last calls to consider().
 * This will eliminate all calls to consider() since the last call to
 * resetUndo(), reset() or object generation.
 *
 * @fn void ColorLutGenerator::reset()
 * Reset the generator.
 * This throws away all results accumulated up to now and starts from scratch
 * with the generation process.
 *
 * @fn void ColorLutGenerator::reset_undo()
 * Reset undo buffer.
 * This throws away all undo information and starts a new undo buffer.
 *
 * @fn bool ColorLutGenerator::has_histograms()
 * Check if this generator has histograms.
 * @return true, if this generator has histograms, false otherwise
 * 
 * @fn std::map< std::string, Histogram *> *  ColorLutGenerator::get_histograms()
 * Get histograms.
 * @return a map of histograms, if any.
 *
 */


/** Virtual empty destructor. */
ColorLutGenerator::~ColorLutGenerator()
{
}
