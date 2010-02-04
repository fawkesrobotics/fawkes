
/**************************************************************************
 *  lookuptable_generator.cpp - interface for generating arbitrary color
 *                              lookup tables
 *
 *  Generated: Tue Mar 27 17:07:15 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 ***************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <fvutils/colormap/generator.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColormapGenerator <fvutils/colormap/generator.h>
 * Interface for colormap generators.
 *
 * @fn void ColormapGenerator::set_buffer(unsigned char *buffer, unsigned int width, unsigned int height)
 * Set image buffer.
 * Set the image buffer that is to be considered next.
 * @param buffer image buffer (YUV422 planar format assumed)
 * @param width width of image in pixels
 * @param height height of image in pixels
 *
 * @fn Colormap *  ColormapGenerator::get_current()
 * Get the current colormap.
 * With this method you can access the current LUT. This is useful to display the
 * current results as "what would happen if we'd use this LUT?".
 * @return current colormap
 *
 * @fn void ColormapGenerator::consider()
 * Considers the given buffer and extracts the needed information.
 * @see set_buffer()
 *
 * @fn void ColormapGenerator::calc()
 * Calculate LUT.
 * Does the calculation of the lookup table without extracting any further information
 * from the given buffer.
 *
 * @fn void ColormapGenerator::undo()
 * Undo last calls to consider().
 * This will eliminate all calls to consider() since the last call to
 * resetUndo(), reset() or object generation.
 *
 * @fn void ColormapGenerator::reset()
 * Reset the generator.
 * This throws away all results accumulated up to now and starts from scratch
 * with the generation process.
 *
 * @fn void ColormapGenerator::reset_undo()
 * Reset undo buffer.
 * This throws away all undo information and starts a new undo buffer.
 *
 * @fn bool ColormapGenerator::has_histograms()
 * Check if this generator has histograms.
 * @return true, if this generator has histograms, false otherwise
 * 
 * @fn std::map< std::string, Histogram *> *  ColormapGenerator::get_histograms()
 * Get histograms.
 * @return a map of histograms, if any.
 *
 */


/** Virtual empty destructor. */
ColormapGenerator::~ColormapGenerator()
{
}

} // end namespace firevision
