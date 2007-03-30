
/***************************************************************************
 *  writer.cpp - Writer interface
 *
 *  Generated: Tue Mar 27 17:24:55 2007
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

#include <fvutils/writers/writer.h>

/** @class Writer <fvutils/writers/writer.h>
 * Interface to write images.
 * The writer interface defines the general API for image writers. These
 * writers are used to write images to files on your harddrive (like JPEGs,
 * PNGs etc.).
 *
 * @author Tim Niemueller
 *
 * @fn void Writer::set_filename(const char *filename)
 * Set filename.
 * @param filename name of file to write to
 *
 * @fn void Writer::set_dimensions(unsigned int width, unsigned int height)
 * Set dimensions of image in pixels.
 * @param width width of image in pixels
 * @param height height of image in pixels.
 *
 * @fn void Writer::set_buffer(colorspace_t cspace, unsigned char *buffer)
 * Set image buffer.
 * @param cspace color space of image
 * @param buffer buffer of image
 *
 * @fn void Writer::write()
 * Write to file.
 */

/** Virtual empty destructor. */
Writer::~Writer()
{
}
