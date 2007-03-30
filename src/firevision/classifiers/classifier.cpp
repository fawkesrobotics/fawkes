
/***************************************************************************
 *  classifier.h - Abstract class defining a (color) classifier
 *
 *  Generated: Tue May 03 19:50:02 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#include <classifiers/classifier.h>

/** @class Classifier <classifiers/classifier.h>
 * Classifier interface.
 * The classifier finds regions of interest (ROI) by some
 * a priori knowledge like known colors or shapes.
 * The list of ROIs returned by classify() _must_
 * be disjunct, meaning that no ROIs overlap each other.
 * Do appropriate merging or shrinking of the ROIs. See
 * the ReallySimpleClassifier for an example.
 *
 * @fn Classifier::setSrcBuffer(unsigned char *buf)
 * Set the src buffer.
 * @param buf buffer
 *
 * @fn const char * Classifier::getName() const
 * Get name of classifier
 * @return name of classifier
 *
 * @fn std::list< ROI > * Classifier::classify()
 * Classify image.
 * @return list of ROIs from classified image. You must free the memory
 * after usage on your own!
 */

/** Virtual empty destructor. */
Classifier::~Classifier()
{
}
