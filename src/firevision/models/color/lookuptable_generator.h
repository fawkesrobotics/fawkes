
/**************************************************************************
 *  lookuptable_generator.h - interface for generating arbitrary color
 *                            lookup tables
 *
 *  Generated: Wed Mar 01 13:51:39 2006
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

#ifndef __FIREVISION_MODELS_COLOR_LOOKUPTABLE_GENERATOR_
#define __FIREVISION_MODELS_COLOR_LOOKUPTABLE_GENERATOR_

#include <string>
#include <map>

class ColorModelLookupTable;
class Histogram2D;

class ColorLutGenerator
{

 public:

  virtual ~ColorLutGenerator();

  virtual void                     setBuffer(unsigned char *buffer,
					     unsigned int width, unsigned int height)  = 0;
  virtual ColorModelLookupTable *  getCurrent()                                        = 0;

  /** Considers the given buffer and extracts the needed information
   */
  virtual void                     consider()                                          = 0;

  /** Does the calculation of the lookup table without extracting any further information
   * from the given buffer.
   */
  virtual void                     calc()                                              = 0;
  virtual void                     undo()                                              = 0;
  virtual void                     reset()                                             = 0;
  virtual void                     resetUndo()                                         = 0;

  virtual bool                     hasHistograms()                                     = 0;
  virtual std::map< std::string, Histogram2D *> *  getHistograms()                     = 0;

};


#endif
