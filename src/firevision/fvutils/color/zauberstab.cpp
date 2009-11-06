
/***************************************************************************
 *  zauberstab.cpp - Implementation of class "Zauberstab"
 *                   which offers methods for finding 
 *                   maximal, color-contiguous region
 *                   around a seed pixel
 *
 *  Created: Mon Jul 02 2005
 *  Copyright  2005       Martin Heracles  <Martin.Heracles@rwth-aachen.de>
 *             2005-2008  Tim Niemueller   [www.niemueller.de]
 *
 ****************************************************************************/

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

#include <fvutils/color/zauberstab.h>
#include <fvutils/color/yuv.h>
#include <core/macros.h>

#include <cstdlib>
#include <iostream>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Zauberstab <fvutils/color/zauberstab.h>
 * Zaubertab selection utility.
 */

/** Constructor. */
ZRegion::ZRegion()
{
  topSliceY = 0;
  slices = new vector<ZSlice*>();
  slices->clear();
}

/** Constructor. */
ZRegion::~ZRegion()
{
	for (std::vector<ZSlice*>::iterator it = slices->begin(); it != slices->end(); ++it)
	{
		delete (*it);
	}
	
	delete slices;
}

/** Clears all slices.
 */
void
ZRegion::clear()
{
	for (std::vector<ZSlice*>::iterator it = slices->begin(); it != slices->end(); ++it)
	{
		delete (*it);
	}

	slices->clear();
}

/** @class Zauberstab <fvutils/color/zauberstab.h>
 * Zaubertab selection utility.
 */

/** Constructor. */
Zauberstab::Zauberstab() {
  // create empty region
  region = new ZRegion();

  buffer = NULL;
  width = 0;
  height = 0;

  /* by default, "Zauberstab" does not allow
     any deviation from seed color */
  this->threshold = 0 ;
}


/** Destructor. */
Zauberstab::~Zauberstab() {
  delete(region);
}


/** Set threshold.
 * @param t new threshold
 */
void
Zauberstab::setThreshold(unsigned int t) {
  this->threshold = t;
}


/** Get threshold.
 * @return threshold
 */
unsigned int
Zauberstab::getThreshold() {
  return this->threshold;
}


/** Set buffer to work on.
 * @param b buffer
 * @param w width of image
 * @param h height of buffer
 */
void 
Zauberstab::setBuffer(unsigned char *b,
		      unsigned int w,
		      unsigned int h) {
  this->buffer = b;
  this->width = w;
  this->height = h;
}


/** Check if region is empty.
 * @return true if empty
 */
bool
Zauberstab::isEmptyRegion() {
  return (region->slices->size() == 0);
}


/** Delete all regions. */
void
Zauberstab::deleteRegion() {
  region->clear();
}

/** Delete region.
 * @param seedX seed x
 * @param seedY seed y
 */
void
Zauberstab::deleteRegion(unsigned int seedX, unsigned int seedY)
{
  // STEP 1:
  // find the region
  ZRegion* region2 = privFindRegion (seedX, seedY);

  // STEP 2:
  // now delete the newly found region2 from the original region
  deleteRegion(region2);

  delete region2;
}


/** Delete region.
 * @param region2 region to delete
 */
void
Zauberstab::deleteRegion(ZRegion *region2)
{
  ZSlice* nSlice; //A slice to be deleted
  ZSlice* oSlice; //A slice currently in the region

	// delete each slice of region 2 from region
	while (region2->slices->size())
	{
		/* check if current slice from region 2 is at
			 at a height different from all slices at region */
    nSlice = region2->slices->back();
    region2->slices->pop_back();
		int heightOfSlice = nSlice->y;

		unsigned int i = 0;
		unsigned int size = region->slices->size();
		
		while(i < size) //for all existing slices (but not the newly added)
		{
			oSlice = region->slices->at(i);
			if (oSlice->y == heightOfSlice) //same height check for overlapping
			{
				if ((oSlice->leftX >= nSlice->leftX) 
						&& (oSlice->leftX < nSlice->rightX))
				{
					//The slice to delete starts before the slice to be deleted
					if (oSlice->rightX > nSlice->rightX) //A part of the region remains
					{
						oSlice->leftX = nSlice->rightX;
					}
					else //The whole slice dissapears
					{
						region->slices->erase(region->slices->begin() + i);
						size--;
						delete oSlice;

						//The index now points to the next element in the region->slices vector
						continue;
					}
				}

				if ((nSlice->leftX >= oSlice->leftX) 
						&& (nSlice->leftX < oSlice->rightX))
				{
					//The slice to be deleted starts before the part that should be deleted
					if (oSlice->rightX <= nSlice->rightX)
					{//just truncate the old slices
						oSlice->rightX = nSlice->leftX;
					}
					else //split the old spice
					{
						ZSlice* newPart = new ZSlice;
						newPart->rightX = oSlice->rightX;
						newPart->leftX = nSlice->rightX;
						newPart->y = heightOfSlice;

						oSlice->rightX = nSlice->leftX;
						region->slices->push_back(newPart);
					}
				}
			}

			i++;
		}
	}
}

/** A private region finder
 * @param seedX seed x
 * @param seedY seed y
 * @return a ZRegion pointer (has to be deleted by the caller)
 */
ZRegion*
Zauberstab::privFindRegion (unsigned int seedX, unsigned int seedY)
{
  unsigned char py __unused;
  unsigned char pu = 0;
  unsigned char pv = 0;

  // STEP 1:
  // first of all find the region around (seedX, seedY)
  // (this is analogously to method "findRegion")
  // (could be done more elegantly without the following redundant code)

  // create empty region
  ZRegion *region2 = new ZRegion();

  /* find seed pixel's v-value
     (consider seed pixel's neighborhood
      and take average v-value) */
  unsigned int uSeed = 0;
  unsigned int vSeed = 0;
  unsigned int cnt = 0;

  for (int x = seedX - 2; x <= (int)seedX + 2; ++x) {
    if (x < 0) continue;
    if ((unsigned int )x >= width) continue;
    for (int y = seedY - 2; y <= (int)seedY + 2; ++y) {
      if (y < 0) continue;
      if ((unsigned int)y >= height) continue;
      YUV422_PLANAR_YUV(buffer, width, height, x, y, py, pu, pv);
      uSeed += pu;
      vSeed += pv;
      ++cnt;
    }
  }

	if (cnt) 
	{
		uSeed = uSeed / cnt;
		vSeed = vSeed / cnt;
	}
  /* initial slice 
     thru seed pixel (seedX, seedY) */
  ZSlice *tmp = NULL;
  tmp = findSlice(seedX, seedY, vSeed, uSeed);
  region2->slices->push_back(tmp);

  /* NOTE: The following search works fine for
     objects that are convex (such as ball, goal, ...).
     For non-convex objects it may miss parts
     (e.g. for a U-shaped object it can only find right or left half). */

  // search downwards for more slices
  tmp = region2->slices->front();
  int tmpY = ((int)seedY >= (int)(height - 1)) ? height -1 : seedY + 1;
  // new "seed" pixel has x-coordinate in the middle of initial slice
  int tmpX = int(float(tmp->leftX + tmp->rightX) / 2.0);

  YUV422_PLANAR_YUV(buffer, width, height, tmpX, tmpY, py, pu, pv);
  while (isSimilarUV(pu, uSeed, pv, vSeed)) {
    tmp = findSlice(tmpX, tmpY, vSeed, uSeed);
    region2->slices->push_back(tmp);
    // new "seed" pixel has x-coordinate in the middle of previous slice
    tmpX = int(float(tmp->leftX + tmp->rightX) / 2.0);
    tmpY++;
    if (tmpY >= (int)this->height) {
      break;
    } else {
      YUV422_PLANAR_YUV(buffer, width, height, tmpX, tmpY, py, pu, pv);
    }
  }

  /* search upwards for more slices
     (start search from initial slice again) */
  tmp = region2->slices->front();
  tmpY = (seedY == 0) ? 0 : seedY - 1;
  // new "seed" pixel has x-coordinate in the middle of initial slice
  tmpX = int(float(tmp->leftX + tmp->rightX) / 2.0);

  YUV422_PLANAR_YUV(buffer, width, height, tmpX, tmpY, py, pu, pv);
  while (isSimilarUV(pu, uSeed, pv, vSeed)) {
    tmp = findSlice(tmpX, tmpY, vSeed, uSeed);
    region2->slices->push_back(tmp);
    // new "seed" pixel has x-coordinate in the middle of previous slice
    tmpX = int(float(tmp->leftX + tmp->rightX) / 2.0);
    tmpY--;
    if (tmpY < 0) {
      break;
    } else {
      YUV422_PLANAR_YUV(buffer, width, height, tmpX, tmpY, py, pu, pv);
    }
  }
  
  region2->topSliceY = tmpY + 1;

	for (std::vector<ZSlice*>::iterator it = region2->slices->begin(); it != region2->slices->end(); ++it)
	{
		cout << "start x: " << ((*it)->leftX) << " end x: " << ((*it)->rightX) << " y: " << ((*it)->y) << endl;
	}
	cout << endl;
  return region2;
}

/** Find region.
 * @param seedX seed x
 * @param seedY seed y
 */
void
Zauberstab::findRegion(unsigned int seedX, unsigned int seedY) {
  if (buffer == NULL) return;
  if (width == 0) return;
  if (height == 0) return;

  delete region;
  region = privFindRegion(seedX, seedY);
}


/** Add region.
 * @param seedX seed x
 * @param seedY seed y
 */
void
Zauberstab::addRegion(unsigned int seedX, unsigned int seedY)
{
  // STEP 1:
  // find the region
  ZRegion* region2 = privFindRegion (seedX, seedY);

  // STEP 2:
  // now merge the newly found region2 with the original region
  addRegion(region2);

  delete region2;
}


/** Find specific slice.
 * @param x x
 * @param y y
 * @param vSeed v seed
 * @return slice
 */
ZSlice*
Zauberstab::findSlice(unsigned int x, unsigned int y,
                      unsigned int vSeed, int uSeed)
{
  // slice with single pixel (x, y)
  ZSlice *slice = new ZSlice;
  slice->y = y;
  slice->leftX = x;
  slice->rightX = x;

  unsigned char py __unused;
  unsigned char pu=0;
  unsigned char pv=0;
  int tmpX = x + 1;

  if ((unsigned int)tmpX < width)
  {
    YUV422_PLANAR_YUV(buffer, width, height, tmpX, y, py, pu, pv);

    // search to the right
		while (uSeed >= 0 ? isSimilarUV(pu, uSeed, pv, vSeed) : isSimilarV(pv, vSeed)) {
      (slice->rightX)++;
      tmpX++;
      if (tmpX >= (int)this->width) {
        break;
      } else {
        YUV422_PLANAR_YUV(buffer, width, height, tmpX, y, py, pu, pv);
      }
    };
  }

  // search to the left
  tmpX = x - 1;
  if (tmpX >= 0)
  {
    YUV422_PLANAR_YUV(buffer, width, height, tmpX, y, py, pu, pv);
    while (uSeed >= 0 ? isSimilarUV(pu, uSeed, pv, vSeed) : isSimilarV(pv, vSeed)) {
      (slice->leftX)--;
      tmpX--;
      if (tmpX < 0) {
        break;
      } else {
        YUV422_PLANAR_YUV(buffer, width, height, tmpX, y, py, pu, pv);
      }
    };
  }
  /*
  cout << "Zauberstab: Slice found." << endl;
  cout << "            (left : " << slice->leftX << endl
       << "             right: " << slice->rightX << endl
       << "             at y = " << slice->y << ")" << endl;
  */
  return slice;
}


/** Add region.
 * @param region2 region to add
 */
void
Zauberstab::addRegion(ZRegion *region2)
{
  ZSlice* nSlice; //A slice to be added
  ZSlice* oSlice; //A slice currently in the region

  // add each slice from region 2 to region
  while (region2->slices->size())
  {
    /* check if current slice from region 2 is at
       at a height different from all slices at region */
    nSlice = region2->slices->back();
    region2->slices->pop_back();
    int heightOfSlice = nSlice->y;

		unsigned int i = 0;

		while(i < region->slices->size()) //for all existing slices
		{
			oSlice = region->slices->at(i);
			if (oSlice->y == heightOfSlice) //same height check for overlapping
			{
				if (((oSlice->leftX >= nSlice->leftX) 
				     && (oSlice->leftX <= nSlice->rightX))
				    || ((nSlice->leftX >= oSlice->leftX) 
				        && (nSlice->leftX <= oSlice->rightX)))
				{
					//They are overlapping so grow the new slice
					nSlice->leftX  = min(nSlice->leftX,  oSlice->leftX);
					nSlice->rightX = max(nSlice->rightX, oSlice->rightX);
					
					//and delete the old one
					region->slices->erase(region->slices->begin() + i);
					delete oSlice;
					
					//The iterator i now points to the next element in the region->slices vector
					continue;
				}
			}

			++i;
		}

		//By now all overlapping slices have been removed
		region->slices->push_back(nSlice);
  }
}


/** True if two V values are similar.
 * @param v1 V value 1
 * @param v2 V value 2
 * @return true if V values are similar (depends on threshold)
 */
bool 
Zauberstab::isSimilarV(unsigned int v1,
		       unsigned int v2) {
  return ( (unsigned int)abs((int)v1 - (int)v2) > this->threshold ) ? false : true;
}


/** True if two U values are similar.
 * @param u1 U value 1
 * @param u2 U value 2
 * @return true if U values are similar (depends on threshold)
 */
bool 
Zauberstab::isSimilarU(unsigned int u1,
		       unsigned int u2) {
  return ( (unsigned int)abs((int)u1 - (int)u2) > this->threshold ) ? false : true;
}


/** True if two u and V values are similar.
 * @param u1 U value 1
 * @param u2 U value 2
 * @param v1 V value 1
 * @param v2 V value 2
 * @return true if V values are similar (depends on threshold)
 */
bool 
Zauberstab::isSimilarUV(unsigned int u1, unsigned int u2,
                       unsigned int v1, unsigned int v2)
{
  return isSimilarU(u1, u2) && isSimilarV(v1, v2);
}


/** Get region.
 * @return region
 */
ZRegion *
Zauberstab::getRegion() const
{
  return region;
}


/** Get selection.
 * @return selection as a vector of rectangles.
 */
vector< rectangle_t >
Zauberstab::getSelection()
{

  vector< rectangle_t > rv;
  rv.clear();

  std::vector< ZSlice *>::iterator it;
  for (it = region->slices->begin(); it != region->slices->end(); it++) {
    rectangle_t rect;
    rect.start.x = (*it)->leftX;
    rect.start.y = (*it)->y;
    rect.extent.w = (*it)->rightX - (*it)->leftX;
    rect.extent.h = 1;
    rv.push_back( rect );
  }

  return rv;
}

} // end namespace firevision
