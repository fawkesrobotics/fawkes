
/***************************************************************************
 *  zauberstab.cpp - Implementation of class "Zauberstab"
 *                   which offers methods for finding 
 *                   maximal, color-contiguous region
 *                   around a seed pixel
 *
 *  Generated: Mon Jul 02 2005
 *  Copyright  2005       Martin Heracles  <Martin.Heracles@rwth-aachen.de>
 *             2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/color/zauberstab.h>
#include <fvutils/color/yuv.h>
#include <core/macros.h>

using namespace std;

/** @class Zauberstab <fvutils/color/zauberstab.h>
 * Zaubertab selection utility.
 */

/** Constructor. */
Zauberstab::Zauberstab() {
  // create empty region
  region = new ZRegion;
  region->topSliceY = 0;
  region->slices = new vector<ZSlice*>;
  region->slices->clear();

  buffer = NULL;
  width = 0;
  height = 0;

  /* by default, "Zauberstab" does not allow
     any deviation from seed color */
  this->threshold = 0 ;
}


/** Destructor. */
Zauberstab::~Zauberstab() {
  // delete all slices
  for (unsigned int i = 0; i < region->slices->size(); i++) {
    delete region->slices->at(i);
  }
  region->slices->clear();
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


/** Delete region. */
void
Zauberstab::deleteRegion() {
  // delete all slices
  for (unsigned int i = 0; i < region->slices->size(); i++) {
    delete region->slices->at(i);
  }
  region->slices->clear();
}


/** Find region.
 * @param seedX seed x
 * @param seedY seed y
 */
void
Zauberstab::findRegion(int seedX,
		       int seedY) {

  if (buffer == NULL) return;
  if (width == 0) return;
  if (height == 0) return;

  unsigned char py __unused;
  unsigned char pu __unused;
  unsigned char pv;

  // delete all slices
  for (unsigned int i = 0; i < region->slices->size(); i++) {
    delete region->slices->at(i);
  }
  region->slices->clear();

  /* find seed pixel´s v-value
     (consider seed pixel´s neighborhood
      and take average v-value) */
  unsigned int vSeed = 0;
  for (int x = seedX - 2; x <= seedX + 2; x++) {
    for (int y = seedY - 2; y <= seedY + 2; y++) {
      YUV422_PLANAR_YUV(buffer, width, height, x, y, py, pu, pv);
      vSeed += pv;
    }
  }
  vSeed = int(float(vSeed) / 25.f);

  /* initial slice 
     thru seed pixel (seedX, seedY) */
  ZSlice *tmp = NULL;
  tmp = findSlice(seedX, seedY, vSeed);
  region->slices->push_back(tmp);

  /* NOTE: The following search works fine for
     objects that are convex (such as ball, goal, ...).
     For non-convex objects it may miss parts
     (e.g. for a U-shaped object it can only find right or left half). */

  // search downwards for more slices
  tmp = region->slices->front();
  int tmpY = seedY + 1;

  // new "seed" pixel has x-coordinate in the middle of initial slice
  int tmpX = int(float(tmp->leftX + tmp->rightX) / 2.0);
  YUV422_PLANAR_YUV(buffer, width, height, tmpX, tmpY, py, pu, pv);

  while (isSimilarV(pv, vSeed)) {
    tmp = findSlice(tmpX, tmpY, vSeed);
    region->slices->push_back(tmp);
    // new "seed" pixel has x-coordinate in the middle of previous slice
    tmpX = int(float(tmp->leftX + tmp->rightX) / 2.0);
    tmpY++;
    if (tmpY > (int)this->height) {
      break;
    } else {
      YUV422_PLANAR_YUV(buffer, width, height, tmpX, tmpY, py, pu, pv);
    }
  }

  /* search upwards for more slices
     (start search from initial slice again) */
  tmp = region->slices->front();
  tmpY = seedY - 1;
  // new "seed" pixel has x-coordinate in the middle of initial slice
  tmpX = int(float(tmp->leftX + tmp->rightX) / 2.0);
  YUV422_PLANAR_YUV(buffer, width, height, tmpX, tmpY, py, pu, pv);

  while (isSimilarV(pv, vSeed)) {
    tmp = findSlice(tmpX, tmpY, vSeed);
    region->slices->push_back(tmp);
    // new "seed" pixel has x-coordinate in the middle of previous slice
    tmpX = int(float(tmp->leftX + tmp->rightX) / 2.0);
    tmpY--;
    if (tmpY < 0) {
      break;
    } else {
      YUV422_PLANAR_YUV(buffer, width, height, tmpX, tmpY, py, pu, pv);
    }
  }
  
  region->topSliceY = tmpY + 1;
}


/** Add region.
 * @param seedX seed x
 * @param seedY seed y
 */
void
Zauberstab::addRegion(int seedX,
		      int seedY) {

  unsigned char py __unused;
  unsigned char pu __unused;
  unsigned char pv=0;

  // STEP 1:
  // first of all find the region around (seedX, seedY)
  // (this is analogously to method "findRegion")
  // (could be done more elegantly without the following redundant code)

  // create empty region
  ZRegion *region2 = new ZRegion;
  region2->topSliceY = 0;
  region2->slices = new vector<ZSlice*>;
  region2->slices->clear();

  /* find seed pixel's v-value
     (consider seed pixel's neighborhood
      and take average v-value) */
  unsigned int vSeed = 0;
  for (int x = seedX - 2; x <= seedX + 2; x++) {
    if (x < 0) continue;
    if ((unsigned int )x >= width) continue;
    for (int y = seedY - 2; y <= seedY + 2; y++) {
      if (y < 0) continue;
      if ((unsigned int)y >= height) continue;
      YUV422_PLANAR_YUV(buffer, width, height, x, y, py, pu, pv);
      vSeed += pv;
    }
  }
  vSeed = int(float(vSeed) / 25.f);

  /* initial slice 
     thru seed pixel (seedX, seedY) */
  ZSlice *tmp = NULL;
  tmp = findSlice(seedX, seedY, vSeed);
  region2->slices->push_back(tmp);

  /* NOTE: The following search works fine for
     objects that are convex (such as ball, goal, ...).
     For non-convex objects it may miss parts
     (e.g. for a U-shaped object it can only find right or left half). */

  // search downwards for more slices
  tmp = region2->slices->front();
  int tmpY = (seedY == (int)(height - 1)) ? height : seedY + 1;
  // new "seed" pixel has x-coordinate in the middle of initial slice
  int tmpX = int(float(tmp->leftX + tmp->rightX) / 2.0);

  YUV422_PLANAR_YUV(buffer, width, height, tmpX, tmpY, py, pu, pv);
  while (isSimilarV(pv, vSeed)) {
    tmp = findSlice(tmpX, tmpY, vSeed);
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
  while (isSimilarV(pv, vSeed)) {
    tmp = findSlice(tmpX, tmpY, vSeed);
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

  // STEP 2:
  // now merge the newly found region2 with the original region
  addRegion(region2);
}


/** Find specific slice.
 * @param x x
 * @param y y
 * @param vSeed v seed
 * @return slice
 */
ZSlice*
Zauberstab::findSlice(int x, 
		      int y,
		      unsigned int vSeed) {  

  // slice with single pixel (x, y)
  ZSlice *slice = new ZSlice;
  slice->y = y;
  slice->leftX = x;
  slice->rightX = x;

  unsigned char py __unused;
  unsigned char pu __unused;
  unsigned char pv=0;
  int tmpX = x + 1;

  YUV422_PLANAR_YUV(buffer, width, height, tmpX, y, py, pu, pv);

  // search to the right
  while (isSimilarV(pv, vSeed)) {
    (slice->rightX)++;
    tmpX++;
    if (tmpX > (int)this->width) {
      break;
    } else {
      YUV422_PLANAR_YUV(buffer, width, height, tmpX, y, py, pu, pv);
    }
  };

  // search to the left
  tmpX = x - 1;
  YUV422_PLANAR_YUV(buffer, width, height, tmpX, y, py, pu, pv);
  while (isSimilarV(pv, vSeed)) {
    (slice->leftX)--;
    tmpX--;
    if (tmpX < 0) {
      break;
    } else {
      YUV422_PLANAR_YUV(buffer, width, height, tmpX, y, py, pu, pv);
    }
  };
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
Zauberstab::addRegion(ZRegion *region2) {
  // add each slice from region 2 to region
  for (unsigned int s2 = 0; s2 < region2->slices->size(); s2++) {
    /* check if current slice from region 2 is at
       at a height different from all slices at region */
    int heightOfSlice = region2->slices->at(s2)->y;
    bool differentY = true;
    unsigned int correspondingSlice = 0;
    for (unsigned int s = 0; s < region->slices->size(); s++) {
      if (region->slices->at(s)->y == heightOfSlice) {
	differentY = false;
	correspondingSlice = s;
	//break;
      }
    }
    if (differentY) {
      // slice from region 2 can be added, no overlap
      region->slices->push_back(region2->slices->at(s2));
    }
    else {
      // check if slices are overlapping
      if (region2->slices->at(s2)->leftX >= region->slices->at(correspondingSlice)->leftX &&
	  region2->slices->at(s2)->rightX <= region->slices->at(correspondingSlice)->rightX) {
	// slice from region 2 is contained by slice from region 
	// do nothing
      }
      else if (region2->slices->at(s2)->leftX < region->slices->at(correspondingSlice)->leftX &&
	       region2->slices->at(s2)->rightX > region->slices->at(correspondingSlice)->rightX) {
	// slice from region2 contains slice from region 
	region->slices->at(correspondingSlice)->leftX = region2->slices->at(s2)->leftX;
	region->slices->at(correspondingSlice)->rightX = region2->slices->at(s2)->rightX;
      }
      else if (region2->slices->at(s2)->leftX < region->slices->at(correspondingSlice)->leftX &&
	       region2->slices->at(s2)->rightX > region->slices->at(correspondingSlice)->leftX) {
	// slice from region2 overlaps left part of slice from region 
	region->slices->at(correspondingSlice)->leftX = region2->slices->at(s2)->leftX;
      }
      else if (region2->slices->at(s2)->rightX > region->slices->at(correspondingSlice)->rightX &&
	       region2->slices->at(s2)->leftX < region->slices->at(correspondingSlice)->rightX) {
	// slice from region2 overlaps right part of slice from region
	region->slices->at(correspondingSlice)->rightX = region2->slices->at(s2)->rightX;
      }
      else {
	// slices are at same height y, but not overlapping
	region->slices->push_back(region2->slices->at(s2));
      }
    }
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
