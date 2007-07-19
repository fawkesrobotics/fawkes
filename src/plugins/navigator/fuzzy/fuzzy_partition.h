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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */
 
#ifndef _FUZZY_PARTITION_H_
#define _FUZZY_PARTITION_H_

#include <vector>


class FuzzySet;
 
class FuzzyPartition
{
 public:
  
  FuzzyPartition();
        
  FuzzyPartition(std::vector<FuzzySet *> *sets);
        
  virtual ~FuzzyPartition();
        
  double getSupportRange();
    
  double getMaxR();
    
  double getMinL();
    
  std::vector<FuzzySet *> *getFuzzySets();
    
  //defuzzification
  double getfirstMax();
    
  double getMiddleMax();
    
  double getWeightedAverageMax();
    
    
 protected:
  
  std::vector<FuzzySet *> *sets;
    
  double minL;
    
  double maxR;    
};

#endif //_FUZZY_PARTITION_H_
