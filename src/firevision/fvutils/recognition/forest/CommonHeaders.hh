#ifndef __commonheaders__hh__
#define __commonheaders__hh__

/***************************************************************************
 *  CommonHeaders.hh - Header file for object recognition with random forests: Headers needed 
 *
 *  Created: April 18/2008
 *  Copyright  2008  Vaishak Belle
 *
 * 
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



// Standard header files (including opencv)
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
// #include <cv.h>
 #include <opencv/highgui.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <dirent.h>
#include <string>
#include <fstream>
#include <map>
#include <iterator>
#include <ctime>

//Those need by the boost library (bare essentials?)
#include <boost/nondet_random.hpp>
#include <boost/random.hpp>
// #include <iostream>
// #include <cstdlib>
// #include <string>
#include <boost/config.hpp>
#include <boost/random.hpp>
#include <boost/progress.hpp>
#include <boost/shared_ptr.hpp>



//Haar Filter Set used
enum {BLOCK , RECT2_HORIZONTAL , RECT2_VERTICAL, RECT3_VERTICAL};

#endif
