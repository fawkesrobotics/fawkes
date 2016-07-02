
/***************************************************************************
 *  filter.h - Laser data filter interface
 *
 *  Created: Fri Oct 10 17:11:04 2008
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_LASER_FILTER_FILTER_H_
#define __PLUGINS_LASER_FILTER_FILTER_H_

#include <vector>
#include <string>

namespace fawkes {
  class Time;
}

class LaserDataFilter
{
 public:
  class Buffer {
   public:
    Buffer(size_t num_values = 0);
    ~Buffer();
    std::string   name; ///< name of the input buffer
    std::string   frame;		///< reference coordinate frame ID
    float        *values;	///< values
    fawkes::Time *timestamp;	///< timestamp of data
  };

  LaserDataFilter(const std::string filter_name,
                  unsigned int in_data_size,
                  std::vector<Buffer *> &in, unsigned int out_size);
  virtual ~LaserDataFilter();

  virtual std::vector<Buffer *>  & get_out_vector();
  virtual void                     set_out_vector(std::vector<Buffer *> &out);
  virtual unsigned int             get_out_data_size();

  virtual void                     filter()   = 0;

  void  set_array_ownership(bool own_in, bool own_out);
  /** Check if input arrays are owned by filter.
   * @return true if arrays are owned by this filter, false otherwise. */
  bool  owns_in()  const { return __own_in;  };
  /** Check if output arrays are owned by filter.
   * @return true if arrays are owned by this filter, false otherwise. */
  bool  owns_out() const { return __own_out; };

 protected:
  virtual void set_out_data_size(unsigned int data_size);

  void reset_outbuf(Buffer *b);
  void copy_to_outbuf(Buffer *outbuf, const Buffer *inbuf);


 protected:
  const std::string    filter_name;
  unsigned int         out_data_size;
  unsigned int         in_data_size;
  std::vector<Buffer *>  in;
  std::vector<Buffer *>  out;

 private:
  bool __own_in;
  bool __own_out;
};


#endif
