
/***************************************************************************
 *  pcl_adapter.h - Thread to exchange point clouds
 *
 *  Created: Tue Nov 08 00:36:10 2011
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
 *             2012       Bastian Klingen
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

#ifndef __LIBS_PCL_UTILS_PCL_ADAPTER_H_
#define __LIBS_PCL_UTILS_PCL_ADAPTER_H_

#include <utils/time/time.h>

#include <map>
#include <vector>
#include <string>
#include <stdint.h>

namespace fawkes {
  class PointCloudManager;
  class Logger;
}

class PointCloudAdapter
{
 public:
  /** Information about the data fields. */
  class PointFieldInfo {
   public:
    std::string name;   ///< Name of field
    uint32_t offset;    ///< Offset from start of point struct
    uint8_t  datatype;  ///< Datatype enumeration see above
    uint32_t count;     ///< How many elements in field

    /** Constructor for pre-allocation. */
    PointFieldInfo() {}
    /** Constructor.
     * @param name field name
     * @param offset data offset
     * @param datatype data type ID, see sensor_msgs::PointField
     * @param count number of data entries
     */
    PointFieldInfo(std::string name, uint32_t offset,
                   uint8_t datatype, uint32_t count)
    : name(name), offset(offset), datatype(datatype), count(count) {}
  };
  /** Vector of PointFieldInfo. */
  typedef std::vector<PointFieldInfo> V_PointFieldInfo;

  PointCloudAdapter(fawkes::PointCloudManager *pcl_manager,
		    fawkes::Logger *logger);
  ~PointCloudAdapter();

  void get_info(const std::string &id,
                unsigned int &width, unsigned int &height,
                std::string &frame_id, bool &is_dense,
                V_PointFieldInfo &pfi);

  void get_data(const std::string &id, std::string &frame_id,
                unsigned int &width, unsigned int &height, fawkes::Time &time,
                void **data_ptr, size_t &point_size, size_t &num_points);

  void get_data_and_info(const std::string &id, std::string &frame_id, bool &is_dense,
			 unsigned int &width, unsigned int &height, fawkes::Time &time,
			 V_PointFieldInfo &pfi, void **data_ptr, size_t &point_size, size_t &num_points);

  void close(const std::string &id);

 private:
  fawkes::PointCloudManager *__pcl_manager;

  class StorageAdapter;
  std::map<std::string, StorageAdapter *> __sas;
};


#endif
