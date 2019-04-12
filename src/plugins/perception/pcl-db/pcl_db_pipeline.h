
/***************************************************************************
 *  pcl_db_pipeline.h - PCL DB processing pipeline base class
 *
 *  Created: Wed Aug 21 17:24:18 2013
 *  Copyright  2012-2013  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_PERCEPTION_PCL_DB_PCL_DB_PIPELINE_H_
#define _PLUGINS_PERCEPTION_PCL_DB_PCL_DB_PIPELINE_H_

#include "mongodb_tf_transformer.h"

#include <config/config.h>
#include <logging/logger.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_utils/comparisons.h>
#include <pcl_utils/storage_adapter.h>
#include <pcl_utils/transforms.h>
#include <pcl_utils/utils.h>

#include <Eigen/Core>
#ifdef USE_TIMETRACKER
#	include <utils/time/tracker.h>
#endif
#include <utils/time/tracker_macros.h>

#define USE_ALIGNMENT
#define USE_ICP_ALIGNMENT
// define USE_NDT_ALIGNMENT

#define CFG_PREFIX "/perception/pcl-db/"

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <bsoncxx/builder/basic/document.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/exception/gridfs_exception.hpp>
#include <mongocxx/exception/operation_exception.hpp>
#include <mongocxx/gridfs/bucket.hpp>
#include <mongocxx/gridfs/downloader.hpp>

#ifdef HAVE_MONGODB_VERSION_H
// we are using mongo-cxx-driver which renamed QUERY to MONGO_QUERY
#	define QUERY MONGO_QUERY
#endif

static const uint8_t cluster_colors[12][3] = {{176, 0, 30},
                                              {0, 0, 255},
                                              {255, 90, 0},
                                              {137, 82, 39},
                                              {56, 23, 90},
                                              {99, 0, 30},
                                              {255, 0, 0},
                                              {0, 255, 0},
                                              {255, 255, 0},
                                              {255, 0, 255},
                                              {0, 255, 255},
                                              {27, 117, 196}};

typedef enum { APPLICABLE = 0, TYPE_MISMATCH, NO_POINTCLOUD, QUERY_FAILED } ApplicabilityStatus;

/** Database point cloud pipeline base class.
 * Common functionality for pcl-db-* plugins operating on
 * point clouds restored from MongoDB.
 * @author Tim Niemueller
 */
template <typename PointType>
class PointCloudDBPipeline
{
protected:
	/** Basic point cloud type. */
	typedef pcl::PointCloud<PointType> Cloud;

	/** Colored point type */
	typedef pcl::PointXYZRGB ColorPointType;
	/** Type for colored point clouds based on ColorPointType. */
	typedef pcl::PointCloud<ColorPointType> ColorCloud;
	/** Shared pointer to cloud. */
	typedef typename Cloud::Ptr CloudPtr;
	/** Shared pointer to constant cloud. */
	typedef typename Cloud::ConstPtr CloudConstPtr;

	/** Shared pointer to colored cloud. */
	typedef typename ColorCloud::Ptr ColorCloudPtr;
	/** Shared pointer to constant colored cloud. */
	typedef typename ColorCloud::ConstPtr ColorCloudConstPtr;

public:
	/** Constructor.
   * @param mongodb_client MongoDB client
   * @param config configuration
   * @param logger Logger
   * @param output output point cloud
   */
	PointCloudDBPipeline(mongocxx::client *     mongodb_client,
	                     fawkes::Configuration *config,
	                     fawkes::Logger *       logger,
	                     ColorCloudPtr          output)
	: mongodb_client_(mongodb_client), logger_(logger), output_(output)
	{
		name_ = "PCL_DB_Pipeline";

		cfg_pcl_age_tolerance_ = (long)round(config->get_float(CFG_PREFIX "pcl-age-tolerance") * 1000.);
		std::vector<float> transform_range = config->get_floats(CFG_PREFIX "transform-range");
		if (transform_range.size() != 2) {
			throw fawkes::Exception("Transform range must be a list with exactly two elements");
		}
		if (transform_range[1] < transform_range[0]) {
			throw fawkes::Exception("Transform range start cannot be smaller than end");
		}
		cfg_transform_range_[0] = (long)round(transform_range[0] * 1000.);
		cfg_transform_range_[1] = (long)round(transform_range[1] * 1000.);
	}

	/** Destructor. */
	virtual ~PointCloudDBPipeline()
	{
	}

	/** Check if this pipeline instance is suitable for the given times.
   * Retrieves information about the point clouds for the specified
   * \p times and checks if this pipeline (depending on the template
   * parameter) is suitable for the processing of these pipelines.
   * @param times times for which to check the point clouds
   * @param database ddatabase from which to retrieve the information
   * @param collection collection from which to retrieve the information
   * @return applicability status
   */
	ApplicabilityStatus
	applicable(std::vector<long long> &times, std::string &database, std::string &collection)
	{
		const unsigned int num_clouds = times.size();

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
		std::vector<pcl::PCLPointField> pfields;
#else
		std::vector<sensor_msgs::PointField> pfields;
#endif
		pcl::for_each_type<typename pcl::traits::fieldList<PointType>::type>(
		  pcl::detail::FieldAdder<PointType>(pfields));

		try {
			for (unsigned int i = 0; i < num_clouds; ++i) {
				using namespace bsoncxx::builder;
				auto result = mongodb_client_->database(database)[collection].find_one(
				  basic::make_document(
				    basic::kvp("timestamp",
				               [&](basic::sub_document subdoc) {
					               subdoc.append(basic::kvp("$lt", static_cast<int64_t>(times[i])));
					               subdoc.append(
					                 basic::kvp("$gt",
					                            static_cast<int64_t>(times[i] - cfg_pcl_age_tolerance_)));
				               })),
				  mongocxx::options::find().sort(basic::make_document(basic::kvp("timestamp", -1))));
				if (result) {
					bsoncxx::document::view pcldoc = result->view()["pointcloud"].get_document().view();
					bsoncxx::array::view    fields = pcldoc["field_info"].get_array();

					if (fields.length() == pfields.size()) {
						for (unsigned int i = 0; i < pfields.size(); ++i) {
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
							pcl::PCLPointField &pf = pfields[i];
#else
							sensor_msgs::PointField &pf = pfields[i];
#endif

							bool found = false;
							for (unsigned int j = 0; j < fields.length(); ++j) {
								if ((fields[j]["name"].get_utf8().value.to_string() == pf.name)
								    && (fields[j]["offset"].get_int64() == (int64_t)pf.offset)
								    && (fields[j]["datatype"].get_int64() == pf.datatype)
								    && (fields[j]["count"].get_int64() == (int64_t)pf.count)) {
									found = true;
									break;
								}
							}
							if (!found) {
								logger_->log_warn(name_,
								                  "Type mismatch (fields) for pointcloud "
								                  "at timestamp %lli",
								                  times[i]);
								return TYPE_MISMATCH;
							}
						}
					} else {
						logger_->log_warn(name_,
						                  "Type mismatch (num fields) for pointcloud "
						                  "at timestamp %lli",
						                  times[i]);
						return TYPE_MISMATCH;
					}
				} else {
					logger_->log_warn(name_,
					                  "No pointclouds for timestamp %lli in %s.%s",
					                  times[i],
					                  database.c_str(),
					                  collection.c_str());
					return NO_POINTCLOUD;
				}
			}
		} catch (mongocxx::operation_exception &e) {
			logger_->log_warn(name_, "MongoDB query failed: %s", e.what());
			return QUERY_FAILED;
		}

		return APPLICABLE;
	}

protected: // methods
	/** Read a file from MongoDB GridFS.
   * @param dataptr Pointer to buffer to read data to. Make sure it is of
   * sufficient size.
   * @param database database from which to read the file
   * @param file_id The bucket ID of the file to read
   */
	void
	read_gridfs_file(void *dataptr, std::string &database, bsoncxx::types::value file_id)
	{
		auto gridfs = mongodb_client_->database(database).gridfs_bucket();
		try {
			auto downloader    = gridfs.open_download_stream(file_id);
			auto file_length   = downloader.file_length();
			auto buffer_size   = std::min(file_length, static_cast<int64_t>(downloader.chunk_size()));
			unsigned char *tmp = (unsigned char *)dataptr;
			while (auto length_read = downloader.read(tmp, static_cast<std::size_t>(buffer_size))) {
				tmp += length_read;
			}
		} catch (mongocxx::gridfs_exception &e) {
			logger_->log_warn(name_, "Grid file does not exist");
			return;
		}
	}

	/** Retrieve point clouds from database.
   * @param times timestamps for when to read the point clouds. The method
   * will retrieve the point clouds with the minimum difference between the
   * desired and actual times.
   * @param actual_times upon return contains the actual times of the point
   * clouds retrieved based on the desired @p times.
   * @param database name of the database to retrieve data from
   * @param collection_name name of the collection to retrieve data from.
   * @return vector of shared pointers to retrieved point clouds
   */
	std::vector<CloudPtr>
	retrieve_clouds(std::vector<long> &times,
	                std::vector<long> &actual_times,
	                std::string &      database,
	                std::string &      collection_name)
	{
		using namespace bsoncxx::builder;
		auto collection = mongodb_client_->database(database)[collection_name];
		collection.create_index(basic::make_document(basic::kvp("timestamp", 1)));

		const unsigned int    num_clouds = times.size();
		std::vector<CloudPtr> pcls(num_clouds);

		// retrieve point clouds
		for (unsigned int i = 0; i < num_clouds; ++i) {
			auto result = collection.find_one(
			  basic::make_document(basic::kvp("timestamp",
			                                  [&](basic::sub_document subdoc) {
				                                  subdoc.append(basic::kvp("$lt", times[i]));
				                                  subdoc.append(
				                                    basic::kvp("$gt", times[i] - cfg_pcl_age_tolerance_));
			                                  })),
			  mongocxx::options::find().sort(basic::make_document(basic::kvp("timestamp", -1))));
			if (result) {
				bsoncxx::document::view pcldoc = result->view()["pointcloud"].get_document().view();
				//bsoncxx::array::view    fields    = pcldoc["field_info"].get_array();
				int64_t timestamp = result->view()["timestamp"].get_int64();
				double  age       = (double)(times[i] - timestamp) / 1000.;

				logger_->log_info(name_, "Restoring point cloud at %li with age %f sec", timestamp, age);

				// reconstruct point cloud
				CloudPtr lpcl(new Cloud());
				pcls[i] = lpcl;

				actual_times[i] = timestamp;
				fawkes::Time actual_time((long)actual_times[i]);

				lpcl->header.frame_id = pcldoc["frame_id"].get_utf8().value.to_string();
				lpcl->is_dense        = pcldoc["is_dense"].get_bool();
				lpcl->width           = pcldoc["width"].get_int64();
				lpcl->height          = pcldoc["height"].get_int64();
				fawkes::pcl_utils::set_time(lpcl, actual_time);
				lpcl->points.resize(pcldoc["num_points"].get_int64());

				read_gridfs_file(&lpcl->points[0], database, pcldoc["data"]["id"].get_value());
			} else {
				logger_->log_warn(name_, "Cannot retrieve document for time %li", times[i]);
				return std::vector<CloudPtr>();
			}
		}

		return pcls;
	}

protected:           // members
	const char *name_; /**< Name of the pipeline. */

	long cfg_pcl_age_tolerance_;  /**< Age tolerance for retrieved point clouds. */
	long cfg_transform_range_[2]; /**< Transform range start and end times. */

	mongocxx::client *mongodb_client_; /**< MongoDB client to retrieve data. */

	fawkes::Logger *logger_; /**< Logger for informative messages. */

	ColorCloudPtr output_; /**< The final (colored) output of the pipeline. */
};

/** Convert applicability status to readable string.
 * @param status the status to convert
 * @return readable string for status
 */
inline const char *
to_string(ApplicabilityStatus status)
{
	switch (status) {
	case APPLICABLE: return "Applicable";
	case TYPE_MISMATCH: return "PointCloud in database does not match type";
	case NO_POINTCLOUD: return "For at least one time no pointcloud found";
	case QUERY_FAILED: return "MongoDB query failed";
	default: return "Unknown error";
	}
}

#endif
