
/***************************************************************************
 *  ffmongodb_save_imgs.cpp - Save images from database to file
 *
 *  Created: Fri Dec 14 00:46:58 2012
 *  Copyright  2010-2012  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/writers/png.h>
#include <utils/misc/string_conversions.h>
#include <utils/system/argparser.h>

#include <mongocxx/client.hpp>
#include <mongocxx/gridfs/bucket.hpp>
#include <mongocxx/gridfs/downloader.hpp>

using namespace firevision;
using namespace mongocxx;
using namespace fawkes;
using namespace bsoncxx;
using namespace bsoncxx::builder;

void
print_usage(const char *progname)
{
	printf("Usage: %s [-h] [-o dir] [-f] [-d database] -c collection items...\n"
	       "  -h             Show this help message\n"
	       "  -o dir         Output directory where to create PNG files\n"
	       "  -f             Use original filenames form database\n"
	       "  -d database    Database to query for images\n"
	       "  -c collection  Collection to query for images\n"
	       "\n"
	       "Items are either timestamps (ms precision) or timestamp ranges in\n"
	       "the form ts1..ts2\n"
	       "\n"
	       "Example: %s -d fflog -c openni_image_rgb 0..1355421345807\n"
	       "\n",
	       progname,
	       progname);
}

int
main(int argc, char **argv)
{
	ArgumentParser argp(argc, argv, "ho:fd:c:");
	if (argp.has_arg("h")) {
		print_usage(argv[0]);
		exit(0);
	}

	const std::vector<const char *> &items = argp.items();

	std::string output_dir = "tmp/";
	std::string db_name    = "fflog";
	std::string collection_name;
	std::string query_coll;
	bool        filename_indexed = !argp.has_arg("f");

	std::vector<std::pair<long long, long long>> times;

	if (argp.has_arg("o")) {
		output_dir = argp.arg("o");
		if (output_dir[output_dir.length() - 1] != '/') {
			output_dir += "/";
		}
	}
	if (argp.has_arg("d")) {
		db_name = argp.arg("d");
	}
	if (argp.has_arg("c")) {
		collection_name = argp.arg("c");
	} else {
		print_usage(argv[0]);
		printf("No collection given\n");
		exit(-1);
	}

	if (items.empty()) {
		times.push_back(std::make_pair(0L, std::numeric_limits<long long>::max()));
	} else {
		for (unsigned int i = 0; i < items.size(); ++i) {
			std::string            item   = items[i];
			std::string::size_type dotpos = item.find("..");
			if (dotpos == std::string::npos) {
				// singular timestamp
				long int ts = argp.parse_item_int(i);
				times.push_back(std::make_pair(ts, ts));
			} else {
				// range
				std::string first_ts, second_ts;
				first_ts  = item.substr(0, dotpos);
				second_ts = item.substr(dotpos + 2);
				times.push_back(std::make_pair(StringConversions::to_long(first_ts),
				                               StringConversions::to_long(second_ts)));
			}
		}
	}

	unsigned int image_n = 0;

	client mongodb_client{uri{"localhost"}};
	auto   collection = mongodb_client[db_name][collection_name];

	gridfs::bucket gridfs = mongodb_client["fflog"].gridfs_bucket();

	for (unsigned int i = 0; i < times.size(); ++i) {
		// Initialize the cursor with a lambda because there is no default
		// constructor and we cannot define the cursor inside the conditional
		// branches.
		cursor cursor = [&]() {
			if (times[i].first == times[i].second) {
				printf("Querying for timestamp %lli\n", times[i].first);
				return collection.find(
				  basic::make_document(basic::kvp("timestamp", static_cast<int64_t>(times[i].first))),
				  options::find().sort(basic::make_document(basic::kvp("timestamp", 1))));
			} else {
				printf("Querying for range %lli..%lli\n", times[i].first, times[i].second);
				return collection.find(
				  basic::make_document(
				    basic::kvp("timestamp",
				               [times, i](basic::sub_document subdoc) {
					               subdoc.append(basic::kvp("$gt", static_cast<int64_t>(times[i].first)));
					               subdoc.append(basic::kvp("$lt", static_cast<int64_t>(times[i].second)));
				               })),
				  options::find().sort(basic::make_document(basic::kvp("timestamp", 1))));
			}
		}();

		//auto it = cursor.begin();
		for (auto doc : cursor) {
			auto imgdoc = doc["image"];
			if (imgdoc["colorspace"].get_utf8().value.to_string() == "RGB") {
				types::value file_id  = imgdoc["data"]["id"].get_value();
				std::string  filename = imgdoc["data"]["filename"].get_utf8().value.to_string();
				std::string  image_id = imgdoc["image_id"].get_utf8().value.to_string();

				std::string out_filename;
				char *      fntmp;
				if (filename_indexed) {
					if (asprintf(&fntmp, "%s%s-%08d.png", output_dir.c_str(), image_id.c_str(), image_n++)
					    != -1) {
						out_filename = fntmp;
						free(fntmp);
					}
				} else {
					if (asprintf(&fntmp, "%s%s.png", output_dir.c_str(), filename.c_str()) != -1) {
						out_filename = fntmp;
						free(fntmp);
					}
					++image_n;
				}

				printf("Restoring RGB image %s (%s)\n", filename.c_str(), out_filename.c_str());

				auto    downloader = gridfs.open_download_stream(file_id);
				int64_t filesize   = downloader.file_length();
				int     width      = imgdoc["width"].get_int32();
				int     height     = imgdoc["height"].get_int32();

				if (colorspace_buffer_size(RGB, width, height) != (size_t)filesize) {
					printf("Buffer size mismatch (DB %li vs. exp. %zu)\n",
					       filesize,
					       colorspace_buffer_size(RGB, width, height));
					continue;
				}

				auto buffer_size      = std::min(filesize, static_cast<int64_t>(downloader.chunk_size()));
				unsigned char *buffer = malloc_buffer(RGB, width, height);

				unsigned char *tmp = buffer;
				while (auto length_read = downloader.read(tmp, buffer_size)) {
					tmp += length_read;
				}

				PNGWriter writer(out_filename.c_str(), width, height);
				writer.set_buffer(RGB, buffer);
				writer.write();

				free(buffer);
			}
			//std::advance(it, 1);
		}
	}
}
