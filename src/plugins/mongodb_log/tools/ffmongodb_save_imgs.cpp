
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
#include <utils/system/argparser.h>
#include <utils/misc/string_conversions.h>

#include <mongo/client/dbclient.h>
#include <mongo/client/gridfs.h>

using namespace firevision;
using namespace mongo;
using namespace fawkes;

#ifdef HAVE_MONGODB_VERSION_H
// we are using mongo-cxx-driver which renamed QUERY to MONGO_QUERY
#  define QUERY MONGO_QUERY
#endif


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
	 "\n", progname, progname);
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
  std::string database = "fflog";
  std::string collection;
  std::string query_coll;
  bool filename_indexed = ! argp.has_arg("f");

  std::vector<std::pair<long long, long long> > times;

  if (argp.has_arg("o")) {
    output_dir = argp.arg("o");
    if (output_dir[output_dir.length() - 1] != '/') {
      output_dir += "/";
    }
  }
  if (argp.has_arg("d")) {
    database = argp.arg("d");
  }
  if (argp.has_arg("c")) {
    collection = argp.arg("c");
  } else {
    print_usage(argv[0]);
    printf("No collection given\n");
    exit(-1);
  }

  query_coll = database + "." + collection;

  if (items.empty()) {
    times.push_back(std::make_pair(0L, std::numeric_limits<long long>::max()));
  } else {
    for (unsigned int i = 0; i < items.size(); ++i) {
      std::string item = items[i];
      std::string::size_type dotpos = item.find("..");
      if (dotpos == std::string::npos) {
	// singular timestamp
	long int ts = argp.parse_item_int(i);
	times.push_back(std::make_pair(ts, ts));	
      } else {
	// range
	std::string first_ts, second_ts;
	first_ts = item.substr(0, dotpos);
	second_ts = item.substr(dotpos + 2);
	times.push_back(std::make_pair(StringConversions::to_long(first_ts),
				       StringConversions::to_long(second_ts)));
      }
    }
  }

  unsigned int image_n = 0;

  DBClientConnection *mongodb_client =
    new DBClientConnection(/* auto reconnect */ true);
  std::string errmsg;
  mongodb_client->connect("localhost", errmsg);

  GridFS *gridfs = new GridFS(*mongodb_client, "fflog");


  for (unsigned int i = 0; i < times.size(); ++i) {
    Query q;

    if (times[i].first == times[i].second) {
      printf("Querying for timestamp %lli\n", times[i].first);
      q = QUERY("timestamp" << times[i].first).sort("timestamp", 1);
    } else {
      printf("Querying for range %lli..%lli\n", times[i].first, times[i].second);
      q = QUERY("timestamp"
		<< mongo::GTE << times[i].first
		<< mongo::LTE << times[i].second)
	.sort("timestamp", 1);
    }

#if __cplusplus >= 201103L
    std::unique_ptr<mongo::DBClientCursor> cursor =
      mongodb_client->query(query_coll, q);
#else
    std::auto_ptr<mongo::DBClientCursor> cursor =
      mongodb_client->query(query_coll, q);
#endif

    while (cursor->more()) {
      BSONObj doc = cursor->next();
      
      BSONObj imgdoc = doc.getObjectField("image");
      if (imgdoc["colorspace"].String() == "RGB") {
	std::string filename = imgdoc.getFieldDotted("data.filename").String();
	long filesize = imgdoc.getFieldDotted("data.length").numberLong();
	std::string image_id = imgdoc["image_id"].String();

	std::string out_filename;
	char *fntmp;
	if (filename_indexed) {
	  if (asprintf(&fntmp, "%s%s-%08d.png", output_dir.c_str(),
		       image_id.c_str(), image_n++) != -1)
	  {
	    out_filename = fntmp;
	    free(fntmp);
	  }
	} else {
	  if (asprintf(&fntmp, "%s%s.png", output_dir.c_str(),
		       filename.c_str()) != -1)
	  {
	    out_filename = fntmp;
	    free(fntmp);
	  }
	  ++image_n;
	}

	printf("Restoring RGB image %s (%s)\n", filename.c_str(), out_filename.c_str());

	GridFile file = gridfs->findFile(filename);
	if (! file.exists()) {
	  printf("File %s does not exist\n", filename.c_str());
	  continue;
	}

	unsigned int width  = imgdoc["width"].Int();
	unsigned int height = imgdoc["height"].Int();

	if (colorspace_buffer_size(RGB, width, height) != (size_t)filesize) {
	  printf("Buffer size mismatch (DB %li vs. exp. %zu)\n",
		 filesize, colorspace_buffer_size(RGB, width, height));
	  continue;
	}

	unsigned char *buffer = malloc_buffer(RGB, width, height);

	unsigned char *tmp = buffer;
	for (int c = 0; c < file.getNumChunks(); ++c) {
	  mongo::GridFSChunk chunk = file.getChunk(c);
	  int len = 0;
	  const char *chunk_data = chunk.data(len);
	  memcpy(tmp, chunk_data, len);
	  tmp += len;
	}

	PNGWriter writer(out_filename.c_str(), width, height);
	writer.set_buffer(RGB, buffer);
	writer.write();
	
	free(buffer);
      }
    }
  }

  delete gridfs;
  delete mongodb_client;

}

