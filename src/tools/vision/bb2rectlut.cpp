
/***************************************************************************
 *  bb2rectlut.cpp - BB2 Rectification LUT utility
 *
 *  Created: Mon Oct 29 19:04:28 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
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

#ifdef HAVE_BUMBLEBEE2_CAM
#include <fvcams/bumblebee2.h>
#endif
#include <fvutils/system/camargp.h>
#include <utils/system/argparser.h>
#include <fvutils/rectification/rectfile.h>
#include <fvutils/rectification/rectinfo_block.h>
#include <fvutils/rectification/rectinfo_lut_block.h>

#ifdef HAVE_TRICLOPS_SDK
#include <fvstereo/triclops.h>
#include <cerrno>
#endif

#include <cstdlib>
#include <cstdio>
#include <unistd.h>

using namespace fawkes;
using namespace firevision;

void
print_usage(ArgumentParser *argp)
{
  printf("Usage: %s <-r|-v|-i> file.rectlut\n", argp->program_name());
  printf("You have to give at least one of -r/-v/-i and a file name\n"
	 "  -r   retrieve rectification lut from live camera,\n"
	 "       uses first found Bumblebee2 camera\n"
	 "  -v   verify rectification lut, compares the identification\n"
	 "       info stored in the file with the first currently\n"
	 "       attached camera\n"
	 "  -d   deep verifiction of rectification LUT, compares the identification\n"
	 "       info stored in the file with the first currently attached camera. It\n"
	 "       also verifies each single mapping on equality.\n"
	 "  -i   print info about rectification LUT file\n\n"
	 );
  exit(1);
}


int
retrieve(ArgumentParser *argp)
{
#ifdef HAVE_BUMBLEBEE2_CAM
#ifdef HAVE_TRICLOPS_SDK
  const char *lut_file = argp->items()[0];

  if ( access(lut_file, F_OK) == 0) {
    fprintf(stderr, "File with name %s exists, delete manually and retry. Aborting.\n", lut_file);
    return -1;
  }
  if ( access(lut_file, W_OK) != 0) {
    // ENOENT is ok, we would have access, but there is no file, yet
    if ( errno != ENOENT ) {
      fprintf(stderr, "Cannot write to file %s, permission problem?\n", lut_file);
      return -2;
    }
  }

  CameraArgumentParser *cap = new CameraArgumentParser("bumblebee2:Bumblebee2 BB2-03S2C");
  Bumblebee2Camera *bb2 = new Bumblebee2Camera(cap);
  bb2->open();

  TriclopsStereoProcessor *triclops = new TriclopsStereoProcessor(bb2);
  triclops->generate_rectification_lut(lut_file);
  delete triclops;

  bb2->close();

  delete bb2;
  delete cap;
#else
  printf("Retrieving the rectification LUT from a camera is not supported,\n"
	 "because the Triclops SDK was not available at compile time.\n");
#endif
#else
  printf("Retrieving the rectification LUT from a camera is not supported,\n"
	 "because the Bumblebee2 support was not available at compile time.\n");
#endif

  return 0;
}


int
verify(ArgumentParser *argp)
{
  int rv = 0;

#ifdef HAVE_BUMBLEBEE2_CAM
  CameraArgumentParser *cap = new CameraArgumentParser("bumblebee2:Bumblebee2 BB2-03S2C");
  Bumblebee2Camera *bb2 = new Bumblebee2Camera(cap);
  bb2->open();

  for (unsigned int i = 0; i < argp->num_items(); ++i) {

    const char *lut_file = argp->items()[i];

    if ( access(lut_file, F_OK) != 0) {
      fprintf(stderr, "File with name %s does not exist. Ignoring.\n", lut_file);
      continue;
    }
    if ( access(lut_file, R_OK) != 0) {
      fprintf(stderr, "Cannot read file %s, permission problem? Ingoring.\n", lut_file);
      continue;
    }

    RectificationInfoFile *rif = new RectificationInfoFile();
    try {
      rif->read(lut_file);

      if ( bb2->verify_guid( rif->guid() ) ) {
	printf("Success. The rectification info file has been created for the "
	       "connected camera\n");
      } else {
	printf("Failure. The rectification info file has *not* been created "
	       "for the connected camera\n");
	rv = 5;
      } 
    } catch (Exception &e) {
      fprintf(stderr, "Failed to read lut file %s\n", lut_file);
      e.print_trace();
    }

    delete rif;

  }

  bb2->close();
    
  delete bb2;
  delete cap;
    
#else
  printf("Verifying the rectification LUT from a camera is not supported,\n"
	 "because the Bumblebee2 support was not available at compile time.\n");
#endif

  return rv;
}


int
deep_verify(ArgumentParser *argp)
{
#ifdef HAVE_BUMBLEBEE2_CAM
#ifdef HAVE_TRICLOPS_SDK
  int rv = 0;

  CameraArgumentParser *cap = new CameraArgumentParser("bumblebee2:Bumblebee2 BB2-03S2C");
  Bumblebee2Camera *bb2 = new Bumblebee2Camera(cap);
  bb2->open();

  TriclopsStereoProcessor *triclops = new TriclopsStereoProcessor(bb2);

  for (unsigned int i = 0; i < argp->num_items(); ++i) {

    const char *lut_file = argp->items()[i];

    if ( access(lut_file, F_OK) != 0) {
      fprintf(stderr, "File with name %s does not exist. Ignoring.\n", lut_file);
      continue;
    }
    if ( access(lut_file, R_OK) != 0) {
      fprintf(stderr, "Cannot read file %s, permission problem? Ingoring.\n", lut_file);
      continue;
    }

    if ( triclops->verify_rectification_lut(lut_file) ) {
      printf("Success. LUT file %s contains matching configuration data.\n", lut_file);
    } else {
      printf("Failure. LUT file %s does not contain matching configuration data.\n", lut_file);
    }

  }

  delete triclops;
  bb2->close();
    
  delete bb2;
  delete cap;
    
  return rv;
#else
  printf("Deep verification of the rectification LUT from a camera is not supported,\n"
	 "because the Triclops SDK was not available at compile time.\n");
  return 0;
#endif
#else
  printf("Deep verification of the rectification LUT from a camera is not supported,\n"
	 "because the Bumblebee2 support was not available at compile time.\n");
  return 0;
#endif
}


void
print_info(ArgumentParser *argp)
{
  for (unsigned int i = 0; i < argp->num_items(); ++i) {

    const char *lut_file = argp->items()[i];

    if ( access(lut_file, F_OK) != 0) {
      fprintf(stderr, "File with name %s does not exist. Ignoring.\n", lut_file);
      continue;
    }
    if ( access(lut_file, R_OK) != 0) {
      fprintf(stderr, "Cannot read file %s, permission problem? Ingoring.\n", lut_file);
      continue;
    }

    RectificationInfoFile *rif = new RectificationInfoFile();
    try {
      rif->read(lut_file);
      RectificationInfoFile::RectInfoBlockVector *blocks = rif->rectinfo_blocks();

      printf("File:         %s\n"
	     "Version:      %u\n"
	     "Endianess:    %s\n"
	     "Num Blocks:   %zu/%zu (header/read)\n"
#if __WORDSIZE == 64
	     "GUID:         0x%016lX\n"
#else
	     "GUID:         0x%016llX\n"
#endif
	     "Camera Model: %s\n",
	     lut_file, rif->version(),
	     rif->is_little_endian() ? "little endian" : "big endian",
	     rif->num_blocks(), blocks->size(),
#if __WORDSIZE == 64
	     (long unsigned int)rif->guid(),
#else
	     (long long unsigned int)rif->guid(),
#endif
	     rif->model());

      unsigned int u = 1;
      RectificationInfoFile::RectInfoBlockVector::const_iterator b;
      for (b = blocks->begin(); b != blocks->end(); ++b) {
	RectificationInfoBlock *rib = *b;

	printf("\nRectInfo Block No. %u\n"
	       "Type:       %s\n"
	       "Camera:     %s\n"
	       "Size:       %zu\n",
	       u++,
	       rectinfo_type_strings[rib->type()],
	       rectinfo_camera_strings[rib->camera()],
	       rib->block_size());

	switch (rib->type()) {
	case FIREVISION_RECTINFO_TYPE_LUT_16x16:
	  {
	    RectificationLutInfoBlock *rlib = dynamic_cast<RectificationLutInfoBlock *>(rib);
	    if ( rlib == NULL ) {
	      printf("** Failure to access LUT_16x16\n");
	    } else {
	      printf("LUT width:  %hu\n"
		     "LUT height: %hu\n",
		     rlib->pixel_width(), rlib->pixel_height());
	    }
	  }
	  break;
	default:
	  printf("** No additional information available for this info type\n");
	  break;
	}
      }

      delete blocks;
    } catch (Exception &e) {
      fprintf(stderr, "Failed to read lut file %s\n", lut_file);
      e.print_trace();
    }

    delete rif;

  }
}


int
main(int argc, char **argv)
{

  ArgumentParser argp(argc, argv, "rvid");

  if (argp.num_items() == 0) {
    print_usage(&argp);
  }

  if ( argp.has_arg("r") ) {
    return retrieve(&argp);
  } else if ( argp.has_arg("v") ) {
    return verify(&argp);
  } else if ( argp.has_arg("d") ) {
    return deep_verify(&argp);
  } else if ( argp.has_arg("i") ) {
    print_info(&argp);
  } else {
    print_usage(&argp);
  }

  return 0;
}
