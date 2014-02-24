
/***************************************************************************
 *  batch_render.cpp - Render a directory of dot graphs
 *
 *  Created: Sat Mar 21 17:16:01 2009
 *  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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

#include "gvplugin_skillgui_cairo.h"

#include <utils/system/argparser.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <fnmatch.h>
#include <libgen.h>

using namespace fawkes;


/** DOT graph batch renderer. */
class SkillGuiBatchRenderer
  : public SkillGuiCairoRenderInstructor
{
 public:
  /** Constructor.
   * @param argc number of arguments
   * @param argv arguments
   */
  SkillGuiBatchRenderer(int argc, char **argv)
    : argp(argc, argv, "hi:o:f:wps:")
  {
    if (! (argp.has_arg("i") && argp.has_arg("o") && argp.has_arg("f"))
	|| argp.has_arg("h")) {
      usage();
      exit(-1);
    }

    format = argp.arg("f");
    write_to_png = false;
    bbw = bbh = 0;
    white_bg = argp.has_arg("w");
    postproc_required = false;
    do_postproc = argp.has_arg("p");
    maxwidth = maxheight = 0;
    scale = 1.0;

    if ( (format != "pdf") && (format != "svg") && (format != "png") ) {
      printf("Unknown format '%s'\n\n", format.c_str());
      usage();
      exit(-2);
    }

    if ( do_postproc && (format != "png") ) {
      printf("Post-processing only available for PNG output format.\n");
      exit(-7);
    }

    if (argp.has_arg("s")) {
      char *endptr;
      scale = strtod(argp.arg("s"), &endptr);
      if ( *endptr != 0 ) {
	printf("Invalid scale value '%s', could not convert to number (failed at '%s').\n",
	       argp.arg("s"), endptr);
	exit(-8);
      }
    }

    indir  = argp.arg("i");
    outdir = argp.arg("o");

    struct stat statbuf_in, statbuf_out;
    if (stat(indir.c_str(), &statbuf_in) != 0) {
      perror("Unable to stat input directory");
      exit(-3);
    }
    if (stat(outdir.c_str(), &statbuf_out) != 0) {
      perror("Unable to stat output directory");
      exit(-4);
    }
    if (! S_ISDIR(statbuf_in.st_mode) || ! S_ISDIR(statbuf_out.st_mode)) {
      printf("Input or output directory is not a directory.\n\n");
      exit(-5);
    }

    char outdir_real[PATH_MAX];
    if (realpath(outdir.c_str(), outdir_real)) {
      outdir = outdir_real;
    }

    directory = opendir(indir.c_str());
    if (! directory) {
      printf("Could not open input directory\n");
      exit(-6);
    }

    gvc = gvContext();
    gvplugin_skillgui_cairo_setup(gvc, this);
  }

  /** Destructor. */
  ~SkillGuiBatchRenderer()
  {
    gvFreeContext(gvc);
    closedir(directory);
  }

  /** Show usage instructions. */
  void usage()
  {
    printf("\nUsage: %s -i <dir> -o <dir> -f <format> [-w] [-s scale]\n"
	   " -i dir     Input directory containing dot graphs\n"
	   " -o dir     Output directory for generated graphs\n"
	   " -f format  Output format, one of pdf, svg, or png\n"
	   " -w         White background\n"
	   " -p         Postprocess frames to same size (PNG only)\n"
	   " -s scale   Scale factor to apply during rendering\n"
	   "\n",
	   argp.program_name());
  }

  virtual Cairo::RefPtr<Cairo::Context> get_cairo()
  {
    if (! cairo) {
      if (format == "pdf") {
	surface = Cairo::PdfSurface::create(outfile, bbw * scale, bbh * scale);
	printf("Creating PDF context of size %f x %f\n", bbw * scale, bbh * scale);
      } else if (format == "svg") {
	surface = Cairo::SvgSurface::create(outfile, bbw * scale, bbh * scale);
      } else if (format == "png") {
	surface = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32,
					      (int)ceilf(bbw * scale),
					      (int)ceilf(bbh * scale));
	write_to_png = true;
      }
      cairo = Cairo::Context::create(surface);
      if (white_bg) {
	cairo->set_source_rgb(1, 1, 1);
	cairo->paint();
      }
    }
    return cairo;
  }

  virtual bool scale_override() { return true; }

  virtual void get_dimensions(double &width, double &height)
  {
    width  = bbw * scale;
    height = bbh * scale;
  }

  virtual double get_scale() { return scale; }
  virtual void   set_scale(double scale) {};
  virtual void   set_translation(double tx, double ty) {};

  virtual void   get_translation(double &tx, double &ty)
  {
    // no padding
    tx = pad_x * scale;
    ty = (bbh - pad_y) * scale;
  }

  virtual void   set_bb(double bbw, double bbh)
  {
    this->bbw = bbw;
    this->bbh = bbh;

    if ( bbw * scale > maxwidth  ) {
      postproc_required = (maxwidth  != 0);
      maxwidth  = bbw * scale;
    }
    if ( bbh * scale > maxheight * scale ) {
      postproc_required = (maxheight != 0);
      maxheight = bbh * scale;
    }
  }

  virtual void   set_pad(double pad_x, double pad_y)
  {
    this->pad_x = pad_x;
    this->pad_y = pad_y;
  }


  virtual void   get_pad(double &pad_x, double &pad_y)
  {
    pad_x = 0;
    pad_y = 0;
  }

  /** Render graph. */
  void render()
  {

    FILE *f = fopen(infile.c_str(), "r");
#if defined(GRAPHVIZ_ATLEAST_230) && defined(WITH_CGRAPH)
    Agraph_t *g = agread(f,0);
#else
    Agraph_t *g = agread(f);
#endif
    if (g) {
      gvLayout(gvc, g, (char *)"dot");
      gvRender(gvc, g, (char *)"skillguicairo", NULL);
      gvFreeLayout(gvc, g);
      agclose(g);
    }
    fclose(f);

    if (write_to_png) {
      surface->write_to_png(outfile);
    }

    cairo.clear();
    surface.clear();
  }

  /** Run the renderer. */
  void run()
  {
    struct dirent *d;

    while ((d = readdir(directory)) != NULL) {
      if (fnmatch("*.dot", d->d_name, FNM_PATHNAME | FNM_PERIOD) == 0) {
	char infile_real[PATH_MAX];
	infile  = indir  + "/" + d->d_name;
	if (realpath(infile.c_str(), infile_real)) {
	  infile = infile_real;
	}
	char *basefile = strdup(infile.c_str());
	std::string basen = basename(basefile);
	free(basefile);
	outfile = outdir + "/" + basen.substr(0, basen.length() - 3) + format;
	printf("Converting %s to %s\n", infile.c_str(), outfile.c_str());
	render();
      } else {
	printf("%s does not match pattern\n", d->d_name);
      }
    }

    if (do_postproc && postproc_required) {
      postprocess();
    }
  }

  /** Write function for Cairo.
   * @param closure contains the file handle
   * @param data data to write
   * @param length length of data
   * @return Cairo status
   */
  static cairo_status_t write_func(void *closure,
				   const unsigned char *data, unsigned int length)
  {
    FILE *f = (FILE *)closure;
    if (fwrite(data, length, 1, f)) {
      return CAIRO_STATUS_SUCCESS;
    } else {
      return CAIRO_STATUS_WRITE_ERROR;
    }
  }

  /** Post-process files. Only valid for PNGs. */
  void postprocess()
  {
    printf("Post-processing PNG files, resizing to %fx%f\n", maxwidth, maxheight);
    struct dirent *d;
    DIR *output_dir = opendir(outdir.c_str());
    while ((d = readdir(output_dir)) != NULL) {
      if (fnmatch("*.png", d->d_name, FNM_PATHNAME | FNM_PERIOD) == 0) {
	infile = outdir + "/" + d->d_name;
	Cairo::RefPtr<Cairo::ImageSurface> imgs = Cairo::ImageSurface::create_from_png(infile);
	if ( (imgs->get_height() != maxheight) || (imgs->get_width() != maxwidth)) {
	  // need to re-create
	  char *tmpout = strdup((outdir + "/tmpXXXXXX").c_str());
	  FILE *f = fdopen(mkstemp(tmpout), "w");
	  outfile = tmpout;
	  free(tmpout);

	  Cairo::RefPtr<Cairo::ImageSurface> outs = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32,
										(int)ceilf(maxwidth),
										(int)ceilf(maxheight));
	  double tx = (maxwidth  - imgs->get_width()) / 2.0;
	  double ty = (maxheight - imgs->get_height()) / 2.0;
	  printf("Re-creating %s for post-processing, "
		 "resizing from %ix%i, tx=%f, ty=%f\n", infile.c_str(),
		 imgs->get_width(), imgs->get_height(), tx, ty);
	  Cairo::RefPtr<Cairo::Context> cc = Cairo::Context::create(outs);
	  if (white_bg) {
	    cc->set_source_rgb(1, 1, 1);
	    cc->paint();
	  }
	  cc->set_source(imgs, tx, ty);
	  cc->paint();
	  outs->write_to_png(&SkillGuiBatchRenderer::write_func, f);
	  imgs.clear();
	  cc.clear();
	  outs.clear();
	  fclose(f);
	  rename(outfile.c_str(), infile.c_str());
	}
      }
    }
    closedir(output_dir);
  }

 private:
  GVC_t *gvc;
  ArgumentParser argp;
  std::string format;
  Cairo::RefPtr<Cairo::Surface> surface;
  Cairo::RefPtr<Cairo::Context> cairo;
  bool write_to_png;
  bool white_bg;
  double bbw, bbh;
  double pad_x, pad_y;
  std::string infile;
  std::string outfile;
  std::string indir;
  std::string outdir;
  DIR *directory;
  double maxwidth, maxheight;
  bool postproc_required;
  bool do_postproc;
  double scale;
};

/** This is the main program of the Skill GUI.
 */
int
main(int argc, char **argv)
{
  SkillGuiBatchRenderer renderer(argc, argv);
  renderer.run();
  return 0;
}
