
/***************************************************************************
 *  rrd_descriptions.h - Fawkes RRD descriptions
 *
 *  Created: Sat Dec 18 02:19:03 2010
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_RRD_ASPECT_RRD_DESCRIPTIONS_H_
#define __PLUGINS_RRD_ASPECT_RRD_DESCRIPTIONS_H_

#include <vector>
#include <ctime>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RRDManager;

class RRDDataSource
{
 public:
  /** Data source type. */
  typedef enum {
    GAUGE,	/**< Gauge value. */
    COUNTER,	/**< Counter value. */
    DERIVE,	/**< Derived value. */
    ABSOLUTE,	/**< Absolute value. */
    COMPUTE	/**< Computed value. */
  } Type;

  static const float UNKNOWN;

  RRDDataSource(const char *name, Type type, unsigned int heartbeat = 30,
		float min = 0, float max = UNKNOWN);
  RRDDataSource(const char *name, const char *rpn_expression);
  RRDDataSource(const RRDDataSource &other);
  ~RRDDataSource();
  RRDDataSource & operator=(const RRDDataSource &other);


  const char *  to_string() const;

  /** Get name. @return name */
  const char *  get_name() const { return __name; };
  /** Get type. @return type */
  Type          get_type() const { return __type; };
  /** Get heartbeat. @return heartbeat */
  unsigned int  get_heartbeat() const { return __heartbeat; };
  /** Get minimum. @return minimum */
  float         get_min() const { return __min; };
  /** Get maximum. @return maximum */
  float         get_max() const { return __max; };
  /** Get RPN expression. @return RPN expression */
  const char *  get_rpn_expression() const { return __rpn_expression; };
  

 private:
  char          *__name;
  Type           __type;
  unsigned int   __heartbeat;
  float          __min;
  float          __max;
  char          *__rpn_expression;

  mutable char *  __string;
};

class RRDArchive
{
 public:
  /** Consolidation function type. */
  typedef enum {
    AVERAGE,	/**< Averaging consolidation function. */
    MIN,	/**< Minimum consolidation function. */
    MAX,	/**< Maximum consolidation function. */
    LAST	/**< Last value consolidation function. */
  } ConsolidationFunction;

  RRDArchive(ConsolidationFunction cf,
	     float xff, unsigned int steps, unsigned int rows);
  RRDArchive(const RRDArchive &rra);
  ~RRDArchive();

  const char *  to_string() const;
  static const char *  cf_to_string(ConsolidationFunction cf);

  RRDArchive & operator=(const RRDArchive &rra);

  /** Get consolidation function type. @return consolidation function type */
  ConsolidationFunction get_cf() const { return __cf; }
  /** Get xfiles factor. @return xfiles factor */
  float get_xff() const { return __xff; }
  /** Get number of steps. @return number of steps  */
  unsigned int get_steps() const { return __steps; }
  /** Get number of rows. @return number of rows  */
  unsigned int get_rows() const { return __rows; }

 private:
  ConsolidationFunction __cf;
  float                 __xff;
  unsigned int          __steps;
  unsigned int          __rows;

  mutable char *__string;
};

class RRDDefinition
{
 public:
  RRDDefinition(const char *name,
		std::vector<RRDDataSource> &ds,
		unsigned int step_sec = 10, bool recreate = false);

  RRDDefinition(const char *name,
		std::vector<RRDDataSource> &ds,
		std::vector<RRDArchive> &rra,
		unsigned int step_sec = 10, bool recreate = false);
  RRDDefinition(const RRDDefinition &other);
  ~RRDDefinition();
  RRDDefinition & operator=(const RRDDefinition &other);

  size_t find_ds_index(const char *ds_name) const;
  void   set_filename(const char *filename);
  static const std::vector<RRDArchive> get_default_rra();

  /** Get name. @return name  */
  const char *                         get_name() const { return __name; }
  /** Get step size in sec. @return step size */
  unsigned int                         get_step_sec() const { return __step_sec; }
  /** Check recreation flag.
   * @return true if files should be overwritte, false otherwise */
  bool                                 get_recreate() const { return __recreate; }
  /** Get data sources. * @return data sources  */
  const std::vector<RRDDataSource> &   get_ds() const { return __ds; }
  /** Get specific data source.
   * @param i index of data source
   * @return data source  */
  const RRDDataSource &                get_ds(size_t i) const { return __ds[i]; }
  /** Get RRD archives. @return RRD archive */
  const std::vector<RRDArchive> &      get_rra() const { return __rra; }
  /** Get file name. @return file name */
  const char *                         get_filename() const { return __filename; }

  void set_rrd_manager(RRDManager *rrd_manager);

 private:
  char                       *__name;
  unsigned int                __step_sec;
  bool                        __recreate;
  std::vector<RRDDataSource>  __ds;
  std::vector<RRDArchive>     __rra;
  char                       *__filename;

  RRDManager                 *__rrd_manager;
};

class RRDGraphDataDefinition
{
 public:
  RRDGraphDataDefinition(const char *name, RRDArchive::ConsolidationFunction cf,
			 const RRDDefinition *rrd_def, const char *ds_name = NULL);
  RRDGraphDataDefinition(const char *name, const char *rpn_expression);
  RRDGraphDataDefinition(const RRDGraphDataDefinition &other);
  ~RRDGraphDataDefinition();
  RRDGraphDataDefinition & operator=(const RRDGraphDataDefinition &rra);

  const char *  to_string() const;

  /** Get name. @return name */
  const char *           get_name() const { return __name; };
  /** Get RRD definition. @return RRD definition */
  const RRDDefinition *  get_rrd_def() const { return __rrd_def; };
  /** Get data source name. @return data source name */
  const char *           get_ds_name() const { return __ds_name; };
  /** Get RPN expression. @return RPN expression */
  const char *           get_rpn_expression() const { return __rpn_expression; };
  /** Get consolidation function type. @return consolidation function type */
  RRDArchive::ConsolidationFunction  get_cf() const { return __cf; };

 private:
  char                                    *__name;
  const RRDDefinition                     *__rrd_def;
  char                                    *__ds_name;
  char                                    *__rpn_expression;
  RRDArchive::ConsolidationFunction        __cf;

  mutable char *__string;
};

class RRDGraphElement
{
 public:
  virtual ~RRDGraphElement() {}
  virtual RRDGraphElement * clone() const = 0;
  virtual const char *  to_string() const;
};


class RRDGraphGPrint : public RRDGraphElement
{
 public:
  RRDGraphGPrint(const char *def_name, RRDArchive::ConsolidationFunction cf,
		 const char *format);
  RRDGraphGPrint(const RRDGraphGPrint &other);
  virtual ~RRDGraphGPrint();

  RRDGraphGPrint &  operator=(const RRDGraphGPrint &g);

  virtual RRDGraphElement * clone() const { return new RRDGraphGPrint(*this); }

  virtual const char *  to_string() const;

  /** Get definition name. @return definition name */
  const char *                       get_def_name() const { return __def_name; }
  /** Get consolidation function type. @return consolidation function type */
  RRDArchive::ConsolidationFunction  get_cf() const { return __cf; }
  /** Get format string. @return format string  */
  const char *                       get_format() const { return __format; }
  
 private:
  char                              *__def_name;
  RRDArchive::ConsolidationFunction  __cf;
  char                              *__format;

  mutable char *__string;
};

class RRDGraphLine : public RRDGraphElement
{
 public:
  RRDGraphLine(const char *def_name, float width, const char *color,
	       const char *legend, bool stacked = false);
  RRDGraphLine(const RRDGraphLine &other);
  virtual ~RRDGraphLine();

  virtual RRDGraphElement * clone() const { return new RRDGraphLine(*this); }

  RRDGraphLine &  operator=(const RRDGraphLine &g);

  virtual const char *  to_string() const;

  /** Get definition name. @return definition name */
  const char *  get_def_name() const { return __def_name; }
  /** Get line width. @return line width */
  float         get_width() const { return __width; }
  /** Get color string. @return color string  */
  const char *  get_color() const { return __color; }
  /** Get legend label. @return legend label */
  const char *  get_legend() const { return __legend; }
  /** Get stacked flag. @return true if line should be stacked, false otherwise. */
  bool          get_stacked() const { return __stacked; }

 private:
  char        *__def_name;
  float        __width;
  char        *__color;
  char        *__legend;
  bool         __stacked;

  mutable char *__string;
};

class RRDGraphArea : public RRDGraphElement
{
 public:
  RRDGraphArea(const char *def_name, const char *color,
	       const char *legend, bool stacked = false);
  RRDGraphArea(const RRDGraphArea &other);
  virtual ~RRDGraphArea();

  virtual RRDGraphElement * clone() const { return new RRDGraphArea(*this); }

  RRDGraphArea &  operator=(const RRDGraphArea &g);

  virtual const char *  to_string() const;

  /** Get definition name. @return definition name */
  const char *  get_def_name() const { return __def_name; }
  /** Get color string. @return color string  */
  const char *  get_color() const { return __color; }
  /** Get legend label. @return legend label */
  const char *  get_legend() const { return __legend; }
  /** Get stacked flag. @return true if line should be stacked, false otherwise. */
  bool          get_stacked() const { return __stacked; }

 private:
  char       *__def_name;
  char       *__color;
  char       *__legend;
  bool        __stacked;

  mutable char *__string;
};

class RRDGraphDefinition
{
 public:
  RRDGraphDefinition(const char *name, RRDDefinition *rrd_def,
		     const char *title, const char *vertical_label,
		     std::vector<RRDGraphDataDefinition> &def,
		     std::vector<RRDGraphElement *> &elements,
		     time_t start = -600, time_t end = -10, unsigned int step = 10,
		     unsigned int update_interval = 10, bool slope_mode = false);
  RRDGraphDefinition(const RRDGraphDefinition &other);
  ~RRDGraphDefinition();

  void set_filename(const char *filename);
  const char ** get_argv(size_t &argc) const;

  
  /** Get graph definition name. @return graph definition name */
  const char *  get_name() const { return __name; }
  /** Get RRD definition. @return RRD definition */
  const RRDDefinition * get_rrd_def() const { return __rrd_def; }
  /** Get start time. @return start time  */
  time_t                get_start() const { return __start; }
  /** Get end time. @return end time */
  time_t                get_end() const { return __end; }
  /** Get step size. @return step size */
  unsigned int          get_step() const { return __step; }
  /** Get title. @return tile */
  const char *          get_title() const { return __title; }
  /** Get vertical label. @return vertical label */
  const char *          get_vertical_label() const { return __vertical_label; }
  /** Get update interval. @return update interval */
  const unsigned int    get_update_interval() const { return __update_interval; }
  /** Get slope moe. @return slope mode  */
  const bool            get_slope_mode() const { return __slope_mode; }
  /** Get definitions. @return definitions  */
  const std::vector<RRDGraphDataDefinition> & get_defs() const { return __defs; }
  /** Get graph elements. @return graph elements */
  const std::vector<RRDGraphElement *> & get_elements() const
  { return __elements; }
  /** Get line width. @return line width.  */
  unsigned int          get_width() const { return __width; }
  /** Get fonts. @return fonts */
  const std::vector<const char *>  get_fonts() const { return __fonts; }
  /** Get filename. @return filename */
  const char *          get_filename() const { return __filename; }

 private:
  char                                *__name;
  const RRDDefinition                 *__rrd_def;
  const time_t                         __start;
  const time_t                         __end;
  unsigned int                         __step;
  char                                *__title;
  char                                *__vertical_label;
  const unsigned int                   __update_interval;
  const bool                           __slope_mode;
  std::vector<RRDGraphDataDefinition>  __defs;
  std::vector<RRDGraphElement *>       __elements;
  unsigned int                         __width;
  char                                *__width_s; 
  char                                *__start_s; 
  char                                *__end_s; 
  char                                *__step_s; 
  std::vector<const char *>            __fonts;
  char                                *__filename;
  mutable size_t                       __argc;
  mutable const char                 **__argv;
};

} // end namespace fawkes

#endif
