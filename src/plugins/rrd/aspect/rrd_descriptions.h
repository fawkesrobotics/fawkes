
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

#ifndef _PLUGINS_RRD_ASPECT_RRD_DESCRIPTIONS_H_
#define _PLUGINS_RRD_ASPECT_RRD_DESCRIPTIONS_H_

#include <vector>
#include <ctime>

namespace fawkes {

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
  const char *  get_name() const { return name_; };
  /** Get type. @return type */
  Type          get_type() const { return type_; };
  /** Get heartbeat. @return heartbeat */
  unsigned int  get_heartbeat() const { return heartbeat_; };
  /** Get minimum. @return minimum */
  float         get_min() const { return min_; };
  /** Get maximum. @return maximum */
  float         get_max() const { return max_; };
  /** Get RPN expression. @return RPN expression */
  const char *  get_rpn_expression() const { return rpn_expression_; };
  

 private:
  char          *name_;
  Type           type_;
  unsigned int   heartbeat_;
  float          min_;
  float          max_;
  char          *rpn_expression_;

  mutable char *  string_;
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
  ConsolidationFunction get_cf() const { return cf_; }
  /** Get xfiles factor. @return xfiles factor */
  float get_xff() const { return xff_; }
  /** Get number of steps. @return number of steps  */
  unsigned int get_steps() const { return steps_; }
  /** Get number of rows. @return number of rows  */
  unsigned int get_rows() const { return rows_; }

 private:
  ConsolidationFunction cf_;
  float                 xff_;
  unsigned int          steps_;
  unsigned int          rows_;

  mutable char *string_;
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
  const char *                         get_name() const { return name_; }
  /** Get step size in sec. @return step size */
  unsigned int                         get_step_sec() const { return step_sec_; }
  /** Check recreation flag.
   * @return true if files should be overwritte, false otherwise */
  bool                                 get_recreate() const { return recreate_; }
  /** Get data sources. * @return data sources  */
  const std::vector<RRDDataSource> &   get_ds() const { return ds_; }
  /** Get specific data source.
   * @param i index of data source
   * @return data source  */
  const RRDDataSource &                get_ds(size_t i) const { return ds_[i]; }
  /** Get RRD archives. @return RRD archive */
  const std::vector<RRDArchive> &      get_rra() const { return rra_; }
  /** Get file name. @return file name */
  const char *                         get_filename() const { return filename_; }

  void set_rrd_manager(RRDManager *rrd_manager);

 private:
  char                       *name_;
  unsigned int                step_sec_;
  bool                        recreate_;
  std::vector<RRDDataSource>  ds_;
  std::vector<RRDArchive>     rra_;
  char                       *filename_;

  RRDManager                 *rrd_manager_;
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
  const char *           get_name() const { return name_; };
  /** Get RRD definition. @return RRD definition */
  const RRDDefinition *  get_rrd_def() const { return rrd_def_; };
  /** Get data source name. @return data source name */
  const char *           get_ds_name() const { return ds_name_; };
  /** Get RPN expression. @return RPN expression */
  const char *           get_rpn_expression() const { return rpn_expression_; };
  /** Get consolidation function type. @return consolidation function type */
  RRDArchive::ConsolidationFunction  get_cf() const { return cf_; };

 private:
  char                                    *name_;
  const RRDDefinition                     *rrd_def_;
  char                                    *ds_name_;
  char                                    *rpn_expression_;
  RRDArchive::ConsolidationFunction        cf_;

  mutable char *string_;
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
  const char *                       get_def_name() const { return def_name_; }
  /** Get consolidation function type. @return consolidation function type */
  RRDArchive::ConsolidationFunction  get_cf() const { return cf_; }
  /** Get format string. @return format string  */
  const char *                       get_format() const { return format_; }
  
 private:
  char                              *def_name_;
  RRDArchive::ConsolidationFunction  cf_;
  char                              *format_;

  mutable char *string_;
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
  const char *  get_def_name() const { return def_name_; }
  /** Get line width. @return line width */
  float         get_width() const { return width_; }
  /** Get color string. @return color string  */
  const char *  get_color() const { return color_; }
  /** Get legend label. @return legend label */
  const char *  get_legend() const { return legend_; }
  /** Get stacked flag. @return true if line should be stacked, false otherwise. */
  bool          get_stacked() const { return stacked_; }

 private:
  char        *def_name_;
  float        width_;
  char        *color_;
  char        *legend_;
  bool         stacked_;

  mutable char *string_;
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
  const char *  get_def_name() const { return def_name_; }
  /** Get color string. @return color string  */
  const char *  get_color() const { return color_; }
  /** Get legend label. @return legend label */
  const char *  get_legend() const { return legend_; }
  /** Get stacked flag. @return true if line should be stacked, false otherwise. */
  bool          get_stacked() const { return stacked_; }

 private:
  char       *def_name_;
  char       *color_;
  char       *legend_;
  bool        stacked_;

  mutable char *string_;
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
  const char *  get_name() const { return name_; }
  /** Get RRD definition. @return RRD definition */
  const RRDDefinition * get_rrd_def() const { return rrd_def_; }
  /** Get start time. @return start time  */
  time_t                get_start() const { return start_; }
  /** Get end time. @return end time */
  time_t                get_end() const { return end_; }
  /** Get step size. @return step size */
  unsigned int          get_step() const { return step_; }
  /** Get title. @return tile */
  const char *          get_title() const { return title_; }
  /** Get vertical label. @return vertical label */
  const char *          get_vertical_label() const { return vertical_label_; }
  /** Get update interval. @return update interval */
  unsigned int          get_update_interval() const { return update_interval_; }
  /** Get slope moe. @return slope mode  */
  bool                  get_slope_mode() const { return slope_mode_; }
  /** Get definitions. @return definitions  */
  const std::vector<RRDGraphDataDefinition> & get_defs() const { return defs_; }
  /** Get graph elements. @return graph elements */
  const std::vector<RRDGraphElement *> & get_elements() const
  { return elements_; }
  /** Get line width. @return line width.  */
  unsigned int          get_width() const { return width_; }
  /** Get fonts. @return fonts */
  const std::vector<const char *>  get_fonts() const { return fonts_; }
  /** Get filename. @return filename */
  const char *          get_filename() const { return filename_; }

 private:
  char                                *name_;
  const RRDDefinition                 *rrd_def_;
  const time_t                         start_;
  const time_t                         end_;
  unsigned int                         step_;
  char                                *title_;
  char                                *vertical_label_;
  const unsigned int                   update_interval_;
  const bool                           slope_mode_;
  std::vector<RRDGraphDataDefinition>  defs_;
  std::vector<RRDGraphElement *>       elements_;
  unsigned int                         width_;
  char                                *width_s_; 
  char                                *start_s_; 
  char                                *end_s_; 
  char                                *step_s_; 
  std::vector<const char *>            fonts_;
  char                                *filename_;
  mutable size_t                       argc_;
  mutable const char                 **argv_;
};

} // end namespace fawkes

#endif
