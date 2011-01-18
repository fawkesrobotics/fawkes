
/***************************************************************************
 *  rrd_descriptions.cpp - Fawkes RRD descriptions
 *
 *  Created: Sat Dec 18 11:41:32 2010
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

#include <plugins/rrd/aspect/rrd_descriptions.h>
#include <plugins/rrd/aspect/rrd_manager.h>
#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <utils/misc/string_conversions.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cfloat>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class RRDDataSource <plugins/rrd/aspect/rrd_descriptions.h>
 * Class to represent a RRD data source.
 * @author Tim Niemueller
 */

/** Use for unknown min or max values */
const float RRDDataSource::UNKNOWN = FLT_MIN;

/** Constructor for regular data source.
 * @param name name of the data source
 * @param type type of the data source, may not be COMPUTE.
 * @param heartbeat Number of seconds after which a new value must be received
 * before the value is considered to be unknown.
 * @param min minimum value, use UNKNOWN constant if not known
 * @param max maximum value, use UNKNOWN constant if not known
 */
RRDDataSource::RRDDataSource(const char *name, Type type, unsigned int heartbeat,
			     float min, float max)
  : __name(strdup(name)), __type(type), __heartbeat(heartbeat),
    __min(min), __max(max), __rpn_expression(NULL), __string(NULL)
{
  if (__type == COMPUTE) {
    throw IllegalArgumentException("Non-compute data source ctor used with "
				   "COMPUTE type for DS %s", name);
  }
}


/** Constructor for expression RRDs.
 * @param name name of the data source
 * @param rpn_expression RPN expression
 */
RRDDataSource::RRDDataSource(const char *name, const char *rpn_expression)
  : __name(strdup(name)), __type(COMPUTE), __heartbeat(300), __min(UNKNOWN),
    __max(UNKNOWN), __rpn_expression(strdup(rpn_expression)), __string(NULL)
{
}

/** Copy constructor.
 * @param other other instance to copy
 */
RRDDataSource::RRDDataSource(const RRDDataSource &other)
  : __name(strdup(other.__name)), __type(other.__type),
    __heartbeat(other.__heartbeat),
    __min(other.__min), __max(other.__max),
    __rpn_expression(other.__rpn_expression ? strdup(other.__rpn_expression) : 0),
    __string(NULL)
{
}

/** Destructor. */
RRDDataSource::~RRDDataSource()
{
  if (__string) free(__string);
  if (__name)  free(__name);
  if (__rpn_expression)  free(__rpn_expression);
}

/** Assignment operator.
 * @param other Instance to copy data from.
 * @return reference to this instance
 */
RRDDataSource &
RRDDataSource::operator=(const RRDDataSource &other)
{
  if (__string) free(__string);
  if (__name)  free(__name);
  if (__rpn_expression)  free(__rpn_expression);
  __string = NULL;
  __rpn_expression = NULL;
  __name = strdup(other.__name);
  __type = other.__type;
  __heartbeat = other.__heartbeat;
  __min = other.__min;
  __max = other.__max;
  if (other.__rpn_expression)  __rpn_expression = strdup(other.__rpn_expression);

  return *this;
}


/** Get string reprensetation.
 * @return string representation suitable to be bassed to rrd_create().
 */
const char *
RRDDataSource::to_string() const
{
  if (! __string) {
    if (__type == COMPUTE) {
      if (asprintf(&__string, "DS:%s:COMPUTE:%s", __name, __rpn_expression) == -1) {
	throw OutOfMemoryException("Failed to create DS string for %s", __name);
      }
    } else {
      const char *type_string;
      switch (__type) {
      case COUNTER:  type_string = "COUNTER";  break;
      case DERIVE:   type_string = "DERIVE";   break;
      case ABSOLUTE: type_string = "ABSOLUTE"; break;
      default:       type_string = "GAUGE";    break;
      }
      char min_s[32];
      char max_s[32];
      if (__min == UNKNOWN) {
	strcpy(min_s, "U");
      } else {
	snprintf(min_s, 32, "%f", __min);
      }
      if (__max == UNKNOWN) {
	strcpy(max_s, "U");
      } else {
	snprintf(max_s, 32, "%f", __max);
      }
      if (asprintf(&__string, "DS:%s:%s:%u:%s:%s", __name, type_string,
		   __heartbeat, min_s, max_s) == -1) {
	throw OutOfMemoryException("Failed to create DS string for %s", __name);
      }
    }
  }

  return __string;
}


/** @class RRDArchive <plugins/rrd/aspect/rrd_descriptions.h>
 * RRD Archive description.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cf consolidation function
 * @param xff The xfiles factor defines what part of a consolidation interval
 * may be made up from *UNKNOWN* data while the consolidated value is still
 * regarded as known. It is given as the ratio of allowed *UNKNOWN* PDPs to
 * the number of PDPs in the interval. Thus, it ranges from 0 to 1 (exclusive).
 * @param steps defines how many of these primary data points are used to build
 * a consolidated data point which then goes into the archive.
 * @param rows defines how many generations of data values are kept in an RRA.
 * Obviously, this has to be greater than zero.
 */
RRDArchive::RRDArchive(ConsolidationFunction cf, float xff,
		       unsigned int steps, unsigned int rows)
  : __cf(cf), __xff(xff), __steps(steps), __rows(rows), __string(NULL)
{
}


/** Copy constructor.
 * @param rra instance to copy
 */
RRDArchive::RRDArchive(const RRDArchive &rra)
  : __cf(rra.__cf), __xff(rra.__xff), __steps(rra.__steps), __rows(rra.__rows),
    __string(NULL)
{
}


/** Destructor. */
RRDArchive::~RRDArchive()
{
  if (__string) free(__string);
}

/** Assignment operator.
 * @param rra instance to copy from
 * @return reference to this instance
 */
RRDArchive &
RRDArchive::operator=(const RRDArchive &rra)
{
  if (__string) free(__string);
  __string = NULL;
  __cf     = rra.__cf;
  __xff    = rra.__xff;
  __steps  = rra.__steps;
  __rows   = rra.__rows;
  return *this;
}

/** Get string representation.
 * @return string representation suitable to be passed to rrd_create().
 */
const char *
RRDArchive::to_string() const
{
  if (! __string) {
    const char *cf_string;
    switch (__cf) {
    case MIN:  cf_string = "MIN";  break;
    case MAX:  cf_string = "MAX";   break;
    case LAST: cf_string = "LAST"; break;
    default:   cf_string = "AVERAGE";    break;
    }
    if (asprintf(&__string, "RRA:%s:%f:%u:%u", cf_string,
		 __xff, __steps, __rows) == -1) {
      throw OutOfMemoryException("Failed to create RRA string");
    }
  }

  return __string;
}

/** Convert consolidation function type to string.
 * @param cf consolidation function type
 * @return string representation of @p cf, suitable for RRA lines.
 */
const char *
RRDArchive::cf_to_string(ConsolidationFunction cf)
{
  switch (cf) {
  case RRDArchive::MIN:  return "MIN";     break;
  case RRDArchive::MAX:  return "MAX";     break;
  case RRDArchive::LAST: return "LAST";    break;
  default:               return "AVERAGE"; break;
  }
}

/** @class RRDDefinition <plugins/rrd/aspect/rrd_descriptions.h>
 * RRD Definition.
 * This class describes everything required to create an RRD file.
 * It does not represent all the options rrdtool provides, but a reasonable
 * subset.
 * @author Tim Niemueller
 */

/** Constructor with default RRAs.
 * This creates the RRD definition with the default RRAs produced by
 * get_default_rra().
 * @param name RRD name
 * @param ds data sources
 * @param step_sec Specifies the base interval in seconds with which data
 * will be fed into the RRD.
 * @param recreate if true existing RRD files will be overwritten, otherwise
 * data is appended.
 */
RRDDefinition::RRDDefinition(const char *name, std::vector<RRDDataSource> &ds,
			     unsigned int step_sec, bool recreate)
  : __name(strdup(name)), __step_sec(step_sec), __recreate(recreate),
    __ds(ds), __rra(get_default_rra()), __filename(NULL), __rrd_manager(NULL)
{
}

/** Constructor.
 * @param name RRD name
 * @param ds data sources
 * @param rra RRAs for this RRD.
 * @param step_sec Specifies the base interval in seconds with which data
 * will be fed into the RRD.
 * @param recreate if true existing RRD files will be overwritten, otherwise
 * data is appended.
 */
RRDDefinition::RRDDefinition(const char *name,
			     std::vector<RRDDataSource> &ds,
			     std::vector<RRDArchive> &rra,
			     unsigned int step_sec, bool recreate)
  : __name(strdup(name)), __step_sec(step_sec), __recreate(recreate), __ds(ds),
    __rra(rra), __filename(NULL), __rrd_manager(NULL)
{
}


/** Copy constructor.
 * @param other instance to clone
 */
RRDDefinition::RRDDefinition(const RRDDefinition &other)
  : __name(strdup(other.__name)), __step_sec(other.__step_sec),
    __recreate(other.__recreate), __ds(other.__ds), __rra(other.__rra),
    __filename(other.__filename ? strdup(other.__filename) : 0),
    __rrd_manager(NULL)
{
}

/** Assignment operator.
 * @param other other instance to copy from
 * @return reference to this instance.
 */
RRDDefinition &
RRDDefinition::operator=(const RRDDefinition &other)
{
  if (__name)  free(__name);
  if (__filename) free(__filename);
  if (__rrd_manager)  __rrd_manager->remove_rrd(this);
  __filename    = NULL;
  __rrd_manager = NULL;
  __name        = strdup(other.__name);
  __step_sec    = other.__step_sec;
  __recreate    = other.__recreate;
  __ds          = other.__ds;
  __rra         = other.__rra;
  if (other.__filename) __filename=strdup(other.__filename);
  return *this;
}

/** Destructor. */
RRDDefinition::~RRDDefinition()
{
  if (__rrd_manager)  __rrd_manager->remove_rrd(this);

  if (__name)  free(__name);
  if (__filename)  free(__filename);
}

/** Get default RRAs. They correspond to the following and assume a
 * 10 second step size.
 * @code
 * "RRA:AVERAGE:0.5:1:720"    #  2 hours of 10 sec  averages
 * "RRA:AVERAGE:0.5:3:1680"   # 12 hours of 30 sec  averages
 * "RRA:AVERAGE:0.5:30:456"   #  1 day   of  5 min  averages
 * "RRA:AVERAGE:0.5:180:412"  #  7 days  of 30 min  averages
 * "RRA:AVERAGE:0.5:720:439"  #  4 weeks of  2 hour averages
 * "RRA:AVERAGE:0.5:8640:402" #  1 year  of  1 day averages
 * "RRA:MIN:0.5:1:720"
 * "RRA:MIN:0.5:3:1680"
 * "RRA:MIN:0.5:30:456"
 * "RRA:MIN:0.5:180:412"
 * "RRA:MIN:0.5:720:439"
 * "RRA:MIN:0.5:8640:402"
 * "RRA:MAX:0.5:1:720"
 * "RRA:MAX:0.5:3:1680"
 * "RRA:MAX:0.5:30:456"
 * "RRA:MAX:0.5:180:412"
 * "RRA:MAX:0.5:720:439"
 * "RRA:MAX:0.5:8640:402"
 * @endcode
 * @return vector of RRDArchive representing the described RRAs.
 */
const std::vector<RRDArchive>
RRDDefinition::get_default_rra()
{
  std::vector<RRDArchive> rv;
  rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5,    1,  720));
  rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5,    3, 1680));
  rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5,   30,  456));
  rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5,  180,  412));
  rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5,  720,  439));
  rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5, 8640,  402));
  rv.push_back(RRDArchive(RRDArchive::MIN,     0.5,    1,  720));
  rv.push_back(RRDArchive(RRDArchive::MIN,     0.5,    3, 1680));
  rv.push_back(RRDArchive(RRDArchive::MIN,     0.5,   30,  456));
  rv.push_back(RRDArchive(RRDArchive::MIN,     0.5,  180,  412));
  rv.push_back(RRDArchive(RRDArchive::MIN,     0.5,  720,  439));
  rv.push_back(RRDArchive(RRDArchive::MIN,     0.5, 8640,  402));
  rv.push_back(RRDArchive(RRDArchive::MAX,     0.5,    1,  720));
  rv.push_back(RRDArchive(RRDArchive::MAX,     0.5,    3, 1680));
  rv.push_back(RRDArchive(RRDArchive::MAX,     0.5,   30,  456));
  rv.push_back(RRDArchive(RRDArchive::MAX,     0.5,  180,  412));
  rv.push_back(RRDArchive(RRDArchive::MAX,     0.5,  720,  439));
  rv.push_back(RRDArchive(RRDArchive::MAX,     0.5, 8640,  402));
  return rv;
}


/** Find data source index.
 * @param ds_name name of the data source
 * @return index of found data source
 * @exception Exception thrown if the data source could not be found
 */
size_t
RRDDefinition::find_ds_index(const char *ds_name) const
{
  for (size_t i = 0; i < __ds.size(); ++i) {
    if (strcmp(__ds[i].get_name(), ds_name) == 0) return i;
  }

  throw Exception("Data source name %s not found", ds_name);
}

/** Set filename.
 * This can be done only once. Do not do this manually, rather let the RRDManager
 * handle this!
 * @param filename new filename, should be absolute, otherwise considered
 * relative to current working directory.
 */
void
RRDDefinition::set_filename(const char *filename)
{
  if (__filename) {
    throw Exception("Graph definition %s: filename has already been set!", __name);
  }
  __filename = strdup(filename);
}


/** Set RRD manager.
 * This can be done only once. Do not do this manually, rather let the RRDManager
 * handle this! The RRD manager is used to unregister this RRD if it is deleted.
 * This is a precaution to avoid dangling RRDs.
 * @param rrd_manager RRD manager to use
 */
void
RRDDefinition::set_rrd_manager(RRDManager *rrd_manager)
{
  if (__rrd_manager) {
    throw Exception("RRD definition %s: RRD manager has already been set", __name);
  }
  __rrd_manager = rrd_manager;
}



/** @class RRDGraphDataDefinition <plugins/rrd/aspect/rrd_descriptions.h>
 * Represent data definition in graph arguments.
 * @author Tim Niemueller
 * Currently supports only DEF and CDEF definitions.
 */

/** DEF constructor.
 * @param name name of the graph data source
 * @param cf consolidation function to apply if needed
 * @param rrd_def RRD definition to use
 * @param ds_name data source name in RRD, @p rrd_def will be queried for the
 * data source. If ds_name is NULL, @p name will be used as the data source name.
 */
RRDGraphDataDefinition::RRDGraphDataDefinition(const char *name,
					       RRDArchive::ConsolidationFunction cf,
					       const RRDDefinition *rrd_def,
					       const char *ds_name)
  : __name(strdup(name)), __rrd_def(rrd_def),
    __ds_name(ds_name ? strdup(ds_name) : strdup(name)),
    __rpn_expression(NULL), __cf(cf), __string(NULL)
{
}


/** CDEF constructor.
 * @param name name of the graph data source
 * @param rpn_expression RPN expression
 */
RRDGraphDataDefinition::RRDGraphDataDefinition(const char *name,
					       const char *rpn_expression)
  : __name(strdup(name)), __rrd_def(0), __ds_name(NULL),
    __rpn_expression(strdup(rpn_expression)),
    __cf(RRDArchive::AVERAGE), __string(NULL)
{
}


/** Copy constructor.
 * @param other instance to clone
 */
RRDGraphDataDefinition::RRDGraphDataDefinition(const RRDGraphDataDefinition &other)
  : __name(strdup(other.__name)), __rrd_def(other.__rrd_def),
    __ds_name(other.__ds_name ? strdup(other.__ds_name) : NULL),
    __rpn_expression(other.__rpn_expression ? strdup(other.__rpn_expression) : 0),
    __cf(other.__cf), __string(NULL)
{
}


/** Destructor. */
RRDGraphDataDefinition::~RRDGraphDataDefinition()
{
  if (__name)  free(__name);
  if (__ds_name)  free(__ds_name);
  if (__rpn_expression)  free(__rpn_expression);
  if (__string)   free(__string);
}

/** Assignment operator.
 * @param other instance to copy from
 * @return reference to this instance
 */
RRDGraphDataDefinition &
RRDGraphDataDefinition::operator=(const RRDGraphDataDefinition &other)
{
  if (__string)  free(__string);
  if (__ds_name) free(__ds_name);
  if (__name)  free(__name);
  if (__rpn_expression) free(__rpn_expression);

  __string         = NULL;
  __rpn_expression = NULL;
  __name           = strdup(other.__name);
  __rrd_def        = other.__rrd_def;
  if (other.__ds_name)  __ds_name = strdup(other.__ds_name);
  if (other.__rpn_expression)  __rpn_expression = other.__rpn_expression;
  __cf             = other.__cf;

  return *this;
}


/** Create string representation.
 * @return string representation suitable for rrd_graph_v().
 */
const char *
RRDGraphDataDefinition::to_string() const
{
  if (! __string) {
    if (__rpn_expression) {
      if (asprintf(&__string, "CDEF:%s=%s", __name, __rpn_expression) == -1) {
	throw OutOfMemoryException("Failed to create RRA string");
      }
    } else {
      size_t ds_index = __rrd_def->find_ds_index(__ds_name);

      if (asprintf(&__string, "DEF:%s=%s:%s:%s", __name, __rrd_def->get_filename(),
		   __rrd_def->get_ds(ds_index).get_name(),
		   RRDArchive::cf_to_string(__cf)) == -1) {
	throw OutOfMemoryException("Failed to create RRA string");
      }
    }
  }

  return __string;
}

/** @class RRDGraphElement <plugins/rrd/aspect/rrd_descriptions.h>
 * Interface for graph elements.
 * This super class provides the general interface for the different
 * existing graph elements.
 * @author Tim Niemueller
 *
 * @fn RRDGraphElement * RRDGraphElement::clone() const
 * Clone this element.
 * The clone function is needed to copy an object without knowing its type and
 * therefore without calling its copy constructor.
 * @return new copied instance
 */

/** Create string representation.
 * @return string suitable for rrd_graph_v().
 */
const char *
RRDGraphElement::to_string() const
{
  throw NotImplementedException("Invalid graph element");
}



/** @class RRDGraphGPrint <plugins/rrd/aspect/rrd_descriptions.h>
 * Print string inside graph.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param def_name Data definition for this graph element.
 * @param cf consolidation function to use
 * @param format Format string, cf. man rrdgraph_graph(1).
 */
RRDGraphGPrint::RRDGraphGPrint(const char *def_name,
			       RRDArchive::ConsolidationFunction cf,
			       const char *format)
  : __def_name(strdup(def_name)), __cf(cf), __format(strdup(format)),
    __string(NULL)
{
}

/** Copy constructor.
 * @param other instance to copy
 */
RRDGraphGPrint::RRDGraphGPrint(const RRDGraphGPrint &other)
  : __def_name(strdup(other.__def_name)), __cf(other.__cf),
    __format(strdup(other.__format)), __string(NULL)
{
}


/** Destructor. */
RRDGraphGPrint::~RRDGraphGPrint()
{
  if (__def_name)  free(__def_name);
  if (__format) free(__format);
  if (__string) free(__string);
}

/** Assignment operator.
 * @param g matching graph element to assign
 * @return reference to this instance
 */
RRDGraphGPrint &
RRDGraphGPrint::operator=(const RRDGraphGPrint &g)
{
  if (__def_name)  free(__def_name);
  if (__format) free(__format);
  if (__string) free(__string);

  __string   = NULL;
  __def_name = strdup(g.__def_name);
  __cf       = g.__cf;
  __format   = strdup(g.__format);

  return *this;
}


const char *
RRDGraphGPrint::to_string() const
{
  if (! __string) {
    if (asprintf(&__string, "GPRINT:%s:%s:%s", __def_name,
		 RRDArchive::cf_to_string(__cf), __format) == -1) {
      throw OutOfMemoryException("Failed to create RRD graph GPRINT string");
    }
  }

  return __string;
}


/** @class RRDGraphLine <plugins/rrd/aspect/rrd_descriptions.h>
 * Print graph line.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param def_name Data definition for this graph element.
 * @param width line width
 * @param color color hash string (HTML style, e.g. FF0000)
 * @param legend legend string
 * @param stacked true to stack on previous graph element
 */
RRDGraphLine::RRDGraphLine(const char *def_name, float width, const char *color,
			   const char *legend, bool stacked)
  : __def_name(strdup(def_name)), __width(width), __color(strdup(color)),
    __legend(strdup(legend)), __stacked(stacked), __string(NULL)
{
}

/** Copy ctor.
 * @param other instance to copy
 */
RRDGraphLine::RRDGraphLine(const RRDGraphLine &other)
  : __def_name(strdup(other.__def_name)), __width(other.__width),
    __color(strdup(other.__color)),
    __legend(strdup(other.__legend)), __stacked(other.__stacked), __string(NULL)
{
}


/** Destructor. */
RRDGraphLine::~RRDGraphLine()
{
  if (__def_name)  free(__def_name);
  if (__color)  free(__color);
  if (__legend)  free(__legend);
  if (__string) free(__string);
}


/** Assignment operator.
 * @param g matching graph element to assign
 * @return reference to this instance
 */
RRDGraphLine &
RRDGraphLine::operator=(const RRDGraphLine &g)
{
  if (__def_name)  free(__def_name);
  if (__color)  free(__color);
  if (__legend)  free(__legend);
  if (__string) free(__string);

  __string   = NULL;
  __def_name = strdup(g.__def_name);
  __width    = g.__width;
  __color    = strdup(g.__color);
  __legend   = strdup(g.__legend);
  __stacked  = g.__stacked;

  return *this;
}


const char *
RRDGraphLine::to_string() const
{
  if (! __string) {
    if (asprintf(&__string, "LINE%f:%s#%s:%s%s", __width, __def_name, __color,
		 __legend, __stacked ? ":STACK" : "") == -1) {
      throw OutOfMemoryException("Failed to create RRD graph LINE string");
    }
  }

  return __string;
}


/** @class RRDGraphArea <plugins/rrd/aspect/rrd_descriptions.h>
 * Print graph area.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param def_name Data definition for this graph element.
 * @param color color hash string (HTML style, e.g. FF0000)
 * @param legend legend string
 * @param stacked true to stack on previous graph element
 */
RRDGraphArea::RRDGraphArea(const char *def_name,const char *color,
			   const char *legend, bool stacked)
  : __def_name(strdup(def_name)), __color(strdup(color)),
    __legend(strdup(legend)), __stacked(stacked), __string(NULL)
{
}


/** Copy ctor.
 * @param other instance to copy
 */
RRDGraphArea::RRDGraphArea(const RRDGraphArea &other)
  : __def_name(strdup(other.__def_name)), __color(strdup(other.__color)),
    __legend(strdup(other.__legend)), __stacked(other.__stacked), __string(NULL)
{
}


/** Destructor. */
RRDGraphArea::~RRDGraphArea()
{
  if (__def_name)  free(__def_name);
  if (__color)  free(__color);
  if (__legend)  free(__legend);
  if (__string) free(__string);
}


/** Assignment operator.
 * @param g matching graph element to assign
 * @return reference to this instance
 */
RRDGraphArea &
RRDGraphArea::operator=(const RRDGraphArea &g)
{
  if (__def_name)  free(__def_name);
  if (__color)  free(__color);
  if (__legend)  free(__legend);
  if (__string) free(__string);

  __string   = NULL;
  __def_name = strdup(g.__def_name);
  __color    = strdup(g.__color);
  __legend   = strdup(g.__legend);
  __stacked  = g.__stacked;

  return *this;
}


const char *
RRDGraphArea::to_string() const
{
  if (! __string) {
    if (asprintf(&__string, "AREA:%s#%s:%s%s", __def_name, __color, __legend,
		 __stacked ? ":STACK" : "") == -1) {
      throw OutOfMemoryException("Failed to create RRD graph AREA string");
    }
  }

  return __string;
}


/** @class RRDGraphDefinition <plugins/rrd/aspect/rrd_descriptions.h>
 * Class representing a graph definition.
 * This graph definition is used to generate all required parameters to create
 * a graph from an RRD.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param name name of this graph definition, used internally, name must be
 * unique among all registered graphs.
 * @param rrd_def pointer to definition of the RRD to graph
 * @param start time from where to start graphing. Maybe an absolute time or
 * a negative number for relative times, e.g. "-300" for 5 minutes back from now.
 * @param end time where to end graphing. Maybe an absolute time or a negative
 * number for relative times, e.g. "-300" for 5 minutes back from now.
 * @param step step size in seconds
 * @param title Graph title to print on top of graph
 * @param vertical_label string printed rotated by 90° counter-clockwise besides
 * the vertical axis. Usually should carry description of the Y axis units.
 * @param update_interval The interval at which the graph should be generated.
 * @param slope_mode true to enable slope mode when graphing
 * @param def data definitions for the graph
 * @param elements elements to print in the graph. This graph definition takes
 * ownership of the graph elemenets and will delete them in its dtor.
 */
RRDGraphDefinition::RRDGraphDefinition(const char *name, RRDDefinition *rrd_def,
				       const char *title,
				       const char *vertical_label,
				       std::vector<RRDGraphDataDefinition> &def,
				       std::vector<RRDGraphElement *> &elements,
				       time_t start, time_t end, unsigned int step,
				       unsigned int update_interval,
				       bool slope_mode)
  : __name(strdup(name)), __rrd_def(rrd_def),
    __start(start), __end(end), __step(step),
    __title(strdup(title)), __vertical_label(strdup(vertical_label)),
    __update_interval(update_interval), __slope_mode(slope_mode),
    __defs(def), __elements(elements)
{
  __filename = NULL;
  __argv = NULL;
  __argc = 0;
  __fonts.push_back("LEGEND:10:");
  __fonts.push_back("UNIT:8:");
  __fonts.push_back("TITLE:12:");
  __fonts.push_back("AXIS:8:");
  __width = 560;
  __width_s = strdup(StringConversions::to_string(__width).c_str());
  __start_s = strdup(StringConversions::to_string(__start).c_str());
  __end_s   = strdup(StringConversions::to_string(__end).c_str());
  __step_s  = strdup(StringConversions::to_string(__step).c_str());
}

/** Copy constructor.
 * @param other instance to copy
 */
RRDGraphDefinition::RRDGraphDefinition(const RRDGraphDefinition &other)
  : __name(strdup(other.__name)), __rrd_def(other.__rrd_def),
    __start(other.__start), __end(other.__end), __step(other.__step),
    __title(strdup(other.__title)),
    __vertical_label(strdup(other.__vertical_label)),
    __update_interval(other.__update_interval),
    __slope_mode(other.__slope_mode), __defs(other.__defs),
    __width(other.__width), __fonts(other.__fonts),
    __filename(strdup(other.__filename))
{
  std::vector<RRDGraphElement *>::const_iterator i;
  for (i = other.__elements.begin(); i != other.__elements.end(); ++i) {
    __elements.push_back((*i)->clone());
  }

  __argv = NULL;
  __argc = 0;
  __width_s = strdup(StringConversions::to_string(__width).c_str());
  __start_s = strdup(StringConversions::to_string(__start).c_str());
  __end_s   = strdup(StringConversions::to_string(__end).c_str());
  __step_s  = strdup(StringConversions::to_string(__step).c_str());
}


/** Destructor. */
RRDGraphDefinition::~RRDGraphDefinition()
{
  if (__filename) free(__filename);
  if (__argv)  free(__argv);
  if (__name)  free(__name);
  if (__title)  free(__title);
  if (__vertical_label)  free(__vertical_label);

  free(__width_s);
  free(__start_s);
  free(__end_s);
  free(__step_s);

  std::vector<RRDGraphElement *>::iterator i;
  for (i = __elements.begin(); i != __elements.end(); ++i) {
    delete *i;
  }
}

/** Set filename.
 * This can be done only once. Do not do this manually, rather let the RRDManager
 * handle this!
 * @param filename new filename, should be absolute, otherwise considered
 * relative to current working directory.
 */
void
RRDGraphDefinition::set_filename(const char *filename)
{
  if (__filename) {
    throw Exception("Graph definition for RRD %s: filename has already been set!",
		    __rrd_def->get_name());
  }
  __filename = strdup(filename);
}

/** Get argument array and size.
 * @param argc upon completion contains the number of arguments in the
 * return value.
 * @return argument array suitable for rrd_create_v().
 */
const char **
RRDGraphDefinition::get_argv(size_t &argc) const
{
  if (__argv == NULL) {
    // "graph" filename --disable-rrdtool-tag --width ... --start ... --end ...
    // [fonts] --title ... --vertical-label ... [--slope-mode] DEFS... ELEMS...
    __argc = 16 + __fonts.size() * 2 + __defs.size() + __elements.size();
    __argv = (const char **)malloc(__argc * sizeof(char *));
    size_t i = 0;
    __argv[i++] = "graph";
    __argv[i++] = __filename;
    __argv[i++] = "--disable-rrdtool-tag";
    __argv[i++] = "--width";
    __argv[i++] = __width_s;
    __argv[i++] = "--start";
    __argv[i++] = __start_s;
    __argv[i++] = "--end";
    __argv[i++] = __end_s;
    __argv[i++] = "--step";
    __argv[i++] = __step_s;
    __argv[i++] = "--title";
    __argv[i++] = __title;
    __argv[i++] = "--vertical-label";

    if (strcmp(__vertical_label, "") == 0) {
      __argv[i++] = " ";
    } else {
      __argv[i++] = __vertical_label;
    }

    if (__slope_mode) __argv[i++] = "--slope-mode";

    std::vector<const char *>::const_iterator f;
    for (f = __fonts.begin(); f != __fonts.end(); ++f) {
      __argv[i++] = "--font";
      __argv[i++] = *f;
    }

    std::vector<RRDGraphDataDefinition>::const_iterator d;
    for (d = __defs.begin(); d != __defs.end(); ++d) {
      __argv[i++] = d->to_string();
    }

    std::vector<RRDGraphElement *>::const_iterator e;
    for (e = __elements.begin(); e != __elements.end(); ++e) {
      __argv[i++] = (*e)->to_string();
    }

    __argc = i;
  }

  argc = __argc;
  return __argv;
}


} // end namespace fawkes
