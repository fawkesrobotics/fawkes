
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

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <plugins/rrd/aspect/rrd_descriptions.h>
#include <plugins/rrd/aspect/rrd_manager.h>
#include <utils/misc/string_conversions.h>

#include <cfloat>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace fawkes {

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
RRDDataSource::RRDDataSource(const char * name,
                             Type         type,
                             unsigned int heartbeat,
                             float        min,
                             float        max)
: name_(strdup(name)),
  type_(type),
  heartbeat_(heartbeat),
  min_(min),
  max_(max),
  rpn_expression_(NULL),
  string_(NULL)
{
	if (type_ == COMPUTE) {
		throw IllegalArgumentException("Non-compute data source ctor used with "
		                               "COMPUTE type for DS %s",
		                               name);
	}
}

/** Constructor for expression RRDs.
 * @param name name of the data source
 * @param rpn_expression RPN expression
 */
RRDDataSource::RRDDataSource(const char *name, const char *rpn_expression)
: name_(strdup(name)),
  type_(COMPUTE),
  heartbeat_(300),
  min_(UNKNOWN),
  max_(UNKNOWN),
  rpn_expression_(strdup(rpn_expression)),
  string_(NULL)
{
}

/** Copy constructor.
 * @param other other instance to copy
 */
RRDDataSource::RRDDataSource(const RRDDataSource &other)
: name_(strdup(other.name_)),
  type_(other.type_),
  heartbeat_(other.heartbeat_),
  min_(other.min_),
  max_(other.max_),
  rpn_expression_(other.rpn_expression_ ? strdup(other.rpn_expression_) : 0),
  string_(NULL)
{
}

/** Destructor. */
RRDDataSource::~RRDDataSource()
{
	if (string_)
		free(string_);
	if (name_)
		free(name_);
	if (rpn_expression_)
		free(rpn_expression_);
}

/** Assignment operator.
 * @param other Instance to copy data from.
 * @return reference to this instance
 */
RRDDataSource &
RRDDataSource::operator=(const RRDDataSource &other)
{
	if (string_)
		free(string_);
	if (name_)
		free(name_);
	if (rpn_expression_)
		free(rpn_expression_);
	string_         = NULL;
	rpn_expression_ = NULL;
	name_           = strdup(other.name_);
	type_           = other.type_;
	heartbeat_      = other.heartbeat_;
	min_            = other.min_;
	max_            = other.max_;
	if (other.rpn_expression_)
		rpn_expression_ = strdup(other.rpn_expression_);

	return *this;
}

/** Get string reprensetation.
 * @return string representation suitable to be bassed to rrd_create().
 */
const char *
RRDDataSource::to_string() const
{
	if (!string_) {
		if (type_ == COMPUTE) {
			if (asprintf(&string_, "DS:%s:COMPUTE:%s", name_, rpn_expression_) == -1) {
				throw OutOfMemoryException("Failed to create DS string for %s", name_);
			}
		} else {
			const char *type_string;
			switch (type_) {
			case COUNTER: type_string = "COUNTER"; break;
			case DERIVE: type_string = "DERIVE"; break;
			case ABSOLUTE: type_string = "ABSOLUTE"; break;
			default: type_string = "GAUGE"; break;
			}
			char min_s[32];
			char max_s[32];
			if (min_ == UNKNOWN) {
				strcpy(min_s, "U");
			} else {
				snprintf(min_s, 32, "%f", min_);
			}
			if (max_ == UNKNOWN) {
				strcpy(max_s, "U");
			} else {
				snprintf(max_s, 32, "%f", max_);
			}
			if (asprintf(&string_, "DS:%s:%s:%u:%s:%s", name_, type_string, heartbeat_, min_s, max_s)
			    == -1) {
				throw OutOfMemoryException("Failed to create DS string for %s", name_);
			}
		}
	}

	return string_;
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
RRDArchive::RRDArchive(ConsolidationFunction cf, float xff, unsigned int steps, unsigned int rows)
: cf_(cf), xff_(xff), steps_(steps), rows_(rows), string_(NULL)
{
}

/** Copy constructor.
 * @param rra instance to copy
 */
RRDArchive::RRDArchive(const RRDArchive &rra)
: cf_(rra.cf_), xff_(rra.xff_), steps_(rra.steps_), rows_(rra.rows_), string_(NULL)
{
}

/** Destructor. */
RRDArchive::~RRDArchive()
{
	if (string_)
		free(string_);
}

/** Assignment operator.
 * @param rra instance to copy from
 * @return reference to this instance
 */
RRDArchive &
RRDArchive::operator=(const RRDArchive &rra)
{
	if (string_)
		free(string_);
	string_ = NULL;
	cf_     = rra.cf_;
	xff_    = rra.xff_;
	steps_  = rra.steps_;
	rows_   = rra.rows_;
	return *this;
}

/** Get string representation.
 * @return string representation suitable to be passed to rrd_create().
 */
const char *
RRDArchive::to_string() const
{
	if (!string_) {
		const char *cf_string;
		switch (cf_) {
		case MIN: cf_string = "MIN"; break;
		case MAX: cf_string = "MAX"; break;
		case LAST: cf_string = "LAST"; break;
		default: cf_string = "AVERAGE"; break;
		}
		if (asprintf(&string_, "RRA:%s:%f:%u:%u", cf_string, xff_, steps_, rows_) == -1) {
			throw OutOfMemoryException("Failed to create RRA string");
		}
	}

	return string_;
}

/** Convert consolidation function type to string.
 * @param cf consolidation function type
 * @return string representation of @p cf, suitable for RRA lines.
 */
const char *
RRDArchive::cf_to_string(ConsolidationFunction cf)
{
	switch (cf) {
	case RRDArchive::MIN: return "MIN"; break;
	case RRDArchive::MAX: return "MAX"; break;
	case RRDArchive::LAST: return "LAST"; break;
	default: return "AVERAGE"; break;
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
RRDDefinition::RRDDefinition(const char *                name,
                             std::vector<RRDDataSource> &ds,
                             unsigned int                step_sec,
                             bool                        recreate)
: name_(strdup(name)),
  step_sec_(step_sec),
  recreate_(recreate),
  ds_(ds),
  rra_(get_default_rra()),
  filename_(NULL),
  rrd_manager_(NULL)
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
RRDDefinition::RRDDefinition(const char *                name,
                             std::vector<RRDDataSource> &ds,
                             std::vector<RRDArchive> &   rra,
                             unsigned int                step_sec,
                             bool                        recreate)
: name_(strdup(name)),
  step_sec_(step_sec),
  recreate_(recreate),
  ds_(ds),
  rra_(rra),
  filename_(NULL),
  rrd_manager_(NULL)
{
}

/** Copy constructor.
 * @param other instance to clone
 */
RRDDefinition::RRDDefinition(const RRDDefinition &other)
: name_(strdup(other.name_)),
  step_sec_(other.step_sec_),
  recreate_(other.recreate_),
  ds_(other.ds_),
  rra_(other.rra_),
  filename_(other.filename_ ? strdup(other.filename_) : 0),
  rrd_manager_(NULL)
{
}

/** Assignment operator.
 * @param other other instance to copy from
 * @return reference to this instance.
 */
RRDDefinition &
RRDDefinition::operator=(const RRDDefinition &other)
{
	if (name_)
		free(name_);
	if (filename_)
		free(filename_);
	if (rrd_manager_)
		rrd_manager_->remove_rrd(this);
	filename_    = NULL;
	rrd_manager_ = NULL;
	name_        = strdup(other.name_);
	step_sec_    = other.step_sec_;
	recreate_    = other.recreate_;
	ds_          = other.ds_;
	rra_         = other.rra_;
	if (other.filename_)
		filename_ = strdup(other.filename_);
	return *this;
}

/** Destructor. */
RRDDefinition::~RRDDefinition()
{
	if (rrd_manager_)
		rrd_manager_->remove_rrd(this);

	if (name_)
		free(name_);
	if (filename_)
		free(filename_);
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
	rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5, 1, 720));
	rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5, 3, 1680));
	rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5, 30, 456));
	rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5, 180, 412));
	rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5, 720, 439));
	rv.push_back(RRDArchive(RRDArchive::AVERAGE, 0.5, 8640, 402));
	rv.push_back(RRDArchive(RRDArchive::MIN, 0.5, 1, 720));
	rv.push_back(RRDArchive(RRDArchive::MIN, 0.5, 3, 1680));
	rv.push_back(RRDArchive(RRDArchive::MIN, 0.5, 30, 456));
	rv.push_back(RRDArchive(RRDArchive::MIN, 0.5, 180, 412));
	rv.push_back(RRDArchive(RRDArchive::MIN, 0.5, 720, 439));
	rv.push_back(RRDArchive(RRDArchive::MIN, 0.5, 8640, 402));
	rv.push_back(RRDArchive(RRDArchive::MAX, 0.5, 1, 720));
	rv.push_back(RRDArchive(RRDArchive::MAX, 0.5, 3, 1680));
	rv.push_back(RRDArchive(RRDArchive::MAX, 0.5, 30, 456));
	rv.push_back(RRDArchive(RRDArchive::MAX, 0.5, 180, 412));
	rv.push_back(RRDArchive(RRDArchive::MAX, 0.5, 720, 439));
	rv.push_back(RRDArchive(RRDArchive::MAX, 0.5, 8640, 402));
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
	for (size_t i = 0; i < ds_.size(); ++i) {
		if (strcmp(ds_[i].get_name(), ds_name) == 0)
			return i;
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
	if (filename_) {
		throw Exception("Graph definition %s: filename has already been set!", name_);
	}
	filename_ = strdup(filename);
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
	if (rrd_manager_) {
		throw Exception("RRD definition %s: RRD manager has already been set", name_);
	}
	rrd_manager_ = rrd_manager;
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
RRDGraphDataDefinition::RRDGraphDataDefinition(const char *                      name,
                                               RRDArchive::ConsolidationFunction cf,
                                               const RRDDefinition *             rrd_def,
                                               const char *                      ds_name)
: name_(strdup(name)),
  rrd_def_(rrd_def),
  ds_name_(ds_name ? strdup(ds_name) : strdup(name)),
  rpn_expression_(NULL),
  cf_(cf),
  string_(NULL)
{
}

/** CDEF constructor.
 * @param name name of the graph data source
 * @param rpn_expression RPN expression
 */
RRDGraphDataDefinition::RRDGraphDataDefinition(const char *name, const char *rpn_expression)
: name_(strdup(name)),
  rrd_def_(0),
  ds_name_(NULL),
  rpn_expression_(strdup(rpn_expression)),
  cf_(RRDArchive::AVERAGE),
  string_(NULL)
{
}

/** Copy constructor.
 * @param other instance to clone
 */
RRDGraphDataDefinition::RRDGraphDataDefinition(const RRDGraphDataDefinition &other)
: name_(strdup(other.name_)),
  rrd_def_(other.rrd_def_),
  ds_name_(other.ds_name_ ? strdup(other.ds_name_) : NULL),
  rpn_expression_(other.rpn_expression_ ? strdup(other.rpn_expression_) : 0),
  cf_(other.cf_),
  string_(NULL)
{
}

/** Destructor. */
RRDGraphDataDefinition::~RRDGraphDataDefinition()
{
	if (name_)
		free(name_);
	if (ds_name_)
		free(ds_name_);
	if (rpn_expression_)
		free(rpn_expression_);
	if (string_)
		free(string_);
}

/** Assignment operator.
 * @param other instance to copy from
 * @return reference to this instance
 */
RRDGraphDataDefinition &
RRDGraphDataDefinition::operator=(const RRDGraphDataDefinition &other)
{
	if (string_)
		free(string_);
	if (ds_name_)
		free(ds_name_);
	if (name_)
		free(name_);
	if (rpn_expression_)
		free(rpn_expression_);

	string_         = NULL;
	rpn_expression_ = NULL;
	name_           = strdup(other.name_);
	rrd_def_        = other.rrd_def_;
	if (other.ds_name_)
		ds_name_ = strdup(other.ds_name_);
	if (other.rpn_expression_)
		rpn_expression_ = other.rpn_expression_;
	cf_ = other.cf_;

	return *this;
}

/** Create string representation.
 * @return string representation suitable for rrd_graph_v().
 */
const char *
RRDGraphDataDefinition::to_string() const
{
	if (!string_) {
		if (rpn_expression_) {
			if (asprintf(&string_, "CDEF:%s=%s", name_, rpn_expression_) == -1) {
				throw OutOfMemoryException("Failed to create RRA string");
			}
		} else {
			size_t ds_index = rrd_def_->find_ds_index(ds_name_);

			if (asprintf(&string_,
			             "DEF:%s=%s:%s:%s",
			             name_,
			             rrd_def_->get_filename(),
			             rrd_def_->get_ds(ds_index).get_name(),
			             RRDArchive::cf_to_string(cf_))
			    == -1) {
				throw OutOfMemoryException("Failed to create RRA string");
			}
		}
	}

	return string_;
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
RRDGraphGPrint::RRDGraphGPrint(const char *                      def_name,
                               RRDArchive::ConsolidationFunction cf,
                               const char *                      format)
: def_name_(strdup(def_name)), cf_(cf), format_(strdup(format)), string_(NULL)
{
}

/** Copy constructor.
 * @param other instance to copy
 */
RRDGraphGPrint::RRDGraphGPrint(const RRDGraphGPrint &other)
: def_name_(strdup(other.def_name_)), cf_(other.cf_), format_(strdup(other.format_)), string_(NULL)
{
}

/** Destructor. */
RRDGraphGPrint::~RRDGraphGPrint()
{
	if (def_name_)
		free(def_name_);
	if (format_)
		free(format_);
	if (string_)
		free(string_);
}

/** Assignment operator.
 * @param g matching graph element to assign
 * @return reference to this instance
 */
RRDGraphGPrint &
RRDGraphGPrint::operator=(const RRDGraphGPrint &g)
{
	if (def_name_)
		free(def_name_);
	if (format_)
		free(format_);
	if (string_)
		free(string_);

	string_   = NULL;
	def_name_ = strdup(g.def_name_);
	cf_       = g.cf_;
	format_   = strdup(g.format_);

	return *this;
}

const char *
RRDGraphGPrint::to_string() const
{
	if (!string_) {
		if (asprintf(&string_, "GPRINT:%s:%s:%s", def_name_, RRDArchive::cf_to_string(cf_), format_)
		    == -1) {
			throw OutOfMemoryException("Failed to create RRD graph GPRINT string");
		}
	}

	return string_;
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
RRDGraphLine::RRDGraphLine(const char *def_name,
                           float       width,
                           const char *color,
                           const char *legend,
                           bool        stacked)
: def_name_(strdup(def_name)),
  width_(width),
  color_(strdup(color)),
  legend_(strdup(legend)),
  stacked_(stacked),
  string_(NULL)
{
}

/** Copy ctor.
 * @param other instance to copy
 */
RRDGraphLine::RRDGraphLine(const RRDGraphLine &other)
: def_name_(strdup(other.def_name_)),
  width_(other.width_),
  color_(strdup(other.color_)),
  legend_(strdup(other.legend_)),
  stacked_(other.stacked_),
  string_(NULL)
{
}

/** Destructor. */
RRDGraphLine::~RRDGraphLine()
{
	if (def_name_)
		free(def_name_);
	if (color_)
		free(color_);
	if (legend_)
		free(legend_);
	if (string_)
		free(string_);
}

/** Assignment operator.
 * @param g matching graph element to assign
 * @return reference to this instance
 */
RRDGraphLine &
RRDGraphLine::operator=(const RRDGraphLine &g)
{
	if (def_name_)
		free(def_name_);
	if (color_)
		free(color_);
	if (legend_)
		free(legend_);
	if (string_)
		free(string_);

	string_   = NULL;
	def_name_ = strdup(g.def_name_);
	width_    = g.width_;
	color_    = strdup(g.color_);
	legend_   = strdup(g.legend_);
	stacked_  = g.stacked_;

	return *this;
}

const char *
RRDGraphLine::to_string() const
{
	if (!string_) {
		if (asprintf(&string_,
		             "LINE%f:%s#%s:%s%s",
		             width_,
		             def_name_,
		             color_,
		             legend_,
		             stacked_ ? ":STACK" : "")
		    == -1) {
			throw OutOfMemoryException("Failed to create RRD graph LINE string");
		}
	}

	return string_;
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
RRDGraphArea::RRDGraphArea(const char *def_name,
                           const char *color,
                           const char *legend,
                           bool        stacked)
: def_name_(strdup(def_name)),
  color_(strdup(color)),
  legend_(strdup(legend)),
  stacked_(stacked),
  string_(NULL)
{
}

/** Copy ctor.
 * @param other instance to copy
 */
RRDGraphArea::RRDGraphArea(const RRDGraphArea &other)
: def_name_(strdup(other.def_name_)),
  color_(strdup(other.color_)),
  legend_(strdup(other.legend_)),
  stacked_(other.stacked_),
  string_(NULL)
{
}

/** Destructor. */
RRDGraphArea::~RRDGraphArea()
{
	if (def_name_)
		free(def_name_);
	if (color_)
		free(color_);
	if (legend_)
		free(legend_);
	if (string_)
		free(string_);
}

/** Assignment operator.
 * @param g matching graph element to assign
 * @return reference to this instance
 */
RRDGraphArea &
RRDGraphArea::operator=(const RRDGraphArea &g)
{
	if (def_name_)
		free(def_name_);
	if (color_)
		free(color_);
	if (legend_)
		free(legend_);
	if (string_)
		free(string_);

	string_   = NULL;
	def_name_ = strdup(g.def_name_);
	color_    = strdup(g.color_);
	legend_   = strdup(g.legend_);
	stacked_  = g.stacked_;

	return *this;
}

const char *
RRDGraphArea::to_string() const
{
	if (!string_) {
		if (asprintf(&string_, "AREA:%s#%s:%s%s", def_name_, color_, legend_, stacked_ ? ":STACK" : "")
		    == -1) {
			throw OutOfMemoryException("Failed to create RRD graph AREA string");
		}
	}

	return string_;
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
RRDGraphDefinition::RRDGraphDefinition(const char *                         name,
                                       RRDDefinition *                      rrd_def,
                                       const char *                         title,
                                       const char *                         vertical_label,
                                       std::vector<RRDGraphDataDefinition> &def,
                                       std::vector<RRDGraphElement *> &     elements,
                                       time_t                               start,
                                       time_t                               end,
                                       unsigned int                         step,
                                       unsigned int                         update_interval,
                                       bool                                 slope_mode)
: name_(strdup(name)),
  rrd_def_(rrd_def),
  start_(start),
  end_(end),
  step_(step),
  title_(strdup(title)),
  vertical_label_(strdup(vertical_label)),
  update_interval_(update_interval),
  slope_mode_(slope_mode),
  defs_(def),
  elements_(elements)
{
	filename_ = NULL;
	argv_     = NULL;
	argc_     = 0;
	fonts_.push_back("LEGEND:10:");
	fonts_.push_back("UNIT:8:");
	fonts_.push_back("TITLE:12:");
	fonts_.push_back("AXIS:8:");
	width_   = 560;
	width_s_ = strdup(StringConversions::to_string(width_).c_str());
	start_s_ = strdup(StringConversions::to_string(start_).c_str());
	end_s_   = strdup(StringConversions::to_string(end_).c_str());
	step_s_  = strdup(StringConversions::to_string(step_).c_str());
}

/** Copy constructor.
 * @param other instance to copy
 */
RRDGraphDefinition::RRDGraphDefinition(const RRDGraphDefinition &other)
: name_(strdup(other.name_)),
  rrd_def_(other.rrd_def_),
  start_(other.start_),
  end_(other.end_),
  step_(other.step_),
  title_(strdup(other.title_)),
  vertical_label_(strdup(other.vertical_label_)),
  update_interval_(other.update_interval_),
  slope_mode_(other.slope_mode_),
  defs_(other.defs_),
  width_(other.width_),
  fonts_(other.fonts_),
  filename_(strdup(other.filename_))
{
	std::vector<RRDGraphElement *>::const_iterator i;
	for (i = other.elements_.begin(); i != other.elements_.end(); ++i) {
		elements_.push_back((*i)->clone());
	}

	argv_    = NULL;
	argc_    = 0;
	width_s_ = strdup(StringConversions::to_string(width_).c_str());
	start_s_ = strdup(StringConversions::to_string(start_).c_str());
	end_s_   = strdup(StringConversions::to_string(end_).c_str());
	step_s_  = strdup(StringConversions::to_string(step_).c_str());
}

/** Destructor. */
RRDGraphDefinition::~RRDGraphDefinition()
{
	if (filename_)
		free(filename_);
	if (argv_)
		free(argv_);
	if (name_)
		free(name_);
	if (title_)
		free(title_);
	if (vertical_label_)
		free(vertical_label_);

	free(width_s_);
	free(start_s_);
	free(end_s_);
	free(step_s_);

	std::vector<RRDGraphElement *>::iterator i;
	for (i = elements_.begin(); i != elements_.end(); ++i) {
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
	if (filename_) {
		throw Exception("Graph definition for RRD %s: filename has already been set!",
		                rrd_def_->get_name());
	}
	filename_ = strdup(filename);
}

/** Get argument array and size.
 * @param argc upon completion contains the number of arguments in the
 * return value.
 * @return argument array suitable for rrd_create_v().
 */
const char **
RRDGraphDefinition::get_argv(size_t &argc) const
{
	if (argv_ == NULL) {
		// "graph" filename --disable-rrdtool-tag --width ... --start ... --end ...
		// [fonts] --title ... --vertical-label ... [--slope-mode] DEFS... ELEMS...
		argc_      = 16 + fonts_.size() * 2 + defs_.size() + elements_.size();
		argv_      = (const char **)malloc(argc_ * sizeof(char *));
		size_t i   = 0;
		argv_[i++] = "graph";
		argv_[i++] = filename_;
		argv_[i++] = "--disable-rrdtool-tag";
		argv_[i++] = "--width";
		argv_[i++] = width_s_;
		argv_[i++] = "--start";
		argv_[i++] = start_s_;
		argv_[i++] = "--end";
		argv_[i++] = end_s_;
		argv_[i++] = "--step";
		argv_[i++] = step_s_;
		argv_[i++] = "--title";
		argv_[i++] = title_;
		argv_[i++] = "--vertical-label";

		if (strcmp(vertical_label_, "") == 0) {
			argv_[i++] = " ";
		} else {
			argv_[i++] = vertical_label_;
		}

		if (slope_mode_)
			argv_[i++] = "--slope-mode";

		std::vector<const char *>::const_iterator f;
		for (f = fonts_.begin(); f != fonts_.end(); ++f) {
			argv_[i++] = "--font";
			argv_[i++] = *f;
		}

		std::vector<RRDGraphDataDefinition>::const_iterator d;
		for (d = defs_.begin(); d != defs_.end(); ++d) {
			argv_[i++] = d->to_string();
		}

		std::vector<RRDGraphElement *>::const_iterator e;
		for (e = elements_.begin(); e != elements_.end(); ++e) {
			argv_[i++] = (*e)->to_string();
		}

		argc_ = i;
	}

	argc = argc_;
	return argv_;
}

} // end namespace fawkes
