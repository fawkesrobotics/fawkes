
/***************************************************************************
 *  MetricFamilyInterface.cpp - Fawkes BlackBoard Interface - MetricFamilyInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2017  Tim Niemueller
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

#include <interfaces/MetricFamilyInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class MetricFamilyInterface <interfaces/MetricFamilyInterface.h>
 * MetricFamilyInterface Fawkes BlackBoard Interface.
 * 
		  Each MetricFamily within the same exposition must have a unique
		  name. Each Metric within the same MetricFamily must have a
		  unique set of LabelPair fields. Otherwise, the ingestion
		  behavior is undefined.

		  To determine all metrics, all interfaces of the appropriate type
		  with the same prefix as a MetricFamilyInterface ID are opened
		  and exported. Metrics of non-matching type are silently ignored.
	  
 * @ingroup FawkesInterfaces
 */



/** Constructor */
MetricFamilyInterface::MetricFamilyInterface() : Interface()
{
  data_size = sizeof(MetricFamilyInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (MetricFamilyInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_MetricType[(int)NOT_INITIALIZED] = "NOT_INITIALIZED";
  enum_map_MetricType[(int)COUNTER] = "COUNTER";
  enum_map_MetricType[(int)GAUGE] = "GAUGE";
  enum_map_MetricType[(int)UNTYPED] = "UNTYPED";
  enum_map_MetricType[(int)HISTOGRAM] = "HISTOGRAM";
  add_fieldinfo(IFT_STRING, "name", 128, data->name);
  add_fieldinfo(IFT_STRING, "help", 256, data->help);
  add_fieldinfo(IFT_ENUM, "metric_type", 1, &data->metric_type, "MetricType", &enum_map_MetricType);
  unsigned char tmp_hash[] = {0x4a, 0xdc, 0x96, 0x7f, 0x52, 00, 0x90, 0xe1, 0x48, 0x93, 0xa0, 0xae, 0xb4, 0xce, 0xcf, 0xec};
  set_hash(tmp_hash);
}

/** Destructor */
MetricFamilyInterface::~MetricFamilyInterface()
{
  free(data_ptr);
}
/** Convert MetricType constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
MetricFamilyInterface::tostring_MetricType(MetricType value) const
{
  switch (value) {
  case NOT_INITIALIZED: return "NOT_INITIALIZED";
  case COUNTER: return "COUNTER";
  case GAUGE: return "GAUGE";
  case UNTYPED: return "UNTYPED";
  case HISTOGRAM: return "HISTOGRAM";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get name value.
 * 
	    The metric family name.
    
 * @return name value
 */
char *
MetricFamilyInterface::name() const
{
  return data->name;
}

/** Get maximum length of name value.
 * @return length of name value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MetricFamilyInterface::maxlenof_name() const
{
  return 128;
}

/** Set name value.
 * 
	    The metric family name.
    
 * @param new_name new name value
 */
void
MetricFamilyInterface::set_name(const char * new_name)
{
  strncpy(data->name, new_name, sizeof(data->name));
  data_changed = true;
}

/** Get help value.
 * 
	    The metric family name.
    
 * @return help value
 */
char *
MetricFamilyInterface::help() const
{
  return data->help;
}

/** Get maximum length of help value.
 * @return length of help value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MetricFamilyInterface::maxlenof_help() const
{
  return 256;
}

/** Set help value.
 * 
	    The metric family name.
    
 * @param new_help new help value
 */
void
MetricFamilyInterface::set_help(const char * new_help)
{
  strncpy(data->help, new_help, sizeof(data->help));
  data_changed = true;
}

/** Get metric_type value.
 * 
	    The type of metrics to look for.
    
 * @return metric_type value
 */
MetricFamilyInterface::MetricType
MetricFamilyInterface::metric_type() const
{
  return (MetricFamilyInterface::MetricType)data->metric_type;
}

/** Get maximum length of metric_type value.
 * @return length of metric_type value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MetricFamilyInterface::maxlenof_metric_type() const
{
  return 1;
}

/** Set metric_type value.
 * 
	    The type of metrics to look for.
    
 * @param new_metric_type new metric_type value
 */
void
MetricFamilyInterface::set_metric_type(const MetricType new_metric_type)
{
  data->metric_type = new_metric_type;
  data_changed = true;
}

/* =========== message create =========== */
Message *
MetricFamilyInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
MetricFamilyInterface::copy_values(const Interface *other)
{
  const MetricFamilyInterface *oi = dynamic_cast<const MetricFamilyInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(MetricFamilyInterface_data_t));
}

const char *
MetricFamilyInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "MetricType") == 0) {
    return tostring_MetricType((MetricType)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
MetricFamilyInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(MetricFamilyInterface)
/// @endcond


} // end namespace fawkes
