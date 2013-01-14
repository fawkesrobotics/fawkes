
/***************************************************************************
 *  config.cpp - Fawkes configuration interface
 *
 *  Created: Mon Dec 18 14:54:23 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <config/config.h>
#include <config/change_handler.h>
#include <cstring>

namespace fawkes {

/** @class Configuration <config/config.h>
 * Interface for configuration handling.
 * We know that half of robotics is about parameter tuning. The Configuration
 * interface defines a unified way of storing parameters and other
 * configuration options no matter of how the database is implemented.
 * This is mainly done to allow for testing different solutions for ticket #10.
 *
 * @fn Configuration::~Configuration()
 * Virtual empty destructor.
 *
 * @fn void Configuration::load(const char *file_path)
 * Load configuration.
 * Loads configuration data, or opens a file, depending on the implementation. After
 * this call access to all other methods shall be possible.
 * @param file_path path of the configuration file.
 * 
 *
 * @fn void Configuration::copy(Configuration *copyconf)
 * Copy all values from the given configuration.
 * All values from the given configuration are copied. Old values are not erased
 * so that the copied values will overwrite existing values, new values are
 * created, but values existent in current config but not in the copie config
 * will remain unchanged.
 * @param copyconf configuration to copy
 * 
 * @fn bool Configuration::exists(const char *path)
 * Check if a given value exists.
 * @param path path to value
 * @return true if the value exists, false otherwise
 * 
 * @fn bool Configuration::is_float(const char *path)
 * Check if a value is of type float
 * @param path path to value
 * @return true if the value exists and is of type float
 * 
 * @fn bool Configuration::is_uint(const char *path)
 * Check if a value is of type unsigned int
 * @param path path to value
 * @return true if the value exists and is of type unsigned int
 * 
 * @fn bool Configuration::is_int(const char *path)
 * Check if a value is of type int
 * @param path path to value
 * @return true if the value exists and is of type int
 * 
 * @fn bool Configuration::is_bool(const char *path)
 * Check if a value is of type bool
 * @param path path to value
 * @return true if the value exists and is of type bool
 * 
 * @fn bool Configuration::is_string(const char *path)
 * Check if a value is of type string
 * @param path path to value
 * @return true if the value exists and is of type string
 *
 * @fn bool Configuration::is_list(const char *path)
 * Check if a value is a list.
 * @param path path to value
 * @return true if the value exists and is a list
 * 
 * @fn bool Configuration::is_default(const char *path)
 * Check if a value was read from the default config.
 * @param path path to value
 * @return true if the value exists and is only stored in the default config
 * 
 * @fn float Configuration::get_float(const char *path)
 * Get value from configuration which is of type float
 * @param path path to value
 * @return value
 * 
 * @fn unsigned int Configuration::get_uint(const char *path)
 * Get value from configuration which is of type unsigned int
 * @param path path to value
 * @return value
 * 
 * @fn int Configuration::get_int(const char *path)
 * Get value from configuration which is of type int
 * @param path path to value
 * @return value
 * 
 * @fn bool Configuration::get_bool(const char *path)
 * Get value from configuration which is of type bool
 * @param path path to value
 * @return value
 * 
 * @fn std::string Configuration::get_string(const char *path)
 * Get value from configuration which is of type string
 * @param path path to value
 * @return value
 *
 * @fn std::vector<float> Configuration::get_floats(const char *path)
 * Get list of values from configuration which is of type float
 * @param path path to value
 * @return value
 * 
 * @fn std::vector<unsigned int> Configuration::get_uints(const char *path)
 * Get list of values from configuration which is of type unsigned int
 * @param path path to value
 * @return value
 * 
 * @fn std::vector<int> Configuration::get_ints(const char *path)
 * Get list of values from configuration which is of type int
 * @param path path to value
 * @return value
 * 
 * @fn std::vector<bool> Configuration::get_bools(const char *path)
 * Get list of values from configuration which is of type bool
 * @param path path to value
 * @return value
 * 
 * @fn std::vector<std::string> Configuration::get_strings(const char *path)
 * Get list of values from configuration which is of type string
 * @param path path to value
 * @return value
 *
 * @fn Configuration::ValueIterator * Configuration::get_value(const char *path)
 * Get value from configuration.
 * @param path path to value
 * @return value iterator for just this one value, maybe invalid if value does not
 * exists.
 *
 * @fn std::string Configuration::get_type(const char *path)
 * Get type of value at given path.
 * @param path path to value
 * @return string representation of type, one of float, unsigned int, int, bool,
 * or string
 * @exception ConfigurationException shall be thrown if value does not exist or
 * on any other error.
 *
 * @fn std::string Configuration::get_comment(const char *path)
 * Get comment of value at given path.
 * The value at the given path must exist in the host-specific configuration.
 * @param path path to value
 * @return comment
 * @exception ConfigEntryNotFoundException shall be thrown if value does not exist
 * @exception ConfigurationException shall be thrown on any other error
 *
 * @fn std::string Configuration::get_default_comment(const char *path)
 * Get comment of value at given path.
 * The value at the given path must exist in the default configuration.
 * @param path path to value
 * @return comment
 * @exception ConfigEntryNotFoundException shall be thrown if value does not exist
 * @exception ConfigurationException shall be thrown on any other error
 *
 * 
 * @fn void Configuration::set_float(const char *path, float f)
 * Set new value in configuration of type float
 * @param path path to value
 * @param f new float value
 * 
 * @fn void Configuration::set_uint(const char *path, unsigned int uint)
 * Set new value in configuration of type unsigned int
 * @param path path to value
 * @param uint new unsigned int value
 * 
 * @fn void Configuration::set_int(const char *path, int i)
 * Set new value in configuration of type int
 * @param path path to value
 * @param i new int value
 * 
 * @fn void Configuration::set_bool(const char *path, bool b)
 * Set new value in configuration of type bool
 * @param path path to value
 * @param b new bool value
 * 
 * @fn void Configuration::set_string(const char *path, std::string &s)
 * Set new value in configuration of type string
 * @param path path to value
 * @param s new string value
 *
 * @fn void Configuration::set_string(const char *path, const char *s)
 * Set new value in configuration of type string. Works like the aforementioned method.
 * Just takes an good ol' char array instead of a std::string.
 * @param path path to value
 * @param s new string value
 *
 * @fn void Configuration::set_floats(const char *path, std::vector<float> &f)
 * Set new value in configuration of type float
 * @param path path to value
 * @param f new float values
 * 
 * @fn void Configuration::set_uints(const char *path, std::vector<unsigned int> &uint)
 * Set new value in configuration of type unsigned int
 * @param path path to value
 * @param uint new unsigned int values
 * 
 * @fn void Configuration::set_ints(const char *path, std::vector<int> &i)
 * Set new value in configuration of type int
 * @param path path to value
 * @param i new int values
 * 
 * @fn void Configuration::set_bools(const char *path, std::vector<bool> &b)
 * Set new value in configuration of type bool
 * @param path path to value
 * @param b new bool values
 * 
 * @fn void Configuration::set_strings(const char *path, std::vector<std::string> &s)
 * Set new value in configuration of type string
 * @param path path to value
 * @param s new string values
 *
 * @fn void Configuration::set_strings(const char *path, std::vector<const char *> &s)
 * Set new value in configuration of type string. Works like the aforementioned method.
 * Just takes an good ol' char array instead of a std::string.
 * @param path path to value
 * @param s new string values

 *
 * @fn void Configuration::set_comment(const char *path, std::string &comment)
 * Set new comment for existing value.
 * @param path path to value
 * @param comment new comment string
 *
 * @fn void Configuration::set_comment(const char *path, const char *comment)
 * Set new comment for existing value. Works like the aforementioned method.
 * Just takes an good ol' char array instead of a std::string.
 * @param path path to value
 * @param comment new comment string
 *
 * @fn void Configuration::erase(const char *path)
 * Erase the given value from the configuration. It is not an error if the value does
 * not exists before deletion.
 * @param path path to value
 *
 * @fn void Configuration::set_default_float(const char *path, float f)
 * Set new default value in configuration of type float
 * @param path path to value
 * @param f new float value
 * 
 * @fn void Configuration::set_default_uint(const char *path, unsigned int uint)
 * Set new default value in configuration of type unsigned int
 * @param path path to value
 * @param uint new unsigned int value
 * 
 * @fn void Configuration::set_default_int(const char *path, int i)
 * Set new default value in configuration of type int
 * @param path path to value
 * @param i new int value
 * 
 * @fn void Configuration::set_default_bool(const char *path, bool b)
 * Set new default value in configuration of type bool
 * @param path path to value
 * @param b new bool value
 * 
 * @fn void Configuration::set_default_string(const char *path, std::string &s)
 * Set new default value in configuration of type string
 * @param path path to value
 * @param s new string value
 *
 * @fn void Configuration::set_default_string(const char *path, const char *s)
 * Set new default value in configuration of type string. Works like the aforementioned method.
 * Just takes an good ol' char array instead of a std::string.
 * @param path path to value
 * @param s new string value
 *
 * @fn void Configuration::set_default_comment(const char *path, std::string &comment)
 * Set new default comment for existing default configuration value.
 * @param path path to value
 * @param comment new comment string
 *
 * @fn void Configuration::set_default_comment(const char *path, const char *comment)
 * Set new default comment for existing default configuration value.
 * Works like the aforementioned method. Just takes an good ol' char array
 * instead of a std::string.
 * @param path path to value
 * @param comment new comment string
 *
 * @fn void Configuration::erase_default(const char *path)
 * Erase the given default value from the configuration. It is not an error if the value does
 * not exists before deletion.
 * @param path path to value
 *
 * @fn Configuration::ValueIterator * Configuration::iterator()
 * Iterator for all values.
 * Returns an iterator that can be used to iterate over all values in the current
 * configuration, it will value the overlay. If a default and a host-specific value
 * exists you will only see the host-specific value.
 * @return iterator over all values
 *
 * @fn Configuration::ValueIterator * Configuration::search(const char *path)
 * Iterator with search results.
 * Returns an iterator that can be used to iterate over the search results. All values
 * whose path start with the given strings are returned.
 * A call like
 * @code
 *   config->search("");
 * @endcode
 * is effectively the same as a call to iterator().
 * @param path start of path
 * @return iterator to search results
 *
 * @fn void Configuration::lock()
 * Lock the config.
 * No further changes or queries can be executed on the configuration and will block until
 * the config is unlocked.
 *
 * @fn bool Configuration::try_lock()
 * Try to lock the config.
 * @see Configuration::lock()
 * @return true, if the lock has been aquired, false otherwise
 *
 * @fn void Configuration::unlock()
 * Unlock the config.
 * Modifications and queries are possible again.
 *
 * @fn void Configuration::try_dump()
 * Try to dump configuration.
 * For configuration methods that transform configuration files in a binary
 * format this can be used to write out the text representation on shutdown
 * of Fawkes.
 * @exception Exception thrown if dumping fails
 *
 */

/** @class ConfigurationException config/config.h
 * Generic configuration exception.
 * Thrown if there is no other matching exception.
 */


/** Constructor.
 * @param msg message
 */
ConfigurationException::ConfigurationException(const char *msg)
  : Exception(msg)
{
}


/** Constructor.
 * @param prefix Put as "prefix: " before the message, can be used to have a prefix
 * and put an error message from another API into msg.
 * @param msg message
 */
ConfigurationException::ConfigurationException(const char *prefix, const char *msg)
  : Exception()
{
  append("%s: %s", prefix, msg);
}


/** @class ConfigEntryNotFoundException config/config.h
 * Thrown if a config entry could not be found.
 */


/** Constructor.
 * @param path path of value
 */
ConfigEntryNotFoundException::ConfigEntryNotFoundException( const char *path)
  : Exception("Config value for '%s' not found", path)
{
}


/** @class ConfigTypeMismatchException config/config.h
 * Thrown if there a type problem was detected for example if you tried
 * to query a float with get_int().
 */

/** Constructor.
 * @param path path of value
 * @param actual actual type
 * @param requested requested type
 */
ConfigTypeMismatchException::ConfigTypeMismatchException(const char *path,
							 const char *actual,
							 const char *requested)
  : Exception()
{
  append("Config value for '%s' is not of type '%s', but of type '%s'",
	 path, requested, actual);
}

/** @class CouldNotOpenConfigException <config/config.h>
 * Thrown if config could not be opened.
 * This is most likely to happen during the constructor or load().
 */

/** Constructor.
 * @param format format of message to describe cause or symptom of failure
 */
CouldNotOpenConfigException::CouldNotOpenConfigException(const char *format, ...)
  : Exception()
{
  va_list va;
  va_start(va, format);
  append_va(format, va);
  va_end(va);
}


/** @class Configuration::ValueIterator <config/config.h>
 * Iterator interface to iterate over config values. This does not implement a
 * classic iterator interface with begin and end nodes but rather mimics a more
 * Java-like interface where you iterate over the entries in a while loop until
 * you covered all entries (much like a queue).
 * If you implement this for your own configuration system you should not make
 * the constructor publically accessible.
 *
 * @fn Configuration::ValueIterator::~ValueIterator()
 * Virtual emptry destructor.
 *
 * @fn bool Configuration::ValueIterator::next()
 * Check if there is another element and advance to this if possible.
 * This advances to the next element, if there is one.
 * @return true, if another element has been reached, false otherwise
 *
 * @fn bool Configuration::ValueIterator::valid() const
 * Check if the current element is valid.
 * This is much like the classic end element for iterators. If the iterator is
 * invalid there all subsequent calls to next() shall fail.
 * @return true, if the iterator is still valid, false otherwise
 *
 * @fn const char * Configuration::ValueIterator::path() const
 * Path of value.
 * @return path of value
 *
 * @fn const char * Configuration::ValueIterator::type() const
 * Type of value.
 * @return string representation of value type.
 *
 * @fn bool Configuration::ValueIterator::is_float() const
 * Check if current value is a float.
 * @return true, if value is a float, false otherwise
 *
 * @fn bool Configuration::ValueIterator::is_uint() const
 * Check if current value is a unsigned int.
 * @return true, if value is a unsigned int, false otherwise
 *
 * @fn bool Configuration::ValueIterator::is_int() const
 * Check if current value is a int.
 * @return true, if value is a int, false otherwise
 *
 * @fn bool Configuration::ValueIterator::is_bool() const
 * Check if current value is a bool.
 * @return true, if value is a bool, false otherwise
 *
 * @fn bool Configuration::ValueIterator::is_string() const
 * Check if current value is a string.
 * @return true, if value is a string, false otherwise
 *
 * @fn bool Configuration::ValueIterator::is_list() const
 * Check if a value is a list.
 * @return true if the value exists and is a list
 *
 * @fn size_t Configuration::ValueIterator::get_list_size() const
 * Get number of elements in list value.
 * @return number of elements in list value
 * @throw Exception thrown if the element is not a list.
 *
 * @fn bool Configuration::ValueIterator::is_default() const
 * Check if current value was read from the default config.
 * @return true, if value was read from the default config, false otherwise
 *
 * @fn float Configuration::ValueIterator::get_float() const
 * Get float value.
 * @return value
 *
 * @fn unsigned int Configuration::ValueIterator::get_uint() const
 * Get unsigned int value.
 * @return value
 *
 * @fn int Configuration::ValueIterator::get_int() const
 * Get int value.
 * @return value
 *
 * @fn bool Configuration::ValueIterator::get_bool() const
 * Get bool value.
 * @return value
 *
 * @fn std::string Configuration::ValueIterator::get_string() const
 * Get string value.
 * @return value
 *
 * @fn std::vector<float> Configuration::ValueIterator::get_floats() const
 * Get list of values from configuration which is of type float
 * @return value
 * 
 * @fn std::vector<unsigned int> Configuration::ValueIterator::get_uints() const
 * Get list of values from configuration which is of type unsigned int
 * @return value
 * 
 * @fn std::vector<int> Configuration::ValueIterator::get_ints() const
 * Get list of values from configuration which is of type int
 * @return value
 * 
 * @fn std::vector<bool> Configuration::ValueIterator::get_bools() const
 * Get list of values from configuration which is of type bool
 * @return value
 * 
 * @fn std::vector<std::string> Configuration::ValueIterator::get_strings() const
 * Get list of values from configuration which is of type string
 * @return value
 *
 * @fn std::string Configuration::ValueIterator::get_comment() const
 * Get comment of value.
 * @return comment
 *
 * @fn std::string Configuration::ValueIterator::get_as_string() const
 * Get value as string.
 * @return value as string
 *
 */



/** Add a configuration change handler.
 * The added handler is called whenever a value changes and the handler
 * desires to get notified for the given component.
 * @param h configuration change handler
 */
void
Configuration::add_change_handler(ConfigurationChangeHandler *h)
{
  const char *c = h->config_monitor_prefix();
  if ( c == NULL ) {
    c = "";
  }

  _change_handlers.insert(ChangeHandlerMultimap::value_type(c, h));
}


/** Remove a configuration change handler.
 * The handler is removed from the change handler list and no longer called on
 * config changes.
 * @param h configuration change handler
 */
void
Configuration::rem_change_handler(ConfigurationChangeHandler *h)
{
  const char *c = h->config_monitor_prefix();
  if ( c == NULL ) {
    c = "";
  }
  bool changed = true;
  while (changed) {
    changed = false;
    for (ChangeHandlerMultimap::const_iterator j = _change_handlers.begin(); !changed && (j != _change_handlers.end()); ++j) {
      _ch_range = _change_handlers.equal_range((*j).first);
      for (ChangeHandlerMultimap::iterator i = _ch_range.first; !changed && (i != _ch_range.second); ++i) {
	if ( (*i).second == h ) {
	  _change_handlers.erase(i);
	  changed = true;
	  break;
	}
      }
      if ( changed)  break;
    }
  }
}


/** Find handlers for given path.
 * @param path path to get handlers for
 * @return list with config change handlers.
 */
Configuration::ChangeHandlerList *
Configuration::find_handlers(const char *path)
{
  ChangeHandlerList *rv = new ChangeHandlerList();
  for (ChangeHandlerMultimap::const_iterator j = _change_handlers.begin(); j != _change_handlers.end(); ++j) {
    if ( strstr(path, (*j).first) == path ) {
      _ch_range = _change_handlers.equal_range((*j).first);
      for (ChangeHandlerMultimap::const_iterator i = _ch_range.first; i != _ch_range.second; ++i) {
	rv->push_back((*i).second);
      }
    }
  }

  return rv;
}


/** Notify handlers for given path.
 * @param path path to notify handlers for
 * @param comment_changed true if the change is about a comment change,
 * false otherwise
 */
void
Configuration::notify_handlers(const char *path, bool comment_changed)
{
  ChangeHandlerList *h = find_handlers(path);
  Configuration::ValueIterator *value = get_value(path);
  if (value->next()) {
    for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
      if (comment_changed) {
	(*i)->config_comment_changed(value);
      } else {
	(*i)->config_value_changed(value);
      }
    }
  } else {
    for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
      (*i)->config_value_erased(path);
    }
  }
  delete value;
  delete h;
}

} // end namespace fawkes
