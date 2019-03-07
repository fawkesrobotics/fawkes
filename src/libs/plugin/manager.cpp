
/***************************************************************************
 *  manager.cpp - Fawkes plugin manager
 *
 *  Created: Wed Nov 15 23:31:55 2006 (on train to Cologne)
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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
#include <core/exception.h>
#include <core/plugin.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/thread_collector.h>
#include <core/threading/thread_initializer.h>
#include <logging/liblogger.h>
#include <plugin/listener.h>
#include <plugin/loader.h>
#include <plugin/manager.h>
#include <sys/types.h>
#include <utils/misc/string_split.h>
#include <utils/system/dynamic_module/module_manager.h>
#include <utils/system/fam_thread.h>

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <dirent.h>

namespace fawkes {

/// @cond INTERNALS
class plname_eq
{
public:
	plname_eq(std::string name)
	{
		name_ = name;
	}
	bool
	operator()(Plugin *plugin)
	{
		return (name_ == plugin->name());
	}

private:
	std::string name_;
};
/// @endcond INTERNALS

/** @class PluginManager <plugin/manager.h>
 * Fawkes Plugin Manager.
 * This class provides a manager for the plugins used in fawkes. It can
 * load and unload modules.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param thread_collector thread manager plugin threads will be added to
 * and removed from appropriately.
 * @param config Fawkes configuration
 * @param meta_plugin_prefix Path prefix for meta plugins
 * @param module_flags flags to use to open plugin modules
 * @param init_cache true to initialize the plugin cache, false to skip this
 * step. Note that some functions like transmitting a list of available plugins
 * is unavailable until the cache has been initialized. You can defer
 * initialization of the cache if required.
 */
PluginManager::PluginManager(ThreadCollector *   thread_collector,
                             Configuration *     config,
                             const char *        meta_plugin_prefix,
                             Module::ModuleFlags module_flags,
                             bool                init_cache)
: ConfigurationChangeHandler(meta_plugin_prefix)
{
	mutex_                 = new Mutex();
	this->thread_collector = thread_collector;
	plugin_loader          = new PluginLoader(PLUGINDIR, config);
	plugin_loader->get_module_manager()->set_open_flags(module_flags);
	next_plugin_id      = 1;
	config_             = config;
	meta_plugin_prefix_ = meta_plugin_prefix;

	if (init_cache) {
		init_pinfo_cache();
	}

	config_->add_change_handler(this);

	fam_thread_ = new FamThread();
#ifdef HAVE_INOTIFY
	RefPtr<FileAlterationMonitor> fam = fam_thread_->get_fam();
	fam->add_filter("^[^.].*\\." SOEXT "$");
	fam->add_listener(this);
	fam->watch_dir(PLUGINDIR);
	fam_thread_->start();
#else
	LibLogger::log_warn("PluginManager",
	                    "File alteration monitoring not available, "
	                    "cannot detect changed plugins on disk.");
#endif
}

/** Destructor. */
PluginManager::~PluginManager()
{
#ifdef HAVE_INOTIFY
	fam_thread_->cancel();
	fam_thread_->join();
#endif
	delete fam_thread_;
	config_->rem_change_handler(this);
	pinfo_cache_.lock();
	pinfo_cache_.clear();
	pinfo_cache_.unlock();
	// Unload all plugins
	for (rpit = plugins.rbegin(); rpit != plugins.rend(); ++rpit) {
		try {
			thread_collector->force_remove((*rpit)->threads());
		} catch (Exception &e) {
			// We want it to be quiet on destruction, i.e. Fawkes quitting
			//LibLogger::log_warn("PluginManager", "Forced unloading of %s caused exception",
			//		  (*rpit)->name());
			//LibLogger::log_warn("PluginManager", e);
		}
		plugin_loader->unload(*rpit);
	}
	plugins.clear();
	plugin_ids.clear();
	delete plugin_loader;
	delete mutex_;
}

/** Set flags to open modules with.
 * @param flags flags to pass to modules when opening them
 */
void
PluginManager::set_module_flags(Module::ModuleFlags flags)
{
	plugin_loader->get_module_manager()->set_open_flags(flags);
}

/** Initialize plugin info cache. */
void
PluginManager::init_pinfo_cache()
{
	pinfo_cache_.lock();

	DIR *          plugin_dir;
	struct dirent *dirp;
	const char *   file_ext = "." SOEXT;

	if (NULL == (plugin_dir = opendir(PLUGINDIR))) {
		throw Exception(errno, "Plugin directory %s could not be opened", PLUGINDIR);
	}

	for (unsigned int i = 0; NULL != (dirp = readdir(plugin_dir)); ++i) {
		char *      file_name = dirp->d_name;
		char *      pos       = strstr(file_name, file_ext);
		std::string plugin_name =
		  std::string(file_name).substr(0, strlen(file_name) - strlen(file_ext));
		if (NULL != pos) {
			try {
				pinfo_cache_.push_back(
				  make_pair(plugin_name, plugin_loader->get_description(plugin_name.c_str())));
			} catch (Exception &e) {
				LibLogger::log_warn("PluginManager",
				                    "Could not get description of plugin %s, "
				                    "exception follows",
				                    plugin_name.c_str());
				LibLogger::log_warn("PluginManager", e);
			}
		}
	}

	closedir(plugin_dir);

	try {
		Configuration::ValueIterator *i = config_->search(meta_plugin_prefix_.c_str());
		while (i->next()) {
			if (i->is_string()) {
				std::string p = std::string(i->path()).substr(meta_plugin_prefix_.length());
				std::string s = std::string("Meta: ") + i->get_string();

				pinfo_cache_.push_back(make_pair(p, s));
			}
		}
		delete i;
	} catch (Exception &e) {
	}

	pinfo_cache_.sort();
	pinfo_cache_.unlock();
}

/** Generate list of all available plugins.
 * @return list of plugins that are available, each plugin is represented by
 * a pair of strings. The first string is the plugin name, the second is its
 * description.
 */
std::list<std::pair<std::string, std::string>>
PluginManager::get_available_plugins()
{
	std::list<std::pair<std::string, std::string>> rv;

	std::list<std::pair<std::string, std::string>>::iterator i;
	for (i = pinfo_cache_.begin(); i != pinfo_cache_.end(); ++i) {
		rv.push_back(*i);
	}

	return rv;
}

/** Get list of loaded plugins.
 * @return list of names of real and meta plugins currently loaded
 */
std::list<std::string>
PluginManager::get_loaded_plugins()
{
	std::list<std::string> rv;

	plugins.lock();
	for (pit = plugins.begin(); pit != plugins.end(); ++pit) {
		rv.push_back((*pit)->name());
	}
	plugins.unlock();
	meta_plugins_.lock();
	for (mpit_ = meta_plugins_.begin(); mpit_ != meta_plugins_.end(); ++mpit_) {
		rv.push_back(mpit_->first);
	}
	meta_plugins_.unlock();

	return rv;
}

/** Check if plugin is loaded.
 * @param plugin_name plugin to check if it is loaded
 * @return true if the plugin is currently loaded, false otherwise
 */
bool
PluginManager::is_loaded(const std::string &plugin_name)
{
	if (plugin_loader->is_loaded(plugin_name.c_str())) {
		return true;
	} else {
		// Could still be a meta plugin
		return (meta_plugins_.find(plugin_name) != meta_plugins_.end());
	}
}

/** Check if plugin is a meta plugin.
 * @param plugin_name plugin to check
 * @return true if the plugin is a meta plugin, false otherwise
 */
bool
PluginManager::is_meta_plugin(const std::string &plugin_name)
{
	try {
		std::string meta_plugin_path = meta_plugin_prefix_ + plugin_name;
		return (config_->is_string(meta_plugin_path.c_str()));
	} catch (ConfigEntryNotFoundException &e) {
		return false;
	}
}

/** Get meta plugin children.
 * @param plugin_name plugin to check
 * @return List of plugins which would be loaded for this plugin.
 */
std::list<std::string>
PluginManager::get_meta_plugin_children(const std::string &plugin_name)
{
	std::string meta_plugin_path = meta_plugin_prefix_ + plugin_name;
	std::string meta_plugin_str  = config_->get_string(meta_plugin_path.c_str());
	return parse_plugin_list(meta_plugin_str.c_str());
}

/** Parse a list of plugin types.
 * Takes a comma-separated list of plugins and parses them into the individual
 * plugin names.
 * @param plugin_type_list string containing a comma-separated list of plugin types
 * @return parsed list of plugin types
 */
std::list<std::string>
PluginManager::parse_plugin_list(const char *plugin_list)
{
	std::list<std::string> rv;

	char *plugins = strdup(plugin_list);
	char *saveptr;
	char *plugin;

	plugin = strtok_r(plugins, ",", &saveptr);
	while (plugin) {
		rv.push_back(plugin);
		plugin = strtok_r(NULL, ",", &saveptr);
	}
	free(plugins);

	return rv;
}

/** Load plugin.
 * The loading is interrupted if any of the plugins does not load properly.
 * The already loaded plugins are *not* unloaded, but kept.
 * @param plugin_list list of plugin names to load. The plugin list can contain meta plugins.
 */
void
PluginManager::load(const std::string &plugin_list)
{
	load(parse_plugin_list(plugin_list.c_str()));
}

/** Load plugin.
 * The loading is interrupted if any of the plugins does not load properly.
 * The already loaded plugins are *not* unloaded, but kept.
 * @param plugin_list string containing a comma-separated list of plugins
 * to load. The plugin list can contain meta plugins.
 */
void
PluginManager::load(const std::list<std::string> &plugin_list)
{
	for (std::list<std::string>::const_iterator i = plugin_list.begin(); i != plugin_list.end();
	     ++i) {
		if (i->length() == 0)
			continue;

		bool try_real_plugin = true;
		if (meta_plugins_.find(*i) == meta_plugins_.end()) {
			std::string            meta_plugin = meta_plugin_prefix_ + *i;
			bool                   found_meta  = false;
			std::list<std::string> pset;
			try {
				if (config_->is_list(meta_plugin.c_str())) {
					std::vector<std::string> tmp = config_->get_strings(meta_plugin.c_str());
					pset.insert(pset.end(), tmp.begin(), tmp.end());
				} else
					pset = parse_plugin_list(config_->get_string(meta_plugin.c_str()).c_str());
				found_meta = true;
			} catch (ConfigEntryNotFoundException &e) {
				// no meta plugin defined by that name
				//printf("No meta plugin defined with the name %s\n", i->c_str());
				try_real_plugin = true;
			}

			if (found_meta) {
				if (pset.size() == 0) {
					throw Exception("Refusing to load an empty meta plugin");
				}
				//printf("Going to load meta plugin %s (%s)\n", i->c_str(), pset.c_str());
				meta_plugins_.lock();
				// Setting has to happen here, so that a meta plugin will not cause an
				// endless loop if it references itself!
				meta_plugins_[*i] = pset;
				meta_plugins_.unlock();
				try {
					LibLogger::log_info("PluginManager",
					                    "Loading plugins %s for meta plugin %s",
					                    str_join(pset.begin(), pset.end(), ",").c_str(),
					                    i->c_str());
					load(pset);
					LibLogger::log_debug("PluginManager", "Loaded meta plugin %s", i->c_str());
					notify_loaded(i->c_str());
				} catch (Exception &e) {
					e.append("Could not initialize meta plugin %s, aborting loading.", i->c_str());
					meta_plugins_.erase_locked(*i);
					throw;
				}

				try_real_plugin = false;
			}
		}

		if (try_real_plugin
		    && (find_if(plugins.begin(), plugins.end(), plname_eq(*i)) == plugins.end())) {
			try {
				//printf("Going to load real plugin %s\n", i->c_str());
				Plugin *plugin = plugin_loader->load(i->c_str());
				plugins.lock();
				try {
					thread_collector->add(plugin->threads());
					plugins.push_back(plugin);
					plugin_ids[*i] = next_plugin_id++;
					LibLogger::log_debug("PluginManager", "Loaded plugin %s", i->c_str());
					notify_loaded(i->c_str());
				} catch (CannotInitializeThreadException &e) {
					e.append("Plugin >>> %s <<< could not be initialized, unloading", i->c_str());
					plugins.unlock();
					plugin_loader->unload(plugin);
					throw;
				}
				plugins.unlock();
			} catch (Exception &e) {
				MutexLocker lock(meta_plugins_.mutex());
				if (meta_plugins_.find(*i) == meta_plugins_.end()) {
					// only throw exception if no meta plugin with that name has
					// already been loaded
					throw;
				}
			}
		}
	}
}

/** Unload plugin.
 * Note that this method does not allow to pass a list of plugins, but it will
 * only accept a single plugin at a time.
 * @param plugin_name plugin to unload, can be a meta plugin.
 */
void
PluginManager::unload(const std::string &plugin_name)
{
	MutexLocker lock(plugins.mutex());
	if ((pit = find_if(plugins.begin(), plugins.end(), plname_eq(plugin_name))) != plugins.end()) {
		try {
			thread_collector->remove((*pit)->threads());
			plugin_loader->unload(*pit);
			plugins.erase(pit);
			plugin_ids.erase(plugin_name);
			notify_unloaded(plugin_name.c_str());
			// find all meta plugins that required this module, this can no longer
			// be considered loaded
			meta_plugins_.lock();
			mpit_ = meta_plugins_.begin();
			while (mpit_ != meta_plugins_.end()) {
				std::list<std::string> pp = mpit_->second;

				bool erase = false;
				for (std::list<std::string>::iterator i = pp.begin(); i != pp.end(); ++i) {
					if (*i == plugin_name) {
						erase = true;
						break;
					}
				}
				if (erase) {
					LockMap<std::string, std::list<std::string>>::iterator tmp = mpit_;
					++mpit_;
					notify_unloaded(tmp->first.c_str());
					meta_plugins_.erase(tmp);
				} else {
					++mpit_;
				}
			}
			meta_plugins_.unlock();

		} catch (Exception &e) {
			LibLogger::log_error("PluginManager",
			                     "Could not finalize one or more threads "
			                     "of plugin %s, NOT unloading plugin",
			                     plugin_name.c_str());
			throw;
		}
	} else if (meta_plugins_.find(plugin_name) != meta_plugins_.end()) {
		std::list<std::string> pp = meta_plugins_[plugin_name];

		for (std::list<std::string>::reverse_iterator i = pp.rbegin(); i != pp.rend(); ++i) {
			if (i->length() == 0)
				continue;
			if ((find_if(plugins.begin(), plugins.end(), plname_eq(*i)) == plugins.end())
			    && (meta_plugins_.find(*i) != meta_plugins_.end())) {
				continue;
			}

			meta_plugins_.erase_locked(*i);
			LibLogger::log_info("PluginManager",
			                    "UNloading plugin %s for meta plugin %s",
			                    i->c_str(),
			                    plugin_name.c_str());
			unload(i->c_str());
		}
	}
}

void
PluginManager::config_tag_changed(const char *new_tag)
{
}

void
PluginManager::config_value_changed(const Configuration::ValueIterator *v)
{
	if (v->is_string()) {
		pinfo_cache_.lock();
		std::string p = std::string(v->path()).substr(meta_plugin_prefix_.length());
		std::string s = std::string("Meta: ") + v->get_string();
		std::list<std::pair<std::string, std::string>>::iterator i;
		bool                                                     found = false;
		for (i = pinfo_cache_.begin(); i != pinfo_cache_.end(); ++i) {
			if (p == i->first) {
				i->second = s;
				found     = true;
				break;
			}
		}
		if (!found) {
			pinfo_cache_.push_back(make_pair(p, s));
		}
		pinfo_cache_.unlock();
	}
}

void
PluginManager::config_comment_changed(const Configuration::ValueIterator *v)
{
}

void
PluginManager::config_value_erased(const char *path)
{
	pinfo_cache_.lock();
	std::string p = std::string(path).substr(meta_plugin_prefix_.length());
	std::list<std::pair<std::string, std::string>>::iterator i;
	for (i = pinfo_cache_.begin(); i != pinfo_cache_.end(); ++i) {
		if (p == i->first) {
			pinfo_cache_.erase(i);
			break;
		}
	}
	pinfo_cache_.unlock();
}

void
PluginManager::fam_event(const char *filename, unsigned int mask)
{
	const char *file_ext = "." SOEXT;

	const char *pos = strstr(filename, file_ext);
	std::string p   = std::string(filename).substr(0, strlen(filename) - strlen(file_ext));
	if (NULL != pos) {
		pinfo_cache_.lock();
		bool                                                     found = false;
		std::list<std::pair<std::string, std::string>>::iterator i;
		for (i = pinfo_cache_.begin(); i != pinfo_cache_.end(); ++i) {
			if (p == i->first) {
				found = true;
				if ((mask & FAM_DELETE) || (mask & FAM_MOVED_FROM)) {
					pinfo_cache_.erase(i);
				} else {
					try {
						i->second = plugin_loader->get_description(p.c_str());
					} catch (Exception &e) {
						LibLogger::log_warn("PluginManager",
						                    "Could not get possibly modified "
						                    "description of plugin %s, exception follows",
						                    p.c_str());
						LibLogger::log_warn("PluginManager", e);
					}
				}
				break;
			}
		}
		if (!found && !(mask & FAM_ISDIR)
		    && ((mask & FAM_MODIFY) || (mask & FAM_MOVED_TO) || (mask & FAM_CREATE))) {
#ifndef HAVE_LIBELF
			if (plugin_loader->is_loaded(p.c_str())) {
				LibLogger::log_info("PluginManager",
				                    "Plugin %s changed on disk, but is "
				                    "loaded, no new info can be loaded, keeping old.",
				                    p.c_str());
			}
#endif
			try {
				std::string s = plugin_loader->get_description(p.c_str());
				LibLogger::log_info("PluginManager", "Reloaded meta-data of %s on file change", p.c_str());
				pinfo_cache_.push_back(make_pair(p, s));
			} catch (Exception &e) {
				// ignore, all it means is that the file has not been finished writing
				/*
	LibLogger::log_warn("PluginManager", "Could not get possibly modified "
			    "description of plugin %s, exception follows",
			    p.c_str());
	LibLogger::log_warn("PluginManager", e);
	*/
			}
		}

		pinfo_cache_.sort();
		pinfo_cache_.unlock();
	}
}

/** Add listener.
 * Listeners are notified of plugin load and unloda events.
 * @param listener listener to add
 */
void
PluginManager::add_listener(PluginManagerListener *listener)
{
	listeners_.lock();
	listeners_.push_back(listener);
	listeners_.sort();
	listeners_.unique();
	listeners_.unlock();
}

/** Remove listener.
 * @param listener listener to remove
 */
void
PluginManager::remove_listener(PluginManagerListener *listener)
{
	listeners_.remove_locked(listener);
}

void
PluginManager::notify_loaded(const char *plugin_name)
{
	listeners_.lock();
	for (lit_ = listeners_.begin(); lit_ != listeners_.end(); ++lit_) {
		try {
			(*lit_)->plugin_loaded(plugin_name);
		} catch (Exception &e) {
			LibLogger::log_warn("PluginManager",
			                    "PluginManagerListener threw exception "
			                    "during notification of plugin loaded, exception follows.");
			LibLogger::log_warn("PluginManager", e);
		}
	}
	listeners_.unlock();
}

void
PluginManager::notify_unloaded(const char *plugin_name)
{
	listeners_.lock();
	for (lit_ = listeners_.begin(); lit_ != listeners_.end(); ++lit_) {
		try {
			(*lit_)->plugin_unloaded(plugin_name);
		} catch (Exception &e) {
			LibLogger::log_warn("PluginManager",
			                    "PluginManagerListener threw exception "
			                    "during notification of plugin unloaded, exception follows.");
			LibLogger::log_warn("PluginManager", e);
		}
	}
	listeners_.unlock();
}

/** Lock plugin manager.
 * This is an utility method that you can use for mutual access to the plugin
 * manager. The mutex is not used internally, but meant to be used from
 * callers.
 */
void
PluginManager::lock()
{
	mutex_->lock();
}

/** Try to lock plugin manager.
 * This is an utility method that you can use for mutual access to the plugin
 * manager. The mutex is not used internally, but meant to be used from
 * callers.
 * @return true if the lock was acquired, false otherwise
 */
bool
PluginManager::try_lock()
{
	return mutex_->try_lock();
}

/** Unlock plugin manager. */
void
PluginManager::unlock()
{
	mutex_->unlock();
}

} // end namespace fawkes
