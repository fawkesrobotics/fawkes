
/***************************************************************************
 *  clips_env_manager.cpp - CLIPS environment manager
 *
 *  Created: Thu Aug 15 18:57:58 2013
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include <plugins/clips/aspect/clips_env_manager.h>
#include <plugins/clips/aspect/clips_feature.h>
#include <logging/logger.h>
#include <utils/time/time.h>

#include <cstring>

extern "C" {
#include <clips/clips.h>
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


#define ROUTER_NAME "fawkeslog"

/// @cond INTERNALS
class CLIPSLogger
{
 public:
  CLIPSLogger(Logger *logger, const char *component = NULL)
  {
    logger_ = logger;
    if (component) {
      component_ = strdup(component);
    } else {
      component_ = NULL;
    }
  }

  ~CLIPSLogger()
  {
    if (component_) {
      free(component_);
    }
  }

  void log(const char *logical_name, const char *str)
  {
    if (strcmp(str, "\n") == 0) {
      if (strcmp(logical_name, "debug") == 0 || strcmp(logical_name, "logdebug") == 0 ||
	  strcmp(logical_name, WTRACE) == 0)
      {
	logger_->log_debug(component_ ? component_ : "CLIPS", "%s", buffer_.c_str());
      } else if (strcmp(logical_name, "warn") == 0 || strcmp(logical_name, "logwarn") == 0 ||
		 strcmp(logical_name, WWARNING) == 0)
      {
	logger_->log_warn(component_ ? component_ : "CLIPS", "%s", buffer_.c_str());
      } else if (strcmp(logical_name, "error") == 0 || strcmp(logical_name, "logerror") == 0 ||
		 strcmp(logical_name, WERROR) == 0)
      {
	logger_->log_error(component_ ? component_ : "CLIPS", "%s", buffer_.c_str());
      } else if (strcmp(logical_name, WDIALOG) == 0) {
        // ignored
      } else {
	logger_->log_info(component_ ? component_ : "CLIPS", "%s", buffer_.c_str());
      }

      buffer_.clear();
    } else {
      buffer_ += str;
    }
  }

 private:
  Logger *logger_;
  char *component_;
  std::string buffer_;
};

class CLIPSContextMaintainer {
 public:
  CLIPSContextMaintainer(Logger *logger, const char *log_component_name)
  {
    this->logger = new CLIPSLogger(logger, log_component_name);
  }

  ~CLIPSContextMaintainer()
  {
    delete logger;
  }

 public:
  CLIPSLogger *logger;
};


static int
log_router_query(void *env, char *logical_name)
{
  if (strcmp(logical_name, "l") == 0) return TRUE;
  if (strcmp(logical_name, "info") == 0) return TRUE;
  if (strcmp(logical_name, "debug") == 0) return TRUE;
  if (strcmp(logical_name, "warn") == 0) return TRUE;
  if (strcmp(logical_name, "error") == 0) return TRUE;
  if (strcmp(logical_name, "loginfo") == 0) return TRUE;
  if (strcmp(logical_name, "logdebug") == 0) return TRUE;
  if (strcmp(logical_name, "logwarn") == 0) return TRUE;
  if (strcmp(logical_name, "logerror") == 0) return TRUE;
  if (strcmp(logical_name, "stdout") == 0) return TRUE;
  if (strcmp(logical_name, WTRACE) == 0) return TRUE;
  if (strcmp(logical_name, WDIALOG) == 0) return TRUE;
  if (strcmp(logical_name, WWARNING) == 0) return TRUE;
  if (strcmp(logical_name, WERROR) == 0) return TRUE;
  if (strcmp(logical_name, WDISPLAY) == 0) return TRUE;
  return FALSE;
}

static int
log_router_print(void *env, char *logical_name, char *str)
{
  void *rc = GetEnvironmentRouterContext(env);
  CLIPSLogger *logger = static_cast<CLIPSLogger *>(rc);
  logger->log(logical_name, str);
  return TRUE;
}

static int
log_router_exit(void *env, int exit_code)
{
  return TRUE;
}

/// @endcond


/** @class CLIPSEnvManager <plugins/clips/aspect/clips_env_manager.h>
 * CLIPS environment manager.
 * The CLIPS environment manager creates and maintains CLIPS
 * environments, registers features and provides them to the CLIPS
 * environments, and allows access to any and all CLIPS environments.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger to log messages from created environments
 * @param clock clock to get time from for (now)
 * @param clips_dir path where to look for CLIPS files
 */
CLIPSEnvManager::CLIPSEnvManager(Logger *logger, Clock *clock, std::string &clips_dir)
{
  logger_ = logger;
  clock_  = clock;
  clips_dir_ = clips_dir;
}

/** Destructor. */
CLIPSEnvManager::~CLIPSEnvManager()
{
}


/** Create a new environment.
 * This function creates a new plain environment and sets up logging etc.
 * @param log_component_name prefix for log entries
 * @return readily initialized CLIPS environment
 */
LockPtr<CLIPS::Environment>
CLIPSEnvManager::new_env(const std::string &log_component_name)
{
  // CLIPS overwrites the SIGINT handler, restore it after
  // initializing the environment
  struct sigaction oldact;
  if (sigaction(SIGINT, NULL, &oldact) == 0) {
    LockPtr<CLIPS::Environment> clips(new CLIPS::Environment(),
				      /* recursive mutex */ true);

    // by default be silent
    clips->unwatch("all");

    CLIPSContextMaintainer *cm =
      new CLIPSContextMaintainer(logger_, log_component_name.c_str());

    void *env = clips->cobj();

    SetEnvironmentContext(env, cm);

    EnvAddRouterWithContext(env, (char *)ROUTER_NAME,
                            /* exclusive */ 30,
                            log_router_query,
                            log_router_print,
                            /* getc */   NULL,
                            /* ungetc */ NULL,
                            log_router_exit,
                            cm->logger);

    // restore old action
    sigaction(SIGINT, &oldact, NULL);

    return clips;
  } else {
    throw Exception("CLIPS: Unable to backup "
		    "SIGINT sigaction for restoration.");
  }
}


/** Create a new environment.
 * The environment is registered internally under the specified name.
 * It must be destroyed when done with it. Only a single environment
 * can be created for a particular environment name.
 * @param env_name name by which to register environment
 * @param log_component_name prefix for log entries
 * @return readily initialized CLIPS environment
 */
LockPtr<CLIPS::Environment>
CLIPSEnvManager::create_env(const std::string &env_name, const std::string &log_component_name)
{
  LockPtr<CLIPS::Environment> clips;
  if (envs_.find(env_name) != envs_.end()) {
    throw Exception("CLIPS environment '%s' already exists", env_name.c_str());
  }

  clips = new_env(log_component_name);

  if (clips) {
    envs_[env_name].env = clips;

    // add generic functions
    add_functions(env_name, clips);

    // assert all currently available features to environment
    assert_features(clips, true);

    guarded_load(env_name, clips_dir_ + "utils.clp");
    guarded_load(env_name, clips_dir_ + "time.clp");
    guarded_load(env_name, clips_dir_ + "path.clp");

    clips->evaluate("(path-add \"" + clips_dir_ + "\")");

    return clips;
  } else {
    throw Exception("Failed to initialize CLIPS environment '%s'", env_name.c_str());
  }
}

/** Destroy the named environment.
 * Only ever destroy environments which you have created yourself.
 * @param env_name name of the environment to destroy
 */
void
CLIPSEnvManager::destroy_env(const std::string &env_name)
{
  if (envs_.find(env_name) != envs_.end()) {
    void *env = envs_[env_name].env->cobj();
    CLIPSContextMaintainer *cm =
      static_cast<CLIPSContextMaintainer *>(GetEnvironmentContext(env));

    EnvDeleteRouter(env, (char *)ROUTER_NAME);
    SetEnvironmentContext(env, NULL);
    delete cm;

    for (auto feat : envs_[env_name].req_feat) {
      if (features_.find(feat) != features_.end()) {
	features_[feat]->clips_context_destroyed(env_name);
      }
    }

    envs_.erase(env_name);
  }
}


/** Get map of environments.
 * @return map from environment name to environment lock ptr
 */
std::map<std::string, LockPtr<CLIPS::Environment>>
CLIPSEnvManager::environments() const
{
  std::map<std::string, LockPtr<CLIPS::Environment>> rv;
  for (auto envd : envs_) {
    rv[envd.first] = envd.second.env;
  }
  return rv;
}


CLIPS::Value
CLIPSEnvManager::clips_request_feature(std::string env_name, std::string feature_name)
{
  bool rv = true;

  logger_->log_debug("ClipsEnvManager", "Environment %s requests feature %s",
		     env_name.c_str(), feature_name.c_str());

  if (envs_.find(env_name) == envs_.end()) {
    logger_->log_warn("ClipsEnvManager", "Feature %s request from non-existent environment %s",
		      feature_name.c_str(), env_name.c_str());
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }
  if (features_.find(feature_name) == features_.end()) {
    logger_->log_warn("ClipsEnvManager", "Environment requested unavailable feature %s",
		      feature_name.c_str());
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }

  ClipsEnvData &envd = envs_[env_name];
  if (std::binary_search(envd.req_feat.begin(), envd.req_feat.end(), feature_name)) {
    logger_->log_warn("ClipsEnvManager", "Environment %s requested feature %s *again*",
		      env_name.c_str(), feature_name.c_str());
    return CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);
  }

  envd.env.lock();
  features_[feature_name]->clips_context_init(env_name, envd.env);
  envd.req_feat.push_back(feature_name);
  envd.req_feat.sort();

  // deffact so it survives a reset
  std::string deffacts = "(deffacts ff-features-loaded";

  for (auto feat : envd.req_feat) {
    deffacts += " (ff-feature-loaded " + feat + ")";
  }
  deffacts += ")";

  envd.env->assert_fact_f("(ff-feature-loaded %s)", feature_name.c_str());

  if (! envd.env->build(deffacts)) {
    logger_->log_warn("ClipsEnvManager", "Failed to build deffacts ff-features-loaded "
		      "for %s", env_name.c_str());
    rv = false;
  }
  envd.env.unlock();

  return CLIPS::Value(rv ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}


CLIPS::Values
CLIPSEnvManager::clips_now()
{
  CLIPS::Values rv;
  fawkes::Time now(clock_);
  rv.push_back(now.get_sec());
  rv.push_back(now.get_usec());
  return rv;
}


void
CLIPSEnvManager::add_functions(const std::string &env_name, LockPtr<CLIPS::Environment> &clips)
{
  clips->add_function("ff-feature-request", sigc::slot<CLIPS::Value, std::string>(sigc::bind<0>(sigc::mem_fun(*this, &CLIPSEnvManager::clips_request_feature), env_name)));
  clips->add_function("now", sigc::slot<CLIPS::Values>(sigc::mem_fun( *this, &CLIPSEnvManager::clips_now)));
}

void
CLIPSEnvManager::assert_features(LockPtr<CLIPS::Environment> &clips, bool immediate_assert)
{

  // deffact so it survives a reset
  std::string deffacts = "(deffacts ff-features-available";

  for (auto feat : features_) {
    deffacts += " (ff-feature " + feat.first + ")";
    if (immediate_assert) {
      // assert so it is immediately available
      clips->assert_fact_f("(ff-feature %s)", feat.first.c_str());
    }
  }
  deffacts += ")";
  
  if (! clips->build(deffacts)) {
    logger_->log_warn("ClipsEnvManager", "Failed to build deffacts ff-features-available");
  }
}


/** Add a feature by name.
 * @param features CLIPS feature maintainers to add
 */
void
CLIPSEnvManager::add_features(const std::list<CLIPSFeature *> &features)
{
  for (auto feat : features) {
    const std::string &feature_name = feat->clips_feature_name;

    if (features_.find(feature_name) != features_.end()) {
      throw Exception("Feature '%s' has already been registered", feature_name.c_str());
    }

    logger_->log_info("ClipsEnvManager", "Adding feature %s", feature_name.c_str());

    features_[feature_name] = feat;

    // assert fact to indicate feature availability to environments
    for (auto env : envs_) {
      env.second.env.lock();
      assert_features(env.second.env, false);
      // assert so it is immediately available
      env.second.env->assert_fact_f("(ff-feature %s)", feature_name.c_str());
      env.second.env.unlock();
    }
  }
}


/** Assert that a feature can be removed.
 * The feature will not actually be removed, it will just be checked if this
 * would work without problem.
 * @param features list of features to query for removal
 * @exception Exception thrown with a descriptive message if the feature
 * cannot be removed because it is still in use
 */
void
CLIPSEnvManager::assert_can_remove_features(const std::list<CLIPSFeature *> &features)
{
  for (auto feat : features) {
    const std::string &feature_name = feat->clips_feature_name;

    for (auto env : envs_) {
      if (std::binary_search(env.second.req_feat.begin(), env.second.req_feat.end(), feature_name)) {
	throw Exception("Cannot remove feature %s as environment %s depends on it",
			feature_name.c_str(), env.first.c_str());
      }
    }
  }
}

/** Remove a feature by name.
 * @param features list of features to remove
 * @exception Exception thrown with a descriptive message if the feature
 * cannot be removed because it is still in use
 */
void
CLIPSEnvManager::remove_features(const std::list<CLIPSFeature *> &features)
{
  // On plugin unload this would fail because destruction
  // of threads is forced.
  //assert_can_remove_features(features);
  for (auto feat : features) {
    const std::string &feature_name = feat->clips_feature_name;

    if (features_.find(feature_name) != features_.end()) {
      features_.erase(feature_name);
    }
  }
}


void
CLIPSEnvManager::guarded_load(const std::string &env_name, const std::string &filename)
{
  if (envs_.find(env_name) == envs_.end()) {
    throw Exception("guarded_load: env %s has not been registered", env_name.c_str());
  }

  LockPtr<CLIPS::Environment> &clips = envs_[env_name].env;

  int load_rv = 0;
  if ((load_rv = clips->load(filename)) != 1) {
    if (load_rv == 0) {
      destroy_env(env_name);
      throw Exception("%s: cannot find %s", env_name.c_str(), filename.c_str());
    } else {
      destroy_env(env_name);
      throw Exception("%s: CLIPS code error in %s",
		      env_name.c_str(), filename.c_str());
    }
  }
}


} // end namespace fawkes
