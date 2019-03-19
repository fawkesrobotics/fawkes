/***************************************************************************
 *  clingo_access.cpp - Clingo access wrapper for the aspects
 *
 *  Created: Mon Oct 31 13:41:07 2016
 *  Copyright  2016 Björn Schäpers
 *             2018 Tim Niemueller [www.niemueller.org]
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

#include "clingo_access.h"

#include <core/threading/mutex_locker.h>
#include <logging/logger.h>

#include <algorithm>
#include <sstream>
#include <thread>

namespace fawkes {

/** Helper class to incorporate bool into mutex locker with RAII */
class BoolMutexLocker
{
public:
	/** Constructor.
	 * @param mutex mutex to lock
	 * @param associated_bool associated bool value
	 */
	BoolMutexLocker(Mutex *mutex, bool &associated_bool)
	: mutex_(mutex), bool_(associated_bool), initial_locked_(associated_bool)
	{
		if (!initial_locked_) {
			mutex_->lock();
			bool_ = true;
		}
	}

	/** Destructor. */
	~BoolMutexLocker(void)
	{
		if (initial_locked_ != bool_) {
			if (initial_locked_) {
				relock();
			} else {
				unlock();
			}
		}
	}

	/** Unlock mutex. */
	void
	unlock(void)
	{
		bool_ = false;
		mutex_->unlock();
	}

	/** Relock mutex after unlock. */
	void
	relock(void)
	{
		mutex_->lock();
		bool_ = true;
	}

private:
	Mutex *const mutex_;
	bool &       bool_;
	const bool   initial_locked_;
};

/**
 * @class ClingoAccess
 * @brief A wrapper around the clingo control, to control the solving process.
 * This class provides access to the Clingo solver. Callbacks can be used for
 * automated notification on specific events, such if a model has been found.
 * It provides a high-level interface to the solver, i.e., to configure, start,
 * or cancel the solving process. It also supports assigning and releasing
 * externals.
 * @author Björn Schäpers
 *
 * @property ClingoAccess::logger_
 * @brief The used logger.
 *
 * @property ClingoAccess::log_comp_
 * @brief The log component.
 *
 * @property ClingoAccess::num_threads_
 * @brief How many threads Clingo should use for solving.
 *
 * @property ClingoAccess::thread_mode_splitting_
 * @brief Wether splitting should be used.
 *
 * @fn bool	ClingoAccess::assign_external(const Clingo::Symbol& atom, const bool value)
 * @brief Assign external value.
 * @param atom external atom
 * @param value value to assign
 * @return true if external was assigned, false otherwise.
 *
 * @fn bool	ClingoAccess::free_external(const Clingo::Symbol& atom)
 * @brief Release external value.
 * @param atom external atom
 * @return true if external was released, false otherwise.
 */

/**
 * @property ClingoAccess::control_mutex_
 * @brief The mutex to protect the clingo control.
 *
 * @property ClingoAccess::control_is_locked_
 * @brief Mark if control_mutex_ is locked. (Because we can not ask the
 * mutex and need this information for blocked solving.)
 *
 * @property ClingoAccess::control_
 * @brief The clingo control.
 */

/**
 * @property ClingoAccess::model_mutex_
 * @brief The mutex for the model symbols.
 *
 * @property ClingoAccess::model_symbols_
 * @brief The symbols found in the last model.
 *
 * @property ClingoAccess::old_symbols_
 * @brief The symbols found in the before last model.
 *
 * @property ClingoAccess::model_counter_
 * @brief Counts how many models we have computed for one solving process.
 */

/**
 * @property ClingoAccess::solving_
 * @brief Whether the control is in the solving process.
 *
 * @property ClingoAccess::callback_mutex_
 * @brief The mutex to protect the callback vectors.
 *
 * @property ClingoAccess::model_callbacks_
 * @brief The functions to call on a new model.
 *
 * @property ClingoAccess::finish_callbacks_
 * @brief The functions to call, when the solving process finished.
 *
 * @property ClingoAccess::ground_callback_
 * @brief The callback for the grounding.
 *
 * @property ClingoAccess::async_handle_
 * @brief The handle for the async solving.
 *
 * @property ClingoAccess::debug_level_
 * @brief Which debug outputs should be printed.
 */

/**
 * @brief Extracts the symbols from the new model and calls the registered functions.
 * @param[in] model The new model.
 * @return If the solving process should compute more models (if there are any).
 */
bool
ClingoAccess::on_model(Clingo::Model &model)
{
	MutexLocker locker1(&model_mutex_);
	model_symbols_ = model.symbols(
	  debug_level_ >= ASP_DBG_ALL_MODEL_SYMBOLS ? Clingo::ShowType::All : Clingo::ShowType::Shown);

	if (debug_level_ >= ASP_DBG_TIME) {
		logger_->log_info(log_comp_.c_str(),
		                  "New %smodel found: #%d",
		                  model.optimality_proven() ? "optimal " : "",
		                  ++model_counter_);

		if (debug_level_ >= ASP_DBG_MODELS) {
			/* To save (de-)allocations just move found symbols at the end
			 * of the vector and move the end iterator to the front. After
			 * this everything in [begin, end) is in oldSymbols but not in
			 * symbols. */
			auto begin = old_symbols_.begin(), end = old_symbols_.end();

			for (const Clingo::Symbol &symbol : model_symbols_) {
				auto iter = std::find(begin, end, symbol);
				if (iter == end) {
					logger_->log_info(log_comp_.c_str(), "New Symbol: %s", symbol.to_string().c_str());
				} else {
					std::swap(*iter, *--end);
				}
			}

			for (; begin != end; ++begin) {
				logger_->log_info(log_comp_.c_str(), "Symbol removed: %s", begin->to_string().c_str());
			}
		}
	}

	old_symbols_ = model_symbols_;

	bool ret = false;

	MutexLocker locker2(&callback_mutex_);
	for (const auto &cb : model_callbacks_) {
		ret |= (*cb)();
	}

	return ret;
}

/**
 * @brief Calls the callbacks for the end of the solving.
 * @param[in] result The result of the solving process.
 */
void
ClingoAccess::on_finish(Clingo::SolveResult result)
{
	if (debug_level_ >= ASP_DBG_TIME) {
		logger_->log_info(log_comp_.c_str(), "Solving nearly done.");
	}

	BoolMutexLocker locker1(&control_mutex_, control_is_locked_);
	MutexLocker     locker2(&callback_mutex_);
	for (const auto &cb : finish_callbacks_) {
		(*cb)(result);
	}

	std::thread blocking_thread([this](void) {
		async_handle_.wait();
		if (debug_level_ >= ASP_DBG_TIME) {
			logger_->log_info(log_comp_.c_str(), "Solving done.");
		}
		solving_ = false;
		return;
	});
	blocking_thread.detach();
	return;
}

/**
 * @brief Allocates the control object and initializes the logger.
 */
void
ClingoAccess::alloc_control()
{
	assert(!control_);

	/* The arguments to Clingo::Control are given as a Span of const char*, because we need to compose some strings we
	 * save them as std::string, so we have not to take care about memory leaks. The arguments given to Clingo are saved
	 * in another vector, where the c_str() pointers of the strings are saved. */

	std::vector<std::string>  argumentsString;
	std::vector<const char *> argumentsChar;

	if (debug_level_ >= ASP_DBG_EVEN_CLINGO) {
		argumentsChar.push_back("--output-debug=text");
	}

	if (num_threads_ != 1) {
		std::stringstream s("-t ", std::ios_base::ate | std::ios_base::out);
		s << num_threads_ << "," << (thread_mode_splitting_ ? "split" : "compete");
		argumentsString.push_back(s.str());
		argumentsChar.push_back(argumentsString.back().c_str());
	}

	control_ = new Clingo::Control(argumentsChar,
	                               [this](const Clingo::WarningCode code, char const *msg) {
		                               fawkes::Logger::LogLevel level = fawkes::Logger::LL_NONE;
		                               switch (code) {
		                               case Clingo::WarningCode::AtomUndefined:
		                               case Clingo::WarningCode::OperationUndefined:
		                               case Clingo::WarningCode::RuntimeError:
			                               level = fawkes::Logger::LL_ERROR;
			                               break;
		                               case Clingo::WarningCode::Other:
		                               case Clingo::WarningCode::VariableUnbounded:
			                               level = fawkes::Logger::LL_WARN;
			                               break;
		                               case Clingo::WarningCode::FileIncluded:
		                               case Clingo::WarningCode::GlobalVariable:
			                               level = fawkes::Logger::LL_INFO;
			                               break;
		                               }
		                               logger_->log(level, log_comp_.c_str(), msg);
		                               return;
	                               },
	                               100);
	return;
}

/** Constructor.
 * @param[in] logger The used logger.
 * @param[in] log_component The logging component.
 */
ClingoAccess::ClingoAccess(Logger *logger, const std::string &log_component)
: logger_(logger),
  log_comp_(log_component.empty() ? "Clingo" : log_component),
  debug_level_(ASP_DBG_NONE),
  num_threads_(1),
  thread_mode_splitting_(false),
  control_is_locked_(false),
  control_(nullptr),
  model_mutex_(Mutex::RECURSIVE),
  solving_(false)
{
	alloc_control();
}

/** Destructor. */
ClingoAccess::~ClingoAccess(void)
{
	delete control_;
}

/**
 * @brief Registers a callback for the event of a new model. The callback can control with it's return value if the
 *        search for models should continue. If one of the callbacks says yes the search is continued.
 * @param[in] callback The callback to register.
 */
void
ClingoAccess::register_model_callback(std::shared_ptr<std::function<bool(void)>> callback)
{
	MutexLocker locker(&callback_mutex_);
	model_callbacks_.emplace_back(callback);
}

/**
 * @brief Unregisters a callback for the event of a new model.
 * @param[in] callback The callback to unregister.
 */
void
ClingoAccess::unregister_model_callback(std::shared_ptr<std::function<bool(void)>> callback)
{
	MutexLocker locker(&callback_mutex_);
	model_callbacks_.erase(std::find(model_callbacks_.begin(), model_callbacks_.end(), callback));
}

/**
 * @brief Registers a callback for the event of finishing the solving process.
 * @param[in] callback The callback to register.
 */
void
ClingoAccess::register_finish_callback(
  std::shared_ptr<std::function<void(Clingo::SolveResult)>> callback)
{
	MutexLocker locker(&callback_mutex_);
	finish_callbacks_.emplace_back(callback);
}

/**
 * @brief Unregisters a callback for the event of finishing the solving process.
 * @param[in] callback The callback to unregister.
 */
void
ClingoAccess::unregister_finish_callback(
  std::shared_ptr<std::function<void(Clingo::SolveResult)>> callback)
{
	MutexLocker locker(&callback_mutex_);
	finish_callbacks_.erase(std::find(finish_callbacks_.begin(), finish_callbacks_.end(), callback));
}

/**
 * @brief Sets the ground callback, to implement custom functions.
 * @param[in, out] callback The callback, will be moved.
 */
void
ClingoAccess::set_ground_callback(Clingo::GroundCallback &&callback)
{
	MutexLocker locker(&callback_mutex_);
	ground_callback_ = std::move(callback);
}

/** Returns whether the solving process is running.
 * @return true if currently solving, false otherwise
 */
bool
ClingoAccess::solving(void) const noexcept
{
	return solving_;
}

/**
 * @brief Starts the solving process, if it isn't already running.
 * @return If the process is started.
 */
bool
ClingoAccess::start_solving(void)
{
	BoolMutexLocker locker(&control_mutex_, control_is_locked_);
	if (solving_) {
		return false;
	}
	if (debug_level_ >= ASP_DBG_TIME) {
		logger_->log_info(log_comp_.c_str(), "Start async solving.");
	}

	solving_ = true;
	model_mutex_.lock();
	model_counter_ = 0;
	model_mutex_.unlock();
	async_handle_ = control_->solve(Clingo::SymbolicLiteralSpan{}, this, true, true);
	return true;
}

/** Starts the solving process.
 * If it isn't already running, in a blocking manner, that means it
 * does not start the computation in an asynchronous way.
 * @return true, if the process was started
 */
bool
ClingoAccess::start_solving_blocking(void)
{
	if (solving_) {
		return false;
	}

	BoolMutexLocker locker(&control_mutex_, control_is_locked_);
	if (debug_level_ >= ASP_DBG_TIME) {
		logger_->log_info(log_comp_.c_str(), "Start sync solving.");
	}

	solving_ = true;
	model_mutex_.lock();
	model_counter_ = 0;
	model_mutex_.unlock();
	control_->solve(Clingo::SymbolicLiteralSpan{}, this, false, true);
	return true;
}

/** Stops the solving process, if it is running.
 * @return If it was stopped. (Check solving() for actual state!)
 */
bool
ClingoAccess::cancel_solving(void)
{
	BoolMutexLocker locker(&control_mutex_, control_is_locked_);
	if (!solving_) {
		return false;
	}

	if (debug_level_ >= ASP_DBG_TIME) {
		logger_->log_info(log_comp_.c_str(), "Cancel solving.");
	}

	control_->interrupt();
	return true;
}

/** Tries to reset Clingo.
 * That means deletes the control object and creates a new one.
 * @return true, if successful
 */
bool
ClingoAccess::reset(void)
{
	if (solving_) {
		logger_->log_warn(log_comp_.c_str(),
		                  "Could not reset while solving. "
		                  "Please try again when the solving is stopped.");
		cancel_solving();
		return false;
	}
	logger_->log_warn(log_comp_.c_str(), "Resetting Clingo");
	delete control_;
	control_ = nullptr;
	alloc_control();
	return true;
}

/** Sets the number of threads Clingo should use.
 * @param[in] threads The number.
 * @param[in] thread_mode_splitting Wether splitting should be used.
 * @warning This will call reset().
 * @exception Exception If it is called while solving.
 * @exception Exception If it is called with @c threads < 1.
 */
void
ClingoAccess::set_num_threads(const int threads, const bool thread_mode_splitting)
{
	if (solving_) {
		throw Exception("Tried to set the number of threads while Clingo was solving.");
	}
	if (threads < 1) {
		throw Exception("Tried to set thread count to %d, only values >= 1 are valid.", threads);
	}
	logger_->log_info(log_comp_.c_str(),
	                  "Change # of threads for solving from %d to %d "
	                  "and splitting from %s to %s.",
	                  num_threads_,
	                  threads,
	                  thread_mode_splitting_ ? "true" : "false",
	                  thread_mode_splitting ? "true" : "false");
	num_threads_           = threads;
	thread_mode_splitting_ = thread_mode_splitting;
	reset();
	return;
}

/** Returns how many threads Clingo should use.
 * @return number of threads
 * @see ClingoAccess::num_threads_
 */
int
ClingoAccess::num_threads(void) const noexcept
{
	return num_threads_;
}

/** Get current debug level.
 * @return current debug level
 */
ClingoAccess::DebugLevel_t
ClingoAccess::debug_level() const
{
	return debug_level_.load();
}

/** Set debug level.
 * @param log_level new debug level
 */
void
ClingoAccess::set_debug_level(ClingoAccess::DebugLevel_t log_level)
{
	return debug_level_.store(log_level);
}

/** Returns a copy of the last symbols found.
 * @return copy of last symbols found
 */
Clingo::SymbolVector
ClingoAccess::model_symbols(void) const
{
	MutexLocker locker(&model_mutex_);
	return model_symbols_;
}

/** Loads a file in the solver.
 * @param[in] path The path of the file.
 * @return true, if file was loaded
 */
bool
ClingoAccess::load_file(const std::string &path)
{
	BoolMutexLocker locker(&control_mutex_, control_is_locked_);
	if (solving_) {
		return false;
	}

	logger_->log_info(log_comp_.c_str(), "Loading file program file %s.", path.c_str());
	control_->load(path.c_str());
	return true;
}

/** Grounds a program part.
 * @param[in] parts The parts to ground.
 * @return true if parts could be grounded
 */
bool
ClingoAccess::ground(const Clingo::PartSpan &parts)
{
	if (parts.empty()) {
		return true;
	}

	BoolMutexLocker locker(&control_mutex_, control_is_locked_);
	if (solving_) {
		return false;
	}
	if (debug_level_ >= ASP_DBG_TIME) {
		logger_->log_info(log_comp_.c_str(), "Grounding %zu parts:", parts.size());
		if (debug_level_ >= ASP_DBG_PROGRAMS) {
			auto i = 0;
			for (const Clingo::Part &part : parts) {
				std::string params;
				bool        first = true;
				for (const auto &param : part.params()) {
					if (first) {
						first = false;
					} else {
						params += ", ";
					}
					params += param.to_string();
				}
				logger_->log_info(log_comp_.c_str(), "Part #%d: %s [%s]", ++i, part.name(), params.c_str());
			}
		}
	}

	control_->ground(parts, ground_callback_);

	if (debug_level_ >= ASP_DBG_TIME) {
		logger_->log_info(log_comp_.c_str(), "Grounding done.");
	}
	return true;
}

/** Assigns an external value.
 * @param[in] atom The atom to assign.
 * @param[in] value The assigned value.
 * @return If it could be assigned.
 */
bool
ClingoAccess::assign_external(const Clingo::Symbol &atom, const Clingo::TruthValue value)
{
	BoolMutexLocker locker(&control_mutex_, control_is_locked_);
	if (solving_) {
		return false;
	}

	if (debug_level_ >= ASP_DBG_EXTERNALS) {
		logger_->log_info(log_comp_.c_str(),
		                  "Assigning %s to %s.",
		                  [value](void) {
			                  const char *ret = "Unknown Value";
			                  switch (value) {
			                  case Clingo::TruthValue::Free: ret = "Free"; break;
			                  case Clingo::TruthValue::True: ret = "True"; break;
			                  case Clingo::TruthValue::False: ret = "False"; break;
			                  }
			                  return ret;
		                  }(),
		                  atom.to_string().c_str());
	}
	control_->assign_external(atom, value);
	return true;
}

/** Releases an external value.
 * @param[in] atom The atom to release.
 * @return true, if it could be released
 */
bool
ClingoAccess::release_external(const Clingo::Symbol &atom)
{
	BoolMutexLocker locker(&control_mutex_, control_is_locked_);
	if (solving_) {
		return false;
	}

	if (debug_level_ >= ASP_DBG_EXTERNALS) {
		logger_->log_info(log_comp_.c_str(), "Releasing %s.", atom.to_string().c_str());
	}
	control_->release_external(atom);
	return true;
}

} // end namespace fawkes
