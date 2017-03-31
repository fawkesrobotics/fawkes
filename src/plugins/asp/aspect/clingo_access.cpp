/***************************************************************************
 *  clingo_access.cpp - Clingo access wrapper for the aspects
 *
 *  Created: Mon Oct 31 13:41:07 2016
 *  Copyright  2016 Björn Schäpers
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

#include "clingo_access.h"

#include <algorithm>
#include <sstream>
#include <thread>

#include <core/threading/mutex_locker.h>
#include <logging/logger.h>

namespace fawkes {

#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/**
 * @class ClingoAccess
 * @brief A wrapper around the clingo control, to control the solving process.
 * @author Björn Schäpers
 *
 * @property ClingoAccess::Log
 * @brief The used logger.
 *
 * @property ClingoAccess::LogComponent
 * @brief The log component.
 *
 * @property ClingoAccess::NumberOfThreads
 * @brief How many threads Clingo should use for solving.
 *
 * @property ClingoAccess::Splitting
 * @brief Wether splitting should be used.
 */

/**
 * @property ClingoAccess::ControlMutex
 * @brief The mutex to protect the clingo control.
 *
 * @property ClingoAccess::Control
 * @brief The clingo control.
 */

/**
 * @property ClingoAccess::ModelMutex
 * @brief The mutex for the model symbols.
 *
 * @property ClingoAccess::ModelSymbols
 * @brief The symbols found in the last model.
 *
 * @property ClingoAccess::OldSymbols
 * @brief The symbols found in the before last model.
 *
 * @property ClingoAccess::ModelCounter
 * @brief Counts how many models we have computed for one solving process.
 */

/**
 * @property ClingoAccess::Solving
 * @brief Whether the control is in the solving process.
 *
 * @property ClingoAccess::CallbackMutex
 * @brief The mutex to protect the callback vectors.
 *
 * @property ClingoAccess::ModelCallbacks
 * @brief The functions to call on a new model.
 *
 * @property ClingoAccess::FinishCallbacks
 * @brief The functions to call, when the solving process finished.
 *
 * @property ClingoAccess::GroundCallback
 * @brief The callback for the grounding.
 *
 * @property ClingoAccess::AsyncHandle
 * @brief The handle for the async solving.
 */

/**
 * @enum ClingoAccess::DebugLevel_t
 * @brief An enum to define debug levels. The higher levels include the lower values.
 *
 * @var ClingoAccess::DebugLevel_t::None
 * @brief No debug output at all.
 *
 * @var ClingoAccess::DebugLevel_t::Time
 * @brief Print when starting/finishing grounding/solving for analysis.
 *
 * @var ClingoAccess::DebugLevel_t::Programs
 * @brief Print which programs are grounded.
 *
 * @var ClingoAccess::DebugLevel_t::Models
 * @brief Print new models.
 *
 * @var ClingoAccess::DebugLevel_t::Externals
 * @brief Print assignments and releases of externals.
 *
 * @var ClingoAccess::DebugLevel_t::AllModelSymbols
 * @brief Ignore #show statements and print all symbols of a model.
 *
 * @var ClingoAccess::DebugLevel_t::All
 * @brief Print everything.
 *
 * @var ClingoAccess::DebugLevel_t::EvenClingo
 * @brief Activates the --output-debug=text option for clingo. This produces ALOT of output!
 *
 * @property ClingoAccess::DebugLevel
 * @brief Which debug outputs should be printed.
 */


/**
 * @brief Extracts the symbols from the new model and calls the registered functions.
 * @param[in] model The new model.
 * @return If the solving process should compute more models (if there are any).
 */
bool
ClingoAccess::newModel(const Clingo::Model& model)
{
	MutexLocker locker1(&ModelMutex);
	ModelSymbols = model.symbols(DebugLevel >= AllModelSymbols ? Clingo::ShowType::All : Clingo::ShowType::Shown);

	if ( DebugLevel >= Time )
	{
		Log->log_info(LogComponent.c_str(), "New %smodel found: #%d", model.optimality_proven() ? "optimal " : "",
			++ModelCounter);

		if ( DebugLevel >= Models )
		{
			/* To save (de-)allocations just move found symbols at the end of the vector and move the end iterator to
			 * the front. After this everything in [begin, end) is in oldSymbols but not in symbols. */
			auto begin = OldSymbols.begin(), end = OldSymbols.end();

			for ( const Clingo::Symbol& symbol : ModelSymbols )
			{
				auto iter = std::find(begin, end, symbol);
				if ( iter == end )
				{
					Log->log_info(LogComponent.c_str(), "New Symbol: %s", symbol.to_string().c_str());
				} //if ( iter == end )
				else
				{
					std::swap(*iter, *--end);
				} //else -> if ( iter == end )
			} //for ( const Clingo::Symbol& symbol : ModelSymbols )

			for ( ; begin != end; ++begin )
			{
				Log->log_info(LogComponent.c_str(), "Symbol removed: %s", begin->to_string().c_str());
			} //for ( ; begin != end; ++begin )
		} //if ( DebugLevel >= Models )
	} //if ( DebugLevel >= Time )

	OldSymbols = ModelSymbols;

	bool ret = false;

	MutexLocker locker2(&CallbackMutex);
	for ( const auto& cb : ModelCallbacks )
	{
		ret |= (*cb)();
	} //for ( const auto& cb : ModelCallbacks )

	return ret;
}

/**
 * @brief Calls the callbacks for the end of the solving.
 * @param[in] result The result of the solving process.
 */
void
ClingoAccess::solvingFinished(const Clingo::SolveResult result)
{
	if ( DebugLevel >= Time )
	{
		Log->log_info(LogComponent.c_str(), "Solving nearly done.");
	} //if ( DebugLevel >= Time )

	MutexLocker locker1(&ControlMutex), locker2(&CallbackMutex);
	for ( const auto& cb : FinishCallbacks )
	{
		(*cb)(result);
	} //for ( const auto& cb : FinishCallbacks )

	std::thread blockingThread([this](void)
		{
			AsyncHandle.wait();
			if ( DebugLevel >= Time )
			{
				Log->log_info(LogComponent.c_str(), "Solving done.");
			} //if ( DebugLevel >= Time )
			Solving = false;
			return;
		});
	blockingThread.detach();
	return;
}

/**
 * @brief Allocates the control object and initializes the logger.
 */
void
ClingoAccess::allocControl()
{
	assert(!Control);

	/* The arguments to Clingo::Control are given as a Span of const char*, because we need to compose some strings we
	 * save them as std::string, so we have not to take care about memory leaks. The arguments given to Clingo are saved
	 * in another vector, where the c_str() pointers of the strings are saved. */

	std::vector<std::string> argumentsString;
	std::vector<const char*> argumentsChar;

	if ( DebugLevel >= EvenClingo )
	{
		argumentsChar.push_back("--output-debug=text");
	} //if ( DebugLevel >= EvenClingo )

	if ( NumberOfThreads != 1 )
	{
		std::stringstream s("-t ", std::ios_base::ate | std::ios_base::out);
		s<<NumberOfThreads<<","<<(Splitting ? "split" : "compete");
		argumentsString.push_back(s.str());
		argumentsChar.push_back(argumentsString.back().c_str());
	} //if ( NumberOfThreads != 1 )

	Control = new Clingo::Control(argumentsChar,
		[this](const Clingo::WarningCode code, char const *msg)
		{
			fawkes::Logger::LogLevel level = fawkes::Logger::LL_NONE;
			switch ( code )
			{
				case Clingo::WarningCode::AtomUndefined      :
				case Clingo::WarningCode::OperationUndefined :
				case Clingo::WarningCode::RuntimeError       : level = fawkes::Logger::LL_ERROR; break;
				case Clingo::WarningCode::Other              :
				case Clingo::WarningCode::VariableUnbounded  : level = fawkes::Logger::LL_WARN; break;
				case Clingo::WarningCode::FileIncluded       :
				case Clingo::WarningCode::GlobalVariable     : level = fawkes::Logger::LL_INFO; break;
			} //switch ( code )
			Log->log(level, LogComponent.c_str(), msg);
			return;
		}, 100);
	return;
}

/**
 * @brief Constructor.
 * @param[in] log The used logger.
 * @param[in] logComponent The logging component.
 */
ClingoAccess::ClingoAccess(Logger *log, const std::string& logComponent) : Log(log),
		LogComponent(logComponent.empty() ? "Clingo" : logComponent), NumberOfThreads(1), Splitting(false),
		Control(nullptr), ModelMutex(Mutex::RECURSIVE), Solving(false), AsyncHandle(nullptr), DebugLevel(None)
{
	allocControl();
	return;
}

/**
 * @brief The destructor.
 */
ClingoAccess::~ClingoAccess(void)
{
	delete Control;
	return;
}

/**
 * @brief Registers a callback for the event of a new model. The callback can control with it's return value if the
 *        search for models should continue. If one of the callbacks says yes the search is continued.
 * @param[in] callback The callback to register.
 */
void
ClingoAccess::registerModelCallback(std::shared_ptr<std::function<bool(void)>> callback)
{
	MutexLocker locker(&CallbackMutex);
	ModelCallbacks.emplace_back(callback);
	return;
}

/**
 * @brief Unregisters a callback for the event of a new model.
 * @param[in] callback The callback to unregister.
 */
void
ClingoAccess::unregisterModelCallback(std::shared_ptr<std::function<bool(void)>> callback)
{
	MutexLocker locker(&CallbackMutex);
	ModelCallbacks.erase(std::find(ModelCallbacks.begin(), ModelCallbacks.end(), callback));
	return;
}

/**
 * @brief Registers a callback for the event of finishing the solving process.
 * @param[in] callback The callback to register.
 */
void
ClingoAccess::registerFinishCallback(std::shared_ptr<std::function<void(Clingo::SolveResult)>> callback)
{
	MutexLocker locker(&CallbackMutex);
	FinishCallbacks.emplace_back(callback);
	return;
}

/**
 * @brief Unregisters a callback for the event of finishing the solving process.
 * @param[in] callback The callback to unregister.
 */
void
ClingoAccess::unregisterFinishCallback(std::shared_ptr<std::function<void(Clingo::SolveResult)>> callback)
{
	MutexLocker locker(&CallbackMutex);
	FinishCallbacks.erase(std::find(FinishCallbacks.begin(), FinishCallbacks.end(), callback));
	return;
}

/**
 * @brief Sets the ground callback, to implement custom functions.
 * @param[in, out] callback The callback, will be moved.
 */
void
ClingoAccess::setGroundCallback(Clingo::GroundCallback&& callback)
{
	MutexLocker locker(&CallbackMutex);
	GroundCallback = std::move(callback);
	return;
}

/**
 * @brief Returns whether the solving process is running.
 */
bool
ClingoAccess::solving(void) const noexcept
{
	return Solving;
}

/**
 * @brief Starts the solving process, if it isn't already running.
 * @return If the process is started.
 */
bool
ClingoAccess::startSolving(void)
{
	MutexLocker locker(&ControlMutex);
	if ( Solving )
	{
		return false;
	} //if ( Solving )

	if ( DebugLevel >= Time )
	{
		Log->log_info(LogComponent.c_str(), "Start async solving.");
	} //if ( DebugLevel >= Time )
	Solving = true;
	ModelMutex.lock();
	ModelCounter = 0;
	ModelMutex.unlock();
	AsyncHandle = Control->solve_async([this](const Clingo::Model& model) { return newModel(model); },
		[this](const Clingo::SolveResult& result) { solvingFinished(result); return; });
	return true;
}

/**
 * @brief Starts the solving process, if it isn't already running, in a blocking manner, that means it does not start
 *        the computation in an asynchronous way.
 * @return If the process was started.
 */
bool
ClingoAccess::startSolvingBlocking(void)
{
	if ( Solving )
	{
		return false;
	} //if ( Solving )

	MutexLocker locker(&ControlMutex);
	if ( DebugLevel >= Time )
	{
		Log->log_info(LogComponent.c_str(), "Start sync solving.");
	} //if ( DebugLevel >= Time )
	Solving = true;
	ModelMutex.lock();
	ModelCounter = 0;
	ModelMutex.unlock();
	const auto result(Control->solve([this,&locker](const Clingo::Model& model) {
		locker.unlock();
		const auto ret = newModel(model);
		locker.relock();
		return ret;
	}));
	locker.unlock();
	solvingFinished(result);
	return true;
}

/**
 * @brief Stops the solving process, if it is running.
 * @return If it was stopped. (Check solving() for actual state!)
 */
bool
ClingoAccess::cancelSolving(void)
{
	MutexLocker locker(&ControlMutex);
	if ( !Solving )
	{
		return false;
	} //if ( !Solving )

	if ( DebugLevel >= Time )
	{
		Log->log_info(LogComponent.c_str(), "Cancel solving.");
	} //if ( DebugLevel >= Time )

	Control->interrupt();
	return true;
}

/**
 * @brief Tries to reset Clingo, that means deletes the control object and creates a new one.
 * @return If it was an success.
 */
bool
ClingoAccess::reset(void)
{
	if ( Solving )
	{
		Log->log_warn(LogComponent.c_str(),
			"Could not reset while solving. Please try again when the solving is stopped.");
		cancelSolving();
		return false;
	} //if ( Solving )
	Log->log_warn(LogComponent.c_str(), "Clingo will be resetted.");
	delete Control;
	Control = nullptr;
	allocControl();
	return true;
}

/**
 * @brief Sets the number of threads Clingo should use.
 * @param[in] threads The number.
 * @param[in] useSPlitting Wether splitting should be used.
 * @warning This will call reset().
 * @exception Exception If it is called while solving.
 * @exception Exception If it is called with @c threads < 1.
 */
void
ClingoAccess::setNumberOfThreads(const int threads, const bool useSplitting)
{
	if ( Solving )
	{
		throw Exception("Tried to set the number of threads while Clingo was solving.");
	} //if ( Solving )
	if ( threads < 1 )
	{
		throw Exception("Tried to set thread count to %d, only values >= 1 are valid.", threads);
	} //if ( threads < 1 )
	Log->log_info(LogComponent.c_str(), "Change # of threads for solving from %d to %d and splitting from %s to %s.",
		NumberOfThreads, threads, Splitting ? "true" : "false", useSplitting ? "true" : "false");
	NumberOfThreads = threads;
	Splitting = useSplitting;
	reset();
	return;
}

/**
 * @brief Returns how many threads Clingo should use.
 * @return @link ClingoAccess::NumberOfThreads NumberOfThreads @endlink
 */
int
ClingoAccess::numberOfThreads(void) const noexcept
{
	return NumberOfThreads;
}

/**
 * @brief Returns a copy of the last symbols found.
 */
Clingo::SymbolVector
ClingoAccess::modelSymbols(void) const
{
	MutexLocker locker(&ModelMutex);
	return ModelSymbols;
}

/**
 * @brief Loads a file in the solver.
 * @param[in] path The path of the file.
 * @return If the file was loaded.
 */
bool
ClingoAccess::loadFile(const std::string& path)
{
	MutexLocker locker(&ControlMutex);
	if ( Solving )
	{
		return false;
	} //if ( Solving )

	Log->log_info(LogComponent.c_str(), "Loading file program file %s.", path.c_str());
	Control->load(path.c_str());
	return true;
}

/**
 * @brief Grounds a program part.
 * @param[in] parts The parts to ground.
 * @return If the parts could be grounded.
 */
bool
ClingoAccess::ground(const Clingo::PartSpan& parts)
{
	if ( parts.empty() )
	{
		return true;
	} //if ( parts.empty() )

	MutexLocker locker(&ControlMutex);
	if ( Solving )
	{
		return false;
	} //if ( Solving )

	if ( DebugLevel >= Time )
	{
		Log->log_info(LogComponent.c_str(), "Grounding %zu parts:", parts.size());
		if ( DebugLevel >= Programs )
		{
			auto i = 0;
			for ( const Clingo::Part& part : parts )
			{
				std::string params;
				bool first = true;
				for ( const auto& param : part.params() )
				{
					if ( first )
					{
						first = false;
					} //if ( first )
					else
					{
						params += ", ";
					} //else -> if ( first )
					params += param.to_string();
				} //for ( const auto& param : part.params() )
				Log->log_info(LogComponent.c_str(), "Part #%d: %s [%s]", ++i, part.name(), params.c_str());
			} //for ( const auto& part : parts )
		} //if ( DebugLevel >= Programs )
	} //if ( DebugLevel >= Time )

	Control->ground(parts, GroundCallback);

	if ( DebugLevel >= Time )
	{
		Log->log_info(LogComponent.c_str(), "Grounding done.");
	} //if ( DebugLevel >= Time )
	return true;
}

/**
 * @brief Assigns an external value.
 * @param[in] atom The atom to assign.
 * @param[in] value The assigned value.
 * @return If it could be assigned.
 */
bool
ClingoAccess::assign_external(const Clingo::Symbol& atom, const Clingo::TruthValue value)
{
	MutexLocker locker(&ControlMutex);
	if ( Solving )
	{
		return false;
	} //if ( Solving )

	if ( DebugLevel >= Externals )
	{
		Log->log_info(LogComponent.c_str(), "Assigning %s to %s.", [value](void)
			{
				const char *ret = "Unknown Value";
				switch ( value )
				{
					case Clingo::TruthValue::Free  : ret = "Free";  break;
					case Clingo::TruthValue::True  : ret = "True";  break;
					case Clingo::TruthValue::False : ret = "False"; break;
				} //switch ( value )
				return ret;
			}(), atom.to_string().c_str());
	} //if ( DebugLevel >= Externals )
	Control->assign_external(atom, value);
	return true;
}

/**
 * @brief Releases an external value.
 * @param[in] atom The atom to release.
 * @return If it could be released.
 */
bool
ClingoAccess::release_external(const Clingo::Symbol& atom)
{
	MutexLocker locker(&ControlMutex);
	if ( Solving )
	{
		return false;
	} //if ( Solving )

	if ( DebugLevel >= Externals )
	{
		Log->log_info(LogComponent.c_str(), "Releasing %s.", atom.to_string().c_str());
	} //if ( DebugLevel >= Externals )
	Control->release_external(atom);
	return true;
}

} // end namespace fawkes
