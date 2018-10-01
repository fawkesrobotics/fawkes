
/***************************************************************************
 *  yaml_node.h - Utility class for internal YAML config handling
 *
 *  Created: Thu Aug 09 14:08:18 2012
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#ifndef __CONFIG_YAML_NODE_H_
#define __CONFIG_YAML_NODE_H_

#ifndef __CONFIG_YAML_H_
#  error Do not include yaml_node.h directly
#endif

#include <utils/misc/string_conversions.h>
#include <utils/misc/string_split.h>
#include <fstream>
#include <iostream>
#include <stack>
#include <cerrno>
#include <climits>
#include <unistd.h>
#include <algorithm>
#include <yaml-cpp/traits.h>
#include <limits>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <regex>
#include <memory>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/// @cond INTERNALS

#define PATH_REGEX "^[a-zA-Z0-9_-]+$"
#define YAML_REGEX "^[a-zA-Z0-9_-]+\\.yaml$"
// from https://www.ietf.org/rfc/rfc3986.txt
#define URL_REGEX "^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\\?([^#]*))?(#(.*))?"
#define FRAME_REGEX "^([a-zA-Z_][a-zA-Z0-9_/-]*)+$"

namespace yaml_utils {

	namespace detail {
		// we're not gonna mess with the mess that is all the isupper/etc. functions
		inline bool IsLower(char ch) { return 'a' <= ch && ch <= 'z'; }
		inline bool IsUpper(char ch) { return 'A' <= ch && ch <= 'Z'; }
		inline char ToLower(char ch) { return IsUpper(ch) ? ch + 'a' - 'A' : ch; }

		inline std::string tolower(const std::string& str)
		{
			std::string s(str);
			std::transform(s.begin(), s.end(), s.begin(), ToLower);
			return s;
		}

		template <typename T>
			inline bool IsEntirely(const std::string& str, T func)
		{
			for(std::size_t i=0;i<str.size();i++)
				if(!func(str[i]))
					return false;

			return true;
		}

		// IsFlexibleCase
		// . Returns true if 'str' is:
		//   . UPPERCASE
		//   . lowercase
		//   . Capitalized
		inline bool IsFlexibleCase(const std::string& str)
		{
			if(str.empty())
				return true;

			if(IsEntirely(str, IsLower))
				return true;

			bool firstcaps = IsUpper(str[0]);
			std::string rest = str.substr(1);
			return firstcaps && (IsEntirely(rest, IsLower) || IsEntirely(rest, IsUpper));
		}
	}

	inline bool convert(const std::string& input, std::string& output) {
		output = input;
		return true;
	}

	inline bool convert(const std::string& input, bool& output)
	{
		// we can't use iostream bool extraction operators as they don't
		// recognize all possible values in the table below (taken from
		// http://yaml.org/type/bool.html)
		static const struct {
			std::string truename, falsename;
		} names[] = {
			{ "y", "n" },
			{ "yes", "no" },
			{ "true", "false" },
			{ "on", "off" },
		};

		if(! detail::IsFlexibleCase(input))
			return false;

		for(unsigned i=0;i<sizeof(names)/sizeof(names[0]);i++) {
			if(names[i].truename == detail::tolower(input)) {
				output = true;
				return true;
			}

			if(names[i].falsename == detail::tolower(input)) {
				output = false;
				return true;
			}
		}

		return false;
	}

	inline bool
		convert(const std::string& input, YAML::_Null& output)
	{
		return input.empty() || input == "~" || input == "null" || input == "Null" || input == "NULL";
	}

	inline bool convert(const std::string &input, unsigned int &rhs)
	{
		errno = 0;
		char *endptr;
		long int l = strtol(input.c_str(), &endptr, 0);

		if ((errno == ERANGE && (l == LONG_MAX || l == LONG_MIN)) || (errno != 0 && l == 0)) {
			return false;
		}
		if (endptr == input.c_str())  return false;
		if (*endptr != 0)  return false;
		if (l < 0)   return false;

		rhs = (unsigned int)l;

		return true;
	}


	template <typename T>
		inline bool convert(const std::string &input, T &rhs,
		                    typename YAML::enable_if<YAML::is_numeric<T> >::type * = 0)
	{
		std::stringstream stream(input);
		stream.unsetf(std::ios::dec);
		if((stream >> rhs) && (stream >> std::ws).eof())
			return true;
		else {
		}
		if(std::numeric_limits<T>::has_infinity) {
			if(YAML::conversion::IsInfinity(input) || YAML::conversion::IsNegativeInfinity(input) )
			{
				rhs = std::numeric_limits<T>::infinity();
				return true;
			}
		}

		if(std::numeric_limits<T>::has_quiet_NaN && YAML::conversion::IsNaN(input)) {
			rhs = std::numeric_limits<T>::quiet_NaN();
			return true;
		}

		return false;
	}

	static std::regex url_regex{URL_REGEX, std::regex_constants::extended};
	static std::regex frame_regex{FRAME_REGEX, std::regex_constants::extended};
}

class YamlConfigurationNode : public std::enable_shared_from_this<YamlConfigurationNode>
{
 public:
	struct Type {
		enum value { NONE, UINT32, INT32, FLOAT, BOOL, STRING, MAP, SEQUENCE, SEQUENCE_MAP, UNKNOWN };
		static const char * to_string(value v) {
			switch (v) {
			case NONE:     return "NONE";
			case UINT32:   return "unsigned int";
			case INT32:    return "int";
			case FLOAT:    return "float";
			case BOOL:     return "bool";
			case STRING:   return "string";
			case SEQUENCE: return "SEQUENCE";
			case MAP:      return "MAP";
			case SEQUENCE_MAP: return "SEQUENCE_MAP";
			default:       return "UNKNOWN";
			}
		}
	};

	YamlConfigurationNode() : name_("root"), type_(Type::UNKNOWN), is_default_(false) {}

	YamlConfigurationNode(std::string name)
		: name_(name), type_(Type::NONE), is_default_(false)
	{}

	YamlConfigurationNode(const YamlConfigurationNode &n) = delete;

	~YamlConfigurationNode()
	{
	}

	static std::shared_ptr<YamlConfigurationNode>
	create(const YAML::Node &node, const std::string& name = "root")
	{
		auto n = std::make_shared<YamlConfigurationNode>(name);

		switch (node.Type()) {
		case YAML::NodeType::Null:
			n->set_type(Type::NONE);
			break;

		case YAML::NodeType::Scalar:
			n->set_scalar(node.Scalar());
			n->verify_scalar(node);
			break;

		case YAML::NodeType::Sequence:
			n->set_type(Type::SEQUENCE);
			n->set_sequence(node);
			break;

		case YAML::NodeType::Map:
			n->set_type(Type::MAP);
			n->set_map(node);
			break;

		default:
			n->set_type(Type::UNKNOWN);
			break;
		}

		return n;
	}

	void add_child(std::string &p, std::shared_ptr<YamlConfigurationNode> n) {
		if (type_ != Type::MAP && type_ != Type::SEQUENCE_MAP) {
			type_ = Type::MAP;
		}
		children_[p] = n;
	}

	std::map<std::string, std::shared_ptr<YamlConfigurationNode>>::iterator  begin()
	{ return children_.begin(); }

	std::map<std::string, std::shared_ptr<YamlConfigurationNode>>::iterator  end()
	{ return children_.end(); }

	std::map<std::string, std::shared_ptr<YamlConfigurationNode>>::size_type size() const
	{ return children_.size(); }


	std::map<std::string, std::shared_ptr<YamlConfigurationNode>>::const_iterator  begin() const
	{ return children_.begin(); }

	std::map<std::string, std::shared_ptr<YamlConfigurationNode>>::const_iterator  end() const
	{ return children_.end(); }

	std::shared_ptr<YamlConfigurationNode>
	find(std::queue<std::string> &q)
	{

		std::shared_ptr<YamlConfigurationNode> n = shared_from_this();
		std::string path;

		while (! q.empty()) {
			std::string pel = q.front();

			path += "/" + pel;

			if (n->children_.find(pel) == n->children_.end()) {
				throw ConfigEntryNotFoundException(path.c_str());
			}
			n = n->children_[pel];

			q.pop();
		}

		return n;
	}

	std::shared_ptr<YamlConfigurationNode>
	find_or_insert(const char *path)
	{
		std::queue<std::string> q = str_split_to_queue(path);

		std::shared_ptr<YamlConfigurationNode> n = shared_from_this();
		while (! q.empty()) {
			std::string pel = q.front();
			if (n->children_.find(pel) == n->children_.end()) {
				n->add_child(pel, std::make_shared<YamlConfigurationNode>(pel));
			}
			n = n->children_[pel];
			q.pop();
		}

		return n;
	}

	void erase(const char *path)
	{
		std::queue<std::string> q = str_split_to_queue(path);
		std::stack<std::shared_ptr<YamlConfigurationNode>> qs;
		std::string full_path;

		std::shared_ptr<YamlConfigurationNode> n = shared_from_this();
		while (! q.empty()) {

			std::string pel = q.front();
			full_path += "/" + pel;

			if (n->children_.find(pel) == n->children_.end()) {
				throw ConfigEntryNotFoundException(full_path.c_str());
			}
			qs.push(n);
			n = n->children_[pel];

			q.pop();
		}

		if (n->has_children()) {
			throw Exception("YamlConfig: cannot erase non-leaf value");
		}

		std::shared_ptr<YamlConfigurationNode> child = n;
		while (! qs.empty()) {
			std::shared_ptr<YamlConfigurationNode> en = qs.top();

			en->children_.erase(child->name());

			// The node had more nodes than just the child, stop erasing
			if (en->has_children()) {
				break;
			}

			child = en;
			qs.pop();
		}
	}



	std::shared_ptr<YamlConfigurationNode>
	find(const char *path)
	{
		try {
			std::queue<std::string> pel_q = str_split_to_queue(path);
			return find(pel_q);
		} catch (Exception &e) {
			throw;
		}
	}

	void operator=(const YamlConfigurationNode&& n)
		{
			name_        = std::move(n.name_);
			type_        = std::move(n.type_);
			children_    = std::move(n.children_);
			list_values_ = std::move(n.list_values_);
		}

	bool operator< (const YamlConfigurationNode &n) const
	{ return this->name_ < n.name_; }

	std::shared_ptr<YamlConfigurationNode>
	operator[] (const std::string &p)
	{
		std::map<std::string, std::shared_ptr<YamlConfigurationNode>>::iterator i;
		if ((i = children_.find(p)) != children_.end()) {
			return i->second;
		} else {
			return NULL;
		}
	}

	std::shared_ptr<YamlConfigurationNode>
	operator+= (const std::shared_ptr<YamlConfigurationNode> n)
	{
		if (! n) return shared_from_this();

		std::shared_ptr<YamlConfigurationNode> add_to = shared_from_this();

		if (n->name() != "root") {
			if (children_.find(n->name()) == children_.end()) {
				auto new_val = std::make_shared<YamlConfigurationNode>(n->name());
				new_val->set_type(n->get_type());
				children_[n->name()] = new_val;
			}
			add_to = children_[n->name()];
		}

		if (add_to->is_scalar()) {
			if (! n->is_scalar()) {
				throw Exception("YamlConfig: cannot overwrite scalar value %s with non-scalar",
				                add_to->name().c_str());
			}
			add_to->set_scalar(n->get_scalar());
		} else if (add_to->is_list()) {
			if (! n->is_list()) {
				throw Exception("YamlConfig: cannot overwrite list value %s with non-list",
				                add_to->name().c_str());
			}
			if (is_type<unsigned int>()) {
				try {
					int v = get_int();
					if (v >= 0) {
						add_to->set_list(n->get_list<unsigned int>());
					} else {
						add_to->set_list(n->get_list<int>());
					}
				} catch (Exception &e) {
					// can happen if value > MAX_INT
					add_to->set_list(n->get_list<unsigned int>());
				}
			} else if (is_type<int>()) {
				add_to->set_list(n->get_list<int>());
			} else if (is_type<float>()) {
				add_to->set_list(n->get_list<float>());
			} else if (is_type<bool>()) {
				add_to->set_list(n->get_list<bool>());
			} else if (is_type<std::string>()) {
				add_to->set_list(n->get_list<std::string>());
			} else {
				std::vector<std::string> empty;
				add_to->set_list(empty);
			}
		} else if (add_to->get_type() == Type::SEQUENCE_MAP) {
			if (n->get_type() != Type::SEQUENCE_MAP) {
				throw Exception("YamlConfig: cannot overwrite sequence map value %s with non-sequence-map",
				                add_to->name().c_str());
			}
			add_to->children_.clear();
			for (auto i = n->begin(); i != n->end(); ++i) {
				*add_to += i->second;
			}
		} else {
			for (auto i = n->begin(); i != n->end(); ++i) {
				*add_to += i->second;
			}
		}

		return shared_from_this();
	}

	bool operator==(const YamlConfigurationNode &n) const
	{
		return (name_ == n.name_) && (type_ == n.type_) && (scalar_value_ == n.scalar_value_);
	}

	bool operator!=(const YamlConfigurationNode &n) const
	{
		return (name_ != n.name_) || (type_ != n.type_) || (scalar_value_ != n.scalar_value_);
	}

	/** Check for differences in two trees.
	 * This returns a list of all changes that have occured in b opposed
	 * to b. This means keys which have been added or changed in b compared to
	 * a. It also includes keys which have been removed from a, i.e. which exist
	 * in b but not in a.
	 * @param a root node of first tree
	 * @param b root node of second tree
	 * @return list of paths to leaf nodes that changed
	 */
	static std::list<std::string> diff(const std::shared_ptr<YamlConfigurationNode>a, const std::shared_ptr<YamlConfigurationNode>b)
	{
		std::list<std::string> rv;

		std::map<std::string, std::shared_ptr<YamlConfigurationNode>> na, nb;
		a->enum_leafs(na);
		b->enum_leafs(nb);
    
		std::map<std::string, std::shared_ptr<YamlConfigurationNode>>::iterator i;
		for (i = na.begin(); i != na.end(); ++i) {
			if (nb.find(i->first) == nb.end()) {
				// this is a new key in a
				// printf("A %s NOT in B\n", i->first.c_str());
				rv.push_back(i->first);
			} else if (*i->second != *nb[i->first]) {
				// different values/types
				// printf("A %s modified\n", i->first.c_str());
				rv.push_back(i->first);
			}
		}

		for (i = nb.begin(); i != nb.end(); ++i) {
			if (na.find(i->first) == na.end()) {
				// this is a new key in b
				// printf("B %s NOT in A\n", i->first.c_str());
				rv.push_back(i->first);
			}
		}

		return rv;
	}

	/** Retrieve value casted to given type T.
	 * @param path path to query
	 * @return value casted as desired
	 * @throw YAML::ScalarInvalid thrown if value does not exist or is of
	 * a different type.
	 */
	template<typename T>
		T
		get_value() const
	{
		if (type_ == Type::SEQUENCE) {
			throw Exception("YamlConfiguration: value of %s is a list", name_.c_str());
		}
		T rv;
		if (yaml_utils::convert(scalar_value_, rv)) {
			return rv;
		} else {
			// might want to have custom exception here later
			throw Exception("YamlConfig: value or type error on %s", name_.c_str());
		}
	}

	/** Get the list elements as string.
	 * The first element determines the type of the list.
	 * @return value string as list, i.e. as space-separated list of items
	 */
	std::string
		get_list_as_string() const
	{
		if (type_ != Type::SEQUENCE) {
			throw fawkes::Exception("YamlConfiguration: value of %s is not a list", name_.c_str());
		}
		if (list_values_.empty())  return "";

		std::string rv = "";
		bool is_string = (determine_scalar_type() == Type::STRING);
		if (is_string) {
			rv = " \"" + list_values_[0] + "\"";
			for (size_t i = 1; i < list_values_.size(); ++i) {
				rv += " \"" + list_values_[i] + "\"";
			}
		} else {
			rv = list_values_[0];
			for (size_t i = 1; i < list_values_.size(); ++i) {
				rv += " " + list_values_[i];
			}
		}

		return rv;
	}

	/** Retrieve value casted to given type T.
	 * @return value casted as desired
	 * @throw YAML::ScalarInvalid thrown if value does not exist or is of
	 * a different type.
	 */
	template<typename T>
		std::vector<T>
		get_list() const
	{
		if (type_ != Type::SEQUENCE) {
			throw Exception("YamlConfiguration: value of %s is not a list", name_.c_str());
		}
		std::vector<T> rv;
		const typename std::vector<T>::size_type N = list_values_.size();
		rv.resize(N);
		for (typename std::vector<T>::size_type i = 0; i < N; ++i) {
			T t;
			if (! yaml_utils::convert(list_values_[i], t)) {
				// might want to have custom exception here later
				throw Exception("YamlConfig: value or type error on %s[%zi]",
				                name_.c_str(), i);
			}
			rv[i] = t;
		}
		return rv;
	}

	/** Retrieve list size.
	 * @return size of list
	 * @throw YAML::ScalarInvalid thrown if value does not exist or is of
	 * a different type.
	 */
	size_t
		get_list_size() const
	{
		if (type_ != Type::SEQUENCE) {
			throw Exception("YamlConfiguration: value of %s is not a list", name_.c_str());
		}
		return list_values_.size();
	}

	/** Set value of given type T.
	 * @param path path to query
	 * @return value casted as desired
	 * @throw YAML::ScalarInvalid thrown if value does not exist or is of
	 * a different type.
	 */
	template<typename T>
		void
		set_value(const char *path, T t)
	{
		std::shared_ptr<YamlConfigurationNode> n = find_or_insert(path);
		if (n->has_children()) {
			throw Exception("YamlConfig: cannot set value on non-leaf path node %s", path);
		}
		n->set_scalar(StringConversions::to_string(t));
	}

	/** Set list of given type T.
	 * @param path path to query
	 * @return value casted as desired
	 * @throw YAML::ScalarInvalid thrown if value does not exist or is of
	 * a different type.
	 */
	template<typename T>
		void
		set_list(const char *path, std::vector<T> &t)
	{
		std::shared_ptr<YamlConfigurationNode> n = find_or_insert(path);
		if (n->has_children()) {
			throw Exception("YamlConfig: cannot set value on non-leaf path node %s", path);
		}
		std::vector<std::string> v;
		typename std::vector<T>::size_type N = t.size();
		v.resize(N);
		for (typename std::vector<T>::size_type i = 0; i < N; ++i) {
			v[i] = StringConversions::to_string(t[i]);
		}
		n->set_scalar_list(v);
	}

	/** Set value of given type T.
	 * @param path path to query
	 * @return value casted as desired
	 * @throw YAML::ScalarInvalid thrown if value does not exist or is of
	 * a different type.
	 */
	template<typename T>
		void
		set_value(T t)
	{
		if (has_children()) {
			throw Exception("YamlConfig: cannot set value on non-leaf path node %s", name_.c_str());
		}
		set_scalar(StringConversions::to_string(t));
	}


	/** Set list of values of given type T.
	 * @param path path to query
	 * @return value casted as desired
	 * @throw YAML::ScalarInvalid thrown if value does not exist or is of
	 * a different type.
	 */
	template<typename T>
		void
		set_list(const std::vector<T> &t)
	{
		if (has_children()) {
			throw Exception("YamlConfig: cannot set value on non-leaf path node %s", name_.c_str());
		}
		std::vector<std::string> v;
		typename std::vector<T>::size_type N = t.size();
		v.resize(N);
		for (typename std::vector<T>::size_type i = 0; i < N; ++i) {
			v[i] = StringConversions::to_string(t[i]);
		}
		set_scalar_list(v);
	}

	/** Check if value is of given type T.
	 * @param path path to query
	 * @return value casted as desired
	 * @throw YAML::ScalarInvalid thrown if value does not exist or is of
	 * a different type.
	 */
	template<typename T>
		bool
		is_type() const
	{
		T rv;
		if (type_ == Type::SEQUENCE) {
			if (! list_values_.empty()) {
				T rv;
				return (yaml_utils::convert(list_values_[0], rv));
			} else {
				return true;
			}
		} else {
			return (yaml_utils::convert(scalar_value_, rv));
		}
	}

	Type::value get_type() const
	{
		return type_;
	}

	bool is_scalar() const
	{
		switch (type_) {
		case Type::UINT32:
		case Type::INT32:
		case Type::FLOAT:
		case Type::BOOL:
		case Type::STRING:
			return true;
		default:
			return false;
		}
	}

	bool is_list() const
	{
		return (type_ == Type::SEQUENCE);
	}


	float get_float() const
	{
		return get_value<float>();
	}

	unsigned int get_uint() const
	{
		return get_value<unsigned int>();
	}

	int get_int() const
	{
		return get_value<int>();
	}

	bool get_bool() const
	{
		return get_value<bool>();
	}

	std::string get_string() const
	{
		return get_value<std::string>();
	}

	void set_scalar(const std::string &s) {
		scalar_value_ = s;
		type_ = determine_scalar_type();
	}

	void set_scalar_list(const std::vector<std::string> &s) {
		list_values_ = s;
		type_ = Type::SEQUENCE;
	}

	const std::string & get_scalar() const {
		return scalar_value_;
	}

	void set_sequence(const YAML::Node &n)
	{
		if (n.Type() != YAML::NodeType::Sequence) {
#ifdef HAVE_YAMLCPP_NODE_MARK
			throw Exception("Cannot initialize list from non-sequence (line %i, column %i)",
			                n.Mark().line, n.Mark().column);
#else
			throw Exception("Cannot initialize list from non-sequence");
#endif
		}
		type_ = Type::SEQUENCE;
		list_values_.resize(n.size());
		if (n.size() > 0) {
			if (n.begin()->Type() == YAML::NodeType::Scalar) {
				unsigned int i = 0;
				for (YAML::const_iterator it = n.begin(); it != n.end(); ++it) {
					list_values_[i++] = it->as<std::string>();
				}
			} else if (n.begin()->Type() == YAML::NodeType::Map) {
				type_ = Type::SEQUENCE_MAP;
				for (size_t i = 0; i < n.size(); ++i) {
					std::string key{std::to_string(i)};
					add_child(key, YamlConfigurationNode::create(n[i], key));
				}
			} else {
#ifdef HAVE_YAMLCPP_NODE_MARK
				throw Exception("Sequence neither of type scalar nor map (line %i, column %i)",
				                n.Mark().line, n.Mark().column);
#else
				throw Exception("Sequence neither of type scalar nor map");
#endif
			}
		}
	}

	void set_map(const YAML::Node &node)
	{
		for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
			std::string key = it->first.as<std::string>();
			std::shared_ptr<YamlConfigurationNode> in = shared_from_this();
			if (key.find("/") != std::string::npos) {
				// we need to split and find the proper insertion node
				std::vector<std::string> pel = str_split(key);
				for (size_t i = 0; i < pel.size() - 1; ++i) {
					std::shared_ptr<YamlConfigurationNode> n = (*in)[pel[i]];
					if (! n) {
						n = std::make_shared<YamlConfigurationNode>(pel[i]);
            in->add_child(pel[i], n);
          }
          in = n;
				}

        key = pel.back();
      }

			if (children_.find(key) != children_.end()) {
				// we are updating a value
				auto new_value = YamlConfigurationNode::create(it->second, key);
				if (new_value->get_type() != children_[key]->get_type()) {
#ifdef HAVE_YAMLCPP_NODE_MARK
					throw Exception("YamlConfig (line %d, column %d): overwriting value with incompatible type",
					                node.Mark().line, node.Mark().column);
#else
					throw Exception("YamlConfig: overwriting value with incompatible type");
#endif
				}
				in->add_child(key, new_value);
			} else {
				in->add_child(key, YamlConfigurationNode::create(it->second, key));
			}
		}
	}


	bool has_children() const
	{
		return ! children_.empty();
	}


	bool is_default() const
	{
		return is_default_;
	}

	void set_default(const char *path, bool is_default)
	{
		std::shared_ptr<YamlConfigurationNode> n = find(path);
		n->set_default(is_default);
	}

	void set_default(bool is_default)
	{
		is_default_ = is_default;
	}

	void enum_leafs(std::map<std::string, std::shared_ptr<YamlConfigurationNode>> &nodes,
	                std::string prefix = "") const
	{
		std::map<std::string, std::shared_ptr<YamlConfigurationNode>>::const_iterator c;
		for (c = children_.begin(); c != children_.end(); ++c) {
			std::string path = prefix + "/" + c->first;
			if (c->second->has_children()) {
				c->second->enum_leafs(nodes, path);
			} else {
				nodes[path] = c->second;
			}
		}
	}

	void print(std::string indent = "")
	{
		std::cout << indent << name_;
		if (! children_.empty()) {
			std::cout << std::endl;
			std::map<std::string, std::shared_ptr<YamlConfigurationNode>>::iterator c;
			for (c = children_.begin(); c != children_.end(); ++c) {
				c->second->print(indent + "  ");
			}
		} else {
			std::cout << indent << scalar_value_ << " (" << Type::to_string(get_type()) << ")" << std::endl;
		}
	}

 private:
	void emit(YAML::Emitter &ye) {
		if (! children_.empty()) {
			ye << YAML::BeginMap;

			std::map<std::string, std::shared_ptr<YamlConfigurationNode>>::iterator c;
			for (c = children_.begin(); c != children_.end(); ++c) {
				if (c->second->has_children()) {
					// recurse
					ye << YAML::Key << c->first << YAML::Value;
					c->second->emit(ye);
				} else {
					ye << YAML::Key   << c->first
					   << YAML::Value << c->second->get_scalar();
				}
			}

			ye << YAML::EndMap;
		}
	}

 public:
	void emit(std::string &filename)
	{
		if (access(filename.c_str(), W_OK) != 0) {
			if (errno != ENOENT) {
				throw Exception(errno, "YamlConfig: cannot write host file");
			}
		}

		std::ofstream fout(filename.c_str());
		YAML::Emitter ye;
		emit(ye);
		fout << ye.c_str();
	}

	const std::string &
		name() const
	{ return name_; }

 private:
	void set_name(const std::string &name)
	{
		name_ = name;
	}

	void set_type(Type::value type)
	{
		type_ = type;
	}

	Type::value determine_scalar_type() const
	{
		if (is_type<unsigned int>()) {
			try {
				int v = get_int();
				if (v >= 0) {
					return Type::UINT32;
				} else {
					return Type::INT32;
				}
			} catch (Exception &e) {
				// can happen if value > MAX_INT
				return Type::UINT32;
			}
		} else if (is_type<int>()) {
			return Type::INT32;
		} else if (is_type<float>()) {
			return Type::FLOAT;
		} else if (is_type<bool>()) {
			return Type::BOOL;
		} else if (is_type<std::string>()) {
			return Type::STRING;
		} else {
			return Type::UNKNOWN;
		}
	}


	void
	verify_scalar(const YAML::Node &node) const
	{
		if (node.Tag() == "tag:fawkesrobotics.org,cfg/ipv4" ||
		    node.Tag() == "tag:fawkesrobotics.org,cfg/ipv6")
		{
			std::string addr_s;
			try {
				addr_s = get_string();
			} catch (Exception &e) {
#ifdef HAVE_YAMLCPP_NODE_MARK
				e.prepend("YamlConfig (line %d, column %d): Invalid IPv4 or IPv6 address (not a string)",
				          node.Mark().line, node.Mark().column);
#else
				e.prepend("YamlConfig: Invalid IPv4 or IPv6 address (not a string)");
#endif
				throw;
			}

			if (node.Tag() == "tag:fawkesrobotics.org,cfg/ipv4") {
				struct in_addr addr;
				if (inet_pton(AF_INET, addr_s.c_str(), &addr) != 1) {
					throw Exception("YamlConfig: %s is not a valid IPv4 address", addr_s.c_str());
				}
			}
			if (node.Tag() == "tag:fawkesrobotics.org,cfg/ipv6") {
				struct in6_addr addr;
				if (inet_pton(AF_INET6, addr_s.c_str(), &addr) != 1) {
					throw Exception("YamlConfig: %s is not a valid IPv6 address", addr_s.c_str());
				}
			}

		} else if (node.Tag() == "tag:fawkesrobotics.org,cfg/tcp-port" ||
		           node.Tag() == "tag:fawkesrobotics.org,cfg/udp-port")
		{
			unsigned int p = 0;
			try {
				p = get_uint();
			} catch (Exception &e) {
#ifdef HAVE_YAMLCPP_NODE_MARK
				e.prepend("YamlConfig (line %d, column %d): Invalid TCP/UDP port number (not an unsigned int)",
				          node.Mark().line, node.Mark().column);
#else
				e.prepend("YamlConfig: Invalid TCP/UDP port number (not an unsigned int)");
#endif
				throw;
			}
			if (p <= 0 || p >= 65535) {
				throw Exception("YamlConfig: Invalid TCP/UDP port number "
				                "(%u out of allowed range)", p);
			}
		} else if (node.Tag() == "tag:fawkesrobotics.org,cfg/url") {
			std::string scalar = node.Scalar();
			if (! regex_match(scalar, yaml_utils::url_regex)) {
#ifdef HAVE_YAMLCPP_NODE_MARK
				throw Exception("YamlConfig (line %d, column %d): %s is not a valid URL",
				                node.Mark().line, node.Mark().column, scalar.c_str());
#else
				throw Exception("YamlConfig: %s is not a valid URL", scalar.c_str());
#endif
			}
		} else if (node.Tag() == "tag:fawkesrobotics.org,cfg/frame") {
			std::string scalar = node.Scalar();
			if (! regex_match(scalar, yaml_utils::frame_regex)) {
#ifdef HAVE_YAMLCPP_NODE_MARK
				throw Exception("YamlConfig (line %d, column %d): %s is not a valid frame ID",
				                node.Mark().line, node.Mark().column, scalar.c_str());
#else
				throw Exception("YamlConfig: %s is not a valid frame ID", scalar.c_str());
#endif
			}
		}
	}

 private:

	std::string name_;
	Type::value type_;
	std::string scalar_value_;
	std::map<std::string, std::shared_ptr<YamlConfigurationNode>> children_;
	std::vector<std::string> list_values_;
	bool is_default_;
};

/// @endcond

} // end namespace fawkes

#endif
