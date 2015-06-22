
/***************************************************************************
 *  yaml_node.h - Utility class for internal YAML config handling
 *
 *  Created: Thu Aug 09 14:08:18 2012
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef HAVE_YAMLCPP_0_5
#  ifdef HAVE_YAMLCPP_ATLEAST_0_3
// older versions of yaml-cpp had these functions in the
// YAML, rather than in the YAML::conversion namespace.
namespace YAML {
  namespace conversion {
    using YAML::IsInfinity;
    using YAML::IsNegativeInfinity;
    using YAML::IsNaN;
  }
}
#  else
// older versions do not have this at all
namespace YAML {
  namespace conversion {
    inline bool IsInfinity(const std::string& input) {
      return input == ".inf" || input == ".Inf" || input == ".INF" || input == "+.inf" || input == "+.Inf" || input == "+.INF";
    }

    inline bool IsNegativeInfinity(const std::string& input) {
      return input == "-.inf" || input == "-.Inf" || input == "-.INF";
    }

    inline bool IsNaN(const std::string& input) {
      return input == ".nan" || input == ".NaN" || input == ".NAN";
    }
  }
}
#  endif
#endif

namespace fawkes {

/// @cond INTERNALS

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
  }

class YamlConfigurationNode
{
 public:
  struct Type {
    enum value { NONE, UINT32, INT32, FLOAT, BOOL, STRING, MAP, SEQUENCE, UNKNOWN };
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
      default:       return "UNKNOWN";
      }
    }
  };

 YamlConfigurationNode() : name_("root"), type_(Type::UNKNOWN), is_default_(false) {}

  YamlConfigurationNode(const YamlConfigurationNode &n)
    : name_(n.name_), type_(n.type_), scalar_value_(n.scalar_value_),
    list_values_(n.list_values_), is_default_(n.is_default_)
  {}

  YamlConfigurationNode(const YamlConfigurationNode *n)
    : name_(n->name_), type_(n->type_), scalar_value_(n->scalar_value_),
    list_values_(n->list_values_), is_default_(n->is_default_)
  {}

  YamlConfigurationNode(std::string name, const YAML::Node &node)
    : name_(name), type_(Type::UNKNOWN), is_default_(false)
  {
#ifdef HAVE_YAMLCPP_0_5
    scalar_value_ = node.Scalar();
#else
    node.GetScalar(scalar_value_);
#endif
    switch (node.Type()) {
    case YAML::NodeType::Null:      type_ = Type::NONE; break;
    case YAML::NodeType::Scalar:    type_ = determine_scalar_type(); break;
    case YAML::NodeType::Sequence:  type_ = Type::SEQUENCE; set_sequence(node); break;
    case YAML::NodeType::Map:       type_ = Type::MAP; break;
    default:
      type_ = Type::UNKNOWN; break;
    }
  }


  YamlConfigurationNode(std::string name)
    : name_(name), type_(Type::NONE), is_default_(false) {}

  ~YamlConfigurationNode()
  {
    std::map<std::string, YamlConfigurationNode *>::iterator i;
    for (i = children_.begin(); i != children_.end(); ++i) {
      delete i->second;
    }
  }

  void add_child(std::string &p, YamlConfigurationNode *n) {
    type_ = Type::MAP;
    children_[p] = n;
  }

  std::map<std::string, YamlConfigurationNode *>::iterator  begin()
  { return children_.begin(); }

  std::map<std::string, YamlConfigurationNode *>::iterator  end()
  { return children_.end(); }

  std::map<std::string, YamlConfigurationNode *>::size_type size() const
  { return children_.size(); }


  std::map<std::string, YamlConfigurationNode *>::const_iterator  begin() const
  { return children_.begin(); }

  std::map<std::string, YamlConfigurationNode *>::const_iterator  end() const
  { return children_.end(); }

  YamlConfigurationNode * find(std::queue<std::string> &q)
  {

    YamlConfigurationNode *n = this;
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

  YamlConfigurationNode * find_or_insert(const char *path)
  {
    std::queue<std::string> q = str_split_to_queue(path);

    YamlConfigurationNode *n = this;
    while (! q.empty()) {
      std::string pel = q.front();
      if (n->children_.find(pel) == n->children_.end()) {
	n->add_child(pel, new YamlConfigurationNode(pel));
      }
      n = n->children_[pel];
      q.pop();
    }

    return n;
  }

  void erase(const char *path)
  {
    std::queue<std::string> q = str_split_to_queue(path);
    std::stack<YamlConfigurationNode *> qs;
    std::string full_path;

    YamlConfigurationNode *n = this;
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

    YamlConfigurationNode *child = n;
    while (! qs.empty()) {
      YamlConfigurationNode *en = qs.top();

      en->children_.erase(child->name());

      // The node had more nodes than just the child, stop erasing
      if (en->has_children()) {
	break;
      }

      child = en;
      qs.pop();
    }
  }



  YamlConfigurationNode * find(const char *path)
  {
    try {
      std::queue<std::string> pel_q = str_split_to_queue(path);
      return find(pel_q);
    } catch (Exception &e) {
      throw;
    }
  }

  void operator=(const YamlConfigurationNode &n)
  {
    name_        = n.name_;
    type_        = n.type_;
    children_    = n.children_;    
    list_values_ = n.list_values_;
  }

  bool operator< (const YamlConfigurationNode &n) const
  { return this->name_ < n.name_; }

  YamlConfigurationNode * operator[] (const std::string &p)
  {
    std::map<std::string, YamlConfigurationNode *>::iterator i;
     if ((i = children_.find(p)) != children_.end()) {
      return i->second;
    } else {
      return NULL;
    }
  }

  YamlConfigurationNode & operator+= (const YamlConfigurationNode *n)
  {
    if (! n) return *this;

    YamlConfigurationNode *add_to = this;

    if (n->name() != "root") {
      std::map<std::string, YamlConfigurationNode *>::iterator i;
      if ((i = children_.find(n->name())) == children_.end()) {
	children_[n->name()] = new YamlConfigurationNode(n);
      }
      add_to = children_[n->name()];
    }

    if (add_to->is_scalar()) {
      if (! n->is_scalar()) {
        throw Exception("YamlConfig: cannot overwrite scalar value %s with non-scalar",
			add_to->name().c_str());
      }
      add_to->set_scalar(n->get_scalar());
    } else {    
    
      std::map<std::string, YamlConfigurationNode *>::const_iterator i;
      for (i = n->begin(); i != n->end(); ++i) {
	*add_to += i->second;
      }
    }

    return *this;
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
  static std::list<std::string> diff(const YamlConfigurationNode *a, const YamlConfigurationNode *b)
  {
    std::list<std::string> rv;

    std::map<std::string, YamlConfigurationNode *> na, nb;
    a->enum_leafs(na);
    b->enum_leafs(nb);
    
    std::map<std::string, YamlConfigurationNode *>::iterator i;
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
    YamlConfigurationNode *n = find_or_insert(path);
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
    YamlConfigurationNode *n = find_or_insert(path);
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
  set_list(std::vector<T> &t)
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
      throw Exception("Cannot initialize list from non-sequence");
    }
    type_ = Type::SEQUENCE;
    list_values_.resize(n.size());
    unsigned int i = 0;
#ifdef HAVE_YAMLCPP_0_5
    for (YAML::const_iterator it = n.begin(); it != n.end(); ++it) {
      list_values_[i++] = it->as<std::string>();
#else
    for (YAML::Iterator it = n.begin(); it != n.end(); ++it) {
      *it >> list_values_[i++];
#endif
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
    YamlConfigurationNode *n = find(path);
    n->set_default(is_default);
  }

  void set_default(bool is_default)
  {
    is_default_ = is_default;
  }

  void enum_leafs(std::map<std::string, YamlConfigurationNode *> &nodes,
		  std::string prefix = "") const
  {
    std::map<std::string, YamlConfigurationNode *>::const_iterator c;
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
    std::cout << indent << name_ << " : ";
    if (! children_.empty()) {
      std::cout << std::endl;
      std::map<std::string, YamlConfigurationNode *>::iterator c;
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

      std::map<std::string, YamlConfigurationNode *>::iterator c;
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

  std::string name_;
  Type::value type_;
  std::string scalar_value_;
  std::map<std::string, YamlConfigurationNode *> children_;
  std::vector<std::string> list_values_;
  bool is_default_;
};

/// @endcond

} // end namespace fawkes

#endif
