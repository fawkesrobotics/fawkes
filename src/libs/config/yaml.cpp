
/***************************************************************************
 *  yaml.cpp - Fawkes configuration stored in one or more YAML files
 *
 *  Created: Wed Aug 01 16:46:13 2012
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

#include <config/yaml.h>

#include <core/threading/mutex.h>
#include <core/exceptions/software.h>

#include <queue>
#include <fstream>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <yaml-cpp/exceptions.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

#define PATH_REGEX "^[a-zA-Z0-9_-]+$"
#define YAML_REGEX "^[a-zA-Z0-9_-]+\\.yaml$"



/// @cond INTERNALS

class YamlConfiguration::Node
{
 public:
  struct Type {
    enum value { NONE, UINT32, INT32, FLOAT, BOOL, STRING, MAP, SEQUENCE, UNKNOWN };
    static const char * to_string(value v) {
      switch (v) {
      case NONE:     return "NONE";
      case UINT32:   return "UINT32";
      case INT32:    return "INT32";
      case FLOAT:    return "FLOAT";
      case BOOL:     return "BOOL";
      case STRING:   return "STRING";
      case SEQUENCE: return "SEQUENCE";
      case MAP:      return "MAPE";
      default:       return "UNKNOWN";
      }
    }
  };

  Node() {}

  Node(const Node &n)
    : name_(n.name_), type_(n.type_), scalar_value_(n.scalar_value_)
  {}

  Node(const Node *n)
    : name_(n->name_), type_(n->type_), scalar_value_(n->scalar_value_)
  {}

  Node(std::string name, const YAML::Node &node)
    : name_(name)
  {
    node.GetScalar(scalar_value_);
    switch (node.Type()) {
    case YAML::NodeType::Null:      type_ = Type::NONE; break;
    case YAML::NodeType::Scalar:    type_ = determine_scalar_type(); break;
    case YAML::NodeType::Sequence:  type_ = Type::SEQUENCE; break;
    case YAML::NodeType::Map:       type_ = Type::MAP; break;
    default:
      type_ = Type::UNKNOWN; break;
    }
  }


  Node(std::string name)
    : name_(name), type_(Type::NONE) {}

  ~Node()
  {
    std::map<std::string, Node *>::iterator i;
    for (i = children_.begin(); i != children_.end(); ++i) {
      delete i->second;
    }
  }

  void add_child(std::string &p, Node *n) {
    type_ = Type::MAP;
    children_[p] = n;
  }

  std::map<std::string, Node *>::iterator  begin()
  { return children_.begin(); }

  std::map<std::string, Node *>::iterator  end()
  { return children_.end(); }

  std::map<std::string, Node *>::size_type size() const
  { return children_.size(); }


  std::map<std::string, Node *>::const_iterator  begin() const
  { return children_.begin(); }

  std::map<std::string, Node *>::const_iterator  end() const
  { return children_.end(); }

  Node * find(std::queue<std::string> &q)
  {
    std::string pel = q.front();
    if (children_.find(pel) == children_.end()) {
      throw Exception("YamlConfig: path element %s not found", pel.c_str());
    }
    q.pop();
    if (q.empty()) {
      return children_[pel];
    } else {
      return children_[pel]->find(q);
    }
  }


  Node * find(const char *path)
  {
    std::queue<std::string> pel_q = split_to_queue(path);
    return find(pel_q);
  }

  void operator=(const Node &n)
  {
    name_      = n.name_;
    children_  = n.children_;    
  }

  bool operator< (const Node &n) const
  { return this->name_ < n.name_; }

  Node * operator[] (const std::string &p)
  {
    std::map<std::string, Node *>::iterator i;
     if ((i = children_.find(p)) != children_.end()) {
      return i->second;
    } else {
      return NULL;
    }
  }

  Node & operator+= (const Node *n)
  {
    Node *add_to = this;

    if (n->name() != "root") {
      std::map<std::string, Node *>::iterator i;
      if ((i = children_.find(n->name())) == children_.end()) {
	children_[n->name()] = new Node(n);
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
    
      std::map<std::string, Node *>::const_iterator i;
      for (i = n->begin(); i != n->end(); ++i) {
	*add_to += i->second;
      }
    }

    return *this;
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
    T rv;
    if (YAML::Convert(scalar_value_, rv)) {
      return rv;
    } else {
      // might want to have custom exception here later
      throw Exception("YamlConfig: value or type error on %s", name_.c_str());
    }
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
    return YAML::Convert(scalar_value_, rv);
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


  float get_float()
  {
    return get_value<float>();
  }

  unsigned int get_uint()
  {
    return get_value<unsigned int>();
  }

  int get_int()
  {
    return get_value<int>();
  }

  bool get_bool()
  {
    return get_value<bool>();
  }

  Type::value determine_scalar_type()
  {
    try {
      get_uint();
      return Type::UINT32;
    } catch (Exception &e) {}

    try {
      get_int();
      return Type::INT32;
    } catch (Exception &e) {}

    try {
      get_float();
      return Type::FLOAT;
    } catch (Exception &e) {}

    try {
      get_bool();
      return Type::BOOL;
    } catch (Exception &e) {}

    try {
      get_string();
      return Type::STRING;
    } catch (Exception &e) {}

    return Type::UNKNOWN; 
  }

  std::string get_string()
  {
    return get_value<std::string>();
  }

  void set_scalar(const std::string &s) {
    scalar_value_ = s;
  }

  const std::string & get_scalar() const {
    return scalar_value_;
  }

  bool has_children() const
  {
    return ! children_.empty();
  }


  void enum_leafs(std::map<std::string, Node *> &nodes, std::string prefix = "")
  {
    std::map<std::string, Node *>::iterator c;
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
      std::map<std::string, Node *>::iterator c;
      for (c = children_.begin(); c != children_.end(); ++c) {
	c->second->print(indent + "  ");
      }
    } else {
      std::cout << indent << scalar_value_ << " (" << Type::to_string(get_type()) << ")" << std::endl;
    }
  }

  const std::string &
  name() const
  { return name_; }

 private:

  std::string name_;
  Type::value type_;
  std::string scalar_value_;
  std::map<std::string, Node *> children_;
};

/// @endcond

/** @class YamlConfiguration::YamlValueIterator <config/yaml.h>
 * Iterator for YAML config trees.
 * This iterator is used by YamlConfiguration as a result value
 * for queries. Its use is opaque and knowledge of
 * Configuration::ValueIterator will suffice for interaction.
 * @author Tim Niemueller
 */

/** Constructor.
 * Creates an iterator representing the invalid iterator.
 */
YamlConfiguration::YamlValueIterator::YamlValueIterator()
  : first_(true)
{
  current_ = nodes_.end();
}


/** Initializing constructor.
 * @param nodes nodes to iterate over
 */
YamlConfiguration::YamlValueIterator::YamlValueIterator(std::map<std::string, Node *> &nodes)
  : first_(true), nodes_(nodes)
{
  current_ = nodes_.end();
}

bool
YamlConfiguration::YamlValueIterator::next()
{
  if (first_) {
    first_ = false;
    current_ = nodes_.begin();
  } else {
    ++current_;
  }
  return (current_ != nodes_.end());
}

bool
YamlConfiguration::YamlValueIterator::valid() const
{
  return (current_ != nodes_.end());
}
    
const char *
YamlConfiguration::YamlValueIterator::path() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot get path of invalid iterator");
  }
  return current_->first.c_str();
}

const char *
YamlConfiguration::YamlValueIterator::type() const
{
  if (is_uint()) {
    return "unsigned int";
  } else if (is_int()) {
    return "int";
  } else if (is_float()) {
    return "float";
  } else if (is_bool()) {
    return "bool";
  } else if (is_string()) {
    return "string";
  } else {
    throw Exception("YamlValueIterator: value is of no known type");
  }
}
    
bool
YamlConfiguration::YamlValueIterator::is_float() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot check type on invalid iterator");
  }
  return current_->second->is_type<float>();
}

bool
YamlConfiguration::YamlValueIterator::is_uint() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot check type on invalid iterator");
  }
  return current_->second->is_type<unsigned int>();
}

bool
YamlConfiguration::YamlValueIterator::is_int() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot check type on invalid iterator");
  }
  return current_->second->is_type<int>();
}

bool
YamlConfiguration::YamlValueIterator::is_bool() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot check type on invalid iterator");
  }
  return current_->second->is_type<bool>();
}

bool
YamlConfiguration::YamlValueIterator::is_string() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot check type on invalid iterator");
  }
  return current_->second->is_type<std::string>();
}

float
YamlConfiguration::YamlValueIterator::get_float() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot get value of invalid iterator");
  }
  return current_->second->get_value<float>();
}

unsigned int
YamlConfiguration::YamlValueIterator::get_uint() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot get value of invalid iterator");
  }
  return current_->second->get_value<unsigned int>();
}

int
YamlConfiguration::YamlValueIterator::get_int() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot get value of invalid iterator");
  }
  return current_->second->get_value<int>();
}

bool
YamlConfiguration::YamlValueIterator::get_bool() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot get value of invalid iterator");
  }
  return current_->second->get_value<bool>();
}

std::string
YamlConfiguration::YamlValueIterator::get_string() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot get value of invalid iterator");
  }
  return current_->second->get_value<std::string>();
}

std::string
YamlConfiguration::YamlValueIterator::get_as_string() const
{
  if (current_ == nodes_.end()) {
    throw Exception("YamlValueIterator: cannot get value of invalid iterator");
  }
  return current_->second->get_value<std::string>();
}

std::string
YamlConfiguration::YamlValueIterator::get_comment() const
{
  return "";
}

bool
YamlConfiguration::YamlValueIterator::is_default() const
{
  return true;
}



/** @class YamlConfiguration <config/yaml.h>
 * Configuration store using YAML documents.
 * @author Tim Niemueller
 */


/** Constructor. */
YamlConfiguration::YamlConfiguration()
{
  root_ = NULL;
  mutex = new Mutex();

  __sysconfdir   = NULL;
  __userconfdir  = NULL;
  __default_file = NULL;
  __default_sql  = NULL;

#ifdef USE_REGEX_CPP
  __path_regex =  std::regex(PATH_REGEX, std::regex_constants::extended);
  __yaml_regex =  std::regex(YAML_REGEX, std::regex_constants::extended);
#else
  if (regcomp(&__path_regex, PATH_REGEX, REG_EXTENDED) != 0) {
    throw std::runtime_error("Failed to compile path regex");
  }
  if (regcomp(&__yaml_regex, YAML_REGEX, REG_EXTENDED) != 0) {
    throw std::runtime_error("Failed to compile YAML regex");
  }
#endif
}

/** Constructor.
 * @param sysconfdir system configuration directory, will be searched for
 * default configuration file, and system will try to create host-specific
 * database if writable
 * @param userconfdir user configuration directory, will be searched preferably
 * for default configuration file, and will be used to create host-specific
 * database if sysconfdir is not writable. This directory will be created
 * if it does not exist during load().
 */
YamlConfiguration::YamlConfiguration(const char *sysconfdir,
                                     const char *userconfdir)
{
  root_ = NULL;
  mutex = new Mutex();

  __sysconfdir   = strdup(sysconfdir);
  __default_file = NULL;
  __default_sql  = NULL;

#ifdef USE_REGEX_CPP
  __path_regex =  std::regex(PATH_REGEX, std::regex_constants::extended);
  __yaml_regex =  std::regex(YAML_REGEX, std::regex_constants::extended);
#else
  if (regcomp(&__path_regex, PATH_REGEX, REG_EXTENDED) != 0) {
    throw std::runtime_error("Failed to compile path regex");
  }
  if (regcomp(&__yaml_regex, YAML_REGEX, REG_EXTENDED) != 0) {
    throw std::runtime_error("Failed to compile YAML regex");
  }
#endif

  if (userconfdir != NULL) {
    __userconfdir  = strdup(userconfdir);
  } else {
    const char *homedir = getenv("HOME");
    if (homedir == NULL) {
      __userconfdir = strdup(sysconfdir);
    } else {
      if (asprintf(&__userconfdir, "%s/%s", homedir, USERDIR) == -1) {
	__userconfdir = strdup(sysconfdir);
      }
    }
  }
}

/** Destructor. */
YamlConfiguration::~YamlConfiguration()
{
  delete root_;
  root_ = NULL;

  if (__host_file)    free(__host_file);
  if (__default_file) free(__default_file);
  if (__default_sql)  free(__default_sql);
  if (__sysconfdir)   free(__sysconfdir);
  if (__userconfdir)  free(__userconfdir);
#ifndef USE_REGEX_CPP
  regfree(&__path_regex);
  regfree(&__yaml_regex);
#endif
  delete mutex;
}


void
YamlConfiguration::load(const char *file_path,
			const char *tag)
{

  if (file_path == NULL) {
    file_path = "config.yaml";
  }

  std::string filename;
  if (file_path[0] == '/') {
    filename = file_path;
  } else {

    const char *try_paths[] = {__userconfdir, __sysconfdir};
    int try_paths_len = 2;


    for (int i = 0; i < try_paths_len; ++i) {
      char *path;
      if (asprintf(&path, "%s/%s", try_paths[i], file_path) != -1) {
	if (access(path, R_OK) == 0) {
	  filename = path;
	  free(path);
	  break;
	}
	free(path);
      }
    }
    if (filename == "") {
      throw Exception("YamlConfig: cannot find configuration file %s/%s or %s/%s",
		      __userconfdir, file_path, __sysconfdir, file_path);
    }
  }

  root_ = new Node("root");

  std::queue<LoadQueueEntry> load_queue;
  load_queue.push(LoadQueueEntry(filename, false));

  while (! load_queue.empty()) {
    LoadQueueEntry &qe = load_queue.front();

    //printf("Loading %s\n", qe.filename.c_str());

    if (access(qe.filename.c_str(), R_OK) == -1) {
      if (qe.ignore_missing) {
	load_queue.pop();
	continue;
      }
      throw Exception(errno, "YamlConfig: cannot access file %s", qe.filename.c_str());
    }

    std::ifstream fin(qe.filename.c_str());
    YAML::Parser parser;
    YAML::Node doc1, doc2;
    bool have_doc1 = false, have_doc2 = false;

    try {
      parser.Load(fin);
      have_doc1 = parser.GetNextDocument(doc1);
      have_doc2 = parser.GetNextDocument(doc2);
    } catch (YAML::ParserException &e) {
      throw CouldNotOpenConfigException("Failed to parse %s line %i column %i: %s",
					qe.filename.c_str(), e.mark.line, e.mark.column,
					e.msg.c_str());
    }

    Node *sub_root = NULL;

    if (! have_doc1) {
      throw Exception("YamlConfig: file %s contains no document", qe.filename.c_str());
    } else if (have_doc1 && have_doc2) {
      // we have a meta info and a config document
      read_meta_doc(doc1, load_queue);
      read_config_doc(doc2, sub_root);
    } else {
      // only one, assume this to be the config document
      read_config_doc(doc1, sub_root);
    }

    if (sub_root) {
      *root_ += sub_root;
      delete sub_root;
    }

    load_queue.pop();
  }

  //root_->print();

}

/** Create absolute config path.
 * If the @p path starts with / it is considered to be absolute. Otherwise
 * it is prefixed with the config directory.
 * @param path path
 * @return absolute path
 */
static std::string abs_cfg_path(std::string &path)
{
  if (path[0] == '/') {
     return path;
  } else {
    return std::string(CONFDIR) + "/" + path;
  }
}


void
YamlConfiguration::read_meta_doc(YAML::Node &doc, std::queue<LoadQueueEntry> &load_queue)
{
  try {
    const YAML::Node &includes = doc["include"];
    for (YAML::Iterator it = includes.begin(); it != includes.end(); ++it) {
      std::string include;
      bool ignore_missing = false;
      *it >> include;
      if (it->Tag() == "tag:fawkesrobotics.org,cfg/ignore-missing") {
	ignore_missing = true;
      }

      if (include.empty()) {
	throw Exception("YamlConfig: invalid empty include");
      }
 
      if (include[include.size() - 1] == '/') {
	// this should be a directory
	std::string dirname = abs_cfg_path(include);
	struct stat dir_stat;
	if ((stat(dirname.c_str(), &dir_stat) != 0)) {
	  if (ignore_missing) continue;
	  throw Exception(errno, "YamlConfig: Failed to stat directory %s", dirname.c_str());
	}

	if (! S_ISDIR(dir_stat.st_mode)) {
	  throw Exception("YamlConfig: %s is not a directory", dirname.c_str());
	}

	DIR *d = opendir(dirname.c_str());
	if (! d) {
	  throw Exception(errno, "YamlConfig: failed to open directory %s",
			  dirname.c_str());
	}

	std::list<std::string> files;

	struct dirent *dent;
	while ((dent = readdir(d)) != NULL) {
#ifdef USE_REGEX_CPP
	  if (regex_search(dent->d_name, __yaml_regex)) {
#  if 0
	    // just for emacs auto-indentation
	  }
#  endif
#else
	  if (regexec(&__yaml_regex, dent->d_name, 0, NULL, 0) != REG_NOMATCH) {
#endif
	    std::string dn = dent->d_name;
	    //printf("Adding %s\n", (dirname + dn).c_str());
	    files.push_back(dirname + dn);
	  }
	}
	closedir(d);

	files.sort();
	for (std::list<std::string>::iterator f = files.begin(); f != files.end(); ++f) {
	    load_queue.push(LoadQueueEntry(*f, ignore_missing));
	}

      } else {
	load_queue.push(LoadQueueEntry(abs_cfg_path(include), ignore_missing));
      }
    }
  } catch (YAML::KeyNotFound &e) {
    //ignored, no includes
  }
}


void
YamlConfiguration::read_config_doc(const YAML::Node &doc, Node *&node)
{
  if (! node) {
    node = new Node("root");
  }

  if (doc.Type() == YAML::NodeType::Map) {
    //printf("RECURSE %s\n", node->name());
    for (YAML::Iterator it = doc.begin(); it != doc.end(); ++it) {
      std::string key;
      it.first() >> key;
      Node *in = node;
      if (key.find("/") != std::string::npos) {
	// we need to split and find the proper insertion node
	std::vector<std::string> pel = split(key);
	for (size_t i = 0; i < pel.size() - 1; ++i) {
	  Node *n = (*in)[pel[i]];
	  if (! n) {
	    n = new Node(pel[i]);
	    in->add_child(pel[i], n);
	  }
	  in = n;
	}

	key = pel.back();
      }

      Node *tmp = (*in)[key];
      if (tmp) {
	if (tmp->is_scalar() && it.second().Type() != YAML::NodeType::Scalar)
	{
	  throw Exception("YamlConfig: scalar %s cannot be overwritten by non-scalar",
			  tmp->name().c_str());
	}
	std::string s;
	if (it.second().GetScalar(s)) {
	  tmp->set_scalar(s);
	}
      } else {
	Node *tmp = new Node(key, it.second());
	in->add_child(key, tmp);
	read_config_doc(it.second(), tmp);
      }
    }
  } else if (doc.Type() == YAML::NodeType::Scalar) {
    if (doc.Tag() == "tag:fawkesrobotics.org,cfg/tcp-port" ||
	doc.Tag() == "tag:fawkesrobotics.org,cfg/udp-port")
    {
      unsigned int p = 0;
      try {
	p = node->get_uint();
      } catch (Exception &e) {
	e.prepend("YamlConfig: Invalid TCP/UDP port number (not an unsigned int)");
	throw;
      }
      if (p <= 0 || p >= 65535) {
	throw Exception("YamlConfig: Invalid TCP/UDP port number "
			"(%u out of allowed range)", p);
      }
    }
  }
}

void
YamlConfiguration::copy(Configuration *copyconf)
{
  throw NotImplementedException("YamlConfig does not support copying of a configuration");
}

void
YamlConfiguration::tag(const char *tag)
{
  throw NotImplementedException("YamlConfig does not support tagging a configuration");
}

std::list<std::string>
YamlConfiguration::tags()
{
  throw NotImplementedException("YamlConfig does not support tagging a configuration");
}


bool
YamlConfiguration::exists(const char *path)
{
  return false;
}


std::string
YamlConfiguration::get_type(const char *path)
{
  return "";
}

std::string
YamlConfiguration::get_comment(const char *path)
{
  return "";
}


/** Retrieve value casted to given type T.
 * @param root root node of the tree to search
 * @param path path to query
 * @return value casted as desired
 * @throw YAML::ScalarInvalid thrown if value does not exist or is of
 * a different type.
 */
template<typename T>
static inline T
get_value_as(YamlConfiguration::Node *root, const char *path)
{
  YamlConfiguration::Node *n = root->find(path);
  return n->get_value<T>();
}


float
YamlConfiguration::get_float(const char *path)
{
  return get_value_as<float>(root_, path);
}

unsigned int
YamlConfiguration::get_uint(const char *path)
{
  return get_value_as<unsigned int>(root_, path);
}

int
YamlConfiguration::get_int(const char *path)
{
  return get_value_as<int>(root_, path);
}

bool
YamlConfiguration::get_bool(const char *path)
{
  return get_value_as<bool>(root_, path);
}

std::string
YamlConfiguration::get_string(const char *path)
{
  return get_value_as<std::string>(root_, path);
}


/** Check if value is of given type T.
 * @param root root node of the tree to search
 * @param path path to query
 * @return true if value is of desired type, false otherwise
 */
template<typename T>
static inline bool
is_type(YamlConfiguration::Node *root, const char *path)
{
  YamlConfiguration::Node *n = root->find(path);
  return n->is_type<T>();
}


bool
YamlConfiguration::is_float(const char *path)
{
  return is_type<float>(root_, path);
}

bool
YamlConfiguration::is_uint(const char *path)
{
  return is_type<unsigned int>(root_, path);
}

bool
YamlConfiguration::is_int(const char *path)
{
  return is_type<int>(root_, path);
}

bool
YamlConfiguration::is_bool(const char *path)
{
  return is_type<bool>(root_, path);
}

bool
YamlConfiguration::is_string(const char *path)
{
  return is_type<std::string>(root_, path);
}


std::string
YamlConfiguration::get_default_comment(const char *path)
{
  return "";
}



bool
YamlConfiguration::is_default(const char *path)
{
  return true;
}


Configuration::ValueIterator *
YamlConfiguration::get_value(const char *path)
{
  YamlConfiguration::Node *n = root_->find(path);
  std::map<std::string, Node *> nodes;
  nodes[path] = n;
  return new YamlValueIterator(nodes);
}


void
YamlConfiguration::set_float(const char *path, float f)
{
}

void
YamlConfiguration::set_uint(const char *path, unsigned int uint)
{
}

void
YamlConfiguration::set_int(const char *path, int i)
{
}

void
YamlConfiguration::set_bool(const char *path, bool b)
{
}

void
YamlConfiguration::set_string(const char *path, const char *s)
{
}


void
YamlConfiguration::set_string(const char *path, std::string &s)
{
  set_string(path, s.c_str());
}

void
YamlConfiguration::set_comment(const char *path, const char *comment)
{
}

void
YamlConfiguration::set_comment(const char *path, std::string &comment)
{
}

void
YamlConfiguration::erase(const char *path)
{
}

void
YamlConfiguration::set_default_float(const char *path, float f)
{
}

void
YamlConfiguration::set_default_uint(const char *path, unsigned int uint)
{
}

void
YamlConfiguration::set_default_int(const char *path, int i)
{
}

void
YamlConfiguration::set_default_bool(const char *path, bool b)
{
}

void
YamlConfiguration::set_default_string(const char *path,
					const char *s)
{
}

void
YamlConfiguration::set_default_string(const char *path, std::string &s)
{
  set_default_string(path, s.c_str());
}

void
YamlConfiguration::set_default_comment(const char *path, const char *comment)
{
}

void
YamlConfiguration::set_default_comment(const char *path, std::string &comment)
{
  set_default_comment(path, comment.c_str());
}


void
YamlConfiguration::erase_default(const char *path)
{
}

/** Lock the config.
 * No further changes or queries can be executed on the configuration and will block until
 * the config is unlocked.
 */
void
YamlConfiguration::lock()
{
  mutex->lock();
}


/** Try to lock the config.
 * @see Configuration::lock()
 * @return true, if the lock has been aquired, false otherwise
 */
bool
YamlConfiguration::try_lock()
{
  return mutex->try_lock();
}

/** Unlock the config.
 * Modifications and queries are possible again.
 */
void
YamlConfiguration::unlock()
{
  mutex->unlock();
}


void
YamlConfiguration::try_dump()
{
}


Configuration::ValueIterator *
YamlConfiguration::iterator()
{
  std::map<std::string, Node *> nodes;
  root_->enum_leafs(nodes);
  return new YamlValueIterator(nodes);
}


/** Iterator for modified values.
 * Returns an iterator that can be used to iterate over all values that have been
 * modified in the default database in the last load (added, erased or changed).
 * @return iterator over all values
 */
Configuration::ValueIterator *
YamlConfiguration::modified_iterator()
{
  return NULL;
}

Configuration::ValueIterator *
YamlConfiguration::search(const char *path)
{
  Node *n = root_->find(path);
  std::map<std::string, Node *> nodes;
  n->enum_leafs(nodes, path);
  return new YamlValueIterator(nodes);
}

/** Split string into vector of strings at delimiting character.
 * @param s string to split
 * @param delim character delimiting strings
 * @return vector of strings resulting from the parsed string. Empty
 * values are removed.
 */
std::vector<std::string>
YamlConfiguration::split(const std::string &s, char delim)
{
  std::vector<std::string> elems;
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss, item, delim)) {
    if (item != "")  elems.push_back(item);
  }
  return elems;
}


/** Split string into queue of strings at delimiting character.
 * @param s string to split
 * @param delim character delimiting strings
 * @return vector of strings resulting from the parsed string. Empty
 * values are removed.
 */
std::queue<std::string>
YamlConfiguration::split_to_queue(const std::string &s, char delim)
{
  std::queue<std::string> elems;
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss, item, delim)) {
    if (item != "")  elems.push(item);
  }
  return elems;
}

void
YamlConfiguration::verify_name(const char *name) const
{
#ifdef USE_REGEX_CPP
  if (! regex_search(name, __path_regex)) {
#  if 0
    // just for emacs auto-indentation
  }
#  endif
#else
  if (regexec(&__path_regex, name, 0, NULL, 0) == REG_NOMATCH) {
#endif
    throw std::runtime_error("Invalid collection name");
  }
}

/** Query node for a specific path.
 * @param path path to retrieve node for
 * @return node representing requested path query result, if the path only
 * consists of collection and path name returns the whole document.
 */
YamlConfiguration::Node *
YamlConfiguration::query(const char *path) const
{
  std::queue<std::string> pel_q = split_to_queue(path);
  return root_->find(pel_q);
}


} // end namespace fawkes
