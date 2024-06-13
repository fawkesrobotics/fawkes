
/****************************************************************************
 *  TransformsGraph
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Transforms REST API.
 *  Transforms information and some calculations.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

#include "TransformsGraph.h"

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <sstream>

TransformsGraph::TransformsGraph()
{
}

TransformsGraph::TransformsGraph(const std::string &json)
{
	from_json(json);
}

TransformsGraph::TransformsGraph(const rapidjson::Value &v)
{
	from_json_value(v);
}

std::string
TransformsGraph::to_json(bool pretty) const
{
	rapidjson::Document d;

	to_json_value(d, d);

	rapidjson::StringBuffer buffer;
	if (pretty) {
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
		d.Accept(writer);
	} else {
		rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
		d.Accept(writer);
	}

	return buffer.GetString();
}

void
TransformsGraph::to_json_value(rapidjson::Document &d, rapidjson::Value &v) const
{
	rapidjson::Document::AllocatorType &allocator = d.GetAllocator();
	v.SetObject();
	// Avoid unused variable warnings
	(void)allocator;

	if (kind_) {
		rapidjson::Value v_kind;
		v_kind.SetString(*kind_, allocator);
		v.AddMember("kind", v_kind, allocator);
	}
	if (apiVersion_) {
		rapidjson::Value v_apiVersion;
		v_apiVersion.SetString(*apiVersion_, allocator);
		v.AddMember("apiVersion", v_apiVersion, allocator);
	}
	if (dotgraph_) {
		rapidjson::Value v_dotgraph;
		v_dotgraph.SetString(*dotgraph_, allocator);
		v.AddMember("dotgraph", v_dotgraph, allocator);
	}
}

void
TransformsGraph::from_json(const std::string &json)
{
	rapidjson::Document d;
	d.Parse(json);

	from_json_value(d);
}

void
TransformsGraph::from_json_value(const rapidjson::Value &d)
{
	if (d.HasMember("kind") && d["kind"].IsString()) {
		kind_ = d["kind"].GetString();
	}
	if (d.HasMember("apiVersion") && d["apiVersion"].IsString()) {
		apiVersion_ = d["apiVersion"].GetString();
	}
	if (d.HasMember("dotgraph") && d["dotgraph"].IsString()) {
		dotgraph_ = d["dotgraph"].GetString();
	}
}

void
TransformsGraph::validate(bool subcall) const
{
	std::vector<std::string> missing;
	if (!kind_)
		missing.push_back("kind");
	if (!apiVersion_)
		missing.push_back("apiVersion");
	if (!dotgraph_)
		missing.push_back("dotgraph");

	if (!missing.empty()) {
		if (subcall) {
			throw missing;
		} else {
			std::ostringstream s;
			s << "TransformsGraph is missing field" << ((missing.size() > 0) ? "s" : "") << ": ";
			for (std::vector<std::string>::size_type i = 0; i < missing.size(); ++i) {
				s << missing[i];
				if (i < (missing.size() - 1)) {
					s << ", ";
				}
			}
			throw std::runtime_error(s.str());
		}
	}
}
