
------------------------------------------------------------------------
--  dotgrapher.lua - DOT Graph Generator Module
--
--  Created: Sun Mar 15 13:22:56 2009
--  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
--             2010       Carnegie Mellon University
--             2010       Intel Labs Pittsburgh
------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.

require("fawkes.modinit")

--- This module provides a state to create a Hybrid State Machine (HSM)
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)


function graph(name, graphtype)
   assert(name and type(name) == "string", "Invalid name passed for graph")
   local graphtype = graphtype or "graph"
   local g = {name=name, graphtype=graphtype, is_graph=true,
	      nodes={}, subgraphs={}, edges={}, attr={}, ranks={}}
   g.default_node = node(g, "")
   g.default_edge = edge(g, "", "")
   return g
end

function digraph(name)
   return graph(name, "digraph")
end

function subgraph(parentgraph, name)
   local sg = graph(name, "subgraph")
   table.insert(parentgraph.subgraphs, sg)
   return sg
end

function setv(o, attr, val)
   o.attr[attr] = val ~= nil and val or ""
end

function setvl(o, attrtable)
   if not attrtable then return end
   for k, v in pairs(attrtable) do
      o.attr[k] = v
   end
end

function getv(o, attr)
   return o.attr[attr]
end

function delv(o, attr)
   o.attr[attr] = nil
end

function addv(o, attr, val)
   if not o.attr[attr] then
      o.attr[attr] = val
   else
      o.attr[attr] = o.attr[attr] .. "," .. val
   end
end

function get_current_default_node(graph)
   return graph.default_node
end

function get_current_default_edge(graph)
   return graph.default_edge
end


function node(graph, name)
   local n = {name=name, attr={}, is_node=true}
   table.insert(graph.nodes, n)
   if name == "" then
      graph.default_node = n
   end
   return n
end

function edge(graph, from, to)
   local e = {from=from, to=to, attr={}, is_edge=true}
   table.insert(graph.edges, e)
   if name == "" then
      graph.default_edge = e
   end
   return e
end

function align(graph, group, name)
   if not graph.ranks[group] then
      graph.ranks[group] = {}
   end
   table.insert(graph.ranks[group], name)
end

function generate_attrappendix(attr)
   local attrstrings = {}
   for k,v in pairs(attr) do
      local s;
      if v == "" then
	 s = k
      else
	 s = string.gsub(string.format("%s=%q", k, v), "\\\\", "\\")
      end
      table.insert(attrstrings, s)
   end
   if #attrstrings > 0 then
      return " [" .. string.join(", ", attrstrings) .. "]"
   else
      return ""
   end
end


local function sanitize_node_name(name)
   if name ~= "" and not string.match(name, "^[%w_]+$") then
      return string.format("%q", name)
   else
      return name
   end
end

local function generate_node(node, indent)
   local s = ""

   if node.name == "" then
      -- it's a default node
      local attrapp = generate_attrappendix(node.attr)
      if #attrapp > 0 then
	 s = s .. indent .. "node" .. attrapp .. ";\n"
	 -- else ignore, because default nodes are only good for setting attribs
      end
   else
      if node.attr["comment"] then
	 if not node.attr["label"] then
	    node.attr["label"] = string.format("%s\\n(%s)", node.name, node.attr["comment"])
	 else
	    node.attr["label"] = string.format("%s\\n(%s)", node.attr["label"], node.attr["comment"])
	 end
      end
      local attrapp = generate_attrappendix(node.attr)
      s = s .. indent .. sanitize_node_name(node.name) .. attrapp .. ";\n"
   end

   return s
end

local function generate_edge(edge, indent)
   local s = ""
   local attrapp = generate_attrappendix(edge.attr)
   if edge.from == "" or edge.to == "" then
      -- it's a default node
      if #attrapp > 0 then
	 s = s .. indent .. "edge" .. attrapp .. ";\n"
	 -- else ignore, because default nodes are only good for setting attribs
      end
   else
      s = s .. indent .. sanitize_node_name(edge.from) ..
          " -> " .. sanitize_node_name(edge.to) .. attrapp .. ";\n"
   end
   return s
end

local function generate_rank(rank, indent)
   local s = ""
   s = s .. indent .. "{rank=same;\n"
   for _,n in ipairs(rank) do
      s = s .. indent .. "\t" .. sanitize_node_name(n) .. ";\n"
   end
   s = s .. indent .. "}\n"
   return s
end

local function generate_graph(graph, indent)
   local indent = indent or ""
   local s = indent .. graph.graphtype .. " " .. graph.name .. " {\n"

   local graphattr = generate_attrappendix(graph.attr)
   if #graphattr > 0 then
      s = s .. indent .. "\tgraph " .. graphattr .. ";\n"
   end

   s = s .. generate_node(graph.nodes[1], indent.."\t")
   s = s .. generate_edge(graph.edges[1], indent.."\t")


   for i,n in ipairs(graph.nodes) do
      if i > 1 then
	 s = s .. generate_node(n, indent.."\t")
      end
   end

   for _,sg in ipairs(graph.subgraphs) do
      s = s .. generate_graph(sg, indent .. "\t")
   end

   for i,e in ipairs(graph.edges) do
      if i > 1 then
	 s = s .. generate_edge(e, indent.."\t")
      end
   end

   for _,r in pairs(graph.ranks) do
      s = s .. generate_rank(r, indent .. "\t")
   end
   s = s .. indent .. "}\n";

   return s
end

function generate(graph)
   return generate_graph(graph)
end
