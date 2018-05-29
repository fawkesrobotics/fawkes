#!/usr/bin/env python3

# Copyright (c) 2018 Tim Niemueller [www.niemueller.de]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import re
import sys
import yaml
import argparse
from pprint import pprint
import jinja2
from jinja2.ext import Extension
from jinja2.exceptions import TemplateRuntimeError

# Based on https://github.com/duelafn/python-jinja2-apci/blob/master/jinja2_apci/error.py
class RaiseExtension(Extension):
    # This is our keyword(s):
    tags = set(['raise'])

    # See also: jinja2.parser.parse_include()
    def parse(self, parser):
        # the first token is the token that started the tag. In our case we
        # only listen to "raise" so this will be a name token with
        # "raise" as value. We get the line number so that we can give
        # that line number to the nodes we insert.
        lineno = next(parser.stream).lineno

        # Extract the message from the template
        message_node = parser.parse_expression()

        return jinja2.nodes.CallBlock(
            self.call_method('_raise', [message_node], lineno=lineno),
            [], [], [], lineno=lineno
        )

    def _raise(self, msg, caller):
        raise TemplateRuntimeError(msg)

def filter_reftype(value):
	parts = value.split('/')
	if len(parts) != 4:
		return value
	if parts[0] == '#' and parts[1] == 'components' and parts[2] == 'schemas':
		return parts[3]
	else:
		return value

def filter_sanitize(name):
	# input[] => input
	name = name.replace("\\[\\]", "");

	# input[a][b] => input_a_b
	name = name.replace("\\[", "_");
	name = name.replace("\\]", "");

	# input(a)(b) => input_a_b
	name = name.replace("\\(", "_");
	name = name.replace("\\)", "");

	# input.name => input_name
	name = name.replace("\\.", "_");

	# input-name => input_name
	name = name.replace("-", "_");

	# input name and age => input_name_and_age
	name = name.replace(" ", "_");

	keywords = ["class", "operator"]
	if name in keywords:
		name = "_" + name

	return name

def filter_refs(allOf):
	rv = []
	for a in allOf:
		if '$ref' in a:
			rv.append(a['$ref'])
	return rv

def filter_quote(value):
	return "'%s'" % str(value)

def filter_rstrip(value):
	return str(value).rstrip()

def filter_prefix(value, prefix):
	rv = ""
	for l in value.splitlines(True):
		rv += prefix + l
	return rv

def recursive_transitive_types(types, type, all_schemas):
	if type in types: return
	types.add(type)
	if type in all_schemas:
		for n, s in all_schemas.items():
			if n not in types:
				if 'allOf' in s:
					for a in s['allOf']:
						if '$ref' in a:
							t = filter_reftype(a['$ref'])
							if t in types:
								recursive_transitive_types(types, n, all_schemas)
	return types

def filter_transitive_types(type, all_schemas):
	types = set()
	recursive_transitive_types(types, type, all_schemas)
	return sorted(types)

def filter_path_reftypes(spec):
	rv = set()
	for p in spec['paths']:
		for method in spec['paths'][p]:
			if 'parameters' in spec['paths'][p][method]:
				for param in spec['paths'][p][method]['parameters']:
					if param['in'] == 'body':
						if 'schema' in param:
							schema = param['schema']
							if 'type' in schema and schema['type'] == 'array':
								if '$ref' in schema['items']:
									rv.add(filter_reftype(schema['items']['$ref']))
								else:
									raise TemplateRuntimeError("Arrays as parameter types are only supported "+
									                           "for referenced types (%s)" % param)
							elif '$ref' in schema:
								rv.add(filter_reftype(schema['$ref']))
							else:
								raise TemplateRuntimeError("Only referenced schemas are supported "
								                           "as parameter types (%s %s %s)" %
								                           (param, method, param['name']))

			for code in spec['paths'][p][method]['responses']:
				if 'content' in spec['paths'][p][method]['responses'][code]:
					content = spec['paths'][p][method]['responses'][code]['content']
					if 'application/json' in content:
						schema = content['application/json']['schema']
						if 'type' in schema and schema['type'] == 'array':
							if '$ref' in schema['items']:
								rv.add(filter_reftype(schema['items']['$ref']))
							else:
								raise TemplateRuntimeError("Arrays as response types are only supported "+
								                           "for referenced types (%s)" % p)
						elif '$ref' in schema:
							rv.add(filter_reftype(schema['$ref']))
						else:
							raise TemplateRuntimeError("Only referenced schemas are supported "
							                           "as response types (%s %s %s)" % (p, method, code))

	return sorted(rv)

def filter_path_args(op, only_in=''):
	rv = []
	if 'parameters' in op:
		for p in op['parameters']:
			if 'schema' not in p:
				raise TemplateRuntimeError("Parameter requires schema "+
				                           "(%s of %s)" % (p.name, op.operationId))

			required = ('required' in p and p['required'])

			if p['in'] in ['path', 'query'] and (only_in == '' or p['in'] == only_in):
				if 'type' in p['schema'] \
				   and p['schema']['type'] in ['integer', 'number', 'string', 'boolean']:

					rv.append({"name": p['name'],
					           "type": p['schema']['type'],
					           "required": required})
				else:
					raise TemplateRuntimeError("Only primitive types are supported "+
					                           "for path args (%s of %s)" % (p.name, op.operationId))
			elif p['in'] == 'body' and (only_in == '' or p['in'] == only_in):
				if '$ref' in  p['schema']:
					rv.append({"name": p['name'],
					           "type": filter_reftype(p['schema']['$ref']),
					           "required": required})
				else:
					# We can do better here, generating appropriate objects
					rv.append({"name": p['name'], "type": "any", "required": required})
	return rv

def filter_path_substargs(path, op, replacement_pattern):
	if 'parameters' in op:
		for p in op['parameters']:
			if p['in'] == 'path':
				replace_by = replacement_pattern.replace('$$', filter_sanitize(p['name']))
				path = re.sub('\{%s\+?\}' % p['name'], replace_by, path)
	return path	

def filter_regex_replace(value='', pattern='', replacement=''):
    _re = re.compile(pattern)
    return _re.sub(replacement, value)

def write_file(filespec, res):
	if res is not None:
		if args.stdout:
			print(res)
		elif args.dry_run:
			print("Generated %s/%s.%s successfully (not writing, dry-run)" %
			      (args.output_dir, name, f['suffix']))
		else:
			if os.path.lexists(f['filename']) and not args.force:
				print("File %s already exists" % f['filename'])
				sys.exit(-2)
			with open(f['filename'], "w") as output_file:
				output_file.write(res)

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Run RCLL Cluster Sim Jobs')
	parser.add_argument('--api', '-a', metavar="FILE", required=True,
	                    help='OpenAPI specification file')
	parser.add_argument('--output-dir', '-d', metavar="DIR", default='.',
	                    help='Directory for output files (defaults to ".")')
	parser.add_argument('--template-dir', '-t', metavar="DIR", default='.',
	                    help='Directory that contains templates (defaults to ".")')
	parser.add_argument('--dry-run', action='store_true',
	                    help='Disable writing output files.')
	parser.add_argument('--stdout', action='store_true',
	                    help='Write output files to stdout.')
	parser.add_argument('--force', '-f', action='store_true',
	                    help='Overwrite existing files.')
	parser.add_argument('--quiet', '-q', action='store_true',
	                    help='Supress non-error output')
	parser.add_argument('file', metavar="FILE", nargs='+',
	                    help='Template files (must reside in template dir)')
	args = parser.parse_args()

	jinja = jinja2.Environment(loader=jinja2.FileSystemLoader(args.template_dir),
	                           extensions=[RaiseExtension],
	                           autoescape=False,
	                           line_statement_prefix = '%');

	jinja.filters['reftype']          = filter_reftype
	jinja.filters['sanitize']         = filter_sanitize
	jinja.filters['refs']             = filter_refs
	jinja.filters['quote']            = filter_quote
	jinja.filters['prefix']           = filter_prefix
	jinja.filters['rstrip']           = filter_rstrip
	jinja.filters['transitive_types'] = filter_transitive_types
	jinja.filters['path_reftypes']    = filter_path_reftypes
	jinja.filters['path_args']        = filter_path_args
	jinja.filters['path_substargs']   = filter_path_substargs
	jinja.filters['regex_replace']    = filter_regex_replace

	files = []
	for f in args.file:
		fname = os.path.basename(f)
		fname_p = fname.split('.')
		if len(fname_p) < 4:
			print("Invalid format for %s, must be of form 'name.<filext>.<type>.template'" % f)
			sys.exit(-1)
		files.append({"file": f,
		              "type": fname_p[-2],
		              "suffix": fname_p[-3],
		              "name": '.'.join(fname_p[:-2])})

	specfile = open(args.api, "r")
	spec = yaml.load(specfile)


	for f in files:
		template = jinja.get_template(f['file'])

		schemas = spec['components']['schemas']

		res = None

		if f['type'] == "model":
			for name, schema in schemas.items():
				schema = schemas[name]

				f['filename'] = "%s/%s.%s" % (args.output_dir, name, f['suffix'])

				if not args.quiet:
					print("%s" % f['filename'])

				vars = {
					"spec": spec,
					"name": name,
					"schema": schema,
					"all_schemas": schemas
				}

				res = template.render(vars)
				write_file(f, res)

		elif f['type'] == "api":
			vars = {
				"spec": spec,
				"name": filter_sanitize(spec['info']['title']),
				"all_schemas": schemas
			}

			f['filename'] = "%s/%sApiService.%s" % (args.output_dir, vars['name'], f['suffix'])

			if not args.quiet:
				print("%s: %s" % (f['name'], f['filename']))

			try:
				res = template.render(vars)
				write_file(f, res)
			except TemplateRuntimeError as err:
				print("Template error in %s: %s" % (f['file'], err))
				sys.exit(-3)
