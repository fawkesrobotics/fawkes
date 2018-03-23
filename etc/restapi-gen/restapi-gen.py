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
import sys
import yaml
import argparse
from pprint import pprint
import jinja2

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
	                           autoescape=False,
	                           line_statement_prefix = '%');

	jinja.filters['reftype']          = filter_reftype
	jinja.filters['sanitize']         = filter_sanitize
	jinja.filters['refs']             = filter_refs
	jinja.filters['quote']            = filter_quote
	jinja.filters['transitive_types'] = filter_transitive_types

	files = []

	for f in args.file:
		fname = os.path.basename(f)
		fname_p = fname.split('.')
		if len(fname_p) < 3:
			print("Invalid format for %s, must be of form 'name.filext.template'" % f)
			sys.exit(-1)
		files.append({"file": f, "suffix": fname_p[-2], "name": '.'.join(fname_p[:-2])})

	specfile = open(args.api, "r")
	spec = yaml.load(specfile)


	for f in files:
		template = jinja.get_template(f['file'])

		schemas = spec['components']['schemas']
	
		for name, schema in schemas.items():
			schema = schemas[name]
			if not args.quiet:
				print("%s: %s/%s.%s" % (f['name'], args.output_dir, name, f['suffix']))

			vars = {
				"spec": spec,
				"name": name,
				"schema": schema,
				"all_schemas": schemas
			}

			res = template.render(vars)

			if args.stdout:
				print(res)
			elif args.dry_run:
				print("Generated %s/%s.%s successfully (not writing, dry-run)" %
				      (args.output_dir, name, f['suffix']))
			else:
				filename = "%s/%s.%s" % (args.output_dir, name, f['suffix'])
				if os.path.lexists(filename) and not args.force:
					print("File %s already exists" % filename)
					sys.exit(-2)
				with open(filename, "w") as output_file:
					output_file.write(res)
