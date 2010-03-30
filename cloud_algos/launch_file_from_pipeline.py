#!/usr/bin/env python

import sys
if len (sys.argv) < 2:
  print >> sys.stderr, "Usage: %s input.yaml > output.launch" % sys.argv[0]
  sys.exit ()

from yaml import load
try:
    from yaml import CLoader as Loader
    from yaml import CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

filename = sys.argv[1]

stream = open (filename, "r")
# load yaml file into a python dict
data = load(stream, Loader=Loader)

print "<launch>"
print "  <rosparam command=\"load\" file=\"%s\"/>" % filename
for (k,v) in data.iteritems(): 
  print "  <node pkg=\"%s\" name=\"%s\" type=\"%s\"/>" % (v['launch_pkg'], k, v['launch_type'])
print "</launch>"

