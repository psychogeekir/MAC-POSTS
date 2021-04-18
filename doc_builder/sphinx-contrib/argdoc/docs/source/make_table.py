#!/usr/bin/env python
"""This script generates a two-column reStructuredText table in which:

  - each row corresponds to a module in a Python package

  - the first column links to the module documentation

  - the second column links to the module source code generated by
    the 'viewcode' extension
"""
__date__   = "2015-07-4"
__author__ = "Joshua Griffin Dunn"
# coding": utf-8

import sys
import importlib
import argparse

from modulefinder import ModuleFinder
from sphinxcontrib.argdoc.ext import make_rest_table

def get_submodules(package):
    """Find names of all modules in `package`

    Parameters
    ----------
    package : imported Python package


    Returns
    -------
    list
        Sorted list of fully-qualified module names
    """
    mf = ModuleFinder()
    modules = sorted(["%s.%s" % (package.__name__,X) for X in mf.find_all_submodules(package) if X != "__init__"])
    return modules

def get_link_pair(modname):
    """Return a link to the Sphinx documentation and source code for module specified by `modname`

    Parameters
    ----------
    modname : str
        Python module name, fully-qualified


    Returns
    -------
    str
        Link to module documentation

    str
        Link to module source code
    """
    docsummary = importlib.import_module(modname).__doc__.split("\n")[0]

    slashname  = modname.replace(".","/")
    p1 = ":mod:`%s <%s>`" % (docsummary,modname)
    p2 = "`%s <_modules/%s.html>`_" % (modname,slashname)
    return p1, p2

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("package",type=str,help="Python package to document")
    parser.add_argument("outfile",type=str,help="Output file")
    parser.add_argument("--title",default=[],nargs=2,
                        help="Column titles (optional)")

    args    = parser.parse_args(argv)

    print("Importing package '%s'..." % args.package)
    package = importlib.import_module(args.package)
    modules = get_submodules(package)
    print("Found %s submodules..." % len(modules))

    pairs = [get_link_pair(X) for X in modules]
    title = False
    if len(args.title) > 0:
        print("Using column titles '%s' and '%s'" % (args.title[0],args.title[1]))
        title = True
        pairs = [tuple(args.title)] + pairs

    table = u"\n".join(make_rest_table(pairs,title=title))
    print("Writing to '%s'..." % args.outfile)
    with open(args.outfile,"w") as fout:
        fout.write(table)
        fout.write("\n")
        fout.close()

    print("Done.")

if __name__ == "__main__":
    main()